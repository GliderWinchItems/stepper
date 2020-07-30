/******************************************************************************
* File Name          : SwitchTask.c
* Date First Issued  : 02/042020
* Description        : Updating switches from spi shift register
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "SwitchTask.h"
#include "spiserialparallel.h"
#include "SpiOutTask.h"
#include "shiftregbits.h"
#include "GevcuTask.h"
#include "morse.h"

/* Theoretical: 10 spi cycles per millisecond, and each has a switch change
   that puts a uint16_t word on the queue. Obviously, something wrong with
   shift-register and switches. 128 words would be about 12.8 ms before there
   would be a queue overrun. */
#define SWITCHQSIZE 128

osThreadId SwitchTaskHandle    = NULL;
osMessageQId SwitchTaskQHandle = NULL;


/* NOTE: Switch i/o pins have pull-ups and closing the switch pulls the
   pin to ground, hence, a zero bit represents a closed contact. */

enum SWPAIRSTATE
{
	SWP_UNDEFINED, /* 0 Sw state is initially undefined */
	SWP_NC,        /* 1 Normally closed contact */
	SWP_NO,        /* 2 Normally opened contact */
	SWP_DEBOUNCE,  /* 3 NC has made contact, but debounce in progress */
	SWP_DEBOUNCE_OPENING,
	SWP_DEBOUNCE_CLOSING,
};

/* Pushbutton states. */
enum SWPBSTATE
{
	SWPB_OPEN,
	SWPB_CLOSED,
	SWPB_OPENING,
	SWPB_CLOSING
};

/* Reconstructed Local copy (possibly delayed slightly) of spi read word. */
uint16_t spilocal;

uint32_t spitickctr; // Running count of spi time ticks (mostly for debugging)
uint32_t swxctr;

// TODO make these static after debugging
/* Pushbutton linked list head pointer. */
struct SWITCHPTR* phdsw = NULL; // Pointer link head: switches instantiated
struct SWITCHPTR* phddb = NULL; // Pointer link head: active debouncing

/* Array for struct access via switch bit position. */
struct SWITCHPTR* pchange[NUMSWS]  = {0};

/* *************************************************************************
 * static void debouncing_add(struct SWITCHPTR* p);
 *	@brief	: Add 'this' switch to the list that is active debouncing.
 * @param	: p = pointer to struct for this pushbutton 
 * *************************************************************************/
static void debouncing_add(struct SWITCHPTR* p)
{
	if (phddb == NULL)
	{ // Here, no switches are active debouncing 
		phddb = p;       // Point debounce head to this switch struct
		p->pdbnx = NULL; // This switch is the last on the debouncing list
	}
	else
	{ // Here, one or more on debouncing list
		// Add to head of list
		p->pdbnx = phddb;
		phddb = p;
	}
	return;
}
/*******************************************************************************
Inline assembly.  Hacked from code at the following link--
http://balau82.wordpress.com/2011/05/17/inline-assembly-instructions-in-gcc/
*******************************************************************************/
static inline __attribute__((always_inline)) uint32_t arm_clz(uint32_t v) 
{
  unsigned int d;
  asm ("CLZ %[Rd], %[Rm]" : [Rd] "=r" (d) : [Rm] "r" (v));
  return d;
}

/* *************************************************************************
 * struct SWITCHPTR* switch_pb_add(osThreadId tskhandle, uint32_t notebit, 
    uint32_t switchbit,
    uint32_t switchbit1,
    uint8_t type,
    uint8_t db_mode, 
    uint8_t dur_closing,
    uint8_t dur_opening);
 *
 *	@brief	: Add a single on/off (e.g. pushbutton) switch on a linked list
 * @param	: tskhandle = Task handle; NULL to use current task; 
 * @param	: notebit = notification bit; 0 = no notification
 * @param	: switchbit  = bit position in spi read word for this switch
 * @param	: switchbit1 = bit position for 2nd switch if a switch pair
 * @param	: type: 0 = on/off (pushbutton), 1 = switch pair
 * @param	: db_mode = debounce mode: 0 = immediate; 1 = wait debounce
 * @param	: dur_closing = number of spi time tick counts for debounce
 * @param	: dur_opening = number of spi time tick counts for debounce
 * @return	: Pointer to struct for this switch
 * *************************************************************************/
 struct SWITCHPTR* switch_pb_add(osThreadId tskhandle, uint32_t notebit, 
    uint32_t switchbit,
    uint32_t switchbit1,
    uint8_t type,
    uint8_t db_mode, 
    uint8_t dur_closing,
    uint8_t dur_opening)
{
	struct SWITCHPTR* p1 = phdsw;
	struct SWITCHPTR* p2;
	uint16_t n,nn;
	
taskENTER_CRITICAL();

	if (p1 == NULL)
	{ // Here, get first sw struct for list
		p2 = (struct SWITCHPTR*)calloc(1, sizeof(struct SWITCHPTR));
		if (p2 == NULL){taskEXIT_CRITICAL(); morse_trap(524);}
		phdsw = p2; // Head now points to 1st on list.
	}
	else
	{ // One or more on linked list

// TODO Error-check if switch already on linked list

		while (p1->pnext != NULL) // Find last item
		{
			p1 = p1->pnext;
		}
		p2 = (struct SWITCHPTR*)calloc(1, sizeof(struct SWITCHPTR));
		if (p2 == NULL){taskEXIT_CRITICAL(); morse_trap(525);}

		// Point previous last item to newly added item.
		p1->pnext = p2; 
	}

	/* Initialize linked list switch struct */
// Redundant: calloc takes care of this
	p2->pdbnx = NULL; // Not active debouncing

	// Task Notification. notebit = 0 means skip notification.
	if (tskhandle == NULL) // Use calling task handle?
		 tskhandle  = xTaskGetCurrentTaskHandle();
	p2->tskhandle  = tskhandle;
	p2->notebit    = notebit;   // Notification bit to use

	// Designation of the switch(s) in the spi word
	p2->switchbit  = switchbit; // 1st sw position in spi word 
	p2->switchbit1 = switchbit1;// 2nd sw if a switch pair

	// Debouncing logic & timing
// Note: calloc sets to zero, some of the following is redundant
	p2->on         = 0;      // Initial sw contact status (closed)   
	p2->db_on      = 0;      // Initial post-debounce status (closed)
	p2->db_ctr     = 0;      // Working debounce timing counter
	p2->type       = type;   // Switch type: pushbutton
	p2->state      = SWPB_CLOSED; // vars initially zero, so closed
	p2->db_dur_closing = dur_closing; // spi tick count closing
	p2->db_dur_opening = dur_opening; // spi tick count opening
	p2->db_mode        = db_mode;     // Type of logic applied

	/* Map switch bit position (of xor'd changes) to this new struct. */
	// This allows bit position to find sw struct without search
	n = arm_clz(p2->switchbit); // Count leading zeros
	if (n == 32) morse_trap(526); // Prog error: switchbit was zero!
	nn = 31 - n; // n=31 is bit position 0; n=0 is bit pos 31
	// Currently, only two bytes in the spi word.
	if (nn > (SPISERIALPARALLELSIZE*8)) morse_trap(527); // Bit pos out-of-range
	pchange[nn] = p2; // Bit position pointer to new struct for 1st switch

	/* If this is a switch pair set point in array. */
	if (p2->switchbit1 != 0) // Was a 2nd switch identified?
	{ // Here, yes, this is a switch pair
		n = arm_clz(p2->switchbit1); // Count leading zeros
		nn = 31 - n; // n=31 is bit position 0; n=0 is bit pos 31
		if (nn > (SPISERIALPARALLELSIZE*8)) morse_trap(527); // Bit pos out-of-range
		pchange[nn] = p2; // Bit position pointer for 2nd switch
	}

taskEXIT_CRITICAL();
	return p2; // Return pointer to first switch (in case of a pair)
}

/* *************************************************************************
 * static void dbends(struct SWITCHPTR* p);
 *	@brief	: Handle debounce duration ended.
 * @param	: p = pointer to struct for a sw on the debounce linked list
 * *************************************************************************/
static void dbends(struct SWITCHPTR* p)
{
	/* Here, the end of the debounce duration. */
	switch(p->state)
	{
	case SWPB_OPENING: // Switch is in process of opening
		if (p->on == SW_CLOSED)
		{ // Here, sw closed during a sw opening debounce period
			p->state = SWPB_CLOSED;
			if (p->db_mode == SWMODE_NOW)
			{
				p->db_on = SW_CLOSED; // Show representation sw closed
				if (p->notebit != 0)
					xTaskNotify(p->tskhandle, p->notebit, eSetBits);
			}
		}
		else
		{ // Here, sw is open and state is opening (from closed)
			p->state = SWPB_OPEN; // Set constant open state
			if (p->db_mode == SWMODE_WAIT)
			{ // Here, mode is wait until debounce completes
				p->db_on = SW_OPEN;   // Show representation sw open
				if (p->notebit != 0)
					xTaskNotify(p->tskhandle, p->notebit, eSetBits);
			}
		}
		break;		

	case SWPB_CLOSING: // Switch is in process of closing
		if (p->on != SW_CLOSED)			
		{ // Here, sw opened during the debounce duration
			p->state = SWPB_OPEN; // We are back to open state
			if (p->db_mode == SWMODE_NOW)
			{
				p->db_on = SW_OPEN;   // Show representation sw open
				if (p->notebit != 0)
					xTaskNotify(p->tskhandle, p->notebit, eSetBits);
			}
	//    else{ // mode is delay/wait, so db_on remains unchanged}
		}
		else
		{ // Here, debounce complete and sw still closed
			p->state = SWPB_CLOSED;				
			if (p->db_mode == SWMODE_WAIT)
			{
				p->db_on = SW_CLOSED;
				if (p->notebit != 0)
					xTaskNotify(p->tskhandle, p->notebit, eSetBits);
			}
		}
		break;
	}
	return;
}
/* *************************************************************************
 * static void pbxor(struct SWITCHPTR* p);
 *	@brief	: logic for pushbutton bit *changed*
 * @param	: p = pointer to struct for this pushbutton 
 * @param	: mask for clearing bits
 * *************************************************************************/
static void pbxor(struct SWITCHPTR* p)
{
	/* Save present contact state: 1 = open, 0 = closed. */
	p->on = p->switchbit & spilocal; 

	switch(p->state)
	{
	case SWPB_OPEN: // Current debounce state: open
		if (p->on == SW_CLOSED)
		{ // Here, change: open state -> closed contact
			if (p->db_dur_closing == 0) // Debounce count?
			{ // Special case: instantaneous state change, no debounce
				p->db_on = SW_CLOSED;    // Debounced: representative of sw
				p->state = SWPB_CLOSED;  // New sw state
				if (p->notebit != 0)
					xTaskNotify(p->tskhandle, p->notebit, eSetBits);
			}
			else
			{ // One or more debounce ticks required.
				if (p->db_mode == SWMODE_NOW)
				{ // Debounced sw representation shows new contact state "now"
					p->db_on = SW_CLOSED;
					if (p->notebit != 0) // Notify new state, if indicated
						xTaskNotify(p->tskhandle, p->notebit, eSetBits);
				}
				p->state = SWPB_CLOSING; // New state: closing is in process
				p->db_ctr = p->db_dur_closing; // Debounce count
				debouncing_add(p); // Add to debounce active linked list
			}
		}
		else
		{ // Here, change: open state -> open contact (what!?)
			p->db_on = SW_OPEN;
		}
		break;

	case SWPB_CLOSED: // Current debounce state: closed
		if (p->on != SW_CLOSED) // Not closed?
		{ // Here, change: closed state -> open contact
			if (p->db_dur_opening == 0) // Debounce count?
			{ // Special case: instantaneous state change
				p->db_on = SW_OPEN;
				p->state = SWPB_OPEN;
				if (p->notebit != 0)
					xTaskNotify(p->tskhandle, p->notebit, eSetBits);
			}
			else
			{ // One or more debounce ticks required.
				if (p->db_mode == SWMODE_NOW)
				{
					p->db_on = SW_OPEN;
					if (p->notebit != 0)
						xTaskNotify(p->tskhandle, p->notebit, eSetBits);
				}
				p->state = SWPB_OPENING;
				p->db_ctr = p->db_dur_opening; // Set debounce time counter
				debouncing_add(p); // Add to linked list
			}
		}
		else
		{ // Here, change: open state -> open contact (what!?)
			p->db_on = SW_OPEN;
		}
		break;

	case SWPB_OPENING: // sw changed during debounce period
		p->db_ctr = p->db_dur_opening; // Extend debounce time counter
		break;

	case SWPB_CLOSING: // sw changed during debounce period
		p->db_ctr = p->db_dur_closing; // Extend debounce count
		break;			

	default:
		morse_trap(544); // Programming error
	}
	return;
}
/* *************************************************************************
 * static void p2xor(struct SWITCHPTR* p);
 *	@brief	: logic for two switch pair
 * @param	: p = pointer to struct for this switch struct
 * @param	: p->db_on set with sw pair of 0b10 or 0b01.
 * *************************************************************************/
static void p2xor(struct SWITCHPTR* p)
{
	/* Convert sw pair status to a two bit value. */
	p->on = 0x0; // Both sws c;psed
	if ((p->switchbit & spilocal) != SW_CLOSED)
		p->on |= 0x1; // 1st sw open
	if ((p->switchbit1 & spilocal) != SW_CLOSED)
		p->on |= 0x2; // 2nd sw open

	switch (p->on)
	{
	case 0: // Both closed: indeterminant, no change
		break;	
	case 1: // 1st open: 2nd closed
		if (p->db_on == p->on) break;
		p->db_on = p->on;
			if (p->notebit != 0) // Notify task?
				xTaskNotify(p->tskhandle, p->notebit, eSetBits);	
		break;	
	case 2: // 1st closed: 2nd open
		if (p->db_on == p->on) break;
		p->db_on = p->on;
			if (p->notebit != 0) // Notify task?
				xTaskNotify(p->tskhandle, p->notebit, eSetBits);
		break;	
	case 3: // Both open: indeterminant, no change
		break;	
	}
	return;
}

/* *************************************************************************
 * static void spitick(void);
 *	@brief	: Debounce timing
 * *************************************************************************/
static void spitick(void)
{
	spitickctr += 1; // Mostly for Debugging & Test

	/* Traverse linked list of pushbuttons actively debouncing. */
	struct SWITCHPTR* p = phddb;
	struct SWITCHPTR* p2;

	while (p != NULL)
	{
		/* Countdown debounce timing. */
		if (p->db_ctr != 0)
		{ // Here still counting down 
			p->db_ctr -= 1;
			if (p->db_ctr == 0) 
			{ // Here, debounced ended with this tick
				dbends(p); // Do debounce logic
		
		/* Remove from debounce list */
				if (p->pdbnx == NULL)
				{ // Here, 'p' is last on list
					if (p == phddb)
					{ // Here, it was the only one on the list
						phddb = NULL; // Set head: list empty
					}
					else
					{ // Previous list item is now last
						p2->pdbnx = NULL; 
					}
				}
				else
				{ // Here, 'p' is not last
					if (p == phddb)
					{ // Here, it is the first 
						phddb = p->pdbnx; // Head points where 1st pointed to
					}
					else
					{ // Here, somewhere in the middle
						p2->pdbnx = p->pdbnx; // Set new next for previous
					}
				}
			}
		}
		p2 = p; // Save "previous"
		p = p->pnext; // Get next in list
	}
	return;
}
/* *************************************************************************
 * static void swchanges(uint16_t spixor);
 *	@brief	: The queue'd word is not zero, meaning one or more sws changed
 * *************************************************************************/
static void swchanges(uint16_t spixor)
{
	struct SWITCHPTR* p;
	uint32_t n,nn;            // Leading zeros, counts
	uint32_t xortmp = spixor; // Working word of change bits

	/* Deal with switches that have a change. */
	while (xortmp != 0)
	{
		/* Get bit position of changed bit w asm instruction. */
		n = arm_clz(xortmp); // Count leading zeros
		if (n == 32) morse_trap(528); // Shouldn't happen

		nn = (31 - n); // n=31 is bit position 0; n=0 is bit pos 31
		if (nn >= (SPISERIALPARALLELSIZE*8)) morse_trap(529); 

		/* Point to struct for this switch */
		p = pchange[nn]; // Point to struct for this sw or sw pair

		/* If p == NULL, then it would be a non-instantiated sw, 
         which with pullups are ones, but initial vars zero, so
         we end up here with a NULL ptr. */
		if (p != NULL)
		{ // 
			if (p->type == SWTYPE_PB)
			{ // Here, a pushbutton type
				pbxor(p);
				xortmp = (xortmp & ~(p->switchbit) );
			}
			else
			{ // Here, not a pushbutton type
				if (p->type == SWTYPE_PAIR)
				{ // Here, a switch pair
					p2xor(p);
					/* If by chance both sw bits show changed in the spi word
					   then, p2xor will deal with both correctly, and thus
					   both bits are cleared from the spixor word. */
					xortmp = (xortmp & ~(p->switchbit|p->switchbit1) );
				}
				else
				{ // Program error (morse assert?)
					morse_trap(534);
				}
			}
		}
		else
		{ // Clear bit for non-instantiated sw
			xortmp = (xortmp & ~(1 << nn) );
		}
	}
	return;
}
/* *************************************************************************
 * void StartSwitchTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartSwitchTask(void* argument)
{
	BaseType_t qret;
	uint16_t spixor; // spi read-in difference

   /* Infinite loop */
   for(;;)
   {
		qret = xQueueReceive( SwitchTaskQHandle,&spixor,portMAX_DELAY);
		if (qret == pdPASS)
		{ // Here spi interrupt loaded a xor'd read-word onto the queue
			if (spixor == 0)
			{ // Here, spi interrupt countdown "time" tick
				spitick();
			}
			else
			{ // Deal with switch bit changes
				spilocal = (spilocal ^ spixor); // Update local copy
				swchanges(spixor);
			}
		}
   }
}
/* *************************************************************************
 * osThreadId xSwitchTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: SwitchTaskHandle, or NULL if queue or task creation failed
 * *************************************************************************/
osThreadId xSwitchTaskCreate(uint32_t taskpriority)
{
	BaseType_t ret = xTaskCreate(&StartSwitchTask, "SwitchTask",\
     80, NULL, taskpriority, &SwitchTaskHandle);
	if (ret != pdPASS) return NULL;

	SwitchTaskQHandle = xQueueCreate(SWITCHQSIZE, sizeof(uint16_t) );
	if (SwitchTaskQHandle == NULL) return NULL;

	return SwitchTaskHandle;
}


