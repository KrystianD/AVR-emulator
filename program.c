asm ("rjmp main");

#include <stdint.h>

#define ACC_STATUS *((uint8_t*)(0x20))
#define ACC_SWITCH *((uint8_t*)(0x21))
#define ACC_DELAY *((uint16_t*)(0x22)) // 0x23

#define ACC_STATUS_DELAY 0x01

#define ACC_SWITCH_CHARGER 0x01
#define ACC_SWITCH_LOAD    0x02

static inline void enableCharger () { ACC_SWITCH |= ACC_SWITCH_CHARGER; }
static inline void disableCharger () { ACC_SWITCH &= ~ACC_SWITCH_CHARGER; }
static inline void enableLoad () { ACC_SWITCH |= ACC_SWITCH_LOAD; }
static inline void disableLoad () { ACC_SWITCH &= ~ACC_SWITCH_LOAD; }

void delay_ms (uint16_t ms) { ACC_DELAY = ms; ACC_STATUS |= ACC_STATUS_DELAY; }

int main ()
{
	for (;;)
	{
		enableCharger ();
		delay_ms (1000);
		disableCharger ();
		delay_ms (1000);

		enableCharger ();
		delay_ms (200);
		disableCharger ();
		delay_ms (1000);
	}
}

