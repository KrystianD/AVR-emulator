#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>

#include "avr.h"

FILE *file;

int mygetch ()
{
	struct termios oldt,
								 newt;
	int            ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}
uint32_t getTicks ()
{
	struct timeval tv;
	gettimeofday (&tv, 0);
	uint32_t val = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	return val;
}

#define CACHE_SIZE 512
uint8_t cache[CACHE_SIZE];
uint16_t cacheAddr = (uint16_t)(-1);

uint8_t fetchByte (uint16_t addr)
{
	fseek (file, addr, SEEK_SET);
	int read = fread (cache, 1, 1, file);
	return cache[0];

	// printf ("f: %x\n", addr);
	if (cacheAddr == (uint16_t)(-1) || (addr < cacheAddr || addr >= cacheAddr + CACHE_SIZE))
	{
		// printf ("fetch %d from %4d\n", CACHE_SIZE, addr);
		fseek (file, addr, SEEK_SET);
		int read = fread (cache, 1, CACHE_SIZE, file);
		// printf ("read: %d\n", read);
		cacheAddr = addr;
	}
	return cache[addr - cacheAddr];
}
uint16_t fetchWord (uint16_t addr)
{
	uint16_t q;
	q = fetchByte (addr);
	q |= (uint16_t)fetchByte (addr + 1) << 8;
	// printf ("in %04x\n", q);
	return q;
}

int main ()
{
	printf ("Start\n");

	avrInit ();

	file = fopen ("program2_c.bin", "rb");

	uint32_t s = getTicks ();
	uint32_t s2 = getTicks ();
	int c = 0;
	for (;;)
	{
		avrCycle ();
		c++;
		uint32_t t = getTicks ();
		if (t - s >= 1000)
		{
			fprintf (stderr, "CPS: %d\n", c);
			c = 0;
			s = t;
		}


		uint8_t c = avrGetIOMemory (0x00);
		if (c)
		{
			fputc (c, stdout);
			avrSetIOMemory (0x00, 0);
		}
		// printf ("sw: %d\n", avrGetIOMemory (0x01));
		// usleep (100 * 1000);
		// mygetch ();
	}
	
	fclose (file);

	return 0;
}

