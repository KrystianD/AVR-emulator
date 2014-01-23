#include <stdio.h>
#include <string.h>
#include <stdint.h>

uint8_t fetchByte (uint16_t addr, uint8_t* byte);
uint8_t fetchWord (uint16_t addr, uint16_t* word);

void avrInit ();
uint8_t avrCycle ();
uint8_t avrGetIOMemory (uint8_t addr);
void avrSetIOMemory (uint8_t addr, uint8_t val);
