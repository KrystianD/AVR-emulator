#include "avr.h"
#include "avr_opcodes.h"
#include "avr_config.h"

#ifdef AVR_DEBUG
#define DBG(x,...) myprintf (x, ##__VA_ARGS__)
#else
#define DBG(x,...)
#endif

uint8_t memory[AVR_SRAM_SIZE];
uint8_t SREG = 0;
uint16_t PC = 0;

void avrPrintRegs ();

uint8_t getREG (uint8_t reg) { return memory[reg]; }
void setREG (uint8_t reg, uint8_t val) { memory[reg] = val; }

void setFlag (uint8_t flag, uint8_t v) { if (v) SREG |= flag; else SREG &= ~flag; }
uint8_t getFlag (uint8_t flag) { return (SREG & flag) ? 1 : 0; }

uint16_t getSP () { return *(uint16_t*)(memory + 0x20 + 0x3d); }
void setSP (uint16_t val) { *(uint16_t*)(memory + 0x20 + 0x3d) = val; }
void incSP (uint8_t val) { *(uint16_t*)(memory + 0x20 + 0x3d) += val; }
void decSP (uint8_t val) { *(uint16_t*)(memory + 0x20 + 0x3d) -= val; }
void incPC (int16_t val) { PC += val * 2; }
void push8 (uint8_t val) { memory[getSP ()] = val; decSP (1); }
void push16 (uint16_t val) { push8 (val & 0x00ff); push8 (val >> 8); }
uint8_t pop8 () { incSP (1); return memory[getSP ()]; }
uint16_t pop16 () { uint16_t val; val = pop8 () << 8; val |= pop8 (); return val; }
uint16_t* getXPtr () { return (uint16_t*)(memory + 26); }
uint16_t* getYPtr () { return (uint16_t*)(memory + 28); }
uint16_t* getZPtr () { return (uint16_t*)(memory + 30); }

uint8_t getSize (uint16_t addr)
{
	uint16_t opcode;
	fetchWord (addr, &opcode);
	DBG ("test opcode: 0x%04x\r\n", opcode);
	if ((opcode & 0xfe0f) == LDS_OP)
		return 2;
	if ((opcode & 0xfe0f) == STS_OP)
		return 2;
	return 1;
}

int8_t avrProcess1R (uint16_t ins);
int8_t avrProcess2R (uint16_t ins);
int8_t avrProcessBranch (uint16_t ins);
int8_t avrProcessImmed (uint16_t ins);
int8_t avrProcessOther (uint16_t ins);

void updateFlagsADD (uint8_t res, uint8_t src, uint8_t dst)
{
	uint8_t Rd3 = dst & 0x08;
	uint8_t Rr3 = src & 0x08;
	uint8_t R3 = res & 0x08;
	uint8_t Rd7 = dst & 0x80;
	uint8_t Rr7 = src & 0x80;
	uint8_t R7 = res & 0x80;
	setFlag (H_FLAG, (Rd3 & Rr3) | (Rr3 & ~R3) | (~R3 & Rd3));
	setFlag (V_FLAG, (Rd7 & Rr7 & ~R7) | (~Rd7 & ~Rr7 & R7));
	setFlag (N_FLAG, R7);
	setFlag (Z_FLAG, res == 0);
	setFlag (C_FLAG, (Rd7 & Rr7) | (Rr7 & ~R7) | (~R7 & Rd7));
	setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
}
void updateFlagsSBC (uint8_t res, uint8_t src, uint8_t dst)
{
	uint8_t Rd3 = dst & 0x08;
	uint8_t Rr3 = src & 0x08;
	uint8_t R3 = res & 0x08;
	uint8_t Rd7 = dst & 0x80;
	uint8_t Rr7 = src & 0x80;
	uint8_t R7 = res & 0x80;
	setFlag (H_FLAG, (~Rd3 & Rr3) | (Rr3 & ~R3) | (R3 & ~Rd3));
	setFlag (V_FLAG, (Rd7 & ~Rr7 & ~R7) | (~Rd7 & Rr7 & R7));
	setFlag (N_FLAG, R7);
	setFlag (Z_FLAG, (res == 0) & getFlag (Z_FLAG));
	setFlag (C_FLAG, (~Rd7 & Rr7) | (Rr7 & R7) | (R7 & ~Rd7));
	setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
}
void updateFlagsSUB (uint8_t res, uint8_t src, uint8_t dst)
{
	uint8_t Rd3 = dst & 0x08;
	uint8_t Rr3 = src & 0x08;
	uint8_t R3 = res & 0x08;
	uint8_t Rd7 = dst & 0x80;
	uint8_t Rr7 = src & 0x80;
	uint8_t R7 = res & 0x80;
	setFlag (H_FLAG, (~Rd3 & Rr3) | (Rr3 & R3) | (R3 & ~Rd3));
	setFlag (V_FLAG, (Rd7 & ~Rr7 & ~R7) | (~Rd7 & Rr7 & R7));
	setFlag (N_FLAG, R7);
	setFlag (Z_FLAG, res == 0);
	setFlag (C_FLAG, (~Rd7 & Rr7) | (Rr7 & R7) | (R7 & ~Rd7));
	setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
}
void updateStdFlags (uint8_t res)
{
	setFlag (N_FLAG, res & 0x80);
	setFlag (Z_FLAG, res == 0);
	setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
}

void avrInit ()
{
	memset (memory, 0, AVR_SRAM_SIZE);
	SREG = 0;
	PC = 0;
	setSP (AVR_SRAM_SIZE - 1);
	DBG ("AVR core initialized\r\n");
}
uint8_t avrGetIOMemory (uint8_t addr)
{
	return memory[addr + 0x20];
}
void avrSetIOMemory (uint8_t addr, uint8_t val)
{
	memory[addr + 0x20] = val;
}

uint8_t avrCycle ()
{
	uint16_t ins;
	uint8_t res;

 	if (fetchWord (PC, &ins) == 0)
		return 0;

	DBG ("0x%04x = opcode: 0x%04x\r\n", PC, ins);

	do
	{
		res = avrProcess1R (ins);
		if (res == -1) return 0;
		res = avrProcess2R (ins);
		if (res == -1) return 0;
		res = avrProcessImmed (ins);
		if (res == -1) return 0;
		res = avrProcessBranch (ins);
		if (res == -1) return 0;
		res = avrProcessOther (ins);
		if (res == -1) return 0;
	} while (0);
	avrPrintRegs ();
	return 1;
}

int8_t avrProcess1R (uint16_t ins)
{
	uint8_t dstReg = (ins & 0x01f0) >> 4;
	uint8_t dst = getREG (dstReg);
	uint8_t res;
	uint16_t addr;
	switch (ALU_1R_MASK(ins))
	{
	case ASR_OP:
		res = (int8_t)dst >> 1;
		DBG ("ASR Rd: %d res: %d\r\n", dst, res);
		setREG (dstReg, res);
		setFlag (C_FLAG, dst & 0x01);
		setFlag (N_FLAG, res & 0x80);
		setFlag (V_FLAG, getFlag (N_FLAG) ^ getFlag (C_FLAG));
		updateStdFlags (res);
		incPC (1);
		return 1;
	case LSR_OP:
		res = dst >> 1;
		DBG ("LSR Rd: %d res: %d\r\n", dst, res);
		setREG (dstReg, res);
		setFlag (C_FLAG, dst & 0x01);
		setFlag (N_FLAG, 0);
		setFlag (V_FLAG, getFlag (N_FLAG) ^ getFlag (C_FLAG));
		updateStdFlags (res);
		incPC (1);
		return 1;
	case COM_OP:
		res = ~dst;
		DBG ("COM Rd: %d res: %d\r\n", dst, res);
		setREG (dstReg, res);
		setFlag (C_FLAG, 1);
		setFlag (V_FLAG, 0);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case PUSH_OP:
		DBG ("PUSH Rr: %d\r\n", dstReg);
		memory[getSP ()] = getREG (dstReg);
		decSP (1);
		incPC (1);
		return 1;
	case POP_OP:
		DBG ("POP Rd: %d\r\n", dstReg);
		incSP (1);
		setREG (dstReg, memory[getSP ()]);
		incPC (1);
		return 1;
	case STS_OP:
		DBG ("STS Rr: %d\r\n", dstReg);
		incPC (1);
		if (fetchWord (PC, &addr) == 0)
			return -1;
		memory[addr] = getREG (dstReg);
		incPC (1);
		return 1;
	case DEC_OP:
		DBG ("DEC Rd: %d\r\n", dstReg);
		res = dst - 1;
		setREG (dstReg, res);
		setFlag (V_FLAG, res == 0x7f);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case INC_OP:
		DBG ("INC Rd: %d\r\n", dstReg);
		res = dst + 1;
		setREG (dstReg, res);
		setFlag (V_FLAG, res == 0x80);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case SWAP_OP:
		DBG ("SWAP Rd: %d\r\n", dstReg);
		res = ((dst & 0x0f) << 4) | ((dst & 0xf0) >> 4);
		setREG (dstReg, res);
		incPC (1);
		return 1;
	// case XCH_OP:
		// DBG ("XCH Rd: %d\r\n", dstReg);
		// res = memory[*getZPtr ()];
		// memory[*getZPtr ()] = dst;
		// setREG (dstReg, res);
		// incPC (1);
		// return 1;
	case NEG_OP:
	{
		res = -(int8_t)dst;
		DBG ("NEG Rd: %d res: %d\r\n", dst, res);
		setREG (dstReg, res);
		uint8_t Rd3 = dst & 0x08;
		uint8_t R3 = res & 0x08;
		uint8_t Rd7 = dst & 0x80;
		uint8_t R7 = res & 0x80;
		setFlag (H_FLAG, R3 | Rd3);
		setFlag (V_FLAG, res == 0x80);
		setFlag (C_FLAG, res != 0x00);
		updateStdFlags (res);
		incPC (1);
		return 1;
	}
	case ROR_OP:
	{
		res = (dst >> 1) | (getFlag (C_FLAG) ? 0x80 : 0);
		DBG ("ROR Rd: %d res: %d\r\n", dst, res);
		setREG (dstReg, res);
		uint8_t Rd0 = dst & 0x01;
		uint8_t R7 = res & 0x80;
		setFlag (Z_FLAG, res == 0);
		setFlag (N_FLAG, R7);
		setFlag (C_FLAG, Rd0);
		setFlag (V_FLAG, getFlag (N_FLAG) ^ getFlag (C_FLAG));
		setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
		incPC (1);
		return 1;
	}
	// case LPMII_OP:
		// DBG ("LPM Z: %04x Rd: %d\r\n", *getZPtr (), dstReg);
		// setREG (dstReg, fetchByte (*getZPtr ()));
		// return 1;
	// case LPMIII_OP:
		// DBG ("LPM Z+: %04x Rd: %d\r\n", *getZPtr (), dstReg);
		// setREG (dstReg, fetchByte (*getZPtr ()));
		// (*getZPtr ())++;
		// return 1;
	}

	if ((ins & LD_MASK) == LD_OP)
	{
		uint8_t type = ins & 0x000f;
		switch (type)
		{
		case 0x0:
			DBG ("LDS Rd: %d\r\n", dstReg);
			incPC (1);
			if (fetchWord (PC, &addr) == 0)
				return -1;
			setREG (dstReg, memory[addr]);
			break;
		case 0xc:
			DBG ("LD X: %04x Rd: %d\r\n", *getXPtr (), dstReg);
			setREG (dstReg, memory[*getXPtr ()]);
			break;
		case 0xd:
			DBG ("LD X+: %04x Rd: %d\r\n", *getXPtr (), dstReg);
			setREG (dstReg, memory[*getXPtr ()]);
			(*getXPtr ())++;
			break;
		case 0xe:
			(*getXPtr ())--;
			DBG ("LD -X: %04x Rd: %d\r\n", *getXPtr (), dstReg);
			setREG (dstReg, memory[*getXPtr ()]);
			break;
		case 0x9:
			DBG ("LD Y+: %04x Rd: %d\r\n", *getYPtr (), dstReg);
			setREG (dstReg, memory[*getYPtr ()]);
			(*getYPtr ())++;
			break;
		case 0xa:
			(*getYPtr ())--;
			DBG ("LD -Y: %04x Rd: %d\r\n", *getYPtr (), dstReg);
			setREG (dstReg, memory[*getYPtr ()]);
			break;
		case 0x1:
			DBG ("LD Z+: %04x Rd: %d\r\n", *getZPtr (), dstReg);
			setREG (dstReg, memory[*getZPtr ()]);
			(*getZPtr ())++;
			break;
		case 0x2:
			(*getZPtr ())--;
			DBG ("LD -Z: %04x Rd: %d\r\n", *getZPtr (), dstReg);
			setREG (dstReg, memory[*getZPtr ()]);
			break;
		default:
			DBG ("unknown LD\r\n");
			return 0;
		}
		incPC (1);
		return 1;
	}

	if ((ins & ST_MASK) == ST_OP)
	{
		uint8_t type = ins & 0x000f;
		uint16_t *X = getXPtr ();
		uint16_t *Y = getYPtr ();
		uint16_t *Z = getZPtr ();
		switch (type)
		{
		case 0xc: // Unchanged ST X, r16
			DBG ("ST X: %04x Rr: %d\r\n", *X, dstReg);
			memory[*X] = getREG (dstReg);
			break;
		case 0xd: // Post increment ST X+, r16
			DBG ("ST X+: %04x Rr: %d\r\n", *X, dstReg);
			memory[*X] = getREG (dstReg);
			(*X)++;
			break;
		case 0xe: // Pre decrement ST -X, r16
			(*X)--;
			DBG ("ST -X: %04x Rr: %d\r\n", *X, dstReg);
			memory[*X] = getREG (dstReg);
			break;
		case 0x9: // Post increment ST Y+, r16
			DBG ("ST Y+: %04x Rr: %d\r\n", *Y, dstReg);
			memory[*Y] = getREG (dstReg);
			(*Y)++;
			break;
		case 0xa: // Pre decrement ST -Y, r16
			(*Y)--;
			DBG ("ST -Y: %04x Rr: %d\r\n", *Y, dstReg);
			memory[*Y] = getREG (dstReg);
			break;
		case 0x1: // Post increment ST Z+, r16
			DBG ("ST Z+: %04x Rr: %d\r\n", *Z, dstReg);
			memory[*Z] = getREG (dstReg);
			(*Z)++;
			break;
		case 0x2: // Pre decrement ST -Z, r16
			(*Z)--;
			DBG ("ST -Z: %04x Rr: %d\r\n", *Z, dstReg);
			memory[*Z] = getREG (dstReg);
			break;
		default:
			DBG ("unknown ST\r\n");
			return 0;
		}
		incPC (1);
		return 1;
	}

	switch (ins & LDD_MASK)
	{
	case LDDY_OP: // same for LD Rd, Y and LDD Rd, Y+q
	{
		uint8_t q = ((ins & 0x2000) >> 8) | ((ins & 0x0c00) >> 7) | (ins & 0x0007);
		DBG ("LDD Y: %04x Rd: %d q: %d\r\n", *getYPtr (), dstReg, q);
		setREG (dstReg, memory[*getYPtr () + q]);
		incPC (1);
		return 1;
	}
	case LDDZ_OP: // same for LD Rd, Z and LDD Rd, Z+q
	{
		uint8_t q = ((ins & 0x2000) >> 8) | ((ins & 0x0c00) >> 7) | (ins & 0x0007);
		DBG ("LDD Z: %04x Rd: %d q: %d\r\n", *getZPtr (), dstReg, q);
		setREG (dstReg, memory[*getZPtr () + q]);
		incPC (1);
		return 1;
	}
	case STDY_OP: // same for ST Y, Rr and STD Y+q, Rr
	{
		uint8_t q = ((ins & 0x2000) >> 8) | ((ins & 0x0c00) >> 7) | (ins & 0x0007);
		DBG ("STD Y: %04x Rr: %d q: %d\r\n", *getYPtr (), dstReg, q);
		memory[*getYPtr () + q] = getREG (dstReg);
		incPC (1);
		return 1;
	}
	case STDZ_OP: // same for ST Z, Rr and STD Z+q, Rr
	{
		uint8_t q = ((ins & 0x2000) >> 8) | ((ins & 0x0c00) >> 7) | (ins & 0x0007);
		DBG ("STD Z: %04x Rr: %d q: %d\r\n", *getZPtr (), dstReg, q);
		memory[*getZPtr () + q] = getREG (dstReg);
		incPC (1);
		return 1;
	}
	}

	return 0;
}
int8_t avrProcess2R (uint16_t ins)
{
	uint8_t srcReg = ((ins & 0x0200) >> 5) | (ins & 0x000f);
	uint8_t dstReg = (ins & 0x01f0) >> 4;
	uint8_t src = getREG (srcReg);
	uint8_t dst = getREG (dstReg);
	uint8_t res;
	switch (ALU_2R_MASK(ins))
	{
	case ADC_OP:
		res = dst + src + getFlag (C_FLAG);
		DBG ("ADC Rr: %d Rd: %d res: %d\r\n", srcReg, dstReg, res);
		setREG (dstReg, res);
		updateFlagsADD (res, src, dst);
		incPC (1);
		return 1;
	case ADD_OP:
		res = dst + src;
		DBG ("ADD Rr: %d Rd: %d res: %d\r\n", srcReg, dstReg, res);
		setREG (dstReg, res);
		updateFlagsADD (res, src, dst);
		incPC (1);
		return 1;
	case SUB_OP:
		res = dst - src;
		DBG ("SUB Rr: %d Rd: %d res: %d\r\n", srcReg, dstReg, res);
		setREG (dstReg, res);
		updateFlagsSUB (res, src, dst);
		incPC (1);
		return 1;
	case AND_OP:
		res = src & dst;
		DBG ("AND Rr: %d Rd: %d res: %d\r\n", srcReg, dstReg, res);
		setREG (dstReg, res);
		setFlag (V_FLAG, 0);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case MOV_OP:
		res = src;
		DBG ("MOV Rr: %d Rd: %d res: %d\r\n", srcReg, dstReg, res);
		setREG (dstReg, res);
		incPC (1);
		return 1;
	case EOR_OP:
		res = src ^ dst;
		DBG ("EOR Rr: %d Rd: %d\r\n", srcReg, dstReg);
		setREG (dstReg, res);
		setFlag (V_FLAG, 0);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case CP_OP:
	{
		DBG ("CP Rr: %d Rd: %d\r\n", srcReg, dstReg);
		res = dst - src;
		updateFlagsSUB (res, src, dst);
		incPC (1);
		return 1;
	}
	case CPC_OP:
	{
		res = dst - src - getFlag (C_FLAG);
		uint8_t Rd3 = dst & 0x08;
		uint8_t Rr3 = src & 0x08;
		uint8_t R3 = res & 0x08;
		uint8_t Rd7 = dst & 0x80;
		uint8_t Rr7 = src & 0x80;
		uint8_t R7 = res & 0x80;
		DBG ("CPC Rr: %d Rd: %d C: %d\r\n", srcReg, dstReg, getFlag (C_FLAG));
		DBG ("Rd - Rr - C = %u\r\n", res);
		setFlag (H_FLAG, (~Rd3 & Rr3) | (Rr3 & R3) | (R3 & ~Rd3));
		setFlag (V_FLAG, (Rd7 & ~Rr7 & ~R7) | (~Rd7 & Rr7 & R7));
		setFlag (N_FLAG, R7);
		setFlag (Z_FLAG, (res == 0) & getFlag (Z_FLAG));
		setFlag (C_FLAG, (~Rd7 & Rr7) | (Rr7 & R7) | (R7 & ~Rd7));
		setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
		incPC (1);
		return 1;
	}
	case CPSE_OP:
	{
		DBG ("CPSE Rr: %d Rd: %d\r\n", srcReg, dstReg);

		if (src == dst)
		{
			DBG ("skip\r\n");
			uint8_t nextSize = getSize (PC + 1 * 2);
			DBG ("next size: %d\r\n", nextSize);
			incPC (1 + nextSize);
		}
		else
		{
			DBG ("no skip\r\n");
			incPC (1);
		}
		return 1;
	}
	case MUL_OP:
	{
		uint16_t res16 = dst * src;
		DBG ("MUL Rd: %d Rr: %d res: %u\r\n", dstReg, srcReg, res16);
		setREG (0, res16 & 0x00ff);
		setREG (1, res16 >> 8);
		setFlag (C_FLAG, (res16 & 0x8000) ? 1 : 0);
		setFlag (Z_FLAG, res16 == 0);
		incPC (1);
		return 1;
	}
	case OR_OP:
		res = src | dst;
		DBG ("OR Rr: %d Rd: %d res: %d\r\n", srcReg, dstReg, res);
		setREG (dstReg, res);
		setFlag (V_FLAG, 0);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case SBC_OP:
		res = dst - src - getFlag (C_FLAG);
		DBG ("SBC Rr: %d Rd: %d res: %d\r\n", srcReg, dstReg, res);
		setREG (dstReg, res);
		updateFlagsSBC (res, src, dst);
		incPC (1);
		return 1;
	}
	return 0;
}
int8_t avrProcessBranch (uint16_t ins)
{
	int8_t k = (int8_t)(((ins & 0x03f8) >> 3) << 1) >> 1;
	uint8_t s = ins & 0x0007;
	uint8_t res;
	switch (BR_MASK(ins))
	{
	case BRBC_OP:
		DBG ("BRBC k: %d s: %d\r\n", k, s);
		if ((SREG & (1 << s)) == 0)
		{
			DBG ("branch\r\n");
			incPC (k + 1);
		}
		else
		{
			DBG ("no branch\r\n");
			incPC (1);
		}
		return 1;
	case BRBS_OP:
		DBG ("BRBC k: %d s: %d\r\n", k, s);
		if (SREG & (1 << s))
		{
			DBG ("branch\r\n");
			incPC (k + 1);
		}
		else
		{
			DBG ("no branch\r\n");
			incPC (1);
		}
		return 1;
	}
	return 0;
}
int8_t avrProcessImmed (uint16_t ins)
{
	uint8_t K = ((ins & 0x0f00) >> 4) | (ins & 0x000f);
	uint8_t dstReg = 16 + ((ins & 0x00f0) >> 4);
	uint8_t dst = getREG (dstReg);
	uint8_t res;
	switch (ALU_IMMED_MASK(ins))
	{
	case LDI_OP:
		DBG ("LDI Rd: %d val: %d\r\n", dstReg, K);
		setREG (dstReg, K);
		incPC (1);
		return 1;
	case ANDI_OP:
		res = getREG (dstReg) & K;
		DBG ("ANDI Rd: %d K: %d\r\n", dstReg, K);
		setREG (dstReg, res);
		setFlag (V_FLAG, 0);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case SUBI_OP:
		res = dst - K;
		setREG (dstReg, res);
		DBG ("SUBI Rd: %d K: %d\r\n", dstReg, K);
		updateFlagsSUB (res, K, dst);
		incPC (1);
		return 1;
	case CPI_OP:
	{
		DBG ("CPI K: %d Rd: %d\r\n", K, dst);
		res = dst - K;
		DBG ("R (%3u) (%4d)\r\n", res, res);
		uint8_t Rd3 = dst & 0x08;
		uint8_t K3 = K & 0x08;
		uint8_t R3 = res & 0x08;
		uint8_t Rd7 = dst & 0x80;
		uint8_t K7 = K & 0x80;
		uint8_t R7 = res & 0x80;
		setFlag (H_FLAG, (~Rd3 & K3) | (K3 & R3) | (R3 & ~Rd3));
		setFlag (V_FLAG, (Rd7 & ~K7 & ~R7) | (~Rd7 & K7 & R7));
		setFlag (N_FLAG, R7);
		setFlag (Z_FLAG, res == 0);
		setFlag (C_FLAG, (~Rd7 & K7) | (K7 & R7) | (R7 & ~Rd7));
		setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
		incPC (1);
		return 1;
	}
	case ORI_OP:
		res = getREG (dstReg) | K;
		DBG ("ORI Rd: %d K: %d\r\n", dstReg, K);
		setREG (dstReg, res);
		setFlag (V_FLAG, 0);
		updateStdFlags (res);
		incPC (1);
		return 1;
	case SBCI_OP:
		res = dst - K - getFlag (C_FLAG);
		DBG ("SBCI Rd: %d K: %d\r\n", dstReg, K);
		setREG (dstReg, res);
		updateFlagsSBC (res, K, dst);
		incPC (1);
		return 1;
	}
	return 0;
}
int8_t avrProcessOther (uint16_t ins)
{
	int16_t addr;
	uint8_t K, dstReg;
	uint16_t res16;

	// 4-bit opcode
	switch (ins & 0xf000)
	{
	case RJMP_OP:
		addr = (int16_t)((ins & 0x0fff) << 4) >> 4;
		DBG ("RJMP addr: %d\r\n", addr * 2);
		incPC (addr + 1);
		return 1;
	case RCALL_OP:
		addr = (int16_t)((ins & 0x0fff) << 4) >> 4;
		DBG ("RCALL addr: %d\r\n", addr * 2);
		push16 (PC + 1 * 2);
		incPC (addr + 1);
		return 1;
	}

	// 6-bit opcode
	switch (ins & 0xfc00)
	{
	}

	// 7-bit opcode
	switch (ins & 0xfe00)
	{
	case SBRC_OP:
	{
		uint8_t b = ins & 0x0007;
		uint8_t r = (ins & 0x01f0) >> 4;
		DBG ("SBRC b: %d r: %d\r\n", b, r);
		if ((getREG (r) & (1 << b)) == 0)
		{
			DBG ("skip\r\n");
			uint8_t nextSize = getSize (PC + 1 * 2);
			DBG ("next size: %d\r\n", nextSize);
			incPC (1 + nextSize);
		}
		else
		{
			DBG ("no skip\r\n");
			incPC (1);
		}
		return 1;
	}
	case SBRS_OP:
	{
		uint8_t b = ins & 0x0007;
		uint8_t r = (ins & 0x01f0) >> 4;
		DBG ("SBRS b: %d r: %d\r\n", b, r);
		if ((getREG (r) & (1 << b)) != 0)
		{
			DBG ("skip\r\n");
			uint8_t nextSize = getSize (PC + 1 * 2);
			DBG ("next size: %d\r\n", nextSize);
			incPC (1 + nextSize);
		}
		else
		{
			DBG ("no skip\r\n");
			incPC (1);
		}
		return 1;
	}
	}

	// 8-bit opcode
	switch (ins & 0xff00)
	{
	case ADIW_OP:
	{
		K = ((ins & 0x00c0) >> 2) | (ins & 0x000f);
		dstReg = 24 + ((ins & 0x0030) >> 4) * 2;
		uint8_t Rdl = getREG (dstReg);
		uint8_t Rdh = getREG (dstReg + 1);
		res16 = Rdl | (Rdh << 8);
		res16 += K;
		setREG (dstReg, res16 & 0x00ff);
		setREG (dstReg + 1, (res16 & 0xff00) >> 8);
		uint8_t Rdh7 = (Rdh & 0x80) ? 1 : 0;
		uint8_t R15 = (res16 & 0x8000) ? 1 : 0;
		setFlag (V_FLAG, ~Rdh7 & R15);
		setFlag (N_FLAG, R15);
		setFlag (Z_FLAG, res16 == 0);
		setFlag (C_FLAG, ~R15 & Rdh7);
		setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
		DBG ("ADIW K: %d Rd: %d res: %d\r\n", K, dstReg, res16);
		incPC (1);
		return 1;
	}
	case SBIW_OP:
	{
		K = ((ins & 0x00c0) >> 2) | (ins & 0x000f);
		dstReg = 24 + ((ins & 0x0030) >> 4) * 2;
		uint8_t Rdl = getREG (dstReg);
		uint8_t Rdh = getREG (dstReg + 1);
		res16 = Rdl | (Rdh << 8);
		res16 -= K;
		setREG (dstReg, res16 & 0x00ff);
		setREG (dstReg + 1, (res16 & 0xff00) >> 8);
		uint8_t Rdh7 = (Rdh & 0x80) ? 1 : 0;
		uint8_t R15 = (res16 & 0x8000) ? 1 : 0;
		setFlag (V_FLAG, Rdh7 & ~R15);
		setFlag (N_FLAG, R15);
		setFlag (Z_FLAG, res16 == 0);
		setFlag (C_FLAG, R15 & ~Rdh7);
		setFlag (S_FLAG, getFlag (N_FLAG) ^ getFlag (V_FLAG));
		DBG ("SBIW K: %d Rd: %d res: %d\r\n", K, dstReg, res16);
		incPC (1);
		return 1;
	}
	case CBI_OP:
	{
		uint8_t A = (ins & 0x00f8) >> 3;
		uint8_t b = ins & 0x0007;
		DBG ("CBI A: %02x b: %d\r\n", A, b);
		memory[0x20 + A] &= ~(1 << b);
		incPC (1);
		return 1;
	}
	case SBI_OP:
	{
		uint8_t A = (ins & 0x00f8) >> 3;
		uint8_t b = ins & 0x0007;
		DBG ("SBI A: %02x b: %d\r\n", A, b);
		memory[0x20 + A] |= 1 << b;
		incPC (1);
		return 1;
	}
	case MOVW_OP:
	{
		uint8_t Rd = 2 * ((ins & 0x00f0) >> 4);
		uint8_t Rr = 2 * (ins & 0x000f);
		DBG ("MOVW Rd: %d Rr: %d\r\n", Rd, Rr);
		setREG (Rd, getREG (Rr));
		setREG (Rd + 1, getREG (Rr + 1));
		incPC (1);
		return 1;
	}
	case MULS_OP:
	{
		uint8_t Rd = 16 + ((ins & 0x00f0) >> 4);
		uint8_t Rr = 16 + (ins & 0x000f);
		int8_t dst = getREG (Rd);
		int8_t src = getREG (Rr);
		int16_t res = dst * src;
		DBG ("MULS Rd: %d Rr: %d res: %d\r\n", Rd, Rr, res);
		setREG (0, res & 0x00ff);
		setREG (1, res >> 8);
		setFlag (C_FLAG, (res & 0x8000) ? 1 : 0);
		setFlag (Z_FLAG, res == 0);
		incPC (1);
		return 1;
	}
	case SBIS_OP:
	{
		uint8_t A = ((ins & 0x00f8) >> 3);
		uint8_t b = ins & 0x0007;
		DBG ("SBIS A: %d b: %d\r\n", A, b);
		if ((memory[0x20 + A] & (1 << b)) != 0)
		{
			DBG ("skip\r\n");
			uint8_t nextSize = getSize (PC + 1 * 2);
			DBG ("next size: %d\r\n", nextSize);
			incPC (1 + nextSize);
		}
		else
		{
			DBG ("no skip\r\n");
			incPC (1);
		}
		return 1;
	}
	case SBIC_OP:
	{
		uint8_t A = ((ins & 0x00f8) >> 3);
		uint8_t b = ins & 0x0007;
		DBG ("SBIC A: %d b: %d\r\n", A, b);
		if ((memory[0x20 + A] & (1 << b)) == 0)
		{
			DBG ("skip\r\n");
			uint8_t nextSize = getSize (PC + 1 * 2);
			DBG ("next size: %d\r\n", nextSize);
			incPC (1 + nextSize);
		}
		else
		{
			DBG ("no skip\r\n");
			incPC (1);
		}
		return 1;
	}
	}

	//
	switch (ins & 0xff8f)
	{
	case BCLR_OP:
	{
		uint8_t s = (ins & 0x0070) >> 4;
		DBG ("BCLR s: %d\r\n", s);
		SREG &= ~(1 << s);
		incPC (1);
		return 1;
	}
	case BSET_OP:
	{
		uint8_t s = (ins & 0x0070) >> 4;
		DBG ("BSET s: %d\r\n", s);
		SREG |= 1 << s;
		incPC (1);
		return 1;
	}
	}
	
	//
	switch (ins & 0xfe08)
	{
	case BLD_OP:
	{
		uint8_t dstReg = (ins & 0x01f0) >> 4;
		uint8_t bit = ins & 0x0007;
		DBG ("BLD Rd: %d b: %d\r\n", dstReg, bit);
		if (getFlag (T_FLAG))
			setREG (dstReg, getREG (dstReg) | (1 << bit));
		else
			setREG (dstReg, getREG (dstReg) & ~(1 << bit));
		incPC (1);
		return 1;
	}
	case BST_OP:
	{
		uint8_t dstReg = (ins & 0x01f0) >> 4;
		uint8_t bit = ins & 0x0007;
		DBG ("BLD Rd: %d b: %d\r\n", dstReg, bit);
		setFlag (T_FLAG, getREG (dstReg) & (1 << bit));
		incPC (1);
		return 1;
	}
	}
	
	//
	switch (ins & 0xff0f)
	{
	case SER_OP:
	{
		uint8_t dstReg = 16 + ((ins & 0x00f0) >> 4);
		setREG (dstReg, 0xff);
		incPC (1);
		return 1;
	}
	case BST_OP:
	{
		uint8_t dstReg = (ins & 0x01f0) >> 4;
		uint8_t bit = ins & 0x0007;
		DBG ("BLD Rd: %d b: %d\r\n", dstReg, bit);
		setFlag (T_FLAG, getREG (dstReg) & (1 << bit));
		incPC (1);
		return 1;
	}
	}

	//
	switch (ins & 0xf800)
	{
	case IN_OP:
	{
		uint8_t A = ((ins & 0x0600) >> 5) | (ins & 0x000f);
		uint8_t Rd = (ins & 0x01f0) >> 4;
		DBG ("IN Rd: %d A: %d\r\n", Rd, A);
		setREG (Rd, memory[0x20 + A]);
		incPC (1);
		return 1;
	}
	case OUT_OP:
	{
		uint8_t A = ((ins & 0x0600) >> 5) | (ins & 0x000f);
		uint8_t Rr = (ins & 0x01f0) >> 4;
		DBG ("OUT Rr: %d A: %d\r\n", Rr, A);
		memory[0x20 + A] = getREG (Rr);
		incPC (1);
		return 1;
	}
	}

	// 16-bit opcode
	switch (ins)
	{
	case CLC_OP: DBG ("CLC\r\n"); setFlag (C_FLAG, 0); incPC (1); return 1;
	case CLH_OP: DBG ("CLH\r\n"); setFlag (H_FLAG, 0); incPC (1); return 1;
	case CLI_OP: DBG ("CLI\r\n"); setFlag (I_FLAG, 0); incPC (1); return 1;
	case CLN_OP: DBG ("CLN\r\n"); setFlag (N_FLAG, 0); incPC (1); return 1;
	case CLS_OP: DBG ("CLS\r\n"); setFlag (S_FLAG, 0); incPC (1); return 1;
	case CLT_OP: DBG ("CLT\r\n"); setFlag (T_FLAG, 0); incPC (1); return 1;
	case CLV_OP: DBG ("CLV\r\n"); setFlag (V_FLAG, 0); incPC (1); return 1;
	case CLZ_OP: DBG ("CLZ\r\n"); setFlag (Z_FLAG, 0); incPC (1); return 1;

	case SEC_OP: DBG ("SEC\r\n"); setFlag (C_FLAG, 1); incPC (1); return 1;
	case SEH_OP: DBG ("SEH\r\n"); setFlag (H_FLAG, 1); incPC (1); return 1;
	case SEI_OP: DBG ("SEI\r\n"); setFlag (I_FLAG, 1); incPC (1); return 1;
	case SEN_OP: DBG ("SEN\r\n"); setFlag (N_FLAG, 1); incPC (1); return 1;
	case SES_OP: DBG ("SES\r\n"); setFlag (S_FLAG, 1); incPC (1); return 1;
	case SET_OP: DBG ("SET\r\n"); setFlag (T_FLAG, 1); incPC (1); return 1;
	case SEV_OP: DBG ("SEV\r\n"); setFlag (V_FLAG, 1); incPC (1); return 1;
	case SEZ_OP: DBG ("SEZ\r\n"); setFlag (Z_FLAG, 1); incPC (1); return 1;

	case LPM_OP:
	{
		uint8_t byte;
		DBG ("LPM\r\n");
		if (fetchByte (*getZPtr (), &byte) == 0)
			return -1;
		setREG (0, byte);
		incPC (1);
		return 1;
	}

	case RET_OP:
		DBG ("RET\r\n");
		PC = pop16 ();
		return 1;

	case RETI_OP:
		DBG ("RETI\r\n");
		PC = pop16 ();
		setFlag (I_FLAG, 1);
		return 1;

	case ICALL_OP:
		addr = *getZPtr ();
		DBG ("ICALL addr: %04x\r\n", addr);
		push16 (PC + 1 * 2);
		PC = addr * 2;
		return 1;
	case IJMP_OP:
		addr = *getZPtr ();
		DBG ("IJMP addr: %04x\r\n", addr);
		PC = addr * 2;
		return 1;
	
	case NOP_OP: DBG ("NOP\r\n"); incPC (1); return 1;
	}
	return 0;
}

void avrPrintRegs ()
{
#ifdef AVR_DEBUG
	uint8_t i, j;
	for (i = 0; i < 32; i++)
	{
		DBG ("\033[31m%2d\033[0m = %02x (%3u) (%4d) ", i, getREG (i), getREG (i), (int8_t)getREG (i));
		if ((i % 4) == 3) DBG ("\r\n");
	}
	DBG ("\r\n");

	uint16_t r1r0u = *(uint16_t*)(memory + 0);
	int16_t r1r0d = *(int16_t*)(memory + 0);
	DBG ("r1:r0: 0x%04x (%3u) (%4d)\r\n", r1r0u, r1r0u, r1r0d);
	DBG ("X: 0x%04x Y: 0x%04x Z: 0x%04x SP: 0x%x (%3u)\r\n", (uint16_t)*getXPtr (), (uint16_t)*getYPtr (), (uint16_t)*getZPtr (), getSP (), getSP ());
	// DBG ("*X: 0x%02x *Y: 0x%02x *Z: 0x%02x\n\r\n", memory[*getXPtr ()], memory[*getYPtr ()], memory[*getZPtr ()]);

	DBG ("SREG I:%d T:%d H:%d S:%d V:%d N:%d Z:%d C:%d\n\r\n",
		getFlag (I_FLAG), getFlag (T_FLAG), getFlag (H_FLAG), getFlag (S_FLAG),
		getFlag (V_FLAG), getFlag (N_FLAG), getFlag (Z_FLAG), getFlag (C_FLAG));

	for (j = 0; j < 128; j += 25)
	{
		for (i = j; i < j + 25; i++)
		{
			DBG ("\033[31m%2x\033[0m ", i);
		}
		DBG ("\r\n");
		for (i = j; i < j + 25; i++)
		{
			const char *color = "";
			if (i <= 0x1f)
				color = "\033[32m";
			else if (i <= 0x5f)
				color = "\033[33m";
			DBG ("%s%2x\033[0m ", color, memory[i]);
		}
		DBG ("\r\n");
	}
	DBG ("\r\n");
#endif
}
