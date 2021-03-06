#define ADC_OP  (0x1c00)
#define ADD_OP  (0x0c00)
#define ADIW_OP (0x9600)
#define AND_OP  (0x2000)
#define ANDI_OP (0x7000)
#define ASR_OP  (0x9405)
#define BCLR_OP (0x9488)
#define BLD_OP  (0xf800)
#define BRBC_OP (0xf400)
#define BRBS_OP (0xf000)
#define BRCC_OP (0xf400)
#define BRCS_OP (0xf000)
#define BREQ_OP (0xf000 | 0x1)
#define BRGE_OP (0xf400 | 0x4)
#define BRHC_OP (0xf400 | 0x5)
#define BRHS_OP (0xf000 | 0x5)
#define BRID_OP (0xf400 | 0x7)
#define BRIE_OP (0xf000 | 0x7)
#define BRLO_OP (0xf000 | 0x0)
#define BRLT_OP (0xf000 | 0x4)
#define BRMI_OP (0xf000 | 0x2)
#define BRNE_OP (0xf400 | 0x1)
#define BRPL_OP (0xf400 | 0x2)
#define BRSH_OP (0xf400 | 0x0)
#define BRTC_OP (0xf400 | 0x6)
#define BRTS_OP (0xf000 | 0x6)
#define BRVC_OP (0xf400 | 0x3)
#define BRVS_OP (0xf000 | 0x3)
#define BSET_OP (0x9408)
#define BST_OP  (0xfa00)
// #define CALL_OP - not used
#define CBI_OP  (0x9800)
// #define CBR_OP - same as ANDI
#define CLC_OP  (0x9488)
#define CLH_OP  (0x94d8)
#define CLI_OP  (0x94f8)
#define CLN_OP  (0x94a8)
// #define CLR_OP  (0x2400) - same as EOR
#define CLS_OP  (0x94c8)
#define CLT_OP  (0x94e8)
#define CLV_OP  (0x94b8)
#define CLZ_OP  (0x9498)
#define COM_OP  (0x9400)
#define CP_OP   (0x1400)
#define CPC_OP  (0x0400)
#define CPI_OP  (0x3000)
#define CPSE_OP (0x1000)
#define DEC_OP  (0x940a)
// #define DES_OP - not used
// #define EICALL - not used
// #define EIJMP - not used
// #define ELPM - not used
#define EOR_OP  (0x2400)
// #define FMUL_OP - not used
// #define FMULS_OP - not used
// #define FMULSU_OP - not used
#define ICALL_OP (0x9509)
#define IJMP_OP (0x9409)
#define IN_OP   (0xb000)
#define INC_OP  (0x9403)
// #define JMP_OP - not used
//////////////////// #define LAC_OP  (0x9206)
//////////////////// #define LAS_OP  (0x9205)
//////////////////// #define LAT_OP  (0x9207)
#define LD_MASK (0xfe00)
#define LD_OP   (0x9000)
#define LDD_MASK (0xd208)
#define LDDY_OP (0x8008)
#define LDDZ_OP (0x8000)
#define LDI_OP  (0xe000)
#define LDS_OP  (0x9000)
// #define LDS16_OP - not used
#define LPM_OP  (0x95c8)
#define LPMII_OP  (0x9004)
#define LPMIII_OP (0x9005)
// #define LSL_OP - same as ADD Rd,Rd
#define LSR_OP  (0x9406)
#define MOV_OP  (0x2c00)
#define MOVW_OP (0x0100)
#define MUL_OP  (0x9c00)
#define MULS_OP (0x0200)
#define NEG_OP  (0x9401)
#define NOP_OP  (0x0000)
#define OR_OP   (0x2800)
#define ORI_OP  (0x6000)
#define OUT_OP  (0xb800)
#define POP_OP  (0x900f)
#define PUSH_OP (0x920f)
#define RCALL_OP (0xd000)
#define RET_OP  (0x9508)
#define RETI_OP (0x9518)
#define RJMP_OP (0xc000)
// #define ROL_OP  (0x1c00) - same as ADC Rd, Rd
#define ROR_OP  (0x9407)
#define SBC_OP  (0x0800)
#define SBCI_OP (0x4000)
#define SBI_OP  (0x9a00)
#define SBIC_OP (0x9900)
#define SBIS_OP (0x9b00)
#define SBIW_OP (0x9700)
// #define SBR_OP - same as ORI
#define SBRC_OP (0xfc00)
#define SBRS_OP (0xfe00)
#define SEC_OP  (0x9408)
#define SEH_OP  (0x9458)
#define SEI_OP  (0x9478)
#define SEN_OP  (0x9428)
#define SER_OP  (0xef0f)
#define SES_OP  (0x9448)
#define SET_OP  (0x9468)
#define SEV_OP  (0x9438)
#define SEZ_OP  (0x9418)
// #define SLEEP_OP - not used
// #define SPM - not used
#define ST_MASK (0xfe00)
#define ST_OP   (0x9200)
#define STDY_OP (0x8208)
#define STDZ_OP (0x8200)
#define STS_OP  (0x9200)
#define SUB_OP  (0x1800)
#define SUBI_OP (0x5000)
#define SWAP_OP (0x9402)
// #define TST - same as AND Rd, Rd
// #define WDR - not used
// #define XCH_OP  (0x9204) - not used

#define BR_MASK(x) (0xfc00 & (x))
#define ALU_2R_MASK(x) (0xfc00 & (x))
#define ALU_1R_MASK(x) (0xfe0f & (x))
#define ALU_IMMED_MASK(x) (0xf000 & (x))

#define I_FLAG (1 << 7)
#define T_FLAG (1 << 6)
#define H_FLAG (1 << 5)
#define S_FLAG (1 << 4)
#define V_FLAG (1 << 3)
#define N_FLAG (1 << 2)
#define Z_FLAG (1 << 1)
#define C_FLAG (1 << 0)
