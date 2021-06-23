#include "stdio.h"
#include "stdint.h"
#include "string.h"

volatile uint32_t pc;
volatile uint32_t last_pc;
uint8_t last_rd;
uint32_t pc_end;

int32_t regs[] = {
  2201,
  2202,
  2203,
  2204,
  2205,
  2206,
  2207,
  2208,
  2209,
  2210,
  2211,
  2212,
  2213,
  2214,
  2215,
  2216,
  2217,
  2218,
  2219,
  2220,
  2221,
  2222,
  2223,
  2224,
  2225,
  2226,
  2227,
  2228,
  2229,
  2230,
  2231,
  2232,
};

int32_t end[] = {
  4401,
  4402,
  4403,
  4404,
  4405,
  4406,
  4407,
  4408,
  4409,
  4410,
  4411,
  4412,
  4413,
  4414,
  4415,
  4416,
  4417,
  4418,
  4419,
  4420,
  4421,
  4422,
  4423,
  4424,
  4425,
  4426,
  4427,
  4428,
  4429,
  4430,
  4431,
  4432,
};

int nondet_uint();

#define BIT_RANGE(instr,upper,lower) (instr & (((1 << (upper-lower+1)) - 1) << lower))
#define BIT_SINGLE(instr,pos) (instr & (1 << pos))

static const unsigned NUM_REGS = 32;

typedef enum {
  x0 = 0,
  x1,
  x2,
  x3,
  x4,
  x5,
  x6,
  x7,
  x8,
  x9,
  x10,
  x11,
  x12,
  x13,
  x14,
  x15,
  x16,
  x17,
  x18,
  x19,
  x20,
  x21,
  x22,
  x23,
  x24,
  x25,
  x26,
  x27,
  x28,
  x29,
  x30,
  x31,

  zero = x0,
  ra = x1,
  sp = x2,
  gp = x3,
  tp = x4,
  t0 = x5,
  t1 = x6,
  t2 = x7,
  s0 = x8,
  fp = x8,
  s1 = x9,
  a0 = x10,
  a1 = x11,
  a2 = x12,
  a3 = x13,
  a4 = x14,
  a5 = x15,
  a6 = x16,
  a7 = x17,
  s2 = x18,
  s3 = x19,
  s4 = x20,
  s5 = x21,
  s6 = x22,
  s7 = x23,
  s8 = x24,
  s9 = x25,
  s10 = x26,
  s11 = x27,
  t3 = x28,
  t4 = x29,
  t5 = x30,
  t6 = x31,
} reg_name_t;

#define instr_csr(instr) (BIT_RANGE((uint32_t)instr, 31, 20) >> 20)
#define instr_zimm(instr) (BIT_RANGE(instr, 19, 15) >> 15)
#define instr_shamt(instr) (BIT_RANGE(instr, 24, 20) >> 20)
#define instr_funct3(instr) (BIT_RANGE(instr, 14, 12) >> 12)
#define instr_funct12(instr) (BIT_RANGE((uint32_t)instr, 31, 20) >> 20)
#define instr_funct7(instr) (BIT_RANGE((uint32_t)instr, 31, 25) >> 25)
#define instr_funct5(instr) (BIT_RANGE((uint32_t)instr, 31, 27) >> 27)
#define instr_aq(instr) BIT_SINGLE(instr, 26)
#define instr_rl(instr) BIT_SINGLE(instr, 25)
#define instr_opcode(instr) BIT_RANGE(instr, 6, 0)

int32_t instr_J_imm(int32_t instr) {
  return (BIT_SINGLE(instr,31) >> 11) | BIT_RANGE(instr,19,12) | (BIT_SINGLE(instr,20) >> 9) | (BIT_RANGE(instr,30,21) >> 20);
}

int32_t instr_B_imm(int32_t instr) {
  return ((BIT_SINGLE(instr,31) >> 19) | (BIT_SINGLE(instr,7) << 4) | (BIT_RANGE(instr,30,25) >> 20) | (BIT_RANGE(instr,11,8) >> 7));
}

#define instr_I_imm(instr) (((int32_t) BIT_RANGE(instr,31,20)) >> 20)
#define instr_S_imm(instr) (BIT_RANGE(instr,31,25) >> 20) | (BIT_RANGE(instr,11,7) >> 7)
#define instr_U_imm(instr) BIT_RANGE(instr,31,12)
#define instr_rs1(instr) (BIT_RANGE(instr,19,15) >> 15)
#define instr_rs2(instr) (BIT_RANGE(instr,24,20) >> 20)
#define instr_rd(instr) (BIT_RANGE(instr,11,7) >> 7)

uint32_t regs_shamt(uint32_t index) {
  return BIT_RANGE(regs[index], 4, 0);
}

typedef enum {
  OP_LUI    = 0b0110111,
  OP_AUIPC  = 0b0010111,
  OP_JAL    = 0b1101111,
  OP_JALR   = 0b1100111,
  F3_JALR   = 0b000,

  OP_LB     = 0b0000011,
  F3_LB     = 0b000,
  F3_LH     = 0b001,
  F3_LW     = 0b010,
  F3_LBU    = 0b100,
  F3_LHU    = 0b101,

  OP_SB     = 0b0100011,
  F3_SB     = 0b000,
  F3_SH     = 0b001,
  F3_SW     = 0b010,

  OP_BEQ    = 0b1100011,
  F3_BEQ    = 0b000,
  F3_BNE    = 0b001,
  F3_BLT    = 0b100,
  F3_BGE    = 0b101,
  F3_BLTU   = 0b110,
  F3_BGEU   = 0b111,

  OP_ADDI   = 0b0010011,
  F3_ADDI   = 0b000,
  F3_SLTI   = 0b010,
  F3_SLTIU  = 0b011,
  F3_XORI   = 0b100,
  F3_ORI    = 0b110,
  F3_ANDI   = 0b111,
  F3_SLLI   = 0b001,
  F3_SRLI   = 0b101,
  F7_SRLI   = 0b0000000,
  F7_SRAI   = 0b0100000,

  OP_ADD    = 0b0110011,
  F7_ADD    = 0b0000000,
  F7_SUB    = 0b0100000,
  F3_ADD    = 0b000,
  F3_SUB    = 0b000,
  F3_SLL    = 0b001,
  F3_SLT    = 0b010,
  F3_SLTU   = 0b011,
  F3_XOR    = 0b100,
  F3_SRL    = 0b101,
  F3_SRA    = 0b101,
  F3_OR     = 0b110,
  F3_AND    = 0b111,
} opcode_part_t;

typedef enum {
  UNDEF = 0,

  // RV32I Base Instruction Set
  LUI = 1,
  AUIPC,
  JAL,
  JALR,
  BEQ,
  BNE,
  BLT,
  BGE,
  BLTU,
  BGEU,
  LB,
  LH,
  LW,
  LBU,
  LHU,
  SB,
  SH,
  SW,
  ADDI,
  SLTI,
  SLTIU,
  XORI,
  ORI,
  ANDI,
  SLLI,
  SRLI,
  SRAI,
  ADD,
  SUB,
  SLL,
  SLT,
  SLTU,
  XOR,
  SRL,
  SRA,
  OR,
  AND,
  FENCE,
  ECALL,
  EBREAK,
  CSRRW,
  CSRRS,
  CSRRC,
  CSRRWI,
  CSRRSI,
  CSRRCI,

  // RV32M Standard Extension
  MUL,
  MULH,
  MULHSU,
  MULHU,
  DIV,
  DIVU,
  REM,
  REMU,

  // RV32A Standard Extension
  LR_W,
  SC_W,
  AMOSWAP_W,
  AMOADD_W,
  AMOXOR_W,
  AMOAND_W,
  AMOOR_W,
  AMOMIN_W,
  AMOMAX_W,
  AMOMINU_W,
  AMOMAXU_W,

  // privileged instructions
  URET,
  SRET,
  MRET,
  WFI,
  SFENCE_VMA,

  NUMBER_OF_INSTRUCTIONS
} opcode_t;

opcode_t core_decode(int32_t instr) {
  switch (instr_opcode(instr)) {
  case OP_LUI:
    return LUI;

  case OP_AUIPC:
    return AUIPC;

  case OP_JAL:
    return JAL;

  case OP_JALR: {
    __CPROVER_assume(instr_funct3(instr) == F3_JALR);
    return JALR;
  }

  case OP_BEQ: {
    switch (instr_funct3(instr)) {
    case F3_BEQ:
      return BEQ;
    case F3_BNE:
      return BNE;
    case F3_BLT:
      return BLT;
    case F3_BGE:
      return BGE;
    case F3_BLTU:
      return BLTU;
    case F3_BGEU:
      return BGEU;
    }
    break;
  }

  case OP_ADDI: {
    switch (instr_funct3(instr)) {
    case F3_ADDI:
      return ADDI;
    case F3_SLTI:
      return SLTI;
    case F3_SLTIU:
      return SLTIU;
    case F3_XORI:
      return XORI;
    case F3_ORI:
      return ORI;
    case F3_ANDI:
      return ANDI;
    case F3_SLLI:
      __CPROVER_assume(BIT_RANGE(instr, 31, 25) == 0);
      return SLLI;
    case F3_SRLI: {
      switch (instr_funct7(instr)) {
      case F7_SRLI:
	__CPROVER_assume(BIT_RANGE(instr, 31, 25) == 0);
	return SRLI;
      case F7_SRAI:
	__CPROVER_assume(BIT_RANGE(instr, 31, 25) == 16);
	return SRAI;
      }
    }
    }
    break;
  }

  case OP_ADD: {
    switch (instr_funct7(instr)) {
    case F7_ADD:
      switch (instr_funct3(instr)) {
      case F3_ADD:
	return ADD;
      case F3_SLL:
	return SLL;
      case F3_SLT:
	return SLT;
      case F3_SLTU:
	return SLTU;
      case F3_XOR:
	return XOR;
      case F3_SRL:
	return SRL;
      case F3_OR:
	return OR;
      case F3_AND:
	return AND;
      }
      break;

    case F7_SUB:
      switch (instr_funct3(instr)) {
      case F3_SUB:
	return SUB;
      case F3_SRA:
	return SRA;
      }
      break;
    }
    break;
  }
  }
  return UNDEF;
}

void core_exec_step() {
  uint32_t ei = nondet_uint();
  opcode_t op = core_decode(ei);
  pc += 4;
  uint32_t legal_instr = 0;
  uint32_t prev_rd = regs[instr_rd(ei)];

  switch (op) {
  case ADDI:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] + instr_I_imm(ei);
    break;
  case XORI:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] ^ instr_I_imm(ei);
    break;
  case ORI:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] | instr_I_imm(ei);
    break;
  case ANDI:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] & instr_I_imm(ei);
    break;
  case SLTI:
    regs[instr_rd(ei)] = (regs[instr_rs1(ei)] < instr_I_imm(ei)) ? 1 : 0;
    break;
  case SLTIU:
    regs[instr_rd(ei)] = (((uint32_t) regs[instr_rs1(ei)]) < ((uint32_t) instr_I_imm(ei)) ? 1 : 0);
    break;

  case ADD:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] + regs[instr_rs2(ei)];
    break;
  case SUB:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] - regs[instr_rs2(ei)];
    break;
  case XOR:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] ^ regs[instr_rs2(ei)];
    break;
  case OR:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] | regs[instr_rs2(ei)];
    break;
  case AND:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] & regs[instr_rs2(ei)];
    break;

  case SLT:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] < regs[instr_rs2(ei)];
    break;
  case SLTU:
    regs[instr_rd(ei)] = ((uint32_t) regs[instr_rs1(ei)]) < ((uint32_t) regs[instr_rs2(ei)]);
    break; 
  case SLL: 
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] << regs_shamt(instr_rs2(ei));
    break;
  case SRL:
    regs[instr_rd(ei)] = ((uint32_t) regs[instr_rs1(ei)]) >> regs_shamt(instr_rs2(ei));
    break;
  case SRA:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] >> regs_shamt(instr_rs2(ei));
    break;
  case SLLI:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] << instr_shamt(ei);
    break;
  case SRLI:
    regs[instr_rd(ei)] = ((uint32_t) regs[instr_rs1(ei)]) >> instr_shamt(ei);
    break;
  case SRAI:
    regs[instr_rd(ei)] = regs[instr_rs1(ei)] >> instr_shamt(ei);
    break;

  case LUI:
    regs[instr_rd(ei)] = instr_U_imm(ei);
    break;
  case AUIPC:
    regs[instr_rd(ei)] = last_pc + instr_U_imm(ei);
    break;
            
  case JAL:
    if (instr_rd(ei) != zero)
      regs[instr_rd(ei)] = pc;
    pc = last_pc + instr_J_imm(ei);
    __CPROVER_assume((pc & 3) == 0);
    break;

  case JALR: {
    uint32_t link = pc;
    pc = regs[instr_rs1(ei)] + instr_I_imm(ei) & ~1;
    __CPROVER_assume((pc & 3) == 0);
    if (instr_rd(ei) != zero)
      regs[instr_rd(ei)] = link;
  }
    break;

  case BEQ:
    if (regs[instr_rs1(ei)] == regs[instr_rs2(ei)]) {
      pc = last_pc + instr_B_imm(ei);
      __CPROVER_assume((pc & 3) == 0);
    }
    break;

  case BNE:
    if (regs[instr_rs1(ei)] != regs[instr_rs2(ei)]) {
      pc = last_pc + instr_B_imm(ei);
      __CPROVER_assume((pc & 3) == 0);
    }          
    break;

  case BLT:
    if (regs[instr_rs1(ei)] < regs[instr_rs2(ei)]) {
      pc = last_pc + instr_B_imm(ei);
      __CPROVER_assume((pc & 3) == 0);
    }
    break;

  case BGE:
    if (regs[instr_rs1(ei)] >= regs[instr_rs2(ei)]) {
      pc = last_pc + instr_B_imm(ei);
      __CPROVER_assume((pc & 3) == 0);
    }
    break;

  case BLTU:
    if (((uint32_t) regs[instr_rs1(ei)]) < ((uint32_t) regs[instr_rs2(ei)])) {
      pc = last_pc + instr_B_imm(ei);
      __CPROVER_assume((pc & 3) == 0);
    }
    break;

  case BGEU:
    if (((uint32_t) regs[instr_rs1(ei)]) >= ((uint32_t) regs[instr_rs2(ei)])) {
      pc = last_pc + instr_B_imm(ei);
      __CPROVER_assume((pc & 3) == 0);
    }
    break;
        
  default: {
    legal_instr = 1;
  }
  }
  regs[0] = 0;
  __CPROVER_assume(legal_instr != 1);
}

int main() {
  pc = nondet_uint();
  // assume initial pc value before execution
  __CPROVER_assume(pc == 1101);

  // main execution
  for (int i = 0; i < CYCLES; i++) {
    last_pc = pc;
    core_exec_step();
  }

  // assume pc value after execution to force behaviour
  __CPROVER_assume(pc == 1102);
  // these assertions can be used to specify what values are expected in the registers
  __CPROVER_assert(!(
		     regs[1] == end[1] &&
		     regs[2] == end[2] &&
		     regs[3] == end[3] &&
		     regs[4] == end[4] &&
		     regs[5] == end[5] &&
		     regs[6] == end[6] &&
		     regs[7] == end[7] &&
		     regs[8] == end[8] &&
		     regs[9] == end[9] &&
		     regs[10] == end[10] &&
		     regs[11] == end[11] &&
		     regs[12] == end[12] &&
		     regs[13] == end[13] &&
		     regs[14] == end[14] &&
		     regs[15] == end[15] &&
		     regs[16] == end[16] &&
		     regs[17] == end[17] &&
		     regs[18] == end[18] &&
		     regs[19] == end[19] &&
		     regs[20] == end[20] &&
		     regs[21] == end[21] &&
		     regs[22] == end[22] &&
		     regs[23] == end[23] &&
		     regs[24] == end[24] &&
		     regs[25] == end[25] &&
		     regs[26] == end[26] &&
		     regs[27] == end[27] &&
		     regs[28] == end[28] &&
		     regs[29] == end[29] &&
		     regs[30] == end[30] &&
		     regs[31] == end[31]
		     ), "post");
    
  return 0;
}
