#include <cassert>
#include <map>
#include "node.h"
#include "instruction.h"
#include "operand.h"
#include "local_storage_allocation.h"
#include "highlevel.h"
#include "lowlevel.h"
#include "exceptions.h"
#include "lowlevel_codegen.h"

// TODO REMOVE
#include <iostream>

namespace {

// This map has some "obvious" translations of high-level opcodes to
// low-level opcodes.
const std::map<HighLevelOpcode, LowLevelOpcode> HL_TO_LL = {
  { HINS_nop, MINS_NOP},
  { HINS_add_b, MINS_ADDB },
  { HINS_add_w, MINS_ADDW },
  { HINS_add_l, MINS_ADDL },
  { HINS_add_q, MINS_ADDQ },
  { HINS_sub_b, MINS_SUBB },
  { HINS_sub_w, MINS_SUBW },
  { HINS_sub_l, MINS_SUBL },
  { HINS_sub_q, MINS_SUBQ },
  { HINS_mul_l, MINS_IMULL },
  { HINS_mul_q, MINS_IMULQ },
  { HINS_mov_b, MINS_MOVB },
  { HINS_mov_w, MINS_MOVW },
  { HINS_mov_l, MINS_MOVL },
  { HINS_mov_q, MINS_MOVQ },
  { HINS_sconv_bw, MINS_MOVSBW },
  { HINS_sconv_bl, MINS_MOVSBL },
  { HINS_sconv_bq, MINS_MOVSBQ },
  { HINS_sconv_wl, MINS_MOVSWL },
  { HINS_sconv_wq, MINS_MOVSWQ },
  { HINS_sconv_lq, MINS_MOVSLQ },
  { HINS_uconv_bw, MINS_MOVZBW },
  { HINS_uconv_bl, MINS_MOVZBL },
  { HINS_uconv_bq, MINS_MOVZBQ },
  { HINS_uconv_wl, MINS_MOVZWL },
  { HINS_uconv_wq, MINS_MOVZWQ },
  { HINS_uconv_lq, MINS_MOVZLQ },
  { HINS_ret, MINS_RET },
  { HINS_jmp, MINS_JMP },
  { HINS_call, MINS_CALL },

  // For comparisons, it is expected that the code generator will first
  // generate a cmpb/cmpw/cmpl/cmpq instruction to compare the operands,
  // and then generate a setXX instruction to put the result of the
  // comparison into the destination operand. These entries indicate
  // the apprpropriate setXX instruction to use.
  { HINS_cmplt_b, MINS_SETL },
  { HINS_cmplt_w, MINS_SETL },
  { HINS_cmplt_l, MINS_SETL },
  { HINS_cmplt_q, MINS_SETL },
  { HINS_cmplte_b, MINS_SETLE },
  { HINS_cmplte_w, MINS_SETLE },
  { HINS_cmplte_l, MINS_SETLE },
  { HINS_cmplte_q, MINS_SETLE },
  { HINS_cmpgt_b, MINS_SETG },
  { HINS_cmpgt_w, MINS_SETG },
  { HINS_cmpgt_l, MINS_SETG },
  { HINS_cmpgt_q, MINS_SETG },
  { HINS_cmpgte_b, MINS_SETGE },
  { HINS_cmpgte_w, MINS_SETGE },
  { HINS_cmpgte_l, MINS_SETGE },
  { HINS_cmpgte_q, MINS_SETGE },
  { HINS_cmpeq_b, MINS_SETE },
  { HINS_cmpeq_w, MINS_SETE },
  { HINS_cmpeq_l, MINS_SETE },
  { HINS_cmpeq_q, MINS_SETE },
  { HINS_cmpneq_b, MINS_SETNE },
  { HINS_cmpneq_w, MINS_SETNE },
  { HINS_cmpneq_l, MINS_SETNE },
  { HINS_cmpneq_q, MINS_SETNE },
};

}

LowLevelCodeGen::LowLevelCodeGen(bool optimize)
  : m_total_memory_storage(0)
  , m_optimize(optimize) {
}

LowLevelCodeGen::~LowLevelCodeGen() {
}

std::shared_ptr<InstructionSequence> LowLevelCodeGen::generate(const std::shared_ptr<InstructionSequence> &hl_iseq) {
  // TODO: if optimizations are enabled, could do analysis/transformation of high-level code

  std::shared_ptr<InstructionSequence> ll_iseq = translate_hl_to_ll(hl_iseq);

  // TODO: if optimizations are enabled, could do analysis/transformation of low-level code

  return ll_iseq;
}

std::shared_ptr<InstructionSequence> LowLevelCodeGen::translate_hl_to_ll(const std::shared_ptr<InstructionSequence> &hl_iseq) {
  std::shared_ptr<InstructionSequence> ll_iseq(new InstructionSequence());

  // The high-level InstructionSequence will have a pointer to the Node
  // representing the function definition. Useful information could be stored
  // there (for example, about the amount of memory needed for local storage,
  // maximum number of virtual registers used, etc.)
  Node *funcdef_ast = hl_iseq->get_funcdef_ast();
  assert(funcdef_ast != nullptr);

  // It's not a bad idea to store the pointer to the function definition AST
  // in the low-level InstructionSequence as well, in case it's needed by
  // optimization passes.
  ll_iseq->set_funcdef_ast(funcdef_ast);

  const std::string &fn_name = funcdef_ast->get_kid(1)->get_str();

  m_memory_variable_offset = funcdef_ast->get_symbol()->get_offset();
  if (m_memory_variable_offset % 16 != 0)
    m_memory_variable_offset += (16 - (m_memory_variable_offset % 16));

  if (m_memory_variable_offset != 0)
    printf("/* Function \'%s\': placing memory variables at offset -%d from %%rbp */\n", fn_name.c_str(), m_memory_variable_offset);

  int max_temp_vreg = funcdef_ast->get_max_temp_vreg();
  int total_vreg_memory = (max_temp_vreg - 9) * 8;
  printf("/* Function \'%s\': uses %d total bytes of memory storage for vregs */\n", fn_name.c_str(), total_vreg_memory);

  m_vreg_storage_offset = m_memory_variable_offset + total_vreg_memory;
    printf("/* Function \'%s\': placing vreg storage at offset -%d from %%rbp */\n", fn_name.c_str(), m_vreg_storage_offset);

  // Determine the total number of bytes of memory storage
  // that the function needs. This should include both variables that
  // *must* have storage allocated in memory (e.g., arrays), and also
  // any additional memory that is needed for virtual registers,
  // spilled machine registers, etc.  
  m_total_memory_storage = m_vreg_storage_offset; 

  // The function prologue will push %rbp, which should guarantee that the
  // stack pointer (%rsp) will contain an address that is a multiple of 16.
  // If the total memory storage required is not a multiple of 16, add to
  // it so that it is.
  if ((m_total_memory_storage) % 16 != 0)
    m_total_memory_storage += (16 - (m_total_memory_storage % 16));

  printf("/* Function \'%s\': %d bytes of local storage allocated in stack frame */\n", fn_name.c_str(), m_total_memory_storage);

  // Iterate through high level instructions
  for (auto i = hl_iseq->cbegin(); i != hl_iseq->cend(); ++i) {
    Instruction *hl_ins = *i;

    // If the high-level instruction has a label, define an equivalent
    // label in the low-level instruction sequence
    if (i.has_label())
      ll_iseq->define_label(i.get_label());

    // Translate the high-level instruction into one or more low-level instructions
    translate_instruction(hl_ins, ll_iseq);
  }

  return ll_iseq;
}

namespace {

// These helper functions are provided to make it easier to handle
// the way that instructions and operands vary based on operand size
// ('b'=1 byte, 'w'=2 bytes, 'l'=4 bytes, 'q'=8 bytes.)

// Check whether hl_opcode matches a range of opcodes, where base
// is a _b variant opcode. Return true if the hl opcode is any variant
// of that base.
bool match_hl(int base, int hl_opcode) {
  return hl_opcode >= base && hl_opcode < (base + 4);
}

// For a low-level instruction with 4 size variants, return the correct
// variant. base_opcode should be the "b" variant, and operand_size
// should be the operand size in bytes (1, 2, 4, or 8.)
LowLevelOpcode select_ll_opcode(LowLevelOpcode base_opcode, int operand_size) {
  int off;

  switch (operand_size) {
  case 1: // 'b' variant
    off = 0; break;
  case 2: // 'w' variant
    off = 1; break;
  case 4: // 'l' variant
    off = 2; break;
  case 8: // 'q' variant
    off = 3; break;
  default:
    assert(false);
    off = 3;
  }

  return LowLevelOpcode(int(base_opcode) + off);
}

// Get the correct Operand::Kind value for a machine register
// of the specified size (1, 2, 4, or 8 bytes.)
Operand::Kind select_mreg_kind(int operand_size) {
  switch (operand_size) {
  case 1:
    return Operand::MREG8;
  case 2:
    return Operand::MREG16;
  case 4:
    return Operand::MREG32;
  case 8:
    return Operand::MREG64;
  default:
    assert(false);
    return Operand::MREG64;
  }
}

}

void LowLevelCodeGen::translate_instruction(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq) {
  HighLevelOpcode hl_opcode = HighLevelOpcode(hl_ins->get_opcode());

  if (hl_opcode == HINS_enter) {
    // Function prologue: this will create an ABI-compliant stack frame.
    // The local variable area is *below* the address in %rbp, and local storage
    // can be accessed at negative offsets from %rbp. For example, the topmost
    // 4 bytes in the local storage area are at -4(%rbp).
    ll_iseq->append(new Instruction(MINS_PUSHQ, Operand(Operand::MREG64, MREG_RBP)));
    ll_iseq->append(new Instruction(MINS_MOVQ, Operand(Operand::MREG64, MREG_RSP), Operand(Operand::MREG64, MREG_RBP)));
    ll_iseq->append(new Instruction(MINS_SUBQ, Operand(Operand::IMM_IVAL, m_total_memory_storage), Operand(Operand::MREG64, MREG_RSP)));

    return;
  }

  if (hl_opcode == HINS_leave) {
    // Function epilogue: deallocate local storage area and restore original value
    // of %rbp
    ll_iseq->append(new Instruction(MINS_ADDQ, Operand(Operand::IMM_IVAL, m_total_memory_storage), Operand(Operand::MREG64, MREG_RSP)));
    ll_iseq->append(new Instruction(MINS_POPQ, Operand(Operand::MREG64, MREG_RBP)));

    return;
  }

  if (hl_opcode == HINS_ret) {
    ll_iseq->append(new Instruction(MINS_RET));
    return;
  }

  // TODO: handle other high-level instructions
  // Note that you can use the highlevel_opcode_get_source_operand_size() and
  // highlevel_opcode_get_dest_operand_size() functions to determine the
  // size (in bytes, 1, 2, 4, or 8) of either the source operands or
  // destination operand of a high-level instruction. This should be useful
  // for choosing the appropriate low-level instructions and
  // machine register operands.

  // jmp instruction
  if (hl_opcode == HINS_jmp) {
    ll_iseq->append(new Instruction(MINS_JMP, hl_ins->get_operand(0)));
    return;
  }

  // call instruction
  if (hl_opcode == HINS_call) {
    ll_iseq->append(new Instruction(MINS_CALL, hl_ins->get_operand(0)));
    return;
  }

  // Comparisons
  if (match_hl(HINS_cmplte_b, hl_opcode)) {
    hl_cmplte_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  } else if (match_hl(HINS_cmplt_b, hl_opcode)) {
    hl_cmplt_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  } else if (match_hl(HINS_cmpgte_b, hl_opcode)) {
    hl_cmpgte_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  } else if (match_hl(HINS_cmpgte_b, hl_opcode)) {
    hl_cmpgt_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  } else if (match_hl(HINS_cmpeq_b, hl_opcode)) {
    hl_cmpeq_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  } else if (match_hl(HINS_cmpneq_b, hl_opcode)) {
    hl_cmpneq_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  }

  // Conditional jumps
  if (hl_opcode == HINS_cjmp_t) {
    hl_cjmp_t_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  } else if (hl_opcode == HINS_cjmp_f) {
    hl_cjmp_f_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  }

  // Conversions
  if (hl_opcode == HINS_sconv_bw || hl_opcode == HINS_uconv_bw) {
    hl_conv_to_ll_helper(hl_ins, ll_iseq, hl_opcode, 1, 2);
    return;
  } else if (hl_opcode == HINS_sconv_bl || hl_opcode == HINS_uconv_bl) {
    hl_conv_to_ll_helper(hl_ins, ll_iseq, hl_opcode, 1, 4);
    return;
  } else if (hl_opcode == HINS_sconv_bq || hl_opcode == HINS_uconv_bq) {
    hl_conv_to_ll_helper(hl_ins, ll_iseq, hl_opcode, 1, 8);
    return;
  } else if (hl_opcode == HINS_sconv_wl || hl_opcode == HINS_uconv_wl) {
    hl_conv_to_ll_helper(hl_ins, ll_iseq, hl_opcode, 2, 4);
    return;
  } else if (hl_opcode == HINS_sconv_wq|| hl_opcode == HINS_uconv_wq) {
    hl_conv_to_ll_helper(hl_ins, ll_iseq, hl_opcode, 2, 8);
    return;
  } else if (hl_opcode == HINS_sconv_lq || hl_opcode == HINS_uconv_lq) {
    hl_conv_to_ll_helper(hl_ins, ll_iseq, hl_opcode, 4, 8);
    return;
  }
  
  if (hl_opcode == HINS_localaddr) {
    hl_localaddr_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  }

  // mov instruction
  if (match_hl(HINS_mov_b, hl_opcode)) {
    hl_mov_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  }

  // add instruction
  if (match_hl(HINS_add_b, hl_opcode)) {
    hl_add_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  }
  // mul instruction
  if (match_hl(HINS_mul_b, hl_opcode)) {
    hl_mul_to_ll(hl_ins, ll_iseq, hl_opcode);
    return;
  }

  RuntimeError::raise("high level opcode %d not handled", int(hl_opcode));
}

// TODO: implement other private member functions

long LowLevelCodeGen::get_stack_offset(int vreg_num) {
  int base_vreg_num = vreg_num - 10;
  int offset = m_vreg_storage_offset - (base_vreg_num * 8);
  return -1 * offset;
}

Operand LowLevelCodeGen::get_ll_operand(Operand op, int size, const std::shared_ptr<InstructionSequence> &ll_iseq) {
  // Check for immediate value, or label
  if (op.is_imm_ival() || op.is_label()) {
    return op;
  } 
  // TODO: deal with IMM_label?
  
  Operand::Kind mreg_kind = select_mreg_kind(size);
  if (op.is_memref())
    mreg_kind = Operand::MREG64_MEM;

  int vreg_num = op.get_base_reg();

  // Check for reserved registers
  if (vreg_num < 7) {
    switch (vreg_num) {
      case 0: return Operand(mreg_kind, MREG_RAX);
      case 1: return Operand(mreg_kind, MREG_RDI);
      case 2: return Operand(mreg_kind, MREG_RSI);
      case 3: return Operand(mreg_kind, MREG_RDX);
      case 4: return Operand(mreg_kind, MREG_RCX);
      case 5: return Operand(mreg_kind, MREG_R8);
      case 6: return Operand(mreg_kind, MREG_R9);
      default:
        // Unreachable
        assert(false);
    }
  }

  // Deal with VREG
  int vreg_offset = get_stack_offset(vreg_num);
  Operand ll_op(Operand::MREG64_MEM_OFF, MREG_RBP, vreg_offset);

  // TODO: deal with memory references to vregisters...
  // Need to generate instructions for this...

  return ll_op;
}

/**
 * Translates HL mov instruction to LL.
 **/
void LowLevelCodeGen::hl_mov_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  int size = highlevel_opcode_get_source_operand_size(hl_opcode);

  LowLevelOpcode mov_opcode = select_ll_opcode(MINS_MOVB, size);

  Operand src_operand = get_ll_operand(hl_ins->get_operand(1), size, ll_iseq);
  Operand dest_operand = get_ll_operand(hl_ins->get_operand(0), size, ll_iseq);

  if (src_operand.is_memref() && dest_operand.is_memref()) {
    // Move source operand into temporary register
    Operand::Kind mreg_kind = select_mreg_kind(size);
    Operand r10(mreg_kind, MREG_R10);
    ll_iseq->append(new Instruction(mov_opcode, src_operand, r10));
    src_operand = r10;
  }

  ll_iseq->append(new Instruction(mov_opcode, src_operand, dest_operand));
}

/**
 * Translates HL add instruction to LL.
 **/
void LowLevelCodeGen::hl_add_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  int size = highlevel_opcode_get_source_operand_size(hl_opcode);

  LowLevelOpcode mov_opcode = select_ll_opcode(MINS_MOVB, size);
  LowLevelOpcode add_opcode = select_ll_opcode(MINS_ADDB, size);

  Operand src_left_operand = get_ll_operand(hl_ins->get_operand(1), size, ll_iseq);
  Operand src_right_operand = get_ll_operand(hl_ins->get_operand(2), size, ll_iseq);
  Operand dest_operand = get_ll_operand(hl_ins->get_operand(0), size, ll_iseq);

  // TODO: deal with moving into temp register if two or more are mem refs?????
  // I dont think this is necessary with the use of r10 as a helper here...

  Operand::Kind mreg_kind = select_mreg_kind(size);
  Operand r10(mreg_kind, MREG_R10);

  ll_iseq->append(new Instruction(mov_opcode, src_left_operand, r10));
  ll_iseq->append(new Instruction(add_opcode, src_right_operand, r10));
  ll_iseq->append(new Instruction(mov_opcode, r10, dest_operand));
}

/**
 * Translates HL mul instruction to LL.
 **/
void LowLevelCodeGen::hl_mul_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  int size = highlevel_opcode_get_source_operand_size(hl_opcode);

  LowLevelOpcode mov_opcode = select_ll_opcode(MINS_MOVB, size);
  LowLevelOpcode mul_opcode;

  if (size == 8) {
    mul_opcode = MINS_IMULQ;
  } else {
    mul_opcode = MINS_IMULL;
  }

  Operand src_left_operand = get_ll_operand(hl_ins->get_operand(1), size, ll_iseq);
  Operand src_right_operand = get_ll_operand(hl_ins->get_operand(2), size, ll_iseq);
  Operand dest_operand = get_ll_operand(hl_ins->get_operand(0), size, ll_iseq);

  Operand::Kind mreg_kind = select_mreg_kind(size);
  Operand r10(mreg_kind, MREG_R10);

  ll_iseq->append(new Instruction(mov_opcode, src_left_operand, r10));
  ll_iseq->append(new Instruction(mul_opcode, src_right_operand, r10));
  ll_iseq->append(new Instruction(mov_opcode, r10, dest_operand));
}

void LowLevelCodeGen::hl_cmplte_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_SETLE);
}

void LowLevelCodeGen::hl_cmplt_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_SETL);
}

void LowLevelCodeGen::hl_cmpgte_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_SETGE);
}

void LowLevelCodeGen::hl_cmpgt_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_SETG);
}

void LowLevelCodeGen::hl_cmpeq_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_SETE);
}

void LowLevelCodeGen::hl_cmpneq_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_SETNE);
}

void LowLevelCodeGen::hl_cmp_to_ll_helper(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode, LowLevelOpcode comparison) {
  int size = highlevel_opcode_get_source_operand_size(hl_opcode);

  LowLevelOpcode mov_opcode = select_ll_opcode(MINS_MOVB, size);
  LowLevelOpcode cmp_opcode = select_ll_opcode(MINS_CMPB, size);

  Operand src_left_operand = get_ll_operand(hl_ins->get_operand(1), size, ll_iseq);
  Operand src_right_operand = get_ll_operand(hl_ins->get_operand(2), size, ll_iseq);
  Operand dest_operand = get_ll_operand(hl_ins->get_operand(0), size, ll_iseq);

  Operand::Kind mreg_kind = select_mreg_kind(size);
  Operand r10(mreg_kind, MREG_R10);

  ll_iseq->append(new Instruction(mov_opcode, src_left_operand, r10));
  ll_iseq->append(new Instruction(cmp_opcode, src_right_operand, r10));

  LowLevelOpcode movz_opcode;
  bool do_movz = true;
  switch (size) {
    case 2: 
      movz_opcode = MINS_MOVZBW; 
      break;
    case 4: 
      movz_opcode = MINS_MOVZBL;
      break;
    case 8:
      movz_opcode = MINS_MOVZBQ;
      break;
    case 1:
    default: 
      do_movz = false;
      break;
  }

  Operand r10b(select_mreg_kind(1), MREG_R10);
  ll_iseq->append(new Instruction(comparison, r10b));

  if (do_movz) {
    Operand r11(mreg_kind, MREG_R11);
    ll_iseq->append(new Instruction(movz_opcode, r10b, r11));
    ll_iseq->append(new Instruction(mov_opcode, r11, dest_operand));
  } else {
    ll_iseq->append(new Instruction(mov_opcode, r10b, dest_operand));
  }
}

void LowLevelCodeGen::hl_cjmp_t_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cjmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_JNE);
}

void LowLevelCodeGen::hl_cjmp_f_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  hl_cjmp_to_ll_helper(hl_ins, ll_iseq, hl_opcode, MINS_JE);
}

void LowLevelCodeGen::hl_cjmp_to_ll_helper(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode, LowLevelOpcode condition) {
  //int size = highlevel_opcode_get_source_operand_size(hl_opcode);

  // TODO: this should be fine
  //LowLevelOpcode cmp_opcode = select_ll_opcode(MINS_CMPB, size);
  LowLevelOpcode cmp_opcode = MINS_CMPL;

  //Operand jmp_label = get_ll_operand(hl_ins->get_operand(1), size, ll_iseq);
  //Operand cmp_operand = get_ll_operand(hl_ins->get_operand(0), size, ll_iseq);
  // Hardcode 'l' size 
  Operand jmp_label = get_ll_operand(hl_ins->get_operand(1), 4, ll_iseq);
  Operand cmp_operand = get_ll_operand(hl_ins->get_operand(0), 4, ll_iseq);

  ll_iseq->append(new Instruction(cmp_opcode, Operand(Operand::IMM_IVAL, 0), cmp_operand));
  ll_iseq->append(new Instruction(condition, jmp_label));
}


void LowLevelCodeGen::hl_conv_to_ll_helper(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode, int prev_size, int new_size) {
  LowLevelOpcode prev_mov_opcode = select_ll_opcode(MINS_MOVB, prev_size);
  LowLevelOpcode new_mov_opcode = select_ll_opcode(MINS_MOVB, new_size);
  LowLevelOpcode conv_opcode = HL_TO_LL.at(hl_opcode);
  
  Operand before_conv_operand = get_ll_operand(hl_ins->get_operand(1), prev_size, ll_iseq);
  Operand after_conv_operand = get_ll_operand(hl_ins->get_operand(0), new_size, ll_iseq);

  Operand::Kind prev_mreg_kind = select_mreg_kind(prev_size);
  Operand prev_r10(prev_mreg_kind, MREG_R10);
  Operand::Kind new_mreg_kind = select_mreg_kind(new_size);
  Operand new_r10(new_mreg_kind, MREG_R10);

  ll_iseq->append(new Instruction(prev_mov_opcode, before_conv_operand, prev_r10));
  ll_iseq->append(new Instruction(conv_opcode, prev_r10, new_r10));
  ll_iseq->append(new Instruction(new_mov_opcode, new_r10, after_conv_operand));
}

void LowLevelCodeGen::hl_localaddr_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode) {
  Operand vreg_op = get_ll_operand(hl_ins->get_operand(0), 8, ll_iseq);
  Operand offset_op = get_ll_operand(hl_ins->get_operand(1), 8, ll_iseq);

  Operand::Kind mreg_kind = select_mreg_kind(8);
  Operand r10(mreg_kind, MREG_R10);

  int mem_offset = m_memory_variable_offset - offset_op.get_imm_ival();

  Operand mem_var(Operand::MREG64_MEM_OFF, MREG_RBP, mem_offset);

  ll_iseq->append(new Instruction(MINS_LEAQ, mem_var, r10));
  ll_iseq->append(new Instruction(MINS_MOVQ, r10, vreg_op));
}
