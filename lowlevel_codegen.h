#ifndef LOWLEVEL_CODEGEN_H
#define LOWLEVEL_CODEGEN_H

#include <memory>
#include "instruction_seq.h"
#include "lowlevel.h"

// A LowLevelCodeGen object transforms an InstructionSequence containing
// high-level instructions into an InstructionSequence containing
// low-level (x86-64) instructions.
class LowLevelCodeGen {
private:
  int m_total_memory_storage;
  bool m_optimize;

  int m_memory_variable_offset;
  int m_vreg_storage_offset;

  bool m_r10_in_use;
  bool m_r11_in_use;

public:
  LowLevelCodeGen(bool optimize);
  virtual ~LowLevelCodeGen();

  std::shared_ptr<InstructionSequence> generate(const std::shared_ptr<InstructionSequence> &hl_iseq);

private:
  std::shared_ptr<InstructionSequence> translate_hl_to_ll(const std::shared_ptr<InstructionSequence> &hl_iseq);
  void translate_instruction(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq);
  Operand get_ll_operand(Operand hl_opcode, int size, const std::shared_ptr<InstructionSequence> &ll_iseq);

  void hl_mov_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);

  void hl_add_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_sub_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_mul_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);

  void hl_div_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_mod_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);

  
  void hl_cmplte_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cmplt_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cmpgte_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cmpgt_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cmpeq_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cmpneq_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cmp_to_ll_helper(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode, LowLevelOpcode comparison);

  void hl_cjmp_t_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cjmp_f_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  void hl_cjmp_to_ll_helper(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode, LowLevelOpcode condition);

  void hl_conv_to_ll_helper(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode, int prev_size, int new_size);

  void hl_localaddr_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);
  
  // Unary
  void hl_neg_to_ll(Instruction *hl_ins, const std::shared_ptr<InstructionSequence> &ll_iseq, HighLevelOpcode hl_opcode);



  long get_stack_offset(int vreg_num);

};

#endif // LOWLEVEL_CODEGEN_H
