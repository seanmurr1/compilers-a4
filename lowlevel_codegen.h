#ifndef LOWLEVEL_CODEGEN_H
#define LOWLEVEL_CODEGEN_H

#include <memory>
#include "instruction_seq.h"

// A LowLevelCodeGen object transforms an InstructionSequence containing
// high-level instructions into an InstructionSequence containing
// low-level (x86-64) instructions.
class LowLevelCodeGen {
private:
  int m_total_memory_storage;
  bool m_optimize;

  int m_total_vreg_memory;

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

  long get_stack_offset(int vreg_num);

};

#endif // LOWLEVEL_CODEGEN_H
