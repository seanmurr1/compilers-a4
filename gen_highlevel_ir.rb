#! /usr/bin/env ruby

# Arithmetic and data movement operations: these will be generated in
# 4 different "widths": byte (8 bit), word (16 bit), long (32 bit),
# quad (64 bit)
ARITH = [
  # Arithmetic
  :add,
  :sub,
  :mul,
  :div,
  :mod,
  :lshift,
  :rshift,

  # Integer comparisons to compute a boolean value:
  # note that all of these assign to a destination vreg
  :cmplt,
  :cmplte,
  :cmpgt,
  :cmpgte,
  :cmpeq,
  :cmpneq,

  # Bitwise operations
  :and,
  :or,
  :xor,

  # Unary ALU opcodes
  :neg,     # Unary negation (numeric)
  :not,     # Unary not (logical)
  :compl,   # Bitwise complement
  :inc,     # Increment integer
  :dec,     # Decrement integer

  # The (variants of the) mov instruction is used for
  # all moves of data values involving some combination of
  # registers, immediate values, and memory locations.
  :mov,
]

SIZES = [ :b, :w, :l, :q ]

def promotions(s)
  pairs = (0..SIZES.length-1).to_a.product((1..SIZES.length-1).to_a)
  pairs = pairs.filter { |pair| pair[0] < pair[1] }
  return pairs.map { |pair| "#{s}_#{SIZES[pair[0]]}#{SIZES[pair[1]]}".to_sym }
end

OPCODES = [
  :nop,

  # Include the arithmetic and data movement operations,
  # with variations for different operand sizes
  *(ARITH.product(SIZES).map { |pair| "#{pair[0]}_#{pair[1]}".to_sym }),

  # Signed promotions (convert less-precise value to a more-precise type)
  *promotions("sconv"),

  # Unsigned promotions (convert less-precise value to a more-precise type)
  *promotions("uconv"),

  # control flow
  :ret,
  :jmp,
  :call,

  # Enter the stack frame. Allocates specified amount of local storage.
  :enter,

  # Leave the stack frame. Should deallocate the amount of storage
  # that the enter instruction allocated.
  :leave,

  # Compute the address of a variable at a specified (immediate) offset
  # in local storage, storing it in a vreg.
  :localaddr,

  # conditional jump
  :cjmp_t,    # conditional jump if boolean is true
  :cjmp_f,    # conditional jump if boolean is false
]

opcode_names = OPCODES.map { |sym| "HINS_#{sym.to_s}" }

#opcode_names.each do |opcode_name|
#  puts opcode_name
#end

# Generate highlevel.h
File.open('highlevel.h', 'w') do |outf|
  outf.print <<'EOF1'
#ifndef HIGHLEVEL_H
#define HIGHLEVEL_H

#include "formatter.h"

// Generated high-level opcodes and support functions/classes
// Do not edit this file!

enum HighLevelOpcode {
EOF1

  opcode_names.each do |opcode_name|
    outf.puts "  #{opcode_name},"
  end

  outf.print <<'EOF2'
}; // HighLevelOpcode enumeration

// Translate a high-level opcode to its assembler mnemonic.
// Returns nullptr if the opcode is unknown.
const char *highlevel_opcode_to_str(HighLevelOpcode opcode);

#endif // HIGHLEVEL_H
EOF2
end

File.open('highlevel.cpp', 'w') do |outf|
  outf.print <<'EOF3'
#include "highlevel.h"

// Generated high-level opcodes (support functions/classes)
// Do not edit this file!

const char *highlevel_opcode_to_str(HighLevelOpcode opcode) {
  switch (opcode) {
EOF3

  opcode_names.each do |opcode_name|
    len = opcode_name.length
    mnemonic = opcode_name[5..len-1]
    pad = ' ' * (16 - len)
    outf.puts "  case #{opcode_name}:#{pad}return \"#{mnemonic}\";"
  end

  outf.print <<'EOF4'
  default: return nullptr;
  } // end switch
} // end opcode_to_str function
EOF4
end

# Generate highlevel.cpp
