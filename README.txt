I changed all instances of Symbol * to be wrapped in a shared_ptr.
This helped simplify dealing with Symbols in HL code gen.

I store a vreg # or offset value in each var's symbol.

Each Node holds an Operand, which is annotated and used throughout 
HL code gen.

I try to reuse temp vregs whenever possibly, resetting 
the starting temp vreg value after evalutaing an expression.
I think the way I do so is a bit different than the reference solution,
resulting in some different output. However, they are still equivalent.

Currently I do not support String constants or global variables I think.
This will be supported in the next milestone. 

The main logic in HL codegen is getting node operands, manipulating them, and 
annotating the current node such that it can be used up the recursive chain.

(I also don't print out any symbol table entries anymore).