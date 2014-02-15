//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//
class SimpleInstr<dag outs, dag ins, string asmstr, list<dag> pattern>
    : Instruction 
{
  field bits<32> Inst;

  let Namespace = "Simple";
  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString   = asmstr;
  let Pattern = pattern;
}

// Pseudo instruction
class SimplePseudoInstr<dag outs, dag ins, string asmstr, list<dag> pattern>
   : SimpleInstr<outs, ins, asmstr, pattern> 
{
  let isPseudo = 1;
}
