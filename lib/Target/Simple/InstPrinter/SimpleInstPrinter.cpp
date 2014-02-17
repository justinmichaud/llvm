#define DEBUG_TYPE "asm-printer"
#include "SimpleInstPrinter.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#include "SimpleGenAsmWriter.inc"

void SimpleInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const 
{
  OS << StringRef(getRegisterName(RegNo)).lower();
}

void SimpleInstPrinter::printInst(const MCInst *MI, raw_ostream &O,
                                 StringRef Annot) 
{
  printInstruction(MI, O);
}

void SimpleInstPrinter::
printOperand(const MCInst *MI, unsigned OpNo, raw_ostream &O) 
{
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) 
  {
    printRegName(O, Op.getReg());
    return;
  }

  if (Op.isImm()) 
  {
    O << Op.getImm();
    return;
  }

  assert(!"not implemented");
}
