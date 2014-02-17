#ifndef SIMPLE_MCINSTLOWER_H
#define SIMPLE_MCINSTLOWER_H
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/Support/Compiler.h"

namespace llvm {
  class MCContext;
  class MCInst;
  class MCOperand;
  class MachineInstr;
  class MachineFunction;
  class Mangler;
  class AsmPrinter;

/// \brief This class is used to lower an MachineInstr into an MCInst.
class LLVM_LIBRARY_VISIBILITY SimpleMCInstLower {
  typedef MachineOperand::MachineOperandType MachineOperandType;
  MCContext *Ctx;
  Mangler *Mang;
  AsmPrinter &Printer;
public:
  SimpleMCInstLower(class AsmPrinter &asmprinter);
  void Initialize(Mangler *mang, MCContext *C);
  void Lower(const MachineInstr *MI, MCInst &OutMI) const;
  MCOperand LowerOperand(const MachineOperand& MO, unsigned offset = 0) const;
};
}

#endif // SIMPLE_MCINSTLOWER_H
