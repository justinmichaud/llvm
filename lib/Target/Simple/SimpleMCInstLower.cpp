///
/// \file
/// \brief This file contains code to lower Simple MachineInstrs to their
/// corresponding MCInst records.
///
//===----------------------------------------------------------------------===//
#include "SimpleMCInstLower.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"

using namespace llvm;

SimpleMCInstLower::SimpleMCInstLower(class AsmPrinter &asmprinter)
: Printer(asmprinter) {}

void SimpleMCInstLower::Initialize(Mangler *M, MCContext *C) {
  Mang = M;
  Ctx = C;
}

MCOperand SimpleMCInstLower::LowerOperand(const MachineOperand &MO,
                                         unsigned offset) const {
  MachineOperandType MOTy = MO.getType();

  switch (MOTy) 
  {
    default: llvm_unreachable("unknown operand type");
    case MachineOperand::MO_Register:
      return MCOperand::CreateReg(MO.getReg());
    case MachineOperand::MO_Immediate:
      return MCOperand::CreateImm(MO.getImm() + offset);
  }

  return MCOperand();
}

void SimpleMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}
