//===- M6502MCInstLower.h - Lower MachineInstr to MCInst --------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502MCINSTLOWER_H
#define LLVM_LIB_TARGET_M6502_M6502MCINSTLOWER_H

#include "MCTargetDesc/M6502MCExpr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/Support/Compiler.h"

namespace llvm {

class MachineBasicBlock;
class MachineInstr;
class MCContext;
class MCInst;
class MCOperand;
class M6502AsmPrinter;

/// M6502MCInstLower - This class is used to lower an MachineInstr into an
///                   MCInst.
class LLVM_LIBRARY_VISIBILITY M6502MCInstLower {
  using MachineOperandType = MachineOperand::MachineOperandType;

  MCContext *Ctx;
  M6502AsmPrinter &AsmPrinter;

public:
  M6502MCInstLower(M6502AsmPrinter &asmprinter);

  void Initialize(MCContext *C);
  void Lower(const MachineInstr *MI, MCInst &OutMI) const;
  MCOperand LowerOperand(const MachineOperand& MO, unsigned offset = 0) const;

private:
  MCOperand LowerSymbolOperand(const MachineOperand &MO,
                               MachineOperandType MOTy, unsigned Offset) const;
  MCOperand createSub(MachineBasicBlock *BB1, MachineBasicBlock *BB2,
                      M6502MCExpr::M6502ExprKind Kind) const;
  void lowerLongBranchLUi(const MachineInstr *MI, MCInst &OutMI) const;
  void lowerLongBranchADDiu(const MachineInstr *MI, MCInst &OutMI, int Opcode,
                            M6502MCExpr::M6502ExprKind Kind) const;
  bool lowerLongBranch(const MachineInstr *MI, MCInst &OutMI) const;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_M6502_M6502MCINSTLOWER_H
