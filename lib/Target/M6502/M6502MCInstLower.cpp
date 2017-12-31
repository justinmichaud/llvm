//===- M6502MCInstLower.cpp - Convert M6502 MachineInstr to MCInst ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower M6502 MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "M6502MCInstLower.h"
#include "MCTargetDesc/M6502BaseInfo.h"
#include "MCTargetDesc/M6502MCExpr.h"
#include "M6502AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

M6502MCInstLower::M6502MCInstLower(M6502AsmPrinter &asmprinter)
  : AsmPrinter(asmprinter) {}

void M6502MCInstLower::Initialize(MCContext *C) {
  Ctx = C;
}

MCOperand M6502MCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                              MachineOperandType MOTy,
                                              unsigned Offset) const {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_None;
  M6502MCExpr::M6502ExprKind TargetKind = M6502MCExpr::MEK_None;
  bool IsGpOff = false;
  const MCSymbol *Symbol;

  switch(MO.getTargetFlags()) {
  default:
    llvm_unreachable("Invalid target flag!");
  case M6502II::MO_NO_FLAG:
    break;
  case M6502II::MO_GPREL:
    TargetKind = M6502MCExpr::MEK_GPREL;
    break;
  case M6502II::MO_GOT_CALL:
    TargetKind = M6502MCExpr::MEK_GOT_CALL;
    break;
  case M6502II::MO_GOT:
    TargetKind = M6502MCExpr::MEK_GOT;
    break;
  case M6502II::MO_ABS_HI:
    TargetKind = M6502MCExpr::MEK_HI;
    break;
  case M6502II::MO_ABS_LO:
    TargetKind = M6502MCExpr::MEK_LO;
    break;
  case M6502II::MO_TLSGD:
    TargetKind = M6502MCExpr::MEK_TLSGD;
    break;
  case M6502II::MO_TLSLDM:
    TargetKind = M6502MCExpr::MEK_TLSLDM;
    break;
  case M6502II::MO_DTPREL_HI:
    TargetKind = M6502MCExpr::MEK_DTPREL_HI;
    break;
  case M6502II::MO_DTPREL_LO:
    TargetKind = M6502MCExpr::MEK_DTPREL_LO;
    break;
  case M6502II::MO_GOTTPREL:
    TargetKind = M6502MCExpr::MEK_GOTTPREL;
    break;
  case M6502II::MO_TPREL_HI:
    TargetKind = M6502MCExpr::MEK_TPREL_HI;
    break;
  case M6502II::MO_TPREL_LO:
    TargetKind = M6502MCExpr::MEK_TPREL_LO;
    break;
  case M6502II::MO_GPOFF_HI:
    TargetKind = M6502MCExpr::MEK_HI;
    IsGpOff = true;
    break;
  case M6502II::MO_GPOFF_LO:
    TargetKind = M6502MCExpr::MEK_LO;
    IsGpOff = true;
    break;
  case M6502II::MO_GOT_DISP:
    TargetKind = M6502MCExpr::MEK_GOT_DISP;
    break;
  case M6502II::MO_GOT_HI16:
    TargetKind = M6502MCExpr::MEK_GOT_HI16;
    break;
  case M6502II::MO_GOT_LO16:
    TargetKind = M6502MCExpr::MEK_GOT_LO16;
    break;
  case M6502II::MO_GOT_PAGE:
    TargetKind = M6502MCExpr::MEK_GOT_PAGE;
    break;
  case M6502II::MO_GOT_OFST:
    TargetKind = M6502MCExpr::MEK_GOT_OFST;
    break;
  case M6502II::MO_HIGHER:
    TargetKind = M6502MCExpr::MEK_HIGHER;
    break;
  case M6502II::MO_HIGHEST:
    TargetKind = M6502MCExpr::MEK_HIGHEST;
    break;
  case M6502II::MO_CALL_HI16:
    TargetKind = M6502MCExpr::MEK_CALL_HI16;
    break;
  case M6502II::MO_CALL_LO16:
    TargetKind = M6502MCExpr::MEK_CALL_LO16;
    break;
  }

  switch (MOTy) {
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = AsmPrinter.getSymbol(MO.getGlobal());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = AsmPrinter.GetBlockAddressSymbol(MO.getBlockAddress());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = AsmPrinter.GetExternalSymbolSymbol(MO.getSymbolName());
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_MCSymbol:
    Symbol = MO.getMCSymbol();
    Offset += MO.getOffset();
    break;

  case MachineOperand::MO_JumpTableIndex:
    Symbol = AsmPrinter.GetJTISymbol(MO.getIndex());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = AsmPrinter.GetCPISymbol(MO.getIndex());
    Offset += MO.getOffset();
    break;

  default:
    llvm_unreachable("<unknown operand type>");
  }

  const MCExpr *Expr = MCSymbolRefExpr::create(Symbol, Kind, *Ctx);

  if (Offset) {
    // Assume offset is never negative.
    assert(Offset > 0);

    Expr = MCBinaryExpr::createAdd(Expr, MCConstantExpr::create(Offset, *Ctx),
                                   *Ctx);
  }

  if (IsGpOff)
    Expr = M6502MCExpr::createGpOff(TargetKind, Expr, *Ctx);
  else if (TargetKind != M6502MCExpr::MEK_None)
    Expr = M6502MCExpr::create(TargetKind, Expr, *Ctx);

  return MCOperand::createExpr(Expr);
}

MCOperand M6502MCInstLower::LowerOperand(const MachineOperand &MO,
                                        unsigned offset) const {
  MachineOperandType MOTy = MO.getType();

  switch (MOTy) {
  default: llvm_unreachable("unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) break;
    return MCOperand::createReg(MO.getReg());
  case MachineOperand::MO_Immediate:
    return MCOperand::createImm(MO.getImm() + offset);
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_MCSymbol:
  case MachineOperand::MO_JumpTableIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_BlockAddress:
    return LowerSymbolOperand(MO, MOTy, offset);
  case MachineOperand::MO_RegisterMask:
    break;
 }

  return MCOperand();
}

MCOperand M6502MCInstLower::createSub(MachineBasicBlock *BB1,
                                     MachineBasicBlock *BB2,
                                     M6502MCExpr::M6502ExprKind Kind) const {
  const MCSymbolRefExpr *Sym1 = MCSymbolRefExpr::create(BB1->getSymbol(), *Ctx);
  const MCSymbolRefExpr *Sym2 = MCSymbolRefExpr::create(BB2->getSymbol(), *Ctx);
  const MCBinaryExpr *Sub = MCBinaryExpr::createSub(Sym1, Sym2, *Ctx);

  return MCOperand::createExpr(M6502MCExpr::create(Kind, Sub, *Ctx));
}

void M6502MCInstLower::
lowerLongBranchLUi(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(M6502::LUi);

  // Lower register operand.
  OutMI.addOperand(LowerOperand(MI->getOperand(0)));

  // Create %hi($tgt-$baltgt).
  OutMI.addOperand(createSub(MI->getOperand(1).getMBB(),
                             MI->getOperand(2).getMBB(),
                             M6502MCExpr::MEK_HI));
}

void M6502MCInstLower::lowerLongBranchADDiu(
    const MachineInstr *MI, MCInst &OutMI, int Opcode,
    M6502MCExpr::M6502ExprKind Kind) const {
  OutMI.setOpcode(Opcode);

  // Lower two register operands.
  for (unsigned I = 0, E = 2; I != E; ++I) {
    const MachineOperand &MO = MI->getOperand(I);
    OutMI.addOperand(LowerOperand(MO));
  }

  // Create %lo($tgt-$baltgt) or %hi($tgt-$baltgt).
  OutMI.addOperand(createSub(MI->getOperand(2).getMBB(),
                             MI->getOperand(3).getMBB(), Kind));
}

bool M6502MCInstLower::lowerLongBranch(const MachineInstr *MI,
                                      MCInst &OutMI) const {
  switch (MI->getOpcode()) {
  default:
    return false;
  case M6502::LONG_BRANCH_LUi:
    lowerLongBranchLUi(MI, OutMI);
    return true;
  case M6502::LONG_BRANCH_ADDiu:
    lowerLongBranchADDiu(MI, OutMI, M6502::ADDiu, M6502MCExpr::MEK_LO);
    return true;
  case M6502::LONG_BRANCH_DADDiu:
    unsigned TargetFlags = MI->getOperand(2).getTargetFlags();
    if (TargetFlags == M6502II::MO_ABS_HI)
      lowerLongBranchADDiu(MI, OutMI, M6502::DADDiu, M6502MCExpr::MEK_HI);
    else if (TargetFlags == M6502II::MO_ABS_LO)
      lowerLongBranchADDiu(MI, OutMI, M6502::DADDiu, M6502MCExpr::MEK_LO);
    else
      report_fatal_error("Unexpected flags for LONG_BRANCH_DADDiu");
    return true;
  }
}

void M6502MCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  if (lowerLongBranch(MI, OutMI))
    return;

  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}
