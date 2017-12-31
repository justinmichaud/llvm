//===-- M6502MCCodeEmitter.cpp - Convert M6502 Code to Machine Code ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the M6502MCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "M6502MCCodeEmitter.h"
#include "MCTargetDesc/M6502FixupKinds.h"
#include "MCTargetDesc/M6502MCExpr.h"
#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "M6502GenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

MCCodeEmitter *createM6502MCCodeEmitterEB(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new M6502MCCodeEmitter(MCII, Ctx, false);
}

MCCodeEmitter *createM6502MCCodeEmitterEL(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new M6502MCCodeEmitter(MCII, Ctx, true);
}

} // end namespace llvm

// If the D<shift> instruction has a shift amount that is greater
// than 31 (checked in calling routine), lower it to a D<shift>32 instruction
static void LowerLargeShift(MCInst& Inst) {
  assert(Inst.getNumOperands() == 3 && "Invalid no. of operands for shift!");
  assert(Inst.getOperand(2).isImm());

  int64_t Shift = Inst.getOperand(2).getImm();
  if (Shift <= 31)
    return; // Do nothing
  Shift -= 32;

  // saminus32
  Inst.getOperand(2).setImm(Shift);

  switch (Inst.getOpcode()) {
  default:
    // Calling function is not synchronized
    llvm_unreachable("Unexpected shift instruction");
  case M6502::DSLL:
    Inst.setOpcode(M6502::DSLL32);
    return;
  case M6502::DSRL:
    Inst.setOpcode(M6502::DSRL32);
    return;
  case M6502::DSRA:
    Inst.setOpcode(M6502::DSRA32);
    return;
  case M6502::DROTR:
    Inst.setOpcode(M6502::DROTR32);
    return;
  case M6502::DSLL_MM64R6:
    Inst.setOpcode(M6502::DSLL32_MM64R6);
    return;
  case M6502::DSRL_MM64R6:
    Inst.setOpcode(M6502::DSRL32_MM64R6);
    return;
  case M6502::DSRA_MM64R6:
    Inst.setOpcode(M6502::DSRA32_MM64R6);
    return;
  case M6502::DROTR_MM64R6:
    Inst.setOpcode(M6502::DROTR32_MM64R6);
    return;
  }
}

// Fix a bad compact branch encoding for beqc/bnec.
void M6502MCCodeEmitter::LowerCompactBranch(MCInst& Inst) const {
  // Encoding may be illegal !(rs < rt), but this situation is
  // easily fixed.
  unsigned RegOp0 = Inst.getOperand(0).getReg();
  unsigned RegOp1 = Inst.getOperand(1).getReg();

  unsigned Reg0 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp0);
  unsigned Reg1 =  Ctx.getRegisterInfo()->getEncodingValue(RegOp1);

  if (Inst.getOpcode() == M6502::BNEC || Inst.getOpcode() == M6502::BEQC ||
      Inst.getOpcode() == M6502::BNEC64 || Inst.getOpcode() == M6502::BEQC64) {
    assert(Reg0 != Reg1 && "Instruction has bad operands ($rs == $rt)!");
    if (Reg0 < Reg1)
      return;
  } else if (Inst.getOpcode() == M6502::BNVC || Inst.getOpcode() == M6502::BOVC) {
    if (Reg0 >= Reg1)
      return;
  } else if (Inst.getOpcode() == M6502::BNVC_MMR6 ||
             Inst.getOpcode() == M6502::BOVC_MMR6) {
    if (Reg1 >= Reg0)
      return;
  } else
    llvm_unreachable("Cannot rewrite unknown branch!");

  Inst.getOperand(0).setReg(RegOp1);
  Inst.getOperand(1).setReg(RegOp0);
}

bool M6502MCCodeEmitter::isMicroM6502(const MCSubtargetInfo &STI) const {
  return STI.getFeatureBits()[M6502::FeatureMicroM6502];
}

bool M6502MCCodeEmitter::isM650232r6(const MCSubtargetInfo &STI) const {
  return STI.getFeatureBits()[M6502::FeatureM650232r6];
}

void M6502MCCodeEmitter::EmitByte(unsigned char C, raw_ostream &OS) const {
  OS << (char)C;
}

void M6502MCCodeEmitter::EmitInstruction(uint64_t Val, unsigned Size,
                                        const MCSubtargetInfo &STI,
                                        raw_ostream &OS) const {
  // Output the instruction encoding in little endian byte order.
  // Little-endian byte ordering:
  //   m650232r2:   4 | 3 | 2 | 1
  //   microM6502:  2 | 1 | 4 | 3
  if (IsLittleEndian && Size == 4 && isMicroM6502(STI)) {
    EmitInstruction(Val >> 16, 2, STI, OS);
    EmitInstruction(Val, 2, STI, OS);
  } else {
    for (unsigned i = 0; i < Size; ++i) {
      unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
      EmitByte((Val >> Shift) & 0xff, OS);
    }
  }
}

/// encodeInstruction - Emit the instruction.
/// Size the instruction with Desc.getSize().
void M6502MCCodeEmitter::
encodeInstruction(const MCInst &MI, raw_ostream &OS,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const
{
  // Non-pseudo instructions that get changed for direct object
  // only based on operand values.
  // If this list of instructions get much longer we will move
  // the check to a function call. Until then, this is more efficient.
  MCInst TmpInst = MI;
  switch (MI.getOpcode()) {
  // If shift amount is >= 32 it the inst needs to be lowered further
  case M6502::DSLL:
  case M6502::DSRL:
  case M6502::DSRA:
  case M6502::DROTR:
  case M6502::DSLL_MM64R6:
  case M6502::DSRL_MM64R6:
  case M6502::DSRA_MM64R6:
  case M6502::DROTR_MM64R6:
    LowerLargeShift(TmpInst);
    break;
  // Compact branches, enforce encoding restrictions.
  case M6502::BEQC:
  case M6502::BNEC:
  case M6502::BEQC64:
  case M6502::BNEC64:
  case M6502::BOVC:
  case M6502::BOVC_MMR6:
  case M6502::BNVC:
  case M6502::BNVC_MMR6:
    LowerCompactBranch(TmpInst);
  }

  unsigned long N = Fixups.size();
  uint32_t Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);

  // Check for unimplemented opcodes.
  // Unfortunately in M6502 both NOP and SLL will come in with Binary == 0
  // so we have to special check for them.
  unsigned Opcode = TmpInst.getOpcode();
  if ((Opcode != M6502::NOP) && (Opcode != M6502::SLL) &&
      (Opcode != M6502::SLL_MM) && !Binary)
    llvm_unreachable("unimplemented opcode in encodeInstruction()");

  int NewOpcode = -1;
  if (isMicroM6502(STI)) {
    if (isM650232r6(STI)) {
      NewOpcode = M6502::M6502R62MicroM6502R6(Opcode, M6502::Arch_microm6502r6);
      if (NewOpcode == -1)
        NewOpcode = M6502::Std2MicroM6502R6(Opcode, M6502::Arch_microm6502r6);
    }
    else
      NewOpcode = M6502::Std2MicroM6502(Opcode, M6502::Arch_microm6502);

    // Check whether it is Dsp instruction.
    if (NewOpcode == -1)
      NewOpcode = M6502::Dsp2MicroM6502(Opcode, M6502::Arch_mmdsp);

    if (NewOpcode != -1) {
      if (Fixups.size() > N)
        Fixups.pop_back();

      Opcode = NewOpcode;
      TmpInst.setOpcode (NewOpcode);
      Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
    }
  }

  const MCInstrDesc &Desc = MCII.get(TmpInst.getOpcode());

  // Get byte count of instruction
  unsigned Size = Desc.getSize();
  if (!Size)
    llvm_unreachable("Desc.getSize() returns 0");

  EmitInstruction(Binary, Size, STI, OS);
}

/// getBranchTargetOpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTargetOpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_M6502_PC16)));
  return 0;
}

/// getBranchTargetOpValue1SImm16 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTargetOpValue1SImm16(const MCInst &MI, unsigned OpNo,
                              SmallVectorImpl<MCFixup> &Fixups,
                              const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_M6502_PC16)));
  return 0;
}

/// getBranchTargetOpValueMMR6 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTargetOpValueMMR6(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMMR6 expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-2, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_M6502_PC16)));
  return 0;
}

/// getBranchTargetOpValueLsl2MMR6 - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTargetOpValueLsl2MMR6(const MCInst &MI, unsigned OpNo,
                               SmallVectorImpl<MCFixup> &Fixups,
                               const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm())
    return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueLsl2MMR6 expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_M6502_PC16)));
  return 0;
}

/// getBranchTarget7OpValueMM - Return binary encoding of the microM6502 branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTarget7OpValueMM(const MCInst &MI, unsigned OpNo,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMM expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(M6502::fixup_MICROM6502_PC7_S1)));
  return 0;
}

/// getBranchTargetOpValueMMPC10 - Return binary encoding of the microM6502
/// 10-bit branch target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTargetOpValueMMPC10(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValuePC10 expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                   MCFixupKind(M6502::fixup_MICROM6502_PC10_S1)));
  return 0;
}

/// getBranchTargetOpValue - Return binary encoding of the microM6502 branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTargetOpValueMM(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTargetOpValueMM expects only expressions or immediates");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                   MCFixupKind(M6502::
                               fixup_MICROM6502_PC16_S1)));
  return 0;
}

/// getBranchTarget21OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTarget21OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget21OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_M6502_PC21_S2)));
  return 0;
}

/// getBranchTarget21OpValueMM - Return binary encoding of the branch
/// target operand for microM6502. If the machine operand requires
/// relocation, record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTarget21OpValueMM(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
    "getBranchTarget21OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_MICROM6502_PC21_S1)));
  return 0;
}

/// getBranchTarget26OpValue - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getBranchTarget26OpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm() >> 2;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValue expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_M6502_PC26_S2)));
  return 0;
}

/// getBranchTarget26OpValueMM - Return binary encoding of the branch
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::getBranchTarget26OpValueMM(
    const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups,
    const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, divide by 2.
  if (MO.isImm())
    return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getBranchTarget26OpValueMM expects only expressions or immediates");

  const MCExpr *FixupExpression = MCBinaryExpr::createAdd(
      MO.getExpr(), MCConstantExpr::create(-4, Ctx), Ctx);
  Fixups.push_back(MCFixup::create(0, FixupExpression,
                                   MCFixupKind(M6502::fixup_MICROM6502_PC26_S1)));
  return 0;
}

/// getJumpOffset16OpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getJumpOffset16OpValue(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) return MO.getImm();

  assert(MO.isExpr() &&
         "getJumpOffset16OpValue expects only expressions or an immediate");

   // TODO: Push fixup.
   return 0;
}

/// getJumpTargetOpValue - Return binary encoding of the jump
/// target operand. If the machine operand requires relocation,
/// record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getJumpTargetOpValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, divide by 4.
  if (MO.isImm()) return MO.getImm()>>2;

  assert(MO.isExpr() &&
         "getJumpTargetOpValue expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(M6502::fixup_M6502_26)));
  return 0;
}

unsigned M6502MCCodeEmitter::
getJumpTargetOpValueMM(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, divide by 2.
  if (MO.isImm()) return MO.getImm() >> 1;

  assert(MO.isExpr() &&
         "getJumpTargetOpValueMM expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  Fixups.push_back(MCFixup::create(0, Expr,
                                   MCFixupKind(M6502::fixup_MICROM6502_26_S1)));
  return 0;
}

unsigned M6502MCCodeEmitter::
getUImm5Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getUImm5Lsl2Encoding expects only expressions or an immediate");

  return 0;
}

unsigned M6502MCCodeEmitter::
getSImm3Lsa2Value(const MCInst &MI, unsigned OpNo,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    int Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned M6502MCCodeEmitter::
getUImm6Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Value = MO.getImm();
    return Value >> 2;
  }

  return 0;
}

unsigned M6502MCCodeEmitter::
getSImm9AddiuspValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    unsigned Binary = (MO.getImm() >> 2) & 0x0000ffff;
    return (((Binary & 0x8000) >> 7) | (Binary & 0x00ff));
  }

  return 0;
}

unsigned M6502MCCodeEmitter::
getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
               const MCSubtargetInfo &STI) const {
  int64_t Res;

  if (Expr->evaluateAsAbsolute(Res))
    return Res;

  MCExpr::ExprKind Kind = Expr->getKind();
  if (Kind == MCExpr::Constant) {
    return cast<MCConstantExpr>(Expr)->getValue();
  }

  if (Kind == MCExpr::Binary) {
    unsigned Res = getExprOpValue(cast<MCBinaryExpr>(Expr)->getLHS(), Fixups, STI);
    Res += getExprOpValue(cast<MCBinaryExpr>(Expr)->getRHS(), Fixups, STI);
    return Res;
  }

  if (Kind == MCExpr::Target) {
    const M6502MCExpr *M6502Expr = cast<M6502MCExpr>(Expr);

    M6502::Fixups FixupKind = M6502::Fixups(0);
    switch (M6502Expr->getKind()) {
    case M6502MCExpr::MEK_None:
    case M6502MCExpr::MEK_Special:
      llvm_unreachable("Unhandled fixup kind!");
      break;
    case M6502MCExpr::MEK_CALL_HI16:
      FixupKind = M6502::fixup_M6502_CALL_HI16;
      break;
    case M6502MCExpr::MEK_CALL_LO16:
      FixupKind = M6502::fixup_M6502_CALL_LO16;
      break;
    case M6502MCExpr::MEK_DTPREL_HI:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_TLS_DTPREL_HI16
                                   : M6502::fixup_M6502_DTPREL_HI;
      break;
    case M6502MCExpr::MEK_DTPREL_LO:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_TLS_DTPREL_LO16
                                   : M6502::fixup_M6502_DTPREL_LO;
      break;
    case M6502MCExpr::MEK_GOTTPREL:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_GOTTPREL
                                   : M6502::fixup_M6502_GOTTPREL;
      break;
    case M6502MCExpr::MEK_GOT:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_GOT16
                                   : M6502::fixup_M6502_GOT;
      break;
    case M6502MCExpr::MEK_GOT_CALL:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_CALL16
                                   : M6502::fixup_M6502_CALL16;
      break;
    case M6502MCExpr::MEK_GOT_DISP:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_GOT_DISP
                                   : M6502::fixup_M6502_GOT_DISP;
      break;
    case M6502MCExpr::MEK_GOT_HI16:
      FixupKind = M6502::fixup_M6502_GOT_HI16;
      break;
    case M6502MCExpr::MEK_GOT_LO16:
      FixupKind = M6502::fixup_M6502_GOT_LO16;
      break;
    case M6502MCExpr::MEK_GOT_PAGE:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_GOT_PAGE
                                   : M6502::fixup_M6502_GOT_PAGE;
      break;
    case M6502MCExpr::MEK_GOT_OFST:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_GOT_OFST
                                   : M6502::fixup_M6502_GOT_OFST;
      break;
    case M6502MCExpr::MEK_GPREL:
      FixupKind = M6502::fixup_M6502_GPREL16;
      break;
    case M6502MCExpr::MEK_LO:
      // Check for %lo(%neg(%gp_rel(X)))
      if (M6502Expr->isGpOff()) {
        FixupKind = M6502::fixup_M6502_GPOFF_LO;
        break;
      }
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_LO16
                                   : M6502::fixup_M6502_LO16;
      break;
    case M6502MCExpr::MEK_HIGHEST:
      FixupKind = M6502::fixup_M6502_HIGHEST;
      break;
    case M6502MCExpr::MEK_HIGHER:
      FixupKind = M6502::fixup_M6502_HIGHER;
      break;
    case M6502MCExpr::MEK_HI:
      // Check for %hi(%neg(%gp_rel(X)))
      if (M6502Expr->isGpOff()) {
        FixupKind = M6502::fixup_M6502_GPOFF_HI;
        break;
      }
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_HI16
                                   : M6502::fixup_M6502_HI16;
      break;
    case M6502MCExpr::MEK_PCREL_HI16:
      FixupKind = M6502::fixup_M6502_PCHI16;
      break;
    case M6502MCExpr::MEK_PCREL_LO16:
      FixupKind = M6502::fixup_M6502_PCLO16;
      break;
    case M6502MCExpr::MEK_TLSGD:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_TLS_GD
                                   : M6502::fixup_M6502_TLSGD;
      break;
    case M6502MCExpr::MEK_TLSLDM:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_TLS_LDM
                                   : M6502::fixup_M6502_TLSLDM;
      break;
    case M6502MCExpr::MEK_TPREL_HI:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_TLS_TPREL_HI16
                                   : M6502::fixup_M6502_TPREL_HI;
      break;
    case M6502MCExpr::MEK_TPREL_LO:
      FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_TLS_TPREL_LO16
                                   : M6502::fixup_M6502_TPREL_LO;
      break;
    case M6502MCExpr::MEK_NEG:
      FixupKind =
          isMicroM6502(STI) ? M6502::fixup_MICROM6502_SUB : M6502::fixup_M6502_SUB;
      break;
    }
    Fixups.push_back(MCFixup::create(0, M6502Expr, MCFixupKind(FixupKind)));
    return 0;
  }

  if (Kind == MCExpr::SymbolRef) {
    M6502::Fixups FixupKind = M6502::Fixups(0);

    switch(cast<MCSymbolRefExpr>(Expr)->getKind()) {
    default: llvm_unreachable("Unknown fixup kind!");
      break;
    case MCSymbolRefExpr::VK_None:
      FixupKind = M6502::fixup_M6502_32; // FIXME: This is ok for O32/N32 but not N64.
      break;
    } // switch

    Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
    return 0;
  }
  return 0;
}

/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned M6502MCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    return RegNo;
  } else if (MO.isImm()) {
    return static_cast<unsigned>(MO.getImm());
  } else if (MO.isFPImm()) {
    return static_cast<unsigned>(APFloat(MO.getFPImm())
        .bitcastToAPInt().getHiBits(32).getLimitedValue());
  }
  // MO must be an Expr.
  assert(MO.isExpr());
  return getExprOpValue(MO.getExpr(),Fixups, STI);
}

/// Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
template <unsigned ShiftAmount>
unsigned M6502MCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),Fixups, STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  // Apply the scale factor if there is one.
  OffBits >>= ShiftAmount;

  return (OffBits & 0xFFFF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm4(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI);

  return (OffBits & 0xF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm4Lsl1(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 1;

  return (OffBits & 0xF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm4Lsl2(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 6-4, offset is encoded in bits 3-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo),
                                       Fixups, STI) << 4;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return (OffBits & 0xF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMSPImm5Lsl2(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  // Register is encoded in bits 9-5, offset is encoded in bits 4-0.
  assert(MI.getOperand(OpNo).isReg() &&
         (MI.getOperand(OpNo).getReg() == M6502::SP ||
         MI.getOperand(OpNo).getReg() == M6502::SP_64) &&
         "Unexpected base register!");
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return OffBits & 0x1F;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMGPImm7Lsl2(const MCInst &MI, unsigned OpNo,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const {
  // Register is encoded in bits 9-7, offset is encoded in bits 6-0.
  assert(MI.getOperand(OpNo).isReg() &&
         MI.getOperand(OpNo).getReg() == M6502::GP &&
         "Unexpected base register!");

  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1),
                                       Fixups, STI) >> 2;

  return OffBits & 0x7F;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm9(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 8-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo + 1), Fixups, STI);

  return (OffBits & 0x1FF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm11(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 10-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0x07FF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm12(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // opNum can be invalid if instruction had reglist as operand.
  // MemOperand is always last operand of instruction (base + offset).
  switch (MI.getOpcode()) {
  default:
    break;
  case M6502::SWM32_MM:
  case M6502::LWM32_MM:
    OpNo = MI.getNumOperands() - 2;
    break;
  }

  // Base register is encoded in bits 20-16, offset is encoded in bits 11-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0x0FFF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm16(const MCInst &MI, unsigned OpNo,
                      SmallVectorImpl<MCFixup> &Fixups,
                      const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups,
                                       STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0xFFFF) | RegBits;
}

unsigned M6502MCCodeEmitter::
getMemEncodingMMImm4sp(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const {
  // opNum can be invalid if instruction had reglist as operand
  // MemOperand is always last operand of instruction (base + offset)
  switch (MI.getOpcode()) {
  default:
    break;
  case M6502::SWM16_MM:
  case M6502::SWM16_MMR6:
  case M6502::LWM16_MM:
  case M6502::LWM16_MMR6:
    OpNo = MI.getNumOperands() - 2;
    break;
  }

  // Offset is encoded in bits 4-0.
  assert(MI.getOperand(OpNo).isReg());
  // Base register is always SP - thus it is not encoded.
  assert(MI.getOperand(OpNo+1).isImm());
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return ((OffBits >> 2) & 0x0F);
}

// FIXME: should be called getMSBEncoding
//
unsigned
M6502MCCodeEmitter::getSizeInsEncoding(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo-1).isImm());
  assert(MI.getOperand(OpNo).isImm());
  unsigned Position = getMachineOpValue(MI, MI.getOperand(OpNo-1), Fixups, STI);
  unsigned Size = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);

  return Position + Size - 1;
}

template <unsigned Bits, int Offset>
unsigned
M6502MCCodeEmitter::getUImmWithOffsetEncoding(const MCInst &MI, unsigned OpNo,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  unsigned Value = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
  Value -= Offset;
  return Value;
}

unsigned
M6502MCCodeEmitter::getSimm19Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 2'.
    unsigned Res = getMachineOpValue(MI, MO, Fixups, STI);
    assert((Res & 3) == 0);
    return Res >> 2;
  }

  assert(MO.isExpr() &&
         "getSimm19Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  M6502::Fixups FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_PC19_S2
                                            : M6502::fixup_M6502_PC19_S2;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
M6502MCCodeEmitter::getSimm18Lsl3Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm()) {
    // The immediate is encoded as 'immediate << 3'.
    unsigned Res = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
    assert((Res & 7) == 0);
    return Res >> 3;
  }

  assert(MO.isExpr() &&
         "getSimm18Lsl2Encoding expects only expressions or an immediate");

  const MCExpr *Expr = MO.getExpr();
  M6502::Fixups FixupKind = isMicroM6502(STI) ? M6502::fixup_MICROM6502_PC18_S3
                                            : M6502::fixup_M6502_PC18_S3;
  Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
  return 0;
}

unsigned
M6502MCCodeEmitter::getUImm3Mod8Encoding(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  return MO.getImm() % 8;
}

unsigned
M6502MCCodeEmitter::getUImm4AndValue(const MCInst &MI, unsigned OpNo,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const {
  assert(MI.getOperand(OpNo).isImm());
  const MCOperand &MO = MI.getOperand(OpNo);
  unsigned Value = MO.getImm();
  switch (Value) {
    case 128:   return 0x0;
    case 1:     return 0x1;
    case 2:     return 0x2;
    case 3:     return 0x3;
    case 4:     return 0x4;
    case 7:     return 0x5;
    case 8:     return 0x6;
    case 15:    return 0x7;
    case 16:    return 0x8;
    case 31:    return 0x9;
    case 32:    return 0xa;
    case 63:    return 0xb;
    case 64:    return 0xc;
    case 255:   return 0xd;
    case 32768: return 0xe;
    case 65535: return 0xf;
  }
  llvm_unreachable("Unexpected value");
}

unsigned
M6502MCCodeEmitter::getRegisterListOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  unsigned res = 0;

  // Register list operand is always first operand of instruction and it is
  // placed before memory operand (register + imm).

  for (unsigned I = OpNo, E = MI.getNumOperands() - 2; I < E; ++I) {
    unsigned Reg = MI.getOperand(I).getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    if (RegNo != 31)
      res++;
    else
      res |= 0x10;
  }
  return res;
}

unsigned
M6502MCCodeEmitter::getRegisterListOpValue16(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  return (MI.getNumOperands() - 4);
}

unsigned
M6502MCCodeEmitter::getRegisterPairOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  return getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI);
}

unsigned
M6502MCCodeEmitter::getMovePRegPairOpValue(const MCInst &MI, unsigned OpNo,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  unsigned res = 0;

  if (MI.getOperand(0).getReg() == M6502::A1 &&
      MI.getOperand(1).getReg() == M6502::A2)
    res = 0;
  else if (MI.getOperand(0).getReg() == M6502::A1 &&
           MI.getOperand(1).getReg() == M6502::A3)
    res = 1;
  else if (MI.getOperand(0).getReg() == M6502::A2 &&
           MI.getOperand(1).getReg() == M6502::A3)
    res = 2;
  else if (MI.getOperand(0).getReg() == M6502::A0 &&
           MI.getOperand(1).getReg() == M6502::S5)
    res = 3;
  else if (MI.getOperand(0).getReg() == M6502::A0 &&
           MI.getOperand(1).getReg() == M6502::S6)
    res = 4;
  else if (MI.getOperand(0).getReg() == M6502::A0 &&
           MI.getOperand(1).getReg() == M6502::A1)
    res = 5;
  else if (MI.getOperand(0).getReg() == M6502::A0 &&
           MI.getOperand(1).getReg() == M6502::A2)
    res = 6;
  else if (MI.getOperand(0).getReg() == M6502::A0 &&
           MI.getOperand(1).getReg() == M6502::A3)
    res = 7;

  return res;
}

unsigned
M6502MCCodeEmitter::getMovePRegSingleOpValue(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  assert(((OpNo == 2) || (OpNo == 3)) &&
         "Unexpected OpNo for movep operand encoding!");

  MCOperand Op = MI.getOperand(OpNo);
  assert(Op.isReg() && "Operand of movep is not a register!");
  switch (Op.getReg()) {
  default:
    llvm_unreachable("Unknown register for movep!");
  case M6502::ZERO:  return 0;
  case M6502::S1:    return 1;
  case M6502::V0:    return 2;
  case M6502::V1:    return 3;
  case M6502::S0:    return 4;
  case M6502::S2:    return 5;
  case M6502::S3:    return 6;
  case M6502::S4:    return 7;
  }
}

unsigned
M6502MCCodeEmitter::getSimm23Lsl2Encoding(const MCInst &MI, unsigned OpNo,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  assert(MO.isImm() && "getSimm23Lsl2Encoding expects only an immediate");
  // The immediate is encoded as 'immediate >> 2'.
  unsigned Res = static_cast<unsigned>(MO.getImm());
  assert((Res & 3) == 0);
  return Res >> 2;
}

#include "M6502GenMCCodeEmitter.inc"
