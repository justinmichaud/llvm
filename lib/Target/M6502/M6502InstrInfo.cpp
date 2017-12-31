//===- M6502InstrInfo.cpp - M6502 Instruction Information -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M6502 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "M6502InstrInfo.h"
#include "MCTargetDesc/M6502BaseInfo.h"
#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "M6502Subtarget.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOpcodes.h"
#include "llvm/Target/TargetSubtargetInfo.h"
#include <cassert>

using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "M6502GenInstrInfo.inc"

// Pin the vtable to this file.
void M6502InstrInfo::anchor() {}

M6502InstrInfo::M6502InstrInfo(const M6502Subtarget &STI, unsigned UncondBr)
    : M6502GenInstrInfo(M6502::ADJCALLSTACKDOWN, M6502::ADJCALLSTACKUP),
      Subtarget(STI), UncondBrOpc(UncondBr) {}

const M6502InstrInfo *M6502InstrInfo::create(M6502Subtarget &STI) {
  if (STI.inM650216Mode())
    return createM650216InstrInfo(STI);

  return createM6502SEInstrInfo(STI);
}

bool M6502InstrInfo::isZeroImm(const MachineOperand &op) const {
  return op.isImm() && op.getImm() == 0;
}

/// insertNoop - If data hazard condition is found insert the target nop
/// instruction.
// FIXME: This appears to be dead code.
void M6502InstrInfo::
insertNoop(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI) const
{
  DebugLoc DL;
  BuildMI(MBB, MI, DL, get(M6502::NOP));
}

MachineMemOperand *
M6502InstrInfo::GetMemOperand(MachineBasicBlock &MBB, int FI,
                             MachineMemOperand::Flags Flags) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned Align = MFI.getObjectAlignment(FI);

  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MF, FI),
                                 Flags, MFI.getObjectSize(FI), Align);
}

//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//

void M6502InstrInfo::AnalyzeCondBr(const MachineInstr *Inst, unsigned Opc,
                                  MachineBasicBlock *&BB,
                                  SmallVectorImpl<MachineOperand> &Cond) const {
  assert(getAnalyzableBrOpc(Opc) && "Not an analyzable branch");
  int NumOp = Inst->getNumExplicitOperands();

  // for both int and fp branches, the last explicit operand is the
  // MBB.
  BB = Inst->getOperand(NumOp-1).getMBB();
  Cond.push_back(MachineOperand::CreateImm(Opc));

  for (int i = 0; i < NumOp-1; i++)
    Cond.push_back(Inst->getOperand(i));
}

bool M6502InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                  MachineBasicBlock *&TBB,
                                  MachineBasicBlock *&FBB,
                                  SmallVectorImpl<MachineOperand> &Cond,
                                  bool AllowModify) const {
  SmallVector<MachineInstr*, 2> BranchInstrs;
  BranchType BT = analyzeBranch(MBB, TBB, FBB, Cond, AllowModify, BranchInstrs);

  return (BT == BT_None) || (BT == BT_Indirect);
}

void M6502InstrInfo::BuildCondBr(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                const DebugLoc &DL,
                                ArrayRef<MachineOperand> Cond) const {
  unsigned Opc = Cond[0].getImm();
  const MCInstrDesc &MCID = get(Opc);
  MachineInstrBuilder MIB = BuildMI(&MBB, DL, MCID);

  for (unsigned i = 1; i < Cond.size(); ++i) {
    assert((Cond[i].isImm() || Cond[i].isReg()) &&
           "Cannot copy operand for conditional branch!");
    MIB.add(Cond[i]);
  }
  MIB.addMBB(TBB);
}

unsigned M6502InstrInfo::insertBranch(MachineBasicBlock &MBB,
                                     MachineBasicBlock *TBB,
                                     MachineBasicBlock *FBB,
                                     ArrayRef<MachineOperand> Cond,
                                     const DebugLoc &DL,
                                     int *BytesAdded) const {
  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert(!BytesAdded && "code size not handled");

  // # of condition operands:
  //  Unconditional branches: 0
  //  Floating point branches: 1 (opc)
  //  Int BranchZero: 2 (opc, reg)
  //  Int Branch: 3 (opc, reg0, reg1)
  assert((Cond.size() <= 3) &&
         "# of M6502 branch conditions must be <= 3!");

  // Two-way Conditional branch.
  if (FBB) {
    BuildCondBr(MBB, TBB, DL, Cond);
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(FBB);
    return 2;
  }

  // One way branch.
  // Unconditional branch.
  if (Cond.empty())
    BuildMI(&MBB, DL, get(UncondBrOpc)).addMBB(TBB);
  else // Conditional branch.
    BuildCondBr(MBB, TBB, DL, Cond);
  return 1;
}

unsigned M6502InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                     int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();
  unsigned removed = 0;

  // Up to 2 branches are removed.
  // Note that indirect branches are not removed.
  while (I != REnd && removed < 2) {
    // Skip past debug instructions.
    if (I->isDebugValue()) {
      ++I;
      continue;
    }
    if (!getAnalyzableBrOpc(I->getOpcode()))
      break;
    // Remove the branch.
    I->eraseFromParent();
    I = MBB.rbegin();
    ++removed;
  }

  return removed;
}

/// reverseBranchCondition - Return the inverse opcode of the
/// specified Branch instruction.
bool M6502InstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert( (Cond.size() && Cond.size() <= 3) &&
          "Invalid M6502 branch condition!");
  Cond[0].setImm(getOppositeBranchOpc(Cond[0].getImm()));
  return false;
}

M6502InstrInfo::BranchType M6502InstrInfo::analyzeBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
    SmallVectorImpl<MachineOperand> &Cond, bool AllowModify,
    SmallVectorImpl<MachineInstr *> &BranchInstrs) const {
  MachineBasicBlock::reverse_iterator I = MBB.rbegin(), REnd = MBB.rend();

  // Skip all the debug instructions.
  while (I != REnd && I->isDebugValue())
    ++I;

  if (I == REnd || !isUnpredicatedTerminator(*I)) {
    // This block ends with no branches (it just falls through to its succ).
    // Leave TBB/FBB null.
    TBB = FBB = nullptr;
    return BT_NoBranch;
  }

  MachineInstr *LastInst = &*I;
  unsigned LastOpc = LastInst->getOpcode();
  BranchInstrs.push_back(LastInst);

  // Not an analyzable branch (e.g., indirect jump).
  if (!getAnalyzableBrOpc(LastOpc))
    return LastInst->isIndirectBranch() ? BT_Indirect : BT_None;

  // Get the second to last instruction in the block.
  unsigned SecondLastOpc = 0;
  MachineInstr *SecondLastInst = nullptr;

  // Skip past any debug instruction to see if the second last actual
  // is a branch.
  ++I;
  while (I != REnd && I->isDebugValue())
    ++I;

  if (I != REnd) {
    SecondLastInst = &*I;
    SecondLastOpc = getAnalyzableBrOpc(SecondLastInst->getOpcode());

    // Not an analyzable branch (must be an indirect jump).
    if (isUnpredicatedTerminator(*SecondLastInst) && !SecondLastOpc)
      return BT_None;
  }

  // If there is only one terminator instruction, process it.
  if (!SecondLastOpc) {
    // Unconditional branch.
    if (LastInst->isUnconditionalBranch()) {
      TBB = LastInst->getOperand(0).getMBB();
      return BT_Uncond;
    }

    // Conditional branch
    AnalyzeCondBr(LastInst, LastOpc, TBB, Cond);
    return BT_Cond;
  }

  // If we reached here, there are two branches.
  // If there are three terminators, we don't know what sort of block this is.
  if (++I != REnd && isUnpredicatedTerminator(*I))
    return BT_None;

  BranchInstrs.insert(BranchInstrs.begin(), SecondLastInst);

  // If second to last instruction is an unconditional branch,
  // analyze it and remove the last instruction.
  if (SecondLastInst->isUnconditionalBranch()) {
    // Return if the last instruction cannot be removed.
    if (!AllowModify)
      return BT_None;

    TBB = SecondLastInst->getOperand(0).getMBB();
    LastInst->eraseFromParent();
    BranchInstrs.pop_back();
    return BT_Uncond;
  }

  // Conditional branch followed by an unconditional branch.
  // The last one must be unconditional.
  if (!LastInst->isUnconditionalBranch())
    return BT_None;

  AnalyzeCondBr(SecondLastInst, SecondLastOpc, TBB, Cond);
  FBB = LastInst->getOperand(0).getMBB();

  return BT_CondUncond;
}

/// Return the corresponding compact (no delay slot) form of a branch.
unsigned M6502InstrInfo::getEquivalentCompactForm(
    const MachineBasicBlock::iterator I) const {
  unsigned Opcode = I->getOpcode();
  bool canUseShortMicroM6502CTI = false;

  if (Subtarget.inMicroM6502Mode()) {
    switch (Opcode) {
    case M6502::BNE:
    case M6502::BNE_MM:
    case M6502::BEQ:
    case M6502::BEQ_MM:
    // microM6502 has NE,EQ branches that do not have delay slots provided one
    // of the operands is zero.
      if (I->getOperand(1).getReg() == Subtarget.getABI().GetZeroReg())
        canUseShortMicroM6502CTI = true;
      break;
    // For microM6502 the PseudoReturn and PseudoIndirectBranch are always
    // expanded to JR_MM, so they can be replaced with JRC16_MM.
    case M6502::JR:
    case M6502::PseudoReturn:
    case M6502::PseudoIndirectBranch:
    case M6502::TAILCALLREG:
      canUseShortMicroM6502CTI = true;
      break;
    }
  }

  // M6502R6 forbids both operands being the zero register.
  if (Subtarget.hasM650232r6() && (I->getNumOperands() > 1) &&
      (I->getOperand(0).isReg() &&
       (I->getOperand(0).getReg() == M6502::ZERO ||
        I->getOperand(0).getReg() == M6502::ZERO_64)) &&
      (I->getOperand(1).isReg() &&
       (I->getOperand(1).getReg() == M6502::ZERO ||
        I->getOperand(1).getReg() == M6502::ZERO_64)))
    return 0;

  if (Subtarget.hasM650232r6() || canUseShortMicroM6502CTI) {
    switch (Opcode) {
    case M6502::B:
      return M6502::BC;
    case M6502::BAL:
      return M6502::BALC;
    case M6502::BEQ:
    case M6502::BEQ_MM:
      if (canUseShortMicroM6502CTI)
        return M6502::BEQZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BEQC;
    case M6502::BNE:
    case M6502::BNE_MM:
      if (canUseShortMicroM6502CTI)
        return M6502::BNEZC_MM;
      else if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BNEC;
    case M6502::BGE:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BGEC;
    case M6502::BGEU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BGEUC;
    case M6502::BGEZ:
      return M6502::BGEZC;
    case M6502::BGTZ:
      return M6502::BGTZC;
    case M6502::BLEZ:
      return M6502::BLEZC;
    case M6502::BLT:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BLTC;
    case M6502::BLTU:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BLTUC;
    case M6502::BLTZ:
      return M6502::BLTZC;
    case M6502::BEQ64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BEQC64;
    case M6502::BNE64:
      if (I->getOperand(0).getReg() == I->getOperand(1).getReg())
        return 0;
      return M6502::BNEC64;
    case M6502::BGTZ64:
      return M6502::BGTZC64;
    case M6502::BGEZ64:
      return M6502::BGEZC64;
    case M6502::BLTZ64:
      return M6502::BLTZC64;
    case M6502::BLEZ64:
      return M6502::BLEZC64;
    // For M6502R6, the instruction 'jic' can be used for these cases. Some
    // tools will accept 'jrc reg' as an alias for 'jic 0, $reg'.
    case M6502::JR:
    case M6502::PseudoReturn:
    case M6502::PseudoIndirectBranch:
    case M6502::TAILCALLREG:
      if (canUseShortMicroM6502CTI)
        return M6502::JRC16_MM;
      return M6502::JIC;
    case M6502::JALRPseudo:
      return M6502::JIALC;
    case M6502::JR64:
    case M6502::PseudoReturn64:
    case M6502::PseudoIndirectBranch64:
    case M6502::TAILCALLREG64:
      return M6502::JIC64;
    case M6502::JALR64Pseudo:
      return M6502::JIALC64;
    default:
      return 0;
    }
  }

  return 0;
}

/// Predicate for distingushing between control transfer instructions and all
/// other instructions for handling forbidden slots. Consider inline assembly
/// as unsafe as well.
bool M6502InstrInfo::SafeInForbiddenSlot(const MachineInstr &MI) const {
  if (MI.isInlineAsm())
    return false;

  return (MI.getDesc().TSFlags & M6502II::IsCTI) == 0;
}

/// Predicate for distingushing instructions that have forbidden slots.
bool M6502InstrInfo::HasForbiddenSlot(const MachineInstr &MI) const {
  return (MI.getDesc().TSFlags & M6502II::HasForbiddenSlot) != 0;
}

/// Return the number of bytes of code the specified instruction may be.
unsigned M6502InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    return MI.getDesc().getSize();
  case  TargetOpcode::INLINEASM: {       // Inline Asm: Variable size.
    const MachineFunction *MF = MI.getParent()->getParent();
    const char *AsmStr = MI.getOperand(0).getSymbolName();
    return getInlineAsmLength(AsmStr, *MF->getTarget().getMCAsmInfo());
  }
  case M6502::CONSTPOOL_ENTRY:
    // If this machine instr is a constant pool entry, its size is recorded as
    // operand #2.
    return MI.getOperand(2).getImm();
  }
}

MachineInstrBuilder
M6502InstrInfo::genInstrWithNewOpc(unsigned NewOpc,
                                  MachineBasicBlock::iterator I) const {
  MachineInstrBuilder MIB;

  // Certain branches have two forms: e.g beq $1, $zero, dest vs beqz $1, dest
  // Pick the zero form of the branch for readable assembly and for greater
  // branch distance in non-microM6502 mode.
  // Additional M6502R6 does not permit the use of register $zero for compact
  // branches.
  // FIXME: Certain atomic sequences on m650264 generate 32bit references to
  // M6502::ZERO, which is incorrect. This test should be updated to use
  // Subtarget.getABI().GetZeroReg() when those atomic sequences and others
  // are fixed.
  int ZeroOperandPosition = -1;
  bool BranchWithZeroOperand = false;
  if (I->isBranch() && !I->isPseudo()) {
    auto TRI = I->getParent()->getParent()->getSubtarget().getRegisterInfo();
    ZeroOperandPosition = I->findRegisterUseOperandIdx(M6502::ZERO, false, TRI);
    BranchWithZeroOperand = ZeroOperandPosition != -1;
  }

  if (BranchWithZeroOperand) {
    switch (NewOpc) {
    case M6502::BEQC:
      NewOpc = M6502::BEQZC;
      break;
    case M6502::BNEC:
      NewOpc = M6502::BNEZC;
      break;
    case M6502::BGEC:
      NewOpc = M6502::BGEZC;
      break;
    case M6502::BLTC:
      NewOpc = M6502::BLTZC;
      break;
    case M6502::BEQC64:
      NewOpc = M6502::BEQZC64;
      break;
    case M6502::BNEC64:
      NewOpc = M6502::BNEZC64;
      break;
    }
  }

  MIB = BuildMI(*I->getParent(), I, I->getDebugLoc(), get(NewOpc));

  // For M6502R6 JI*C requires an immediate 0 as an operand, JIALC(64) an
  // immediate 0 as an operand and requires the removal of it's %RA<imp-def>
  // implicit operand as copying the implicit operations of the instructio we're
  // looking at will give us the correct flags.
  if (NewOpc == M6502::JIC || NewOpc == M6502::JIALC || NewOpc == M6502::JIC64 ||
      NewOpc == M6502::JIALC64) {

    if (NewOpc == M6502::JIALC || NewOpc == M6502::JIALC64)
      MIB->RemoveOperand(0);

    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      MIB.add(I->getOperand(J));
    }

    MIB.addImm(0);

  } else {
    for (unsigned J = 0, E = I->getDesc().getNumOperands(); J < E; ++J) {
      if (BranchWithZeroOperand && (unsigned)ZeroOperandPosition == J)
        continue;

      MIB.add(I->getOperand(J));
    }
  }

  MIB.copyImplicitOps(*I);

  MIB.setMemRefs(I->memoperands_begin(), I->memoperands_end());
  return MIB;
}

bool M6502InstrInfo::findCommutedOpIndices(MachineInstr &MI, unsigned &SrcOpIdx1,
                                          unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() &&
         "TargetInstrInfo::findCommutedOpIndices() can't handle bundles");

  const MCInstrDesc &MCID = MI.getDesc();
  if (!MCID.isCommutable())
    return false;

  switch (MI.getOpcode()) {
  case M6502::DPADD_U_H:
  case M6502::DPADD_U_W:
  case M6502::DPADD_U_D:
  case M6502::DPADD_S_H:
  case M6502::DPADD_S_W:
  case M6502::DPADD_S_D:
    // The first operand is both input and output, so it should not commute
    if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, 2, 3))
      return false;

    if (!MI.getOperand(SrcOpIdx1).isReg() || !MI.getOperand(SrcOpIdx2).isReg())
      return false;
    return true;
  }
  return TargetInstrInfo::findCommutedOpIndices(MI, SrcOpIdx1, SrcOpIdx2);
}

// ins, ext, dext*, dins have the following constraints:
// 0 <= pos      <  X
// 0 <  size     <= X
// 0 <  pos+size <= x
//
// dinsm and dinsm have the following contraints:
// 0 <= pos      <  X
// 0 <= size     <= X
// 0 <  pos+size <= x

static bool verifyInsExtInstruction(const MachineInstr &MI, StringRef &ErrInfo,
                                    const int64_t PosLow, const int64_t PosHigh,
                                    const int64_t SizeLow,
                                    const int64_t SizeHigh,
                                    const int64_t BothLow,
                                    const int64_t BothHigh) {
  MachineOperand MOPos = MI.getOperand(2);
  if (!MOPos.isImm()) {
    ErrInfo = "Position is not an immediate!";
    return false;
  }
  int64_t Pos = MOPos.getImm();
  if (!((PosLow <= Pos) && (Pos < PosHigh))) {
    ErrInfo = "Position operand is out of range!";
    return false;
  }

  MachineOperand MOSize = MI.getOperand(3);
  if (!MOSize.isImm()) {
    ErrInfo = "Size operand is not an immediate!";
    return false;
  }
  int64_t Size = MOSize.getImm();
  if (!((SizeLow < Size) && (Size <= SizeHigh))) {
    ErrInfo = "Size operand is out of range!";
    return false;
  }

  if (!((BothLow < (Pos + Size)) && ((Pos + Size) <= BothHigh))) {
    ErrInfo = "Position + Size is out of range!";
    return false;
  }

  return true;
}

//  Perform target specific instruction verification.
bool M6502InstrInfo::verifyInstruction(const MachineInstr &MI,
                                      StringRef &ErrInfo) const {
  // Verify that ins and ext instructions are well formed.
  switch (MI.getOpcode()) {
    case M6502::EXT:
    case M6502::EXT_MM:
    case M6502::INS:
    case M6502::INS_MM:
    case M6502::DINS:
    case M6502::DINS_MM64R6:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 32);
    case M6502::DINSM:
    case M6502::DINSM_MM64R6:
      // The ISA spec has a subtle difference here in that it says:
      //  2 <= size <= 64 for 'dinsm', so we change the bounds so that it
      // is in line with the rest of instructions.
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 1, 64, 32, 64);
    case M6502::DINSU:
    case M6502::DINSU_MM64R6:
      // The ISA spec has a subtle difference here in that it says:
      //  2 <= size <= 64 for 'dinsm', so we change the bounds so that it
      // is in line with the rest of instructions.
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 1, 32, 32, 64);
    case M6502::DEXT:
    case M6502::DEXT_MM64R6:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 0, 32, 0, 63);
    case M6502::DEXTM:
    case M6502::DEXTM_MM64R6:
      return verifyInsExtInstruction(MI, ErrInfo, 0, 32, 32, 64, 32, 64);
    case M6502::DEXTU:
    case M6502::DEXTU_MM64R6:
      return verifyInsExtInstruction(MI, ErrInfo, 32, 64, 0, 32, 32, 64);
    default:
      return true;
  }

  return true;
}

std::pair<unsigned, unsigned>
M6502InstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char*>>
M6502InstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
 using namespace M6502II;

 static const std::pair<unsigned, const char*> Flags[] = {
    {MO_GOT,          "m6502-got"},
    {MO_GOT_CALL,     "m6502-got-call"},
    {MO_GPREL,        "m6502-gprel"},
    {MO_ABS_HI,       "m6502-abs-hi"},
    {MO_ABS_LO,       "m6502-abs-lo"},
    {MO_TLSGD,        "m6502-tlsgd"},
    {MO_TLSLDM,       "m6502-tlsldm"},
    {MO_DTPREL_HI,    "m6502-dtprel-hi"},
    {MO_DTPREL_LO,    "m6502-dtprel-lo"},
    {MO_GOTTPREL,     "m6502-gottprel"},
    {MO_TPREL_HI,     "m6502-tprel-hi"},
    {MO_TPREL_LO,     "m6502-tprel-lo"},
    {MO_GPOFF_HI,     "m6502-gpoff-hi"},
    {MO_GPOFF_LO,     "m6502-gpoff-lo"},
    {MO_GOT_DISP,     "m6502-got-disp"},
    {MO_GOT_PAGE,     "m6502-got-page"},
    {MO_GOT_OFST,     "m6502-got-ofst"},
    {MO_HIGHER,       "m6502-higher"},
    {MO_HIGHEST,      "m6502-highest"},
    {MO_GOT_HI16,     "m6502-got-hi16"},
    {MO_GOT_LO16,     "m6502-got-lo16"},
    {MO_CALL_HI16,    "m6502-call-hi16"},
    {MO_CALL_LO16,    "m6502-call-lo16"}
  };
  return makeArrayRef(Flags);
}
