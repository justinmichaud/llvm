//===- M6502LongBranch.cpp - Emit long branches ----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass expands a branch or jump instruction into a long branch if its
// offset is too large to fit into its immediate field.
//
// FIXME: Fix pc-region jump instructions which cross 256MB segment boundaries.
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/M6502ABIInfo.h"
#include "MCTargetDesc/M6502BaseInfo.h"
#include "MCTargetDesc/M6502MCNaCl.h"
#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "M6502.h"
#include "M6502InstrInfo.h"
#include "M6502MachineFunction.h"
#include "M6502Subtarget.h"
#include "M6502TargetMachine.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetSubtargetInfo.h"
#include <cassert>
#include <cstdint>
#include <iterator>

using namespace llvm;

#define DEBUG_TYPE "m6502-long-branch"

STATISTIC(LongBranches, "Number of long branches.");

static cl::opt<bool> SkipLongBranch(
  "skip-m6502-long-branch",
  cl::init(false),
  cl::desc("M6502: Skip long branch pass."),
  cl::Hidden);

static cl::opt<bool> ForceLongBranch(
  "force-m6502-long-branch",
  cl::init(false),
  cl::desc("M6502: Expand all branches to long format."),
  cl::Hidden);

namespace {

  using Iter = MachineBasicBlock::iterator;
  using ReverseIter = MachineBasicBlock::reverse_iterator;

  struct MBBInfo {
    uint64_t Size = 0;
    uint64_t Address;
    bool HasLongBranch = false;
    MachineInstr *Br = nullptr;

    MBBInfo() = default;
  };

  class M6502LongBranch : public MachineFunctionPass {
  public:
    static char ID;

    M6502LongBranch()
        : MachineFunctionPass(ID), ABI(M6502ABIInfo::Unknown()) {}

    StringRef getPassName() const override { return "M6502 Long Branch"; }

    bool runOnMachineFunction(MachineFunction &F) override;

    MachineFunctionProperties getRequiredProperties() const override {
      return MachineFunctionProperties().set(
          MachineFunctionProperties::Property::NoVRegs);
    }

  private:
    void splitMBB(MachineBasicBlock *MBB);
    void initMBBInfo();
    int64_t computeOffset(const MachineInstr *Br);
    void replaceBranch(MachineBasicBlock &MBB, Iter Br, const DebugLoc &DL,
                       MachineBasicBlock *MBBOpnd);
    void expandToLongBranch(MBBInfo &Info);

    MachineFunction *MF;
    SmallVector<MBBInfo, 16> MBBInfos;
    bool IsPIC;
    M6502ABIInfo ABI;
    unsigned LongBranchSeqSize;
  };

} // end anonymous namespace

char M6502LongBranch::ID = 0;

/// Iterate over list of Br's operands and search for a MachineBasicBlock
/// operand.
static MachineBasicBlock *getTargetMBB(const MachineInstr &Br) {
  for (unsigned I = 0, E = Br.getDesc().getNumOperands(); I < E; ++I) {
    const MachineOperand &MO = Br.getOperand(I);

    if (MO.isMBB())
      return MO.getMBB();
  }

  llvm_unreachable("This instruction does not have an MBB operand.");
}

// Traverse the list of instructions backwards until a non-debug instruction is
// found or it reaches E.
static ReverseIter getNonDebugInstr(ReverseIter B, const ReverseIter &E) {
  for (; B != E; ++B)
    if (!B->isDebugValue())
      return B;

  return E;
}

// Split MBB if it has two direct jumps/branches.
void M6502LongBranch::splitMBB(MachineBasicBlock *MBB) {
  ReverseIter End = MBB->rend();
  ReverseIter LastBr = getNonDebugInstr(MBB->rbegin(), End);

  // Return if MBB has no branch instructions.
  if ((LastBr == End) ||
      (!LastBr->isConditionalBranch() && !LastBr->isUnconditionalBranch()))
    return;

  ReverseIter FirstBr = getNonDebugInstr(std::next(LastBr), End);

  // MBB has only one branch instruction if FirstBr is not a branch
  // instruction.
  if ((FirstBr == End) ||
      (!FirstBr->isConditionalBranch() && !FirstBr->isUnconditionalBranch()))
    return;

  assert(!FirstBr->isIndirectBranch() && "Unexpected indirect branch found.");

  // Create a new MBB. Move instructions in MBB to the newly created MBB.
  MachineBasicBlock *NewMBB =
    MF->CreateMachineBasicBlock(MBB->getBasicBlock());

  // Insert NewMBB and fix control flow.
  MachineBasicBlock *Tgt = getTargetMBB(*FirstBr);
  NewMBB->transferSuccessors(MBB);
  NewMBB->removeSuccessor(Tgt, true);
  MBB->addSuccessor(NewMBB);
  MBB->addSuccessor(Tgt);
  MF->insert(std::next(MachineFunction::iterator(MBB)), NewMBB);

  NewMBB->splice(NewMBB->end(), MBB, LastBr.getReverse(), MBB->end());
}

// Fill MBBInfos.
void M6502LongBranch::initMBBInfo() {
  // Split the MBBs if they have two branches. Each basic block should have at
  // most one branch after this loop is executed.
  for (auto &MBB : *MF)
    splitMBB(&MBB);

  MF->RenumberBlocks();
  MBBInfos.clear();
  MBBInfos.resize(MF->size());

  const M6502InstrInfo *TII =
      static_cast<const M6502InstrInfo *>(MF->getSubtarget().getInstrInfo());
  for (unsigned I = 0, E = MBBInfos.size(); I < E; ++I) {
    MachineBasicBlock *MBB = MF->getBlockNumbered(I);

    // Compute size of MBB.
    for (MachineBasicBlock::instr_iterator MI = MBB->instr_begin();
         MI != MBB->instr_end(); ++MI)
      MBBInfos[I].Size += TII->getInstSizeInBytes(*MI);

    // Search for MBB's branch instruction.
    ReverseIter End = MBB->rend();
    ReverseIter Br = getNonDebugInstr(MBB->rbegin(), End);

    if ((Br != End) && !Br->isIndirectBranch() &&
        (Br->isConditionalBranch() || (Br->isUnconditionalBranch() && IsPIC)))
      MBBInfos[I].Br = &*Br;
  }
}

// Compute offset of branch in number of bytes.
int64_t M6502LongBranch::computeOffset(const MachineInstr *Br) {
  int64_t Offset = 0;
  int ThisMBB = Br->getParent()->getNumber();
  int TargetMBB = getTargetMBB(*Br)->getNumber();

  // Compute offset of a forward branch.
  if (ThisMBB < TargetMBB) {
    for (int N = ThisMBB + 1; N < TargetMBB; ++N)
      Offset += MBBInfos[N].Size;

    return Offset + 4;
  }

  // Compute offset of a backward branch.
  for (int N = ThisMBB; N >= TargetMBB; --N)
    Offset += MBBInfos[N].Size;

  return -Offset + 4;
}

// Replace Br with a branch which has the opposite condition code and a
// MachineBasicBlock operand MBBOpnd.
void M6502LongBranch::replaceBranch(MachineBasicBlock &MBB, Iter Br,
                                   const DebugLoc &DL,
                                   MachineBasicBlock *MBBOpnd) {
  const M6502InstrInfo *TII = static_cast<const M6502InstrInfo *>(
      MBB.getParent()->getSubtarget().getInstrInfo());
  unsigned NewOpc = TII->getOppositeBranchOpc(Br->getOpcode());
  const MCInstrDesc &NewDesc = TII->get(NewOpc);

  MachineInstrBuilder MIB = BuildMI(MBB, Br, DL, NewDesc);

  for (unsigned I = 0, E = Br->getDesc().getNumOperands(); I < E; ++I) {
    MachineOperand &MO = Br->getOperand(I);

    if (!MO.isReg()) {
      assert(MO.isMBB() && "MBB operand expected.");
      break;
    }

    MIB.addReg(MO.getReg());
  }

  MIB.addMBB(MBBOpnd);

  if (Br->hasDelaySlot()) {
    // Bundle the instruction in the delay slot to the newly created branch
    // and erase the original branch.
    assert(Br->isBundledWithSucc());
    MachineBasicBlock::instr_iterator II = Br.getInstrIterator();
    MIBundleBuilder(&*MIB).append((++II)->removeFromBundle());
  }
  Br->eraseFromParent();
}

// Expand branch instructions to long branches.
// TODO: This function has to be fixed for beqz16 and bnez16, because it
// currently assumes that all branches have 16-bit offsets, and will produce
// wrong code if branches whose allowed offsets are [-128, -126, ..., 126]
// are present.
void M6502LongBranch::expandToLongBranch(MBBInfo &I) {
  MachineBasicBlock::iterator Pos;
  MachineBasicBlock *MBB = I.Br->getParent(), *TgtMBB = getTargetMBB(*I.Br);
  DebugLoc DL = I.Br->getDebugLoc();
  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator FallThroughMBB = ++MachineFunction::iterator(MBB);
  MachineBasicBlock *LongBrMBB = MF->CreateMachineBasicBlock(BB);
  const M6502Subtarget &Subtarget =
      static_cast<const M6502Subtarget &>(MF->getSubtarget());
  const M6502InstrInfo *TII =
      static_cast<const M6502InstrInfo *>(Subtarget.getInstrInfo());

  MF->insert(FallThroughMBB, LongBrMBB);
  MBB->replaceSuccessor(TgtMBB, LongBrMBB);

  if (IsPIC) {
    MachineBasicBlock *BalTgtMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(FallThroughMBB, BalTgtMBB);
    LongBrMBB->addSuccessor(BalTgtMBB);
    BalTgtMBB->addSuccessor(TgtMBB);

    // We must select between the M650232r6/M650264r6 BAL (which is a normal
    // instruction) and the pre-M650232r6/M650264r6 definition (which is an
    // pseudo-instruction wrapping BGEZAL).
    unsigned BalOp = Subtarget.hasM650232r6() ? M6502::BAL : M6502::BAL_BR;

    if (!ABI.IsN64()) {
      // $longbr:
      //  addiu $sp, $sp, -8
      //  sw $ra, 0($sp)
      //  lui $at, %hi($tgt - $baltgt)
      //  bal $baltgt
      //  addiu $at, $at, %lo($tgt - $baltgt)
      // $baltgt:
      //  addu $at, $ra, $at
      //  lw $ra, 0($sp)
      //  jr $at
      //  addiu $sp, $sp, 8
      // $fallthrough:
      //

      Pos = LongBrMBB->begin();

      BuildMI(*LongBrMBB, Pos, DL, TII->get(M6502::ADDiu), M6502::SP)
        .addReg(M6502::SP).addImm(-8);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(M6502::SW)).addReg(M6502::RA)
        .addReg(M6502::SP).addImm(0);

      // LUi and ADDiu instructions create 32-bit offset of the target basic
      // block from the target of BAL instruction.  We cannot use immediate
      // value for this offset because it cannot be determined accurately when
      // the program has inline assembly statements.  We therefore use the
      // relocation expressions %hi($tgt-$baltgt) and %lo($tgt-$baltgt) which
      // are resolved during the fixup, so the values will always be correct.
      //
      // Since we cannot create %hi($tgt-$baltgt) and %lo($tgt-$baltgt)
      // expressions at this point (it is possible only at the MC layer),
      // we replace LUi and ADDiu with pseudo instructions
      // LONG_BRANCH_LUi and LONG_BRANCH_ADDiu, and add both basic
      // blocks as operands to these instructions.  When lowering these pseudo
      // instructions to LUi and ADDiu in the MC layer, we will create
      // %hi($tgt-$baltgt) and %lo($tgt-$baltgt) expressions and add them as
      // operands to lowered instructions.

      BuildMI(*LongBrMBB, Pos, DL, TII->get(M6502::LONG_BRANCH_LUi), M6502::AT)
        .addMBB(TgtMBB).addMBB(BalTgtMBB);
      MIBundleBuilder(*LongBrMBB, Pos)
          .append(BuildMI(*MF, DL, TII->get(BalOp)).addMBB(BalTgtMBB))
          .append(BuildMI(*MF, DL, TII->get(M6502::LONG_BRANCH_ADDiu), M6502::AT)
                      .addReg(M6502::AT)
                      .addMBB(TgtMBB)
                      .addMBB(BalTgtMBB));

      Pos = BalTgtMBB->begin();

      BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::ADDu), M6502::AT)
        .addReg(M6502::RA).addReg(M6502::AT);
      BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::LW), M6502::RA)
        .addReg(M6502::SP).addImm(0);

      // In NaCl, modifying the sp is not allowed in branch delay slot.
      if (Subtarget.isTargetNaCl())
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::ADDiu), M6502::SP)
          .addReg(M6502::SP).addImm(8);

      if (Subtarget.hasM650232r6())
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::JALR))
          .addReg(M6502::ZERO).addReg(M6502::AT);
      else
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::JR)).addReg(M6502::AT);

      if (Subtarget.isTargetNaCl()) {
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::NOP));
        // Bundle-align the target of indirect branch JR.
        TgtMBB->setAlignment(M6502_NACL_BUNDLE_ALIGN);
      } else
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::ADDiu), M6502::SP)
          .addReg(M6502::SP).addImm(8);

      BalTgtMBB->rbegin()->bundleWithPred();
    } else {
      // $longbr:
      //  daddiu $sp, $sp, -16
      //  sd $ra, 0($sp)
      //  daddiu $at, $zero, %hi($tgt - $baltgt)
      //  dsll $at, $at, 16
      //  bal $baltgt
      //  daddiu $at, $at, %lo($tgt - $baltgt)
      // $baltgt:
      //  daddu $at, $ra, $at
      //  ld $ra, 0($sp)
      //  jr64 $at
      //  daddiu $sp, $sp, 16
      // $fallthrough:
      //

      // We assume the branch is within-function, and that offset is within
      // +/- 2GB.  High 32 bits will therefore always be zero.

      // Note that this will work even if the offset is negative, because
      // of the +1 modification that's added in that case.  For example, if the
      // offset is -1MB (0xFFFFFFFFFFF00000), the computation for %higher is
      //
      // 0xFFFFFFFFFFF00000 + 0x80008000 = 0x000000007FF08000
      //
      // and the bits [47:32] are zero.  For %highest
      //
      // 0xFFFFFFFFFFF00000 + 0x800080008000 = 0x000080007FF08000
      //
      // and the bits [63:48] are zero.

      Pos = LongBrMBB->begin();

      BuildMI(*LongBrMBB, Pos, DL, TII->get(M6502::DADDiu), M6502::SP_64)
        .addReg(M6502::SP_64).addImm(-16);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(M6502::SD)).addReg(M6502::RA_64)
        .addReg(M6502::SP_64).addImm(0);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(M6502::LONG_BRANCH_DADDiu),
              M6502::AT_64).addReg(M6502::ZERO_64)
                          .addMBB(TgtMBB, M6502II::MO_ABS_HI).addMBB(BalTgtMBB);
      BuildMI(*LongBrMBB, Pos, DL, TII->get(M6502::DSLL), M6502::AT_64)
        .addReg(M6502::AT_64).addImm(16);

      MIBundleBuilder(*LongBrMBB, Pos)
          .append(BuildMI(*MF, DL, TII->get(BalOp)).addMBB(BalTgtMBB))
          .append(
              BuildMI(*MF, DL, TII->get(M6502::LONG_BRANCH_DADDiu), M6502::AT_64)
                  .addReg(M6502::AT_64)
                  .addMBB(TgtMBB, M6502II::MO_ABS_LO)
                  .addMBB(BalTgtMBB));

      Pos = BalTgtMBB->begin();

      BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::DADDu), M6502::AT_64)
        .addReg(M6502::RA_64).addReg(M6502::AT_64);
      BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::LD), M6502::RA_64)
        .addReg(M6502::SP_64).addImm(0);

      if (Subtarget.hasM650264r6())
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::JALR64))
          .addReg(M6502::ZERO_64).addReg(M6502::AT_64);
      else
        BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::JR64)).addReg(M6502::AT_64);

      BuildMI(*BalTgtMBB, Pos, DL, TII->get(M6502::DADDiu), M6502::SP_64)
        .addReg(M6502::SP_64).addImm(16);
      BalTgtMBB->rbegin()->bundleWithPred();
    }

    assert(LongBrMBB->size() + BalTgtMBB->size() == LongBranchSeqSize);
  } else {
    // $longbr:
    //  j $tgt
    //  nop
    // $fallthrough:
    //
    Pos = LongBrMBB->begin();
    LongBrMBB->addSuccessor(TgtMBB);
    MIBundleBuilder(*LongBrMBB, Pos)
      .append(BuildMI(*MF, DL, TII->get(M6502::J)).addMBB(TgtMBB))
      .append(BuildMI(*MF, DL, TII->get(M6502::NOP)));

    assert(LongBrMBB->size() == LongBranchSeqSize);
  }

  if (I.Br->isUnconditionalBranch()) {
    // Change branch destination.
    assert(I.Br->getDesc().getNumOperands() == 1);
    I.Br->RemoveOperand(0);
    I.Br->addOperand(MachineOperand::CreateMBB(LongBrMBB));
  } else
    // Change branch destination and reverse condition.
    replaceBranch(*MBB, I.Br, DL, &*FallThroughMBB);
}

static void emitGPDisp(MachineFunction &F, const M6502InstrInfo *TII) {
  MachineBasicBlock &MBB = F.front();
  MachineBasicBlock::iterator I = MBB.begin();
  DebugLoc DL = MBB.findDebugLoc(MBB.begin());
  BuildMI(MBB, I, DL, TII->get(M6502::LUi), M6502::V0)
    .addExternalSymbol("_gp_disp", M6502II::MO_ABS_HI);
  BuildMI(MBB, I, DL, TII->get(M6502::ADDiu), M6502::V0)
    .addReg(M6502::V0).addExternalSymbol("_gp_disp", M6502II::MO_ABS_LO);
  MBB.removeLiveIn(M6502::V0);
}

bool M6502LongBranch::runOnMachineFunction(MachineFunction &F) {
  const M6502Subtarget &STI =
      static_cast<const M6502Subtarget &>(F.getSubtarget());
  const M6502InstrInfo *TII =
      static_cast<const M6502InstrInfo *>(STI.getInstrInfo());

  const TargetMachine& TM = F.getTarget();
  IsPIC = TM.isPositionIndependent();
  ABI = static_cast<const M6502TargetMachine &>(TM).getABI();

  LongBranchSeqSize =
      !IsPIC ? 2 : (ABI.IsN64() ? 10 : (!STI.isTargetNaCl() ? 9 : 10));

  if (STI.inM650216Mode() || !STI.enableLongBranchPass())
    return false;
  if (IsPIC && static_cast<const M6502TargetMachine &>(TM).getABI().IsO32() &&
      F.getInfo<M6502FunctionInfo>()->globalBaseRegSet())
    emitGPDisp(F, TII);

  if (SkipLongBranch)
    return true;

  MF = &F;
  initMBBInfo();

  SmallVectorImpl<MBBInfo>::iterator I, E = MBBInfos.end();
  bool EverMadeChange = false, MadeChange = true;

  while (MadeChange) {
    MadeChange = false;

    for (I = MBBInfos.begin(); I != E; ++I) {
      // Skip if this MBB doesn't have a branch or the branch has already been
      // converted to a long branch.
      if (!I->Br || I->HasLongBranch)
        continue;

      int ShVal = STI.inMicroM6502Mode() ? 2 : 4;
      int64_t Offset = computeOffset(I->Br) / ShVal;

      if (STI.isTargetNaCl()) {
        // The offset calculation does not include sandboxing instructions
        // that will be added later in the MC layer.  Since at this point we
        // don't know the exact amount of code that "sandboxing" will add, we
        // conservatively estimate that code will not grow more than 100%.
        Offset *= 2;
      }

      // Check if offset fits into 16-bit immediate field of branches.
      if (!ForceLongBranch && isInt<16>(Offset))
        continue;

      I->HasLongBranch = true;
      I->Size += LongBranchSeqSize * 4;
      ++LongBranches;
      EverMadeChange = MadeChange = true;
    }
  }

  if (!EverMadeChange)
    return true;

  // Compute basic block addresses.
  if (IsPIC) {
    uint64_t Address = 0;

    for (I = MBBInfos.begin(); I != E; Address += I->Size, ++I)
      I->Address = Address;
  }

  // Do the expansion.
  for (I = MBBInfos.begin(); I != E; ++I)
    if (I->HasLongBranch)
      expandToLongBranch(*I);

  MF->RenumberBlocks();

  return true;
}

/// createM6502LongBranchPass - Returns a pass that converts branches to long
/// branches.
FunctionPass *llvm::createM6502LongBranchPass() { return new M6502LongBranch(); }
