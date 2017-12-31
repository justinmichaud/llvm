//===-- M6502SERegisterInfo.cpp - M650232/64 Register Information -== -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M650232/64 implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#include "M6502SERegisterInfo.h"
#include "M6502.h"
#include "M6502MachineFunction.h"
#include "M6502SEInstrInfo.h"
#include "M6502Subtarget.h"
#include "M6502TargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DebugInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

#define DEBUG_TYPE "m6502-reg-info"

M6502SERegisterInfo::M6502SERegisterInfo() : M6502RegisterInfo() {}

bool M6502SERegisterInfo::
requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool M6502SERegisterInfo::
requiresFrameIndexScavenging(const MachineFunction &MF) const {
  return true;
}

const TargetRegisterClass *
M6502SERegisterInfo::intRegClass(unsigned Size) const {
  if (Size == 4)
    return &M6502::GPR32RegClass;

  assert(Size == 8);
  return &M6502::GPR64RegClass;
}

/// Get the size of the offset supported by the given load/store/inline asm.
/// The result includes the effects of any scale factors applied to the
/// instruction immediate.
static inline unsigned getLoadStoreOffsetSizeInBits(const unsigned Opcode,
                                                    MachineOperand MO) {
  switch (Opcode) {
  case M6502::LD_B:
  case M6502::ST_B:
    return 10;
  case M6502::LD_H:
  case M6502::ST_H:
    return 10 + 1 /* scale factor */;
  case M6502::LD_W:
  case M6502::ST_W:
    return 10 + 2 /* scale factor */;
  case M6502::LD_D:
  case M6502::ST_D:
    return 10 + 3 /* scale factor */;
  case M6502::LL:
  case M6502::LL64:
  case M6502::LLD:
  case M6502::LLE:
  case M6502::SC:
  case M6502::SC64:
  case M6502::SCD:
  case M6502::SCE:
    return 16;
  case M6502::LLE_MM:
  case M6502::LLE_MMR6:
  case M6502::LL_MM:
  case M6502::SCE_MM:
  case M6502::SCE_MMR6:
  case M6502::SC_MM:
    return 12;
  case M6502::LL64_R6:
  case M6502::LL_R6:
  case M6502::LLD_R6:
  case M6502::SC64_R6:
  case M6502::SCD_R6:
  case M6502::SC_R6:
    return 9;
  case M6502::INLINEASM: {
    unsigned ConstraintID = InlineAsm::getMemoryConstraintID(MO.getImm());
    switch (ConstraintID) {
    case InlineAsm::Constraint_ZC: {
      const M6502Subtarget &Subtarget = MO.getParent()
                                           ->getParent()
                                           ->getParent()
                                           ->getSubtarget<M6502Subtarget>();
      if (Subtarget.inMicroM6502Mode())
        return 12;

      if (Subtarget.hasM650232r6())
        return 9;

      return 16;
    }
    default:
      return 16;
    }
  }
  default:
    return 16;
  }
}

/// Get the scale factor applied to the immediate in the given load/store.
static inline unsigned getLoadStoreOffsetAlign(const unsigned Opcode) {
  switch (Opcode) {
  case M6502::LD_H:
  case M6502::ST_H:
    return 2;
  case M6502::LD_W:
  case M6502::ST_W:
    return 4;
  case M6502::LD_D:
  case M6502::ST_D:
    return 8;
  default:
    return 1;
  }
}

void M6502SERegisterInfo::eliminateFI(MachineBasicBlock::iterator II,
                                     unsigned OpNo, int FrameIndex,
                                     uint64_t StackSize,
                                     int64_t SPOffset) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  M6502FunctionInfo *M6502FI = MF.getInfo<M6502FunctionInfo>();

  M6502ABIInfo ABI =
      static_cast<const M6502TargetMachine &>(MF.getTarget()).getABI();
  const M6502RegisterInfo *RegInfo =
    static_cast<const M6502RegisterInfo *>(MF.getSubtarget().getRegisterInfo());

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;

  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  bool EhDataRegFI = M6502FI->isEhDataRegFI(FrameIndex);
  bool IsISRRegFI = M6502FI->isISRRegFI(FrameIndex);
  // The following stack frame objects are always referenced relative to $sp:
  //  1. Outgoing arguments.
  //  2. Pointer to dynamically allocated stack space.
  //  3. Locations for callee-saved registers.
  //  4. Locations for eh data registers.
  //  5. Locations for ISR saved Coprocessor 0 registers 12 & 14.
  // Everything else is referenced relative to whatever register
  // getFrameRegister() returns.
  unsigned FrameReg;

  if ((FrameIndex >= MinCSFI && FrameIndex <= MaxCSFI) || EhDataRegFI ||
      IsISRRegFI)
    FrameReg = ABI.GetStackPtr();
  else if (RegInfo->needsStackRealignment(MF)) {
    if (MFI.hasVarSizedObjects() && !MFI.isFixedObjectIndex(FrameIndex))
      FrameReg = ABI.GetBasePtr();
    else if (MFI.isFixedObjectIndex(FrameIndex))
      FrameReg = getFrameRegister(MF);
    else
      FrameReg = ABI.GetStackPtr();
  } else
    FrameReg = getFrameRegister(MF);

  // Calculate final offset.
  // - There is no need to change the offset if the frame object is one of the
  //   following: an outgoing argument, pointer to a dynamically allocated
  //   stack space or a $gp restore location,
  // - If the frame object is any of the following, its offset must be adjusted
  //   by adding the size of the stack:
  //   incoming argument, callee-saved register location or local variable.
  bool IsKill = false;
  int64_t Offset;

  Offset = SPOffset + (int64_t)StackSize;
  Offset += MI.getOperand(OpNo + 1).getImm();

  DEBUG(errs() << "Offset     : " << Offset << "\n" << "<--------->\n");

  if (!MI.isDebugValue()) {
    // Make sure Offset fits within the field available.
    // For MSA instructions, this is a 10-bit signed immediate (scaled by
    // element size), otherwise it is a 16-bit signed immediate.
    unsigned OffsetBitSize =
        getLoadStoreOffsetSizeInBits(MI.getOpcode(), MI.getOperand(OpNo - 1));
    unsigned OffsetAlign = getLoadStoreOffsetAlign(MI.getOpcode());

    if (OffsetBitSize < 16 && isInt<16>(Offset) &&
        (!isIntN(OffsetBitSize, Offset) ||
         OffsetToAlignment(Offset, OffsetAlign) != 0)) {
      // If we have an offset that needs to fit into a signed n-bit immediate
      // (where n < 16) and doesn't, but does fit into 16-bits then use an ADDiu
      MachineBasicBlock &MBB = *MI.getParent();
      DebugLoc DL = II->getDebugLoc();
      const TargetRegisterClass *PtrRC =
          ABI.ArePtrs64bit() ? &M6502::GPR64RegClass : &M6502::GPR32RegClass;
      MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
      unsigned Reg = RegInfo.createVirtualRegister(PtrRC);
      const M6502SEInstrInfo &TII =
          *static_cast<const M6502SEInstrInfo *>(
              MBB.getParent()->getSubtarget().getInstrInfo());
      BuildMI(MBB, II, DL, TII.get(ABI.GetPtrAddiuOp()), Reg)
          .addReg(FrameReg)
          .addImm(Offset);

      FrameReg = Reg;
      Offset = 0;
      IsKill = true;
    } else if (!isInt<16>(Offset)) {
      // Otherwise split the offset into 16-bit pieces and add it in multiple
      // instructions.
      MachineBasicBlock &MBB = *MI.getParent();
      DebugLoc DL = II->getDebugLoc();
      unsigned NewImm = 0;
      const M6502SEInstrInfo &TII =
          *static_cast<const M6502SEInstrInfo *>(
              MBB.getParent()->getSubtarget().getInstrInfo());
      unsigned Reg = TII.loadImmediate(Offset, MBB, II, DL,
                                       OffsetBitSize == 16 ? &NewImm : nullptr);
      BuildMI(MBB, II, DL, TII.get(ABI.GetPtrAdduOp()), Reg).addReg(FrameReg)
        .addReg(Reg, RegState::Kill);

      FrameReg = Reg;
      Offset = SignExtend64<16>(NewImm);
      IsKill = true;
    }
  }

  MI.getOperand(OpNo).ChangeToRegister(FrameReg, false, false, IsKill);
  MI.getOperand(OpNo + 1).ChangeToImmediate(Offset);
}
