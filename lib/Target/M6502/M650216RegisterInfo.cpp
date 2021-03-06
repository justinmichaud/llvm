//===-- M650216RegisterInfo.cpp - M650216 Register Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M650216 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "M650216RegisterInfo.h"
#include "M6502.h"
#include "M650216InstrInfo.h"
#include "M6502InstrInfo.h"
#include "M6502MachineFunction.h"
#include "M6502Subtarget.h"
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

#define DEBUG_TYPE "m650216-registerinfo"

M650216RegisterInfo::M650216RegisterInfo() : M6502RegisterInfo() {}

bool M650216RegisterInfo::requiresRegisterScavenging
  (const MachineFunction &MF) const {
  return false;
}
bool M650216RegisterInfo::requiresFrameIndexScavenging
  (const MachineFunction &MF) const {
  return false;
}

bool M650216RegisterInfo::useFPForScavengingIndex
  (const MachineFunction &MF) const {
  return false;
}

bool M650216RegisterInfo::saveScavengerRegister
  (MachineBasicBlock &MBB,
   MachineBasicBlock::iterator I,
   MachineBasicBlock::iterator &UseMI,
   const TargetRegisterClass *RC,
   unsigned Reg) const {
  DebugLoc DL;
  const TargetInstrInfo &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  TII.copyPhysReg(MBB, I, DL, M6502::T0, Reg, true);
  TII.copyPhysReg(MBB, UseMI, DL, Reg, M6502::T0, true);
  return true;
}

const TargetRegisterClass *
M650216RegisterInfo::intRegClass(unsigned Size) const {
  assert(Size == 4);
  return &M6502::CPU16RegsRegClass;
}

void M650216RegisterInfo::eliminateFI(MachineBasicBlock::iterator II,
                                     unsigned OpNo, int FrameIndex,
                                     uint64_t StackSize,
                                     int64_t SPOffset) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();

  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;

  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  // The following stack frame objects are always
  // referenced relative to $sp:
  //  1. Outgoing arguments.
  //  2. Pointer to dynamically allocated stack space.
  //  3. Locations for callee-saved registers.
  // Everything else is referenced relative to whatever register
  // getFrameRegister() returns.
  unsigned FrameReg;

  if (FrameIndex >= MinCSFI && FrameIndex <= MaxCSFI)
    FrameReg = M6502::SP;
  else {
    const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
    if (TFI->hasFP(MF)) {
      FrameReg = M6502::S0;
    }
    else {
      if ((MI.getNumOperands()> OpNo+2) && MI.getOperand(OpNo+2).isReg())
        FrameReg = MI.getOperand(OpNo+2).getReg();
      else
        FrameReg = M6502::SP;
    }
  }
  // Calculate final offset.
  // - There is no need to change the offset if the frame object
  //   is one of the
  //   following: an outgoing argument, pointer to a dynamically allocated
  //   stack space or a $gp restore location,
  // - If the frame object is any of the following,
  //   its offset must be adjusted
  //   by adding the size of the stack:
  //   incoming argument, callee-saved register location or local variable.
  int64_t Offset;
  bool IsKill = false;
  Offset = SPOffset + (int64_t)StackSize;
  Offset += MI.getOperand(OpNo + 1).getImm();


  DEBUG(errs() << "Offset     : " << Offset << "\n" << "<--------->\n");

  if (!MI.isDebugValue() &&
      !M650216InstrInfo::validImmediate(MI.getOpcode(), FrameReg, Offset)) {
    MachineBasicBlock &MBB = *MI.getParent();
    DebugLoc DL = II->getDebugLoc();
    unsigned NewImm;
    const M650216InstrInfo &TII =
        *static_cast<const M650216InstrInfo *>(MF.getSubtarget().getInstrInfo());
    FrameReg = TII.loadImmediate(FrameReg, Offset, MBB, II, DL, NewImm);
    Offset = SignExtend64<16>(NewImm);
    IsKill = true;
  }
  MI.getOperand(OpNo).ChangeToRegister(FrameReg, false, false, IsKill);
  MI.getOperand(OpNo + 1).ChangeToImmediate(Offset);


}
