//===-- M6502FrameLowering.h - Define frame lowering for M6502 ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502FRAMELOWERING_H
#define LLVM_LIB_TARGET_M6502_M6502FRAMELOWERING_H

#include "M6502.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {
  class M6502Subtarget;

class M6502FrameLowering : public TargetFrameLowering {
protected:
  const M6502Subtarget &STI;

public:
  explicit M6502FrameLowering(const M6502Subtarget &sti, unsigned Alignment)
    : TargetFrameLowering(StackGrowsDown, Alignment, 0, Alignment), STI(sti) {}

  static const M6502FrameLowering *create(const M6502Subtarget &ST);

  bool hasFP(const MachineFunction &MF) const override;

  bool hasBP(const MachineFunction &MF) const;

  bool isFPCloseToIncomingSP() const override { return false; }

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF,
                                MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const override;

protected:
  uint64_t estimateStackSize(const MachineFunction &MF) const;
};

/// Create M6502FrameLowering objects.
const M6502FrameLowering *createM650216FrameLowering(const M6502Subtarget &ST);
const M6502FrameLowering *createM6502SEFrameLowering(const M6502Subtarget &ST);

} // End llvm namespace

#endif
