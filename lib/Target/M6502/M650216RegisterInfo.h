//===-- M650216RegisterInfo.h - M650216 Register Information ------*- C++ -*-===//
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

#ifndef LLVM_LIB_TARGET_M6502_M650216REGISTERINFO_H
#define LLVM_LIB_TARGET_M6502_M650216REGISTERINFO_H

#include "M6502RegisterInfo.h"

namespace llvm {
class M650216InstrInfo;

class M650216RegisterInfo : public M6502RegisterInfo {
public:
  M650216RegisterInfo();

  bool requiresRegisterScavenging(const MachineFunction &MF) const override;

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override;

  bool useFPForScavengingIndex(const MachineFunction &MF) const override;

  bool saveScavengerRegister(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I,
                                     MachineBasicBlock::iterator &UseMI,
                                     const TargetRegisterClass *RC,
                                     unsigned Reg) const override;

  const TargetRegisterClass *intRegClass(unsigned Size) const override;

private:
  void eliminateFI(MachineBasicBlock::iterator II, unsigned OpNo,
                   int FrameIndex, uint64_t StackSize,
                   int64_t SPOffset) const override;
};

} // end namespace llvm

#endif
