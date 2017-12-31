//===- M6502RegisterInfo.h - M6502 Register Information Impl ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M6502 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502REGISTERINFO_H
#define LLVM_LIB_TARGET_M6502_M6502REGISTERINFO_H

#include "M6502.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include <cstdint>

#define GET_REGINFO_HEADER
#include "M6502GenRegisterInfo.inc"

namespace llvm {

class TargetRegisterClass;

class M6502RegisterInfo : public M6502GenRegisterInfo {
public:
  enum class M6502PtrClass {
    /// The default register class for integer values.
    Default = 0,
    /// The subset of registers permitted in certain microM6502 instructions
    /// such as lw16.
    GPR16MM = 1,
    /// The stack pointer only.
    StackPointer = 2,
    /// The global pointer only.
    GlobalPointer = 3,
  };

  M6502RegisterInfo();

  /// Get PIC indirect call register
  static unsigned getPICCallReg();

  /// Code Generation virtual methods...
  const TargetRegisterClass *getPointerRegClass(const MachineFunction &MF,
                                                unsigned Kind) const override;

  unsigned getRegPressureLimit(const TargetRegisterClass *RC,
                               MachineFunction &MF) const override;
  const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;
  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const override;
  static const uint32_t *getM650216RetHelperMask();

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  bool requiresRegisterScavenging(const MachineFunction &MF) const override;

  bool trackLivenessAfterRegAlloc(const MachineFunction &MF) const override;

  /// Stack Frame Processing Methods
  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  // Stack realignment queries.
  bool canRealignStack(const MachineFunction &MF) const override;

  /// Debug information queries.
  unsigned getFrameRegister(const MachineFunction &MF) const override;

  /// \brief Return GPR register class.
  virtual const TargetRegisterClass *intRegClass(unsigned Size) const = 0;

private:
  virtual void eliminateFI(MachineBasicBlock::iterator II, unsigned OpNo,
                           int FrameIndex, uint64_t StackSize,
                           int64_t SPOffset) const = 0;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_M6502_M6502REGISTERINFO_H
