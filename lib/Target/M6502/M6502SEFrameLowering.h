//===- M6502SEFrameLowering.h - M650232/64 frame lowering ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502SEFRAMELOWERING_H
#define LLVM_LIB_TARGET_M6502_M6502SEFRAMELOWERING_H

#include "M6502FrameLowering.h"
#include <vector>

namespace llvm {

class MachineBasicBlock;
class MachineFunction;
class M6502Subtarget;

class M6502SEFrameLowering : public M6502FrameLowering {
public:
  explicit M6502SEFrameLowering(const M6502Subtarget &STI);

  /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
  /// the function.
  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;

  int getFrameIndexReference(const MachineFunction &MF, int FI,
                             unsigned &FrameReg) const override;

  bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MI,
                                 const std::vector<CalleeSavedInfo> &CSI,
                                 const TargetRegisterInfo *TRI) const override;

  bool hasReservedCallFrame(const MachineFunction &MF) const override;

  void determineCalleeSaves(MachineFunction &MF, BitVector &SavedRegs,
                            RegScavenger *RS) const override;
  unsigned ehDataReg(unsigned I) const;

private:
  void emitInterruptEpilogueStub(MachineFunction &MF,
                                 MachineBasicBlock &MBB) const;
  void emitInterruptPrologueStub(MachineFunction &MF,
                                 MachineBasicBlock &MBB) const;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_M6502_M6502SEFRAMELOWERING_H
