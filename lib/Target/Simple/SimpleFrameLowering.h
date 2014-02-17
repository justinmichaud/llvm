#ifndef SIMPLE_FRAMEINFO_H
#define SIMPLE_FRAMEINFO_H

#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class SimpleSubtarget;

  class SimpleFrameLowering: public TargetFrameLowering 
  {
  public:
    SimpleFrameLowering(const SimpleSubtarget &STI);

    /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
    /// the function.
    void emitPrologue(MachineFunction &MF) const;
    void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const;

    void eliminateCallFramePseudoInstr(MachineFunction &MF,
                                       MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator I) const;

    bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
      MachineBasicBlock::iterator MI,
      const std::vector<CalleeSavedInfo> &CSI,
      const TargetRegisterInfo *TRI) const;
    bool restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
      MachineBasicBlock::iterator MI,
      const std::vector<CalleeSavedInfo> &CSI,
      const TargetRegisterInfo *TRI) const;

    bool hasFP(const MachineFunction &MF) const;
  };
}

#endif // SIMPLE_FRAMEINFO_H
