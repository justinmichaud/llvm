#ifndef SimpleREGISTERINFO_H
#define SimpleREGISTERINFO_H

#include "llvm/Target/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "SimpleGenRegisterInfo.inc"

namespace llvm {

class TargetInstrInfo;

struct SimpleRegisterInfo : public SimpleGenRegisterInfo {
public:
  SimpleRegisterInfo();

  /// Code Generation virtual methods...

  const uint16_t *getCalleeSavedRegs(const MachineFunction *MF = 0) const;

  BitVector getReservedRegs(const MachineFunction &MF) const;
  
  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, unsigned FIOperandNum,
                           RegScavenger *RS = NULL) const;

  unsigned getFrameRegister(const MachineFunction &MF) const;
};

} // end namespace llvm

#endif
