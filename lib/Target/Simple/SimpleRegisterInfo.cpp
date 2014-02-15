#include "Simple.h"
#include "SimpleRegisterInfo.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"

#define GET_REGINFO_TARGET_DESC
#include "SimpleGenRegisterInfo.inc"

using namespace llvm;

SimpleRegisterInfo::SimpleRegisterInfo()
  : SimpleGenRegisterInfo(Simple::R4) {
}


const uint16_t* SimpleRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF)
                                                                         const 
{
  static const uint16_t CalleeSavedRegs[] = {
    Simple::R4,
    0
  };
  return CalleeSavedRegs;
}

BitVector SimpleRegisterInfo::getReservedRegs(const MachineFunction &MF) const 
{
  BitVector Reserved(getNumRegs());
  return Reserved;
}

void
SimpleRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const 
{
  assert(!"NOT IMPLEMENTED");
}


unsigned SimpleRegisterInfo::getFrameRegister(const MachineFunction &MF) const 
{
  assert(!"NOT IMPLEMENTED");
  return 0;
}
