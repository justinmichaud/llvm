#include "SimpleFrameLowering.h"
#include "Simple.h"
#include "SimpleInstrInfo.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

SimpleFrameLowering::SimpleFrameLowering(const SimpleSubtarget &sti)
  : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 4, 0) 
{
  ;
}

bool SimpleFrameLowering::hasFP(const MachineFunction &MF) const {
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
         MF.getFrameInfo()->hasVarSizedObjects();
}

void SimpleFrameLowering::emitPrologue(MachineFunction &MF) const 
{
  ;
}

void SimpleFrameLowering::emitEpilogue(MachineFunction &MF,
                                     MachineBasicBlock &MBB) const 
{
  ;
}

bool SimpleFrameLowering::
spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI,
                          const std::vector<CalleeSavedInfo> &CSI,
                          const TargetRegisterInfo *TRI) const 
{
  assert(!"not implemented");
  return false;
}

bool SimpleFrameLowering::
restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MI,
                            const std::vector<CalleeSavedInfo> &CSI,
                            const TargetRegisterInfo *TRI) const
{
  assert(!"not implemented");
  return false;
}

// This function eliminates ADJCALLSTACKDOWN,
// ADJCALLSTACKUP pseudo instructions
void SimpleFrameLowering::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const 
{
  // TODO: Something reasonable should happen here
  MBB.erase(I);
}
