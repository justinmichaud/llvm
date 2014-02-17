#define DEBUG_TYPE "simple-lower"

#include "SimpleISelLowering.h"
#include "Simple.h"
#include "SimpleSubtarget.h"
#include "SimpleTargetMachine.h"
#include "SimpleTargetObjectFile.h"
#include "SimpleRegisterInfo.h"
//#include "SimpleMachineFunctionInfo.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalAlias.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>

using namespace llvm;

SimpleTargetLowering::SimpleTargetLowering(SimpleTargetMachine &STM)
  : TargetLowering(STM, new SimpleTargetObjectFile()),
    TM(STM),
    Subtarget(*STM.getSubtargetImpl()) {

  // Set up the register classes.
  addRegisterClass(MVT::i32, &Simple::GPR32RegClass);

  // Compute derived properties from the register classes
  computeRegisterProperties();

  setSchedulingPreference(Sched::Source);
}

SDValue SimpleTargetLowering::LowerFormalArguments(
  SDValue Chain, CallingConv::ID CallConv,
  bool isVarArg,
  const SmallVectorImpl<ISD::InputArg> &Ins,
  SDLoc dl, SelectionDAG &DAG,
  SmallVectorImpl<SDValue> &InVals) const
{
  // Implement really simple calling convention
  // Every argument is passed through register
  // If not enough registers - just fail

  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();

  if (Ins.size() >= Simple::GPR32RegClass.getNumRegs())
  {
    assert(!"not implemented");
    return SDValue();
  }

  SmallVector<SDValue, 4> ArgChains;

  // Emit CopyFromReg's
  for (unsigned i = 0, e = Ins.size(); i != e; ++i) 
  {
    if (Ins[i].VT.SimpleTy != MVT::i32)
      assert(!"not implemented");

    unsigned VReg = RegInfo.createVirtualRegister(&Simple::GPR32RegClass);
    RegInfo.addLiveIn(Simple::GPR32RegClass.getRegister(i), VReg);
    SDValue ArgIn = DAG.getCopyFromReg(Chain, dl, VReg, Ins[i].ArgVT);
    InVals.push_back(ArgIn);
    ArgChains.push_back(ArgIn.getValue(ArgIn->getNumValues() - 1));
  }

  // Return chain
  return DAG.getNode(ISD::TokenFactor, dl, MVT::Other, &ArgChains[0],
    ArgChains.size());
}
