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

const char *SimpleTargetLowering::getTargetNodeName(unsigned Opcode) const
{
  switch (Opcode)
  {
  case SimpleISD::RET : return "Simple::RET";
  default             : return NULL;
  }
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

SDValue SimpleTargetLowering::LowerReturn(
  SDValue Chain, CallingConv::ID CallConv,
  bool isVarArg,
  const SmallVectorImpl<ISD::OutputArg> &Outs,
  const SmallVectorImpl<SDValue> &OutVals,
  SDLoc dl, SelectionDAG &DAG) const
{
  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  if (Outs.size() >= Simple::GPR32RegClass.getNumRegs())
  {
    assert(!"not implemented");
    return SDValue();
  }

  // Return is always ret 0
  RetOps.push_back(DAG.getConstant(0, MVT::i32));

  // Build CopyToReg's
  for (unsigned i = 0; i != Outs.size(); ++i) 
  {
    if (Outs[i].VT.SimpleTy != MVT::i32)
      assert(!"not implemented");

    unsigned loc_reg = Simple::GPR32RegClass.getRegister(i);

    Chain = DAG.getCopyToReg(Chain, dl, loc_reg,
      OutVals[i], Flag);

    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(loc_reg, Outs[i].VT));
  }

  RetOps[0] = Chain;  // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  return DAG.getNode(SimpleISD::RET, dl, MVT::Other,
    &RetOps[0], RetOps.size());
}
