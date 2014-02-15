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
