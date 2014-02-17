#ifndef SIMPLE_ISELLOWERING_H
#define SIMPLE_ISELLOWERING_H

#include "Simple.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/Target/TargetLowering.h"

namespace llvm {

  // Forward declarations
  class SimpleSubtarget;
  class SimpleTargetMachine;

  //===--------------------------------------------------------------------===//
  // TargetLowering Implementation
  //===--------------------------------------------------------------------===//
  class SimpleTargetLowering : public TargetLowering
  {
  public:
    explicit SimpleTargetLowering(SimpleTargetMachine &TM);

    virtual SDValue
      LowerFormalArguments(SDValue /*Chain*/, CallingConv::ID /*CallConv*/,
      bool /*isVarArg*/,
      const SmallVectorImpl<ISD::InputArg> &/*Ins*/,
      SDLoc /*dl*/, SelectionDAG &/*DAG*/,
      SmallVectorImpl<SDValue> &/*InVals*/) const;

  private:
    const SimpleTargetMachine &TM;
    const SimpleSubtarget &Subtarget;
  };
}

#endif // SIMPLE_ISELLOWERING_H
