#ifndef SIMPLE_ISELLOWERING_H
#define SIMPLE_ISELLOWERING_H

#include "Simple.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/Target/TargetLowering.h"

namespace llvm {

  // Forward declarations
  class SimpleSubtarget;
  class SimpleTargetMachine;

  namespace SimpleISD
  {
    enum NodeType {
      // Start the numbering where the builtin ops and target ops leave off.
      FIRST_NUMBER = ISD::BUILTIN_OP_END,

      // Return instruction
      RET
    };
  }

  //===--------------------------------------------------------------------===//
  // TargetLowering Implementation
  //===--------------------------------------------------------------------===//
  class SimpleTargetLowering : public TargetLowering
  {
  public:
    explicit SimpleTargetLowering(SimpleTargetMachine &TM);

    virtual const char *getTargetNodeName(unsigned Opcode) const;

    virtual SDValue
      LowerFormalArguments(SDValue /*Chain*/, CallingConv::ID /*CallConv*/,
      bool /*isVarArg*/,
      const SmallVectorImpl<ISD::InputArg> &/*Ins*/,
      SDLoc /*dl*/, SelectionDAG &/*DAG*/,
      SmallVectorImpl<SDValue> &/*InVals*/) const;

    virtual SDValue
      LowerReturn(SDValue /*Chain*/, CallingConv::ID /*CallConv*/,
      bool /*isVarArg*/,
      const SmallVectorImpl<ISD::OutputArg> &/*Outs*/,
      const SmallVectorImpl<SDValue> &/*OutVals*/,
      SDLoc /*dl*/, SelectionDAG &/*DAG*/) const;

  private:
    const SimpleTargetMachine &TM;
    const SimpleSubtarget &Subtarget;
  };
}

#endif // SIMPLE_ISELLOWERING_H
