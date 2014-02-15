#ifndef SIMPLE_H
#define SIMPLE_H

#include "MCTargetDesc/SimpleMCTargetDesc.h"

#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class FunctionPass;
  class ModulePass;
  class TargetMachine;
  class SimpleTargetMachine;
  class formatted_raw_ostream;

  extern Target TheSimpleTarget;

  FunctionPass *createSimpleISelDag(SimpleTargetMachine &TM,
                                    CodeGenOpt::Level OptLevel);
} // End llvm namespace

#endif // SIMPLE_H