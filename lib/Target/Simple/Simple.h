#ifndef SIMPLE_H
#define SIMPLE_H

#include "MCTargetDesc/SimpleMCTargetDesc.h"

#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class TargetMachine;

  extern Target TheSimpleTarget;
} // End llvm namespace

#endif // SIMPLE_H