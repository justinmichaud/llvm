#include "Simple.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::TheSimpleTarget;

extern "C" void LLVMInitializeSimpleTargetInfo() { 
  RegisterTarget<Triple::simple> X(TheSimpleTarget, "simple", "Simple");
}
