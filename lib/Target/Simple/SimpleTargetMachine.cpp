#include "Simple.h"
#include "SimpleTargetMachine.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/IR/Module.h"
#include "llvm/PassManager.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

SimpleTargetMachine::SimpleTargetMachine(const Target &T, StringRef TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Reloc::Model RM, CodeModel::Model CM,
                                         CodeGenOpt::Level OL)
: LLVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL),
  DL("e-m:e-p:32:32-i1:8:32-i8:8:32-i16:16:32-i64:32-f64:32-a:0:32-n32"),
  InstrInfo(),
  Subtarget(TT, CPU, FS),
  FrameLowering(Subtarget),
  TLInfo(*this)
{
  initAsmInfo();
}

namespace {
  /// Simple Code Generator Pass Configuration Options.
  class SimplePassConfig : public TargetPassConfig 
  {
  public:
    SimplePassConfig(SimpleTargetMachine *TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

    SimpleTargetMachine &getSimpleTargetMachine() const {
      return getTM<SimpleTargetMachine>();
    }

    virtual bool addInstSelector();
  };
} // namespace

TargetPassConfig *SimpleTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new SimplePassConfig(this, PM);
}

bool SimplePassConfig::addInstSelector() {
  addPass(createSimpleISelDag(getSimpleTargetMachine(), getOptLevel()));
  return false;
}

// Force static initialization.
extern "C" void LLVMInitializeSimpleTarget() {
  RegisterTargetMachine<SimpleTargetMachine> X(TheSimpleTarget);
}

void SimpleTargetMachine::addAnalysisPasses(PassManagerBase &PM) {
  PM.add(createBasicTargetTransformInfoPass(this));
}
