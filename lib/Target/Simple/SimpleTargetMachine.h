#ifndef SIMPLE_TARGETMACHINE_H
#define SIMPLE_TARGETMACHINE_H

#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class SimpleTargetMachine : public LLVMTargetMachine {
  const DataLayout DL;       // Calculates type size & alignment

public:
  SimpleTargetMachine(const Target &T, StringRef TT,
                      StringRef CPU, StringRef FS, const TargetOptions &Options,
                      Reloc::Model RM, CodeModel::Model CM,
                      CodeGenOpt::Level OL);

  // Pass Pipeline Configuration
  virtual TargetPassConfig *createPassConfig(PassManagerBase &PM);

  virtual void addAnalysisPasses(PassManagerBase &PM);
};

} // end namespace llvm

#endif // SIMPLE_TARGETMACHINE_H
