//===- M6502TargetMachine.h - Define TargetMachine for M6502 ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the M6502 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502TARGETMACHINE_H
#define LLVM_LIB_TARGET_M6502_M6502TARGETMACHINE_H

#include "MCTargetDesc/M6502ABIInfo.h"
#include "M6502Subtarget.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

namespace llvm {

class M6502TargetMachine : public LLVMTargetMachine {
  bool isLittle;
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  // Selected ABI
  M6502ABIInfo ABI;
  M6502Subtarget *Subtarget;
  M6502Subtarget DefaultSubtarget;
  M6502Subtarget NoM650216Subtarget;
  M6502Subtarget M650216Subtarget;

  mutable StringMap<std::unique_ptr<M6502Subtarget>> SubtargetMap;

public:
  M6502TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                    StringRef FS, const TargetOptions &Options,
                    Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                    CodeGenOpt::Level OL, bool JIT, bool isLittle);
  ~M6502TargetMachine() override;

  TargetIRAnalysis getTargetIRAnalysis() override;

  const M6502Subtarget *getSubtargetImpl() const {
    if (Subtarget)
      return Subtarget;
    return &DefaultSubtarget;
  }

  const M6502Subtarget *getSubtargetImpl(const Function &F) const override;

  /// \brief Reset the subtarget for the M6502 target.
  void resetSubtarget(MachineFunction *MF);

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }

  bool isLittleEndian() const { return isLittle; }
  const M6502ABIInfo &getABI() const { return ABI; }

  bool isMachineVerifierClean() const override {
    return false;
  }
};

/// M650232/64 big endian target machine.
///
class M6502ebTargetMachine : public M6502TargetMachine {
  virtual void anchor();

public:
  M6502ebTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

/// M650232/64 little endian target machine.
///
class M6502elTargetMachine : public M6502TargetMachine {
  virtual void anchor();

public:
  M6502elTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                      StringRef FS, const TargetOptions &Options,
                      Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                      CodeGenOpt::Level OL, bool JIT);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_M6502_M6502TARGETMACHINE_H
