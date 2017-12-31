//===-- M6502TargetMachine.cpp - Define TargetMachine for M6502 -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about M6502 target spec.
//
//===----------------------------------------------------------------------===//

#include "M6502TargetMachine.h"
#include "MCTargetDesc/M6502ABIInfo.h"
#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "M6502.h"
#include "M650216ISelDAGToDAG.h"
#include "M6502SEISelDAGToDAG.h"
#include "M6502Subtarget.h"
#include "M6502TargetObjectFile.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetOptions.h"
#include <string>

using namespace llvm;

#define DEBUG_TYPE "m6502"

extern "C" void LLVMInitializeM6502Target() {
  // Register the target.
  RegisterTargetMachine<M6502ebTargetMachine> X(getTheM6502Target());
}

static std::string computeDataLayout(const Triple &TT, StringRef CPU,
                                     const TargetOptions &Options,
                                     bool isLittle) {
  std::string Ret;
  M6502ABIInfo ABI = M6502ABIInfo::computeTargetABI(TT, CPU, Options.MCOptions);

  // There are both little and big endian m6502.
  if (isLittle)
    Ret += "e";
  else
    Ret += "E";

  if (ABI.IsO32())
    Ret += "-m:m";
  else
    Ret += "-m:e";

  // Pointers are 32 bit on some ABIs.
  if (!ABI.IsN64())
    Ret += "-p:32:32";

  // 8 and 16 bit integers only need to have natural alignment, but try to
  // align them to 32 bits. 64 bit integers have natural alignment.
  Ret += "-i8:8:32-i16:16:32-i64:64";

  // 32 bit registers are always available and the stack is at least 64 bit
  // aligned. On N64 64 bit registers are also available and the stack is
  // 128 bit aligned.
  if (ABI.IsN64() || ABI.IsN32())
    Ret += "-n32:64-S128";
  else
    Ret += "-n32-S64";

  return Ret;
}

static Reloc::Model getEffectiveRelocModel(bool JIT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue() || JIT)
    return Reloc::Static;
  return *RM;
}

static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
}

// On function prologue, the stack is created by decrementing
// its pointer. Once decremented, all references are done with positive
// offset from the stack/frame pointer, using StackGrowsUp enables
// an easier handling.
// Using CodeModel::Large enables different CALL behavior.
M6502TargetMachine::M6502TargetMachine(const Target &T, const Triple &TT,
                                     StringRef CPU, StringRef FS,
                                     const TargetOptions &Options,
                                     Optional<Reloc::Model> RM,
                                     Optional<CodeModel::Model> CM,
                                     CodeGenOpt::Level OL, bool JIT,
                                     bool isLittle)
    : LLVMTargetMachine(T, computeDataLayout(TT, CPU, Options, isLittle), TT,
                        CPU, FS, Options, getEffectiveRelocModel(JIT, RM),
                        getEffectiveCodeModel(CM), OL),
      isLittle(isLittle), TLOF(llvm::make_unique<M6502TargetObjectFile>()),
      ABI(M6502ABIInfo::computeTargetABI(TT, CPU, Options.MCOptions)),
      Subtarget(nullptr), DefaultSubtarget(TT, CPU, FS, isLittle, *this,
                                           Options.StackAlignmentOverride),
      NoM650216Subtarget(TT, CPU, FS.empty() ? "-m650216" : FS.str() + ",-m650216",
                        isLittle, *this, Options.StackAlignmentOverride),
      M650216Subtarget(TT, CPU, FS.empty() ? "+m650216" : FS.str() + ",+m650216",
                      isLittle, *this, Options.StackAlignmentOverride) {
  Subtarget = &DefaultSubtarget;
  initAsmInfo();
}

M6502TargetMachine::~M6502TargetMachine() = default;

void M6502ebTargetMachine::anchor() {}

M6502ebTargetMachine::M6502ebTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : M6502TargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, false) {}

void M6502elTargetMachine::anchor() {}

M6502elTargetMachine::M6502elTargetMachine(const Target &T, const Triple &TT,
                                         StringRef CPU, StringRef FS,
                                         const TargetOptions &Options,
                                         Optional<Reloc::Model> RM,
                                         Optional<CodeModel::Model> CM,
                                         CodeGenOpt::Level OL, bool JIT)
    : M6502TargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, JIT, true) {}

const M6502Subtarget *
M6502TargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU = !CPUAttr.hasAttribute(Attribute::None)
                        ? CPUAttr.getValueAsString().str()
                        : TargetCPU;
  std::string FS = !FSAttr.hasAttribute(Attribute::None)
                       ? FSAttr.getValueAsString().str()
                       : TargetFS;
  bool hasM650216Attr =
      !F.getFnAttribute("m650216").hasAttribute(Attribute::None);
  bool hasNoM650216Attr =
      !F.getFnAttribute("nom650216").hasAttribute(Attribute::None);

  bool HasMicroM6502Attr =
      !F.getFnAttribute("microm6502").hasAttribute(Attribute::None);
  bool HasNoMicroM6502Attr =
      !F.getFnAttribute("nomicrom6502").hasAttribute(Attribute::None);

  // FIXME: This is related to the code below to reset the target options,
  // we need to know whether or not the soft float flag is set on the
  // function, so we can enable it as a subtarget feature.
  bool softFloat =
      F.hasFnAttribute("use-soft-float") &&
      F.getFnAttribute("use-soft-float").getValueAsString() == "true";

  if (hasM650216Attr)
    FS += FS.empty() ? "+m650216" : ",+m650216";
  else if (hasNoM650216Attr)
    FS += FS.empty() ? "-m650216" : ",-m650216";
  if (HasMicroM6502Attr)
    FS += FS.empty() ? "+microm6502" : ",+microm6502";
  else if (HasNoMicroM6502Attr)
    FS += FS.empty() ? "-microm6502" : ",-microm6502";
  if (softFloat)
    FS += FS.empty() ? "+soft-float" : ",+soft-float";

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = llvm::make_unique<M6502Subtarget>(TargetTriple, CPU, FS, isLittle, *this,
                                         Options.StackAlignmentOverride);
  }
  return I.get();
}

void M6502TargetMachine::resetSubtarget(MachineFunction *MF) {
  DEBUG(dbgs() << "resetSubtarget\n");

  Subtarget = const_cast<M6502Subtarget *>(getSubtargetImpl(*MF->getFunction()));
  MF->setSubtarget(Subtarget);
}

namespace {

/// M6502 Code Generator Pass Configuration Options.
class M6502PassConfig : public TargetPassConfig {
public:
  M6502PassConfig(M6502TargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {
    // The current implementation of long branch pass requires a scratch
    // register ($at) to be available before branch instructions. Tail merging
    // can break this requirement, so disable it when long branch pass is
    // enabled.
    EnableTailMerge = !getM6502Subtarget().enableLongBranchPass();
  }

  M6502TargetMachine &getM6502TargetMachine() const {
    return getTM<M6502TargetMachine>();
  }

  const M6502Subtarget &getM6502Subtarget() const {
    return *getM6502TargetMachine().getSubtargetImpl();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
};

} // end anonymous namespace

TargetPassConfig *M6502TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new M6502PassConfig(*this, PM);
}

void M6502PassConfig::addIRPasses() {
  TargetPassConfig::addIRPasses();
  addPass(createAtomicExpandPass());
  if (getM6502Subtarget().os16())
    addPass(createM6502Os16Pass());
  if (getM6502Subtarget().inM650216HardFloat())
    addPass(createM650216HardFloatPass());
}
// Install an instruction selector pass using
// the ISelDag to gen M6502 code.
bool M6502PassConfig::addInstSelector() {
  addPass(createM6502ModuleISelDagPass());
  addPass(createM650216ISelDag(getM6502TargetMachine(), getOptLevel()));
  addPass(createM6502SEISelDag(getM6502TargetMachine(), getOptLevel()));
  return false;
}

void M6502PassConfig::addPreRegAlloc() {
  addPass(createM6502OptimizePICCallPass());
}

TargetIRAnalysis M6502TargetMachine::getTargetIRAnalysis() {
  return TargetIRAnalysis([this](const Function &F) {
    if (Subtarget->allowMixed16_32()) {
      DEBUG(errs() << "No Target Transform Info Pass Added\n");
      // FIXME: This is no longer necessary as the TTI returned is per-function.
      return TargetTransformInfo(F.getParent()->getDataLayout());
    }

    DEBUG(errs() << "Target Transform Info Pass Added\n");
    return TargetTransformInfo(BasicTTIImpl(this, F));
  });
}

// Implemented by targets that want to run passes immediately before
// machine code is emitted. return true if -print-machineinstrs should
// print out the code after the passes.
void M6502PassConfig::addPreEmitPass() {
  addPass(createMicroM6502SizeReductionPass());

  // The delay slot filler pass can potientially create forbidden slot (FS)
  // hazards for M6502R6 which the hazard schedule pass (HSP) will fix. Any
  // (new) pass that creates compact branches after the HSP must handle FS
  // hazards itself or be pipelined before the HSP.
  addPass(createM6502DelaySlotFillerPass());
  addPass(createM6502HazardSchedule());
  addPass(createM6502LongBranchPass());
  addPass(createM6502ConstantIslandPass());
}
