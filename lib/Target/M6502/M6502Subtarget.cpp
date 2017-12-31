//===-- M6502Subtarget.cpp - M6502 Subtarget Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the M6502 specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "M6502Subtarget.h"
#include "M6502.h"
#include "M6502MachineFunction.h"
#include "M6502RegisterInfo.h"
#include "M6502TargetMachine.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "m6502-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "M6502GenSubtargetInfo.inc"

// FIXME: Maybe this should be on by default when M650216 is specified
//
static cl::opt<bool>
    Mixed16_32("m6502-mixed-16-32", cl::init(false),
               cl::desc("Allow for a mixture of M650216 "
                        "and M650232 code in a single output file"),
               cl::Hidden);

static cl::opt<bool> M6502_Os16("m6502-os16", cl::init(false),
                               cl::desc("Compile all functions that don't use "
                                        "floating point as M6502 16"),
                               cl::Hidden);

static cl::opt<bool> M650216HardFloat("m650216-hard-float", cl::NotHidden,
                                     cl::desc("Enable mips16 hard float."),
                                     cl::init(false));

static cl::opt<bool>
    M650216ConstantIslands("m650216-constant-islands", cl::NotHidden,
                          cl::desc("Enable mips16 constant islands."),
                          cl::init(true));

static cl::opt<bool>
    GPOpt("m6502-mgpopt", cl::Hidden,
          cl::desc("Enable gp-relative addressing of mips small data items"));

void M6502Subtarget::anchor() {}

M6502Subtarget::M6502Subtarget(const Triple &TT, StringRef CPU, StringRef FS,
                             bool little, const M6502TargetMachine &TM,
                             unsigned StackAlignOverride)
    : M6502GenSubtargetInfo(TT, CPU, FS), M6502ArchVersion(M6502Default),
      IsLittle(little), IsSoftFloat(false), IsSingleFloat(false), IsFPXX(false),
      NoABICalls(false), IsFP64bit(false), UseOddSPReg(true),
      IsNaN2008bit(false), IsGP64bit(false), HasVFPU(false), HasCnM6502(false),
      HasM65023_32(false), HasM65023_32r2(false), HasM65024_32(false),
      HasM65024_32r2(false), HasM65025_32r2(false), InM650216Mode(false),
      InM650216HardFloat(M650216HardFloat), InMicroM6502Mode(false), HasDSP(false),
      HasDSPR2(false), HasDSPR3(false), AllowMixed16_32(Mixed16_32 | M6502_Os16),
      Os16(M6502_Os16), HasMSA(false), UseTCCInDIV(false), HasSym32(false),
      HasEVA(false), DisableMadd4(false), HasMT(false),
      StackAlignOverride(StackAlignOverride), TM(TM), TargetTriple(TT),
      TSInfo(), InstrInfo(M6502InstrInfo::create(
                    initializeSubtargetDependencies(CPU, FS, TM))),
      FrameLowering(M6502FrameLowering::create(*this)),
      TLInfo(M6502TargetLowering::create(TM, *this)) {

  if (M6502ArchVersion == M6502Default)
    M6502ArchVersion = M650232;

  // Don't even attempt to generate code for M6502-I and M6502-V. They have not
  // been tested and currently exist for the integrated assembler only.
  if (M6502ArchVersion == M65021)
    report_fatal_error("Code generation for M6502-I is not implemented", false);
  if (M6502ArchVersion == M65025)
    report_fatal_error("Code generation for M6502-V is not implemented", false);

  // Check if Architecture and ABI are compatible.
  assert(((!isGP64bit() && isABI_O32()) ||
          (isGP64bit() && (isABI_N32() || isABI_N64()))) &&
         "Invalid  Arch & ABI pair.");

  if (hasMSA() && !isFP64bit())
    report_fatal_error("MSA requires a 64-bit FPU register file (FR=1 mode). "
                       "See -mattr=+fp64.",
                       false);

  if (!isABI_O32() && !useOddSPReg())
    report_fatal_error("-mattr=+nooddspreg requires the O32 ABI.", false);

  if (IsFPXX && (isABI_N32() || isABI_N64()))
    report_fatal_error("FPXX is not permitted for the N32/N64 ABI's.", false);

  if (hasM650232r6()) {
    StringRef ISA = hasM650264r6() ? "M650264r6" : "M650232r6";

    assert(isFP64bit());
    assert(isNaN2008());
    if (hasDSP())
      report_fatal_error(ISA + " is not compatible with the DSP ASE", false);
  }

  if (NoABICalls && TM.isPositionIndependent())
    report_fatal_error("position-independent code requires '-mabicalls'");

  if (isABI_N64() && !TM.isPositionIndependent() && !hasSym32())
    NoABICalls = true;

  // Set UseSmallSection.
  UseSmallSection = GPOpt;
  if (!NoABICalls && GPOpt) {
    errs() << "warning: cannot use small-data accesses for '-mabicalls'"
           << "\n";
    UseSmallSection = false;
  }
}

bool M6502Subtarget::isPositionIndependent() const {
  return TM.isPositionIndependent();
}

/// This overrides the PostRAScheduler bit in the SchedModel for any CPU.
bool M6502Subtarget::enablePostRAScheduler() const { return true; }

void M6502Subtarget::getCriticalPathRCs(RegClassVector &CriticalPathRCs) const {
  CriticalPathRCs.clear();
  CriticalPathRCs.push_back(isGP64bit() ? &M6502::GPR64RegClass
                                        : &M6502::GPR32RegClass);
}

CodeGenOpt::Level M6502Subtarget::getOptLevelToEnablePostRAScheduler() const {
  return CodeGenOpt::Aggressive;
}

M6502Subtarget &
M6502Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                               const TargetMachine &TM) {
  std::string CPUName = M6502_MC::selectM6502CPU(TM.getTargetTriple(), CPU);

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  if (InM650216Mode && !IsSoftFloat)
    InM650216HardFloat = true;

  if (StackAlignOverride)
    stackAlignment = StackAlignOverride;
  else if (isABI_N32() || isABI_N64())
    stackAlignment = 16;
  else {
    assert(isABI_O32() && "Unknown ABI for stack alignment!");
    stackAlignment = 8;
  }

  return *this;
}

bool M6502Subtarget::useConstantIslands() {
  DEBUG(dbgs() << "use constant islands " << M650216ConstantIslands << "\n");
  return M650216ConstantIslands;
}

Reloc::Model M6502Subtarget::getRelocationModel() const {
  return TM.getRelocationModel();
}

bool M6502Subtarget::isABI_N64() const { return getABI().IsN64(); }
bool M6502Subtarget::isABI_N32() const { return getABI().IsN32(); }
bool M6502Subtarget::isABI_O32() const { return getABI().IsO32(); }
const M6502ABIInfo &M6502Subtarget::getABI() const { return TM.getABI(); }
