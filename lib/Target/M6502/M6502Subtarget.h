//===-- M6502Subtarget.h - Define Subtarget for the M6502 ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the M6502 specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502SUBTARGET_H
#define LLVM_LIB_TARGET_M6502_M6502SUBTARGET_H

#include "MCTargetDesc/M6502ABIInfo.h"
#include "M6502FrameLowering.h"
#include "M6502ISelLowering.h"
#include "M6502InstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/MC/MCInstrItineraries.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetSubtargetInfo.h"
#include <string>

#define GET_SUBTARGETINFO_HEADER
#include "M6502GenSubtargetInfo.inc"

namespace llvm {
class StringRef;

class M6502TargetMachine;

class M6502Subtarget : public M6502GenSubtargetInfo {
  virtual void anchor();

  enum M6502ArchEnum {
    M6502Default,
    M65021, M65022, M650232, M650232r2, M650232r3, M650232r5, M650232r6, M650232Max,
    M65023, M65024, M65025, M650264, M650264r2, M650264r3, M650264r5, M650264r6
  };

  enum class CPU { P5600 };

  // M6502 architecture version
  M6502ArchEnum M6502ArchVersion;

  // Processor implementation (unused but required to exist by
  // tablegen-erated code).
  CPU ProcImpl;

  // IsLittle - The target is Little Endian
  bool IsLittle;

  // IsSoftFloat - The target does not support any floating point instructions.
  bool IsSoftFloat;

  // IsSingleFloat - The target only supports single precision float
  // point operations. This enable the target to use all 32 32-bit
  // floating point registers instead of only using even ones.
  bool IsSingleFloat;

  // IsFPXX - M6502 O32 modeless ABI.
  bool IsFPXX;

  // NoABICalls - Disable SVR4-style position-independent code.
  bool NoABICalls;

  // IsFP64bit - The target processor has 64-bit floating point registers.
  bool IsFP64bit;

  /// Are odd single-precision registers permitted?
  /// This corresponds to -modd-spreg and -mno-odd-spreg
  bool UseOddSPReg;

  // IsNan2008 - IEEE 754-2008 NaN encoding.
  bool IsNaN2008bit;

  // IsGP64bit - General-purpose registers are 64 bits wide
  bool IsGP64bit;

  // IsPTR64bit - Pointers are 64 bit wide
  bool IsPTR64bit;

  // HasVFPU - Processor has a vector floating point unit.
  bool HasVFPU;

  // CPU supports cnM6502 (Cavium Networks Octeon CPU).
  bool HasCnM6502;

  // isLinux - Target system is Linux. Is false we consider ELFOS for now.
  bool IsLinux;

  // UseSmallSection - Small section is used.
  bool UseSmallSection;

  /// Features related to the presence of specific instructions.

  // HasM65023_32 - The subset of M6502-III instructions added to M650232
  bool HasM65023_32;

  // HasM65023_32r2 - The subset of M6502-III instructions added to M650232r2
  bool HasM65023_32r2;

  // HasM65024_32 - Has the subset of M6502-IV present in M650232
  bool HasM65024_32;

  // HasM65024_32r2 - Has the subset of M6502-IV present in M650232r2
  bool HasM65024_32r2;

  // HasM65025_32r2 - Has the subset of M6502-V present in M650232r2
  bool HasM65025_32r2;

  // InM650216 -- can process M650216 instructions
  bool InM650216Mode;

  // M650216 hard float
  bool InM650216HardFloat;

  // InMicroM6502 -- can process MicroM6502 instructions
  bool InMicroM6502Mode;

  // HasDSP, HasDSPR2, HasDSPR3 -- supports DSP ASE.
  bool HasDSP, HasDSPR2, HasDSPR3;

  // Allow mixed M650216 and M650232 in one source file
  bool AllowMixed16_32;

  // Optimize for space by compiling all functions as M6502 16 unless
  // it needs floating point. Functions needing floating point are
  // compiled as M650232
  bool Os16;

  // HasMSA -- supports MSA ASE.
  bool HasMSA;

  // UseTCCInDIV -- Enables the use of trapping in the assembler.
  bool UseTCCInDIV;

  // Sym32 -- On M650264 symbols are 32 bits.
  bool HasSym32;

  // HasEVA -- supports EVA ASE.
  bool HasEVA;
 
  // nomadd4 - disables generation of 4-operand madd.s, madd.d and
  // related instructions.
  bool DisableMadd4;

  // HasMT -- support MT ASE.
  bool HasMT;

  // Disable use of the `jal` instruction.
  bool UseLongCalls = false;

  /// The minimum alignment known to hold of the stack frame on
  /// entry to the function and which must be maintained by every function.
  unsigned stackAlignment;

  /// The overridden stack alignment.
  unsigned StackAlignOverride;

  InstrItineraryData InstrItins;

  // We can override the determination of whether we are in mips16 mode
  // as from the command line
  enum {NoOverride, M650216Override, NoM650216Override} OverrideMode;

  const M6502TargetMachine &TM;

  Triple TargetTriple;

  const SelectionDAGTargetInfo TSInfo;
  std::unique_ptr<const M6502InstrInfo> InstrInfo;
  std::unique_ptr<const M6502FrameLowering> FrameLowering;
  std::unique_ptr<const M6502TargetLowering> TLInfo;

public:
  bool isPositionIndependent() const;
  /// This overrides the PostRAScheduler bit in the SchedModel for each CPU.
  bool enablePostRAScheduler() const override;
  void getCriticalPathRCs(RegClassVector &CriticalPathRCs) const override;
  CodeGenOpt::Level getOptLevelToEnablePostRAScheduler() const override;

  bool isABI_N64() const;
  bool isABI_N32() const;
  bool isABI_O32() const;
  const M6502ABIInfo &getABI() const;
  bool isABI_FPXX() const { return isABI_O32() && IsFPXX; }

  /// This constructor initializes the data members to match that
  /// of the specified triple.
  M6502Subtarget(const Triple &TT, StringRef CPU, StringRef FS, bool little,
                const M6502TargetMachine &TM, unsigned StackAlignOverride);

  /// ParseSubtargetFeatures - Parses features string setting specified
  /// subtarget options.  Definition of function is auto generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef FS);

  bool hasM65021() const { return M6502ArchVersion >= M65021; }
  bool hasM65022() const { return M6502ArchVersion >= M65022; }
  bool hasM65023() const { return M6502ArchVersion >= M65023; }
  bool hasM65024() const { return M6502ArchVersion >= M65024; }
  bool hasM65025() const { return M6502ArchVersion >= M65025; }
  bool hasM65024_32() const { return HasM65024_32; }
  bool hasM65024_32r2() const { return HasM65024_32r2; }
  bool hasM650232() const {
    return (M6502ArchVersion >= M650232 && M6502ArchVersion < M650232Max) ||
           hasM650264();
  }
  bool hasM650232r2() const {
    return (M6502ArchVersion >= M650232r2 && M6502ArchVersion < M650232Max) ||
           hasM650264r2();
  }
  bool hasM650232r3() const {
    return (M6502ArchVersion >= M650232r3 && M6502ArchVersion < M650232Max) ||
           hasM650264r2();
  }
  bool hasM650232r5() const {
    return (M6502ArchVersion >= M650232r5 && M6502ArchVersion < M650232Max) ||
           hasM650264r5();
  }
  bool hasM650232r6() const {
    return (M6502ArchVersion >= M650232r6 && M6502ArchVersion < M650232Max) ||
           hasM650264r6();
  }
  bool hasM650264() const { return M6502ArchVersion >= M650264; }
  bool hasM650264r2() const { return M6502ArchVersion >= M650264r2; }
  bool hasM650264r3() const { return M6502ArchVersion >= M650264r3; }
  bool hasM650264r5() const { return M6502ArchVersion >= M650264r5; }
  bool hasM650264r6() const { return M6502ArchVersion >= M650264r6; }

  bool hasCnM6502() const { return HasCnM6502; }

  bool isLittle() const { return IsLittle; }
  bool isABICalls() const { return !NoABICalls; }
  bool isFPXX() const { return IsFPXX; }
  bool isFP64bit() const { return IsFP64bit; }
  bool useOddSPReg() const { return UseOddSPReg; }
  bool noOddSPReg() const { return !UseOddSPReg; }
  bool isNaN2008() const { return IsNaN2008bit; }
  bool isGP64bit() const { return IsGP64bit; }
  bool isGP32bit() const { return !IsGP64bit; }
  unsigned getGPRSizeInBytes() const { return isGP64bit() ? 8 : 4; }
  bool isPTR64bit() const { return IsPTR64bit; }
  bool isPTR32bit() const { return !IsPTR64bit; }
  bool hasSym32() const {
    return (HasSym32 && isABI_N64()) || isABI_N32() || isABI_O32();
  }
  bool isSingleFloat() const { return IsSingleFloat; }
  bool isTargetELF() const { return TargetTriple.isOSBinFormatELF(); }
  bool hasVFPU() const { return HasVFPU; }
  bool inM650216Mode() const { return InM650216Mode; }
  bool inM650216ModeDefault() const {
    return InM650216Mode;
  }
  // Hard float for mips16 means essentially to compile as soft float
  // but to use a runtime library for soft float that is written with
  // native mips32 floating point instructions (those runtime routines
  // run in mips32 hard float mode).
  bool inM650216HardFloat() const {
    return inM650216Mode() && InM650216HardFloat;
  }
  bool inMicroM6502Mode() const { return InMicroM6502Mode; }
  bool inMicroM650232r6Mode() const { return InMicroM6502Mode && hasM650232r6(); }
  bool inMicroM650264r6Mode() const { return InMicroM6502Mode && hasM650264r6(); }
  bool hasDSP() const { return HasDSP; }
  bool hasDSPR2() const { return HasDSPR2; }
  bool hasDSPR3() const { return HasDSPR3; }
  bool hasMSA() const { return HasMSA; }
  bool disableMadd4() const { return DisableMadd4; }
  bool hasEVA() const { return HasEVA; }
  bool hasMT() const { return HasMT; }
  bool useSmallSection() const { return UseSmallSection; }

  bool hasStandardEncoding() const { return !inM650216Mode(); }

  bool useSoftFloat() const { return IsSoftFloat; }

  bool useLongCalls() const { return UseLongCalls; }

  bool enableLongBranchPass() const {
    return hasStandardEncoding() || allowMixed16_32();
  }

  /// Features related to the presence of specific instructions.
  bool hasExtractInsert() const { return !inM650216Mode() && hasM650232r2(); }
  bool hasMTHC1() const { return hasM650232r2(); }

  bool allowMixed16_32() const { return inM650216ModeDefault() |
                                        AllowMixed16_32; }

  bool os16() const { return Os16; }

  bool isTargetNaCl() const { return TargetTriple.isOSNaCl(); }

  bool isXRaySupported() const override { return true; }

  // for now constant islands are on for the whole compilation unit but we only
  // really use them if in addition we are in mips16 mode
  static bool useConstantIslands();

  unsigned getStackAlignment() const { return stackAlignment; }

  // Grab relocation model
  Reloc::Model getRelocationModel() const;

  M6502Subtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                 const TargetMachine &TM);

  /// Does the system support unaligned memory access.
  ///
  /// M650232r6/M650264r6 require full unaligned access support but does not
  /// specify which component of the system provides it. Hardware, software, and
  /// hybrid implementations are all valid.
  bool systemSupportsUnalignedAccess() const { return hasM650232r6(); }

  // Set helper classes
  void setHelperClassesM650216();
  void setHelperClassesM6502SE();

  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }
  const M6502InstrInfo *getInstrInfo() const override { return InstrInfo.get(); }
  const TargetFrameLowering *getFrameLowering() const override {
    return FrameLowering.get();
  }
  const M6502RegisterInfo *getRegisterInfo() const override {
    return &InstrInfo->getRegisterInfo();
  }
  const M6502TargetLowering *getTargetLowering() const override {
    return TLInfo.get();
  }
  const InstrItineraryData *getInstrItineraryData() const override {
    return &InstrItins;
  }
};
} // End llvm namespace

#endif
