//===-- M6502TargetObjectFile.cpp - M6502 Object Files ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "M6502TargetObjectFile.h"
#include "M6502Subtarget.h"
#include "M6502TargetMachine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;

static cl::opt<unsigned>
SSThreshold("m6502-ssection-threshold", cl::Hidden,
            cl::desc("Small data and bss section threshold size (default=8)"),
            cl::init(8));

static cl::opt<bool>
LocalSData("m6502-mlocal-sdata", cl::Hidden,
           cl::desc("M6502: Use gp_rel for object-local data."),
           cl::init(true));

static cl::opt<bool>
ExternSData("m6502-mextern-sdata", cl::Hidden,
            cl::desc("M6502: Use gp_rel for data that is not defined by the "
                     "current object."),
            cl::init(true));

static cl::opt<bool>
EmbeddedData("m6502-membedded-data", cl::Hidden,
             cl::desc("M6502: Try to allocate variables in the following"
                      " sections if possible: .rodata, .sdata, .data ."),
             cl::init(false));

void M6502TargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM){
  TargetLoweringObjectFileELF::Initialize(Ctx, TM);
  InitializeELF(TM.Options.UseInitArray);

  SmallDataSection = getContext().getELFSection(
      ".sdata", ELF::SHT_PROGBITS,
      ELF::SHF_WRITE | ELF::SHF_ALLOC | ELF::SHF_MIPS_GPREL);

  SmallBSSSection = getContext().getELFSection(".sbss", ELF::SHT_NOBITS,
                                               ELF::SHF_WRITE | ELF::SHF_ALLOC |
                                                   ELF::SHF_MIPS_GPREL);
  this->TM = &static_cast<const M6502TargetMachine &>(TM);
}

// A address must be loaded from a small section if its size is less than the
// small section size threshold. Data in this section must be addressed using
// gp_rel operator.
static bool IsInSmallSection(uint64_t Size) {
  // gcc has traditionally not treated zero-sized objects as small data, so this
  // is effectively part of the ABI.
  return Size > 0 && Size <= SSThreshold;
}

/// Return true if this global address should be placed into small data/bss
/// section.
bool M6502TargetObjectFile::IsGlobalInSmallSection(
    const GlobalObject *GO, const TargetMachine &TM) const {
  // We first check the case where global is a declaration, because finding
  // section kind using getKindForGlobal() is only allowed for global
  // definitions.
  if (GO->isDeclaration() || GO->hasAvailableExternallyLinkage())
    return IsGlobalInSmallSectionImpl(GO, TM);

  return IsGlobalInSmallSection(GO, TM, getKindForGlobal(GO, TM));
}

/// Return true if this global address should be placed into small data/bss
/// section.
bool M6502TargetObjectFile::
IsGlobalInSmallSection(const GlobalObject *GO, const TargetMachine &TM,
                       SectionKind Kind) const {
  return IsGlobalInSmallSectionImpl(GO, TM) &&
         (Kind.isData() || Kind.isBSS() || Kind.isCommon() ||
          Kind.isReadOnly());
}

/// Return true if this global address should be placed into small data/bss
/// section. This method does all the work, except for checking the section
/// kind.
bool M6502TargetObjectFile::
IsGlobalInSmallSectionImpl(const GlobalObject *GO,
                           const TargetMachine &TM) const {
  const M6502Subtarget &Subtarget =
      *static_cast<const M6502TargetMachine &>(TM).getSubtargetImpl();

  // Return if small section is not available.
  if (!Subtarget.useSmallSection())
    return false;

  // Only global variables, not functions.
  const GlobalVariable *GVA = dyn_cast<GlobalVariable>(GO);
  if (!GVA)
    return false;

  // If the variable has an explicit section, it is placed in that section but
  // it's addressing mode may change.
  if (GVA->hasSection()) {
    StringRef Section = GVA->getSection();

    // Explicitly placing any variable in the small data section overrides
    // the global -G value.
    if (Section == ".sdata" || Section == ".sbss")
      return true;

    // Otherwise reject accessing it through the gp pointer. There are some
    // historic cases which GCC doesn't appear to respect any more. These
    // are .lit4, .lit8 and .srdata. For the moment reject these as well.
    return false;
  }

  // Enforce -mlocal-sdata.
  if (!LocalSData && GVA->hasLocalLinkage())
    return false;

  // Enforce -mextern-sdata.
  if (!ExternSData && ((GVA->hasExternalLinkage() && GVA->isDeclaration()) ||
                       GVA->hasCommonLinkage()))
    return false;

  // Enforce -membedded-data.
  if (EmbeddedData && GVA->isConstant())
    return false;

  Type *Ty = GVA->getValueType();
  return IsInSmallSection(
      GVA->getParent()->getDataLayout().getTypeAllocSize(Ty));
}

MCSection *M6502TargetObjectFile::SelectSectionForGlobal(
    const GlobalObject *GO, SectionKind Kind, const TargetMachine &TM) const {
  // TODO: Could also support "weak" symbols as well with ".gnu.linkonce.s.*"
  // sections?

  // Handle Small Section classification here.
  if (Kind.isBSS() && IsGlobalInSmallSection(GO, TM, Kind))
    return SmallBSSSection;
  if (Kind.isData() && IsGlobalInSmallSection(GO, TM, Kind))
    return SmallDataSection;
  if (Kind.isReadOnly() && IsGlobalInSmallSection(GO, TM, Kind))
    return SmallDataSection;

  // Otherwise, we work the same as ELF.
  return TargetLoweringObjectFileELF::SelectSectionForGlobal(GO, Kind, TM);
}

/// Return true if this constant should be placed into small data section.
bool M6502TargetObjectFile::IsConstantInSmallSection(
    const DataLayout &DL, const Constant *CN, const TargetMachine &TM) const {
  return (static_cast<const M6502TargetMachine &>(TM)
              .getSubtargetImpl()
              ->useSmallSection() &&
          LocalSData && IsInSmallSection(DL.getTypeAllocSize(CN->getType())));
}

/// Return true if this constant should be placed into small data section.
MCSection *M6502TargetObjectFile::getSectionForConstant(const DataLayout &DL,
                                                       SectionKind Kind,
                                                       const Constant *C,
                                                       unsigned &Align) const {
  if (IsConstantInSmallSection(DL, C, *TM))
    return SmallDataSection;

  // Otherwise, we work the same as ELF.
  return TargetLoweringObjectFileELF::getSectionForConstant(DL, Kind, C, Align);
}

const MCExpr *
M6502TargetObjectFile::getDebugThreadLocalSymbol(const MCSymbol *Sym) const {
  const MCExpr *Expr =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  return MCBinaryExpr::createAdd(
      Expr, MCConstantExpr::create(0x8000, getContext()), getContext());
}
