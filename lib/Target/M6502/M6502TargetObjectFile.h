//===-- llvm/Target/M6502TargetObjectFile.h - M6502 Object Info ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502TARGETOBJECTFILE_H
#define LLVM_LIB_TARGET_M6502_M6502TARGETOBJECTFILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {
class M6502TargetMachine;
  class M6502TargetObjectFile : public TargetLoweringObjectFileELF {
    MCSection *SmallDataSection;
    MCSection *SmallBSSSection;
    const M6502TargetMachine *TM;

    bool IsGlobalInSmallSection(const GlobalObject *GO, const TargetMachine &TM,
                                SectionKind Kind) const;
    bool IsGlobalInSmallSectionImpl(const GlobalObject *GO,
                                    const TargetMachine &TM) const;
  public:

    void Initialize(MCContext &Ctx, const TargetMachine &TM) override;

    /// Return true if this global address should be placed into small data/bss
    /// section.
    bool IsGlobalInSmallSection(const GlobalObject *GO,
                                const TargetMachine &TM) const;

    MCSection *SelectSectionForGlobal(const GlobalObject *GO, SectionKind Kind,
                                      const TargetMachine &TM) const override;

    /// Return true if this constant should be placed into small data section.
    bool IsConstantInSmallSection(const DataLayout &DL, const Constant *CN,
                                  const TargetMachine &TM) const;

    MCSection *getSectionForConstant(const DataLayout &DL, SectionKind Kind,
                                     const Constant *C,
                                     unsigned &Align) const override;
    /// Describe a TLS variable address within debug info.
    const MCExpr *getDebugThreadLocalSymbol(const MCSymbol *Sym) const override;
  };
} // end namespace llvm

#endif
