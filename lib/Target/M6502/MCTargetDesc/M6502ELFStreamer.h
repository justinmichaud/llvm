//===- M6502ELFStreamer.h - ELF Object Output --------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This is a custom MCELFStreamer which allows us to insert some hooks before
// emitting data into an actual object file.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502ELFSTREAMER_H
#define LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502ELFSTREAMER_H

#include "M6502OptionRecord.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCELFStreamer.h"
#include <memory>

namespace llvm {

class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCSubtargetInfo;

class M6502ELFStreamer : public MCELFStreamer {
  SmallVector<std::unique_ptr<M6502OptionRecord>, 8> M6502OptionRecords;
  M6502RegInfoRecord *RegInfoRecord;
  SmallVector<MCSymbol*, 4> Labels;

public:
  M6502ELFStreamer(MCContext &Context, std::unique_ptr<MCAsmBackend> MAB,
                  raw_pwrite_stream &OS,
                  std::unique_ptr<MCCodeEmitter> Emitter);

  /// Overriding this function allows us to add arbitrary behaviour before the
  /// \p Inst is actually emitted. For example, we can inspect the operands and
  /// gather sufficient information that allows us to reason about the register
  /// usage for the translation unit.
  void EmitInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                       bool = false) override;

  /// Overriding this function allows us to record all labels that should be
  /// marked as microM6502. Based on this data marking is done in
  /// EmitInstruction.
  void EmitLabel(MCSymbol *Symbol, SMLoc Loc = SMLoc()) override;

  /// Overriding this function allows us to dismiss all labels that are
  /// candidates for marking as microM6502 when .section directive is processed.
  void SwitchSection(MCSection *Section,
                     const MCExpr *Subsection = nullptr) override;

  /// Overriding this function allows us to dismiss all labels that are
  /// candidates for marking as microM6502 when .word directive is emitted.
  void EmitValueImpl(const MCExpr *Value, unsigned Size, SMLoc Loc) override;

  /// Emits all the option records stored up until the point it's called.
  void EmitM6502OptionRecords();

  /// Mark labels as microM6502, if necessary for the subtarget.
  void createPendingLabelRelocs();
};

MCELFStreamer *createM6502ELFStreamer(MCContext &Context,
                                     std::unique_ptr<MCAsmBackend> MAB,
                                     raw_pwrite_stream &OS,
                                     std::unique_ptr<MCCodeEmitter> Emitter,
                                     bool RelaxAll);
} // end namespace llvm

#endif // LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502ELFSTREAMER_H
