//===-------- M6502ELFStreamer.cpp - ELF Object Output ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "M6502ELFStreamer.h"
#include "M6502OptionRecord.h"
#include "M6502TargetStreamer.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

using namespace llvm;

M6502ELFStreamer::M6502ELFStreamer(MCContext &Context,
                                 std::unique_ptr<MCAsmBackend> MAB,
                                 raw_pwrite_stream &OS,
                                 std::unique_ptr<MCCodeEmitter> Emitter)
    : MCELFStreamer(Context, std::move(MAB), OS, std::move(Emitter)) {
  RegInfoRecord = new M6502RegInfoRecord(this, Context);
  M6502OptionRecords.push_back(
      std::unique_ptr<M6502RegInfoRecord>(RegInfoRecord));
}

void M6502ELFStreamer::EmitInstruction(const MCInst &Inst,
                                      const MCSubtargetInfo &STI, bool) {
  MCELFStreamer::EmitInstruction(Inst, STI);

  MCContext &Context = getContext();
  const MCRegisterInfo *MCRegInfo = Context.getRegisterInfo();

  for (unsigned OpIndex = 0; OpIndex < Inst.getNumOperands(); ++OpIndex) {
    const MCOperand &Op = Inst.getOperand(OpIndex);

    if (!Op.isReg())
      continue;

    unsigned Reg = Op.getReg();
    RegInfoRecord->SetPhysRegUsed(Reg, MCRegInfo);
  }

  createPendingLabelRelocs();
}

void M6502ELFStreamer::createPendingLabelRelocs() {
  M6502TargetELFStreamer *ELFTargetStreamer =
      static_cast<M6502TargetELFStreamer *>(getTargetStreamer());

  Labels.clear();
}

void M6502ELFStreamer::EmitLabel(MCSymbol *Symbol, SMLoc Loc) {
  MCELFStreamer::EmitLabel(Symbol);
  Labels.push_back(Symbol);
}

void M6502ELFStreamer::SwitchSection(MCSection *Section,
                                    const MCExpr *Subsection) {
  MCELFStreamer::SwitchSection(Section, Subsection);
  Labels.clear();
}

void M6502ELFStreamer::EmitValueImpl(const MCExpr *Value, unsigned Size,
                                    SMLoc Loc) {
  MCELFStreamer::EmitValueImpl(Value, Size, Loc);
  Labels.clear();
}

void M6502ELFStreamer::EmitM6502OptionRecords() {
  for (const auto &I : M6502OptionRecords)
    I->EmitM6502OptionRecord();
}

MCELFStreamer *llvm::createM6502ELFStreamer(
    MCContext &Context, std::unique_ptr<MCAsmBackend> MAB,
    raw_pwrite_stream &OS, std::unique_ptr<MCCodeEmitter> Emitter,
    bool RelaxAll) {
  return new M6502ELFStreamer(Context, std::move(MAB), OS, std::move(Emitter));
}
