//===-- M6502TargetStreamer.cpp - M6502 Target Streamer Methods -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides M6502 specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "M6502TargetStreamer.h"
#include "InstPrinter/M6502InstPrinter.h"
#include "MCTargetDesc/M6502ABIInfo.h"
#include "M6502ELFStreamer.h"
#include "M6502MCExpr.h"
#include "M6502MCTargetDesc.h"
#include "M6502TargetObjectFile.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

namespace {
static cl::opt<bool> RoundSectionSizes(
    "m6502-round-section-sizes", cl::init(false),
    cl::desc("Round section sizes up to the section alignment"), cl::Hidden);
} // end anonymous namespace

M6502TargetStreamer::M6502TargetStreamer(MCStreamer &S)
    : MCTargetStreamer(S), ModuleDirectiveAllowed(true) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;
}
void M6502TargetStreamer::emitDirectiveSetMicroM6502() {}
void M6502TargetStreamer::emitDirectiveSetNoMicroM6502() {}
void M6502TargetStreamer::setUsesMicroM6502() {}
void M6502TargetStreamer::emitDirectiveSetM650216() {}
void M6502TargetStreamer::emitDirectiveSetNoM650216() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetReorder() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetNoReorder() {}
void M6502TargetStreamer::emitDirectiveSetMacro() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetNoMacro() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetMsa() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetNoMsa() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetMt() {}
void M6502TargetStreamer::emitDirectiveSetNoMt() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetAt() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  forbidModuleDirective();
}
void M6502TargetStreamer::emitDirectiveSetNoAt() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveEnd(StringRef Name) {}
void M6502TargetStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {}
void M6502TargetStreamer::emitDirectiveAbiCalls() {}
void M6502TargetStreamer::emitDirectiveNaN2008() {}
void M6502TargetStreamer::emitDirectiveNaNLegacy() {}
void M6502TargetStreamer::emitDirectiveOptionPic0() {}
void M6502TargetStreamer::emitDirectiveOptionPic2() {}
void M6502TargetStreamer::emitDirectiveInsn() { forbidModuleDirective(); }
void M6502TargetStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                   unsigned ReturnReg) {}
void M6502TargetStreamer::emitMask(unsigned CPUBitmask, int CPUTopSavedRegOff) {}
void M6502TargetStreamer::emitFMask(unsigned FPUBitmask, int FPUTopSavedRegOff) {
}
void M6502TargetStreamer::emitDirectiveSetArch(StringRef Arch) {
  forbidModuleDirective();
}
void M6502TargetStreamer::emitDirectiveSetM65020() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM65021() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM65022() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM65023() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM65024() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM65025() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650232() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650232R2() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650232R3() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650232R5() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650232R6() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650264() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650264R2() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650264R3() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650264R5() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetM650264R6() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetPop() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetPush() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetSoftFloat() {
  forbidModuleDirective();
}
void M6502TargetStreamer::emitDirectiveSetHardFloat() {
  forbidModuleDirective();
}
void M6502TargetStreamer::emitDirectiveSetDsp() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetDspr2() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetNoDsp() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveCpLoad(unsigned RegNo) {}
bool M6502TargetStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  forbidModuleDirective();
  return true;
}
void M6502TargetStreamer::emitDirectiveCpsetup(unsigned RegNo, int RegOrOffset,
                                              const MCSymbol &Sym, bool IsReg) {
}
void M6502TargetStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                               bool SaveLocationIsRegister) {}

void M6502TargetStreamer::emitDirectiveModuleFP() {}

void M6502TargetStreamer::emitDirectiveModuleOddSPReg() {
  if (!ABIFlagsSection.OddSPReg && !ABIFlagsSection.Is32BitABI)
    report_fatal_error("+nooddspreg is only valid for O32");
}
void M6502TargetStreamer::emitDirectiveModuleSoftFloat() {}
void M6502TargetStreamer::emitDirectiveModuleHardFloat() {}
void M6502TargetStreamer::emitDirectiveModuleMT() {}
void M6502TargetStreamer::emitDirectiveSetFp(
    M6502ABIFlagsSection::FpABIKind Value) {
  forbidModuleDirective();
}
void M6502TargetStreamer::emitDirectiveSetOddSPReg() { forbidModuleDirective(); }
void M6502TargetStreamer::emitDirectiveSetNoOddSPReg() {
  forbidModuleDirective();
}

void M6502TargetStreamer::emitR(unsigned Opcode, unsigned Reg0, SMLoc IDLoc,
                               const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void M6502TargetStreamer::emitRX(unsigned Opcode, unsigned Reg0, MCOperand Op1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(Op1);
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void M6502TargetStreamer::emitRI(unsigned Opcode, unsigned Reg0, int32_t Imm,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createImm(Imm), IDLoc, STI);
}

void M6502TargetStreamer::emitRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRX(Opcode, Reg0, MCOperand::createReg(Reg1), IDLoc, STI);
}

void M6502TargetStreamer::emitII(unsigned Opcode, int16_t Imm1, int16_t Imm2,
                                SMLoc IDLoc, const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createImm(Imm1));
  TmpInst.addOperand(MCOperand::createImm(Imm2));
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void M6502TargetStreamer::emitRRX(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 MCOperand Op2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  MCInst TmpInst;
  TmpInst.setOpcode(Opcode);
  TmpInst.addOperand(MCOperand::createReg(Reg0));
  TmpInst.addOperand(MCOperand::createReg(Reg1));
  TmpInst.addOperand(Op2);
  TmpInst.setLoc(IDLoc);
  getStreamer().EmitInstruction(TmpInst, *STI);
}

void M6502TargetStreamer::emitRRR(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 unsigned Reg2, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createReg(Reg2), IDLoc, STI);
}

void M6502TargetStreamer::emitRRI(unsigned Opcode, unsigned Reg0, unsigned Reg1,
                                 int16_t Imm, SMLoc IDLoc,
                                 const MCSubtargetInfo *STI) {
  emitRRX(Opcode, Reg0, Reg1, MCOperand::createImm(Imm), IDLoc, STI);
}

void M6502TargetStreamer::emitAddu(unsigned DstReg, unsigned SrcReg,
                                  unsigned TrgReg, bool Is64Bit,
                                  const MCSubtargetInfo *STI) {
  emitRRR(Is64Bit ? M6502::DADDu : M6502::ADDu, DstReg, SrcReg, TrgReg, SMLoc(),
          STI);
}

void M6502TargetStreamer::emitDSLL(unsigned DstReg, unsigned SrcReg,
                                  int16_t ShiftAmount, SMLoc IDLoc,
                                  const MCSubtargetInfo *STI) {
  if (ShiftAmount >= 32) {
    emitRRI(M6502::DSLL32, DstReg, SrcReg, ShiftAmount - 32, IDLoc, STI);
    return;
  }

  emitRRI(M6502::DSLL, DstReg, SrcReg, ShiftAmount, IDLoc, STI);
}

void M6502TargetStreamer::emitEmptyDelaySlot(bool hasShortDelaySlot, SMLoc IDLoc,
                                            const MCSubtargetInfo *STI) {
  if (hasShortDelaySlot)
    emitRR(M6502::MOVE16_MM, M6502::ZERO, M6502::ZERO, IDLoc, STI);
  else
    emitRRI(M6502::SLL, M6502::ZERO, M6502::ZERO, 0, IDLoc, STI);
}

void M6502TargetStreamer::emitNop(SMLoc IDLoc, const MCSubtargetInfo *STI) {
  emitRRI(M6502::SLL, M6502::ZERO, M6502::ZERO, 0, IDLoc, STI);
}

/// Emit the $gp restore operation for .cprestore.
void M6502TargetStreamer::emitGPRestore(int Offset, SMLoc IDLoc,
                                       const MCSubtargetInfo *STI) {
  emitLoadWithImmOffset(M6502::LW, M6502::GP, M6502::SP, Offset, M6502::GP, IDLoc,
                        STI);
}

/// Emit a store instruction with an immediate offset.
void M6502TargetStreamer::emitStoreWithImmOffset(
    unsigned Opcode, unsigned SrcReg, unsigned BaseReg, int64_t Offset,
    function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, SrcReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // sw $8, offset($8) => lui $at, %hi(offset)
  //                      add $at, $at, $8
  //                      sw $8, %lo(offset)($at)

  unsigned ATReg = GetATReg();
  if (!ATReg)
    return;

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in ATReg.
  emitRI(M6502::LUi, ATReg, HiOffset, IDLoc, STI);
  if (BaseReg != M6502::ZERO)
    emitRRR(M6502::ADDu, ATReg, ATReg, BaseReg, IDLoc, STI);
  // Emit the store with the adjusted base and offset.
  emitRRI(Opcode, SrcReg, ATReg, LoOffset, IDLoc, STI);
}

/// Emit a store instruction with an symbol offset. Symbols are assumed to be
/// out of range for a simm16 will be expanded to appropriate instructions.
void M6502TargetStreamer::emitStoreWithSymOffset(
    unsigned Opcode, unsigned SrcReg, unsigned BaseReg, MCOperand &HiOperand,
    MCOperand &LoOperand, unsigned ATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  // sw $8, sym => lui $at, %hi(sym)
  //               sw $8, %lo(sym)($at)

  // Generate the base address in ATReg.
  emitRX(M6502::LUi, ATReg, HiOperand, IDLoc, STI);
  if (BaseReg != M6502::ZERO)
    emitRRR(M6502::ADDu, ATReg, ATReg, BaseReg, IDLoc, STI);
  // Emit the store with the adjusted base and offset.
  emitRRX(Opcode, SrcReg, ATReg, LoOperand, IDLoc, STI);
}

/// Emit a load instruction with an immediate offset. DstReg and TmpReg are
/// permitted to be the same register iff DstReg is distinct from BaseReg and
/// DstReg is a GPR. It is the callers responsibility to identify such cases
/// and pass the appropriate register in TmpReg.
void M6502TargetStreamer::emitLoadWithImmOffset(unsigned Opcode, unsigned DstReg,
                                               unsigned BaseReg, int64_t Offset,
                                               unsigned TmpReg, SMLoc IDLoc,
                                               const MCSubtargetInfo *STI) {
  if (isInt<16>(Offset)) {
    emitRRI(Opcode, DstReg, BaseReg, Offset, IDLoc, STI);
    return;
  }

  // 1) lw $8, offset($9) => lui $8, %hi(offset)
  //                         add $8, $8, $9
  //                         lw $8, %lo(offset)($9)
  // 2) lw $8, offset($8) => lui $at, %hi(offset)
  //                         add $at, $at, $8
  //                         lw $8, %lo(offset)($at)

  unsigned LoOffset = Offset & 0x0000ffff;
  unsigned HiOffset = (Offset & 0xffff0000) >> 16;

  // If msb of LoOffset is 1(negative number) we must increment HiOffset
  // to account for the sign-extension of the low part.
  if (LoOffset & 0x8000)
    HiOffset++;

  // Generate the base address in TmpReg.
  emitRI(M6502::LUi, TmpReg, HiOffset, IDLoc, STI);
  if (BaseReg != M6502::ZERO)
    emitRRR(M6502::ADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
  // Emit the load with the adjusted base and offset.
  emitRRI(Opcode, DstReg, TmpReg, LoOffset, IDLoc, STI);
}

/// Emit a load instruction with an symbol offset. Symbols are assumed to be
/// out of range for a simm16 will be expanded to appropriate instructions.
/// DstReg and TmpReg are permitted to be the same register iff DstReg is a
/// GPR. It is the callers responsibility to identify such cases and pass the
/// appropriate register in TmpReg.
void M6502TargetStreamer::emitLoadWithSymOffset(unsigned Opcode, unsigned DstReg,
                                               unsigned BaseReg,
                                               MCOperand &HiOperand,
                                               MCOperand &LoOperand,
                                               unsigned TmpReg, SMLoc IDLoc,
                                               const MCSubtargetInfo *STI) {
  // 1) lw $8, sym        => lui $8, %hi(sym)
  //                         lw $8, %lo(sym)($8)
  // 2) ldc1 $f0, sym     => lui $at, %hi(sym)
  //                         ldc1 $f0, %lo(sym)($at)

  // Generate the base address in TmpReg.
  emitRX(M6502::LUi, TmpReg, HiOperand, IDLoc, STI);
  if (BaseReg != M6502::ZERO)
    emitRRR(M6502::ADDu, TmpReg, TmpReg, BaseReg, IDLoc, STI);
  // Emit the load with the adjusted base and offset.
  emitRRX(Opcode, DstReg, TmpReg, LoOperand, IDLoc, STI);
}

M6502TargetAsmStreamer::M6502TargetAsmStreamer(MCStreamer &S,
                                             formatted_raw_ostream &OS)
    : M6502TargetStreamer(S), OS(OS) {}

void M6502TargetAsmStreamer::emitDirectiveSetMicroM6502() {
  OS << "\t.set\tmicrom6502\n";
  forbidModuleDirective();
}

void M6502TargetAsmStreamer::emitDirectiveSetNoMicroM6502() {
  OS << "\t.set\tnomicrom6502\n";
  forbidModuleDirective();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650216() {
  OS << "\t.set\tm650216\n";
  forbidModuleDirective();
}

void M6502TargetAsmStreamer::emitDirectiveSetNoM650216() {
  OS << "\t.set\tnom650216\n";
  M6502TargetStreamer::emitDirectiveSetNoM650216();
}

void M6502TargetAsmStreamer::emitDirectiveSetReorder() {
  OS << "\t.set\treorder\n";
  M6502TargetStreamer::emitDirectiveSetReorder();
}

void M6502TargetAsmStreamer::emitDirectiveSetNoReorder() {
  OS << "\t.set\tnoreorder\n";
  forbidModuleDirective();
}

void M6502TargetAsmStreamer::emitDirectiveSetMacro() {
  OS << "\t.set\tmacro\n";
  M6502TargetStreamer::emitDirectiveSetMacro();
}

void M6502TargetAsmStreamer::emitDirectiveSetNoMacro() {
  OS << "\t.set\tnomacro\n";
  M6502TargetStreamer::emitDirectiveSetNoMacro();
}

void M6502TargetAsmStreamer::emitDirectiveSetMsa() {
  OS << "\t.set\tmsa\n";
  M6502TargetStreamer::emitDirectiveSetMsa();
}

void M6502TargetAsmStreamer::emitDirectiveSetNoMsa() {
  OS << "\t.set\tnomsa\n";
  M6502TargetStreamer::emitDirectiveSetNoMsa();
}

void M6502TargetAsmStreamer::emitDirectiveSetMt() {
  OS << "\t.set\tmt\n";
  M6502TargetStreamer::emitDirectiveSetMt();
}

void M6502TargetAsmStreamer::emitDirectiveSetNoMt() {
  OS << "\t.set\tnomt\n";
  M6502TargetStreamer::emitDirectiveSetNoMt();
}

void M6502TargetAsmStreamer::emitDirectiveSetAt() {
  OS << "\t.set\tat\n";
  M6502TargetStreamer::emitDirectiveSetAt();
}

void M6502TargetAsmStreamer::emitDirectiveSetAtWithArg(unsigned RegNo) {
  OS << "\t.set\tat=$" << Twine(RegNo) << "\n";
  M6502TargetStreamer::emitDirectiveSetAtWithArg(RegNo);
}

void M6502TargetAsmStreamer::emitDirectiveSetNoAt() {
  OS << "\t.set\tnoat\n";
  M6502TargetStreamer::emitDirectiveSetNoAt();
}

void M6502TargetAsmStreamer::emitDirectiveEnd(StringRef Name) {
  OS << "\t.end\t" << Name << '\n';
}

void M6502TargetAsmStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  OS << "\t.ent\t" << Symbol.getName() << '\n';
}

void M6502TargetAsmStreamer::emitDirectiveAbiCalls() { OS << "\t.abicalls\n"; }

void M6502TargetAsmStreamer::emitDirectiveNaN2008() { OS << "\t.nan\t2008\n"; }

void M6502TargetAsmStreamer::emitDirectiveNaNLegacy() {
  OS << "\t.nan\tlegacy\n";
}

void M6502TargetAsmStreamer::emitDirectiveOptionPic0() {
  OS << "\t.option\tpic0\n";
}

void M6502TargetAsmStreamer::emitDirectiveOptionPic2() {
  OS << "\t.option\tpic2\n";
}

void M6502TargetAsmStreamer::emitDirectiveInsn() {
  M6502TargetStreamer::emitDirectiveInsn();
  OS << "\t.insn\n";
}

void M6502TargetAsmStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg) {
  OS << "\t.frame\t$"
     << StringRef(M6502InstPrinter::getRegisterName(StackReg)).lower() << ","
     << StackSize << ",$"
     << StringRef(M6502InstPrinter::getRegisterName(ReturnReg)).lower() << '\n';
}

void M6502TargetAsmStreamer::emitDirectiveSetArch(StringRef Arch) {
  OS << "\t.set arch=" << Arch << "\n";
  M6502TargetStreamer::emitDirectiveSetArch(Arch);
}

void M6502TargetAsmStreamer::emitDirectiveSetM65020() {
  OS << "\t.set\tm65020\n";
  M6502TargetStreamer::emitDirectiveSetM65020();
}

void M6502TargetAsmStreamer::emitDirectiveSetM65021() {
  OS << "\t.set\tm65021\n";
  M6502TargetStreamer::emitDirectiveSetM65021();
}

void M6502TargetAsmStreamer::emitDirectiveSetM65022() {
  OS << "\t.set\tm65022\n";
  M6502TargetStreamer::emitDirectiveSetM65022();
}

void M6502TargetAsmStreamer::emitDirectiveSetM65023() {
  OS << "\t.set\tm65023\n";
  M6502TargetStreamer::emitDirectiveSetM65023();
}

void M6502TargetAsmStreamer::emitDirectiveSetM65024() {
  OS << "\t.set\tm65024\n";
  M6502TargetStreamer::emitDirectiveSetM65024();
}

void M6502TargetAsmStreamer::emitDirectiveSetM65025() {
  OS << "\t.set\tm65025\n";
  M6502TargetStreamer::emitDirectiveSetM65025();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650232() {
  OS << "\t.set\tm650232\n";
  M6502TargetStreamer::emitDirectiveSetM650232();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650232R2() {
  OS << "\t.set\tm650232r2\n";
  M6502TargetStreamer::emitDirectiveSetM650232R2();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650232R3() {
  OS << "\t.set\tm650232r3\n";
  M6502TargetStreamer::emitDirectiveSetM650232R3();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650232R5() {
  OS << "\t.set\tm650232r5\n";
  M6502TargetStreamer::emitDirectiveSetM650232R5();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650232R6() {
  OS << "\t.set\tm650232r6\n";
  M6502TargetStreamer::emitDirectiveSetM650232R6();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650264() {
  OS << "\t.set\tm650264\n";
  M6502TargetStreamer::emitDirectiveSetM650264();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650264R2() {
  OS << "\t.set\tm650264r2\n";
  M6502TargetStreamer::emitDirectiveSetM650264R2();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650264R3() {
  OS << "\t.set\tm650264r3\n";
  M6502TargetStreamer::emitDirectiveSetM650264R3();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650264R5() {
  OS << "\t.set\tm650264r5\n";
  M6502TargetStreamer::emitDirectiveSetM650264R5();
}

void M6502TargetAsmStreamer::emitDirectiveSetM650264R6() {
  OS << "\t.set\tm650264r6\n";
  M6502TargetStreamer::emitDirectiveSetM650264R6();
}

void M6502TargetAsmStreamer::emitDirectiveSetDsp() {
  OS << "\t.set\tdsp\n";
  M6502TargetStreamer::emitDirectiveSetDsp();
}

void M6502TargetAsmStreamer::emitDirectiveSetDspr2() {
  OS << "\t.set\tdspr2\n";
  M6502TargetStreamer::emitDirectiveSetDspr2();
}

void M6502TargetAsmStreamer::emitDirectiveSetNoDsp() {
  OS << "\t.set\tnodsp\n";
  M6502TargetStreamer::emitDirectiveSetNoDsp();
}

void M6502TargetAsmStreamer::emitDirectiveSetPop() {
  OS << "\t.set\tpop\n";
  M6502TargetStreamer::emitDirectiveSetPop();
}

void M6502TargetAsmStreamer::emitDirectiveSetPush() {
 OS << "\t.set\tpush\n";
 M6502TargetStreamer::emitDirectiveSetPush();
}

void M6502TargetAsmStreamer::emitDirectiveSetSoftFloat() {
  OS << "\t.set\tsoftfloat\n";
  M6502TargetStreamer::emitDirectiveSetSoftFloat();
}

void M6502TargetAsmStreamer::emitDirectiveSetHardFloat() {
  OS << "\t.set\thardfloat\n";
  M6502TargetStreamer::emitDirectiveSetHardFloat();
}

// Print a 32 bit hex number with all numbers.
static void printHex32(unsigned Value, raw_ostream &OS) {
  OS << "0x";
  for (int i = 7; i >= 0; i--)
    OS.write_hex((Value & (0xF << (i * 4))) >> (i * 4));
}

void M6502TargetAsmStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  OS << "\t.mask \t";
  printHex32(CPUBitmask, OS);
  OS << ',' << CPUTopSavedRegOff << '\n';
}

void M6502TargetAsmStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  OS << "\t.fmask\t";
  printHex32(FPUBitmask, OS);
  OS << "," << FPUTopSavedRegOff << '\n';
}

void M6502TargetAsmStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  OS << "\t.cpload\t$"
     << StringRef(M6502InstPrinter::getRegisterName(RegNo)).lower() << "\n";
  forbidModuleDirective();
}

bool M6502TargetAsmStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  M6502TargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  OS << "\t.cprestore\t" << Offset << "\n";
  return true;
}

void M6502TargetAsmStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  OS << "\t.cpsetup\t$"
     << StringRef(M6502InstPrinter::getRegisterName(RegNo)).lower() << ", ";

  if (IsReg)
    OS << "$"
       << StringRef(M6502InstPrinter::getRegisterName(RegOrOffset)).lower();
  else
    OS << RegOrOffset;

  OS << ", ";

  OS << Sym.getName();
  forbidModuleDirective();
}

void M6502TargetAsmStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  OS << "\t.cpreturn";
  forbidModuleDirective();
}

void M6502TargetAsmStreamer::emitDirectiveModuleFP() {
  OS << "\t.module\tfp=";
  OS << ABIFlagsSection.getFpABIString(ABIFlagsSection.getFpABI()) << "\n";
}

void M6502TargetAsmStreamer::emitDirectiveSetFp(
    M6502ABIFlagsSection::FpABIKind Value) {
  M6502TargetStreamer::emitDirectiveSetFp(Value);

  OS << "\t.set\tfp=";
  OS << ABIFlagsSection.getFpABIString(Value) << "\n";
}

void M6502TargetAsmStreamer::emitDirectiveModuleOddSPReg() {
  M6502TargetStreamer::emitDirectiveModuleOddSPReg();

  OS << "\t.module\t" << (ABIFlagsSection.OddSPReg ? "" : "no") << "oddspreg\n";
}

void M6502TargetAsmStreamer::emitDirectiveSetOddSPReg() {
  M6502TargetStreamer::emitDirectiveSetOddSPReg();
  OS << "\t.set\toddspreg\n";
}

void M6502TargetAsmStreamer::emitDirectiveSetNoOddSPReg() {
  M6502TargetStreamer::emitDirectiveSetNoOddSPReg();
  OS << "\t.set\tnooddspreg\n";
}

void M6502TargetAsmStreamer::emitDirectiveModuleSoftFloat() {
  OS << "\t.module\tsoftfloat\n";
}

void M6502TargetAsmStreamer::emitDirectiveModuleHardFloat() {
  OS << "\t.module\thardfloat\n";
}

void M6502TargetAsmStreamer::emitDirectiveModuleMT() {
  OS << "\t.module\tmt\n";
}

// This part is for ELF object output.
M6502TargetELFStreamer::M6502TargetELFStreamer(MCStreamer &S,
                                             const MCSubtargetInfo &STI)
    : M6502TargetStreamer(S), MicroM6502Enabled(false), STI(STI) {
  MCAssembler &MCA = getStreamer().getAssembler();

  // It's possible that MCObjectFileInfo isn't fully initialized at this point
  // due to an initialization order problem where LLVMTargetMachine creates the
  // target streamer before TargetLoweringObjectFile calls
  // InitializeMCObjectFileInfo. There doesn't seem to be a single place that
  // covers all cases so this statement covers most cases and direct object
  // emission must call setPic() once MCObjectFileInfo has been initialized. The
  // cases we don't handle here are covered by M6502AsmPrinter.
  Pic = MCA.getContext().getObjectFileInfo()->isPositionIndependent();

  const FeatureBitset &Features = STI.getFeatureBits();

  // Set the header flags that we can in the constructor.
  // FIXME: This is a fairly terrible hack. We set the rest
  // of these in the destructor. The problem here is two-fold:
  //
  // a: Some of the eflags can be set/reset by directives.
  // b: There aren't any usage paths that initialize the ABI
  //    pointer until after we initialize either an assembler
  //    or the target machine.
  // We can fix this by making the target streamer construct
  // the ABI, but this is fraught with wide ranging dependency
  // issues as well.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  // FIXME: Fix a dependency issue by instantiating the ABI object to some
  // default based off the triple. The triple doesn't describe the target
  // fully, but any external user of the API that uses the MCTargetStreamer
  // would otherwise crash on assertion failure.

  ABI = M6502ABIInfo(M6502ABIInfo::O32());

  // Architecture
  if (Features[M6502::FeatureM650264r6])
    EFlags |= ELF::EF_MIPS_ARCH_64R6;
  else if (Features[M6502::FeatureM650264r2] ||
           Features[M6502::FeatureM650264r3] ||
           Features[M6502::FeatureM650264r5])
    EFlags |= ELF::EF_MIPS_ARCH_64R2;
  else if (Features[M6502::FeatureM650264])
    EFlags |= ELF::EF_MIPS_ARCH_64;
  else if (Features[M6502::FeatureM65025])
    EFlags |= ELF::EF_MIPS_ARCH_5;
  else if (Features[M6502::FeatureM65024])
    EFlags |= ELF::EF_MIPS_ARCH_4;
  else if (Features[M6502::FeatureM65023])
    EFlags |= ELF::EF_MIPS_ARCH_3;
  else if (Features[M6502::FeatureM650232r6])
    EFlags |= ELF::EF_MIPS_ARCH_32R6;
  else if (Features[M6502::FeatureM650232r2] ||
           Features[M6502::FeatureM650232r3] ||
           Features[M6502::FeatureM650232r5])
    EFlags |= ELF::EF_MIPS_ARCH_32R2;
  else if (Features[M6502::FeatureM650232])
    EFlags |= ELF::EF_MIPS_ARCH_32;
  else if (Features[M6502::FeatureM65022])
    EFlags |= ELF::EF_MIPS_ARCH_2;
  else
    EFlags |= ELF::EF_MIPS_ARCH_1;

  // Machine
  if (Features[M6502::FeatureCnM6502])
    EFlags |= ELF::EF_MIPS_MACH_OCTEON;

  // Other options.
  if (Features[M6502::FeatureNaN2008])
    EFlags |= ELF::EF_MIPS_NAN2008;

  MCA.setELFHeaderEFlags(EFlags);
}

void M6502TargetELFStreamer::emitLabel(MCSymbol *S) {
  auto *Symbol = cast<MCSymbolELF>(S);
  getStreamer().getAssembler().registerSymbol(*Symbol);
  uint8_t Type = Symbol->getType();
  if (Type != ELF::STT_FUNC)
    return;

  if (isMicroM6502Enabled())
    Symbol->setOther(ELF::STO_MIPS_MICROMIPS);
}

void M6502TargetELFStreamer::finish() {
  MCAssembler &MCA = getStreamer().getAssembler();
  const MCObjectFileInfo &OFI = *MCA.getContext().getObjectFileInfo();

  // .bss, .text and .data are always at least 16-byte aligned.
  MCSection &TextSection = *OFI.getTextSection();
  MCA.registerSection(TextSection);
  MCSection &DataSection = *OFI.getDataSection();
  MCA.registerSection(DataSection);
  MCSection &BSSSection = *OFI.getBSSSection();
  MCA.registerSection(BSSSection);

  TextSection.setAlignment(std::max(16u, TextSection.getAlignment()));
  DataSection.setAlignment(std::max(16u, DataSection.getAlignment()));
  BSSSection.setAlignment(std::max(16u, BSSSection.getAlignment()));

  if (RoundSectionSizes) {
    // Make sections sizes a multiple of the alignment. This is useful for
    // verifying the output of IAS against the output of other assemblers but
    // it's not necessary to produce a correct object and increases section
    // size.
    MCStreamer &OS = getStreamer();
    for (MCSection &S : MCA) {
      MCSectionELF &Section = static_cast<MCSectionELF &>(S);

      unsigned Alignment = Section.getAlignment();
      if (Alignment) {
        OS.SwitchSection(&Section);
        if (Section.UseCodeAlign())
          OS.EmitCodeAlignment(Alignment, Alignment);
        else
          OS.EmitValueToAlignment(Alignment, 0, 1, Alignment);
      }
    }
  }

  const FeatureBitset &Features = STI.getFeatureBits();

  // Update e_header flags. See the FIXME and comment above in
  // the constructor for a full rundown on this.
  unsigned EFlags = MCA.getELFHeaderEFlags();

  // ABI
  // N64 does not require any ABI bits.
  if (getABI().IsO32())
    EFlags |= ELF::EF_MIPS_ABI_O32;
  else if (getABI().IsN32())
    EFlags |= ELF::EF_MIPS_ABI2;

  if (Features[M6502::FeatureGP64Bit]) {
    if (getABI().IsO32())
      EFlags |= ELF::EF_MIPS_32BITMODE; /* Compatibility Mode */
  } else if (Features[M6502::FeatureM650264r2] || Features[M6502::FeatureM650264])
    EFlags |= ELF::EF_MIPS_32BITMODE;

  // -mplt is not implemented but we should act as if it was
  // given.
  if (!Features[M6502::FeatureNoABICalls])
    EFlags |= ELF::EF_MIPS_CPIC;

  if (Pic)
    EFlags |= ELF::EF_MIPS_PIC | ELF::EF_MIPS_CPIC;

  MCA.setELFHeaderEFlags(EFlags);

  // Emit all the option records.
  // At the moment we are only emitting .M6502.options (ODK_REGINFO) and
  // .reginfo.
  M6502ELFStreamer &MEF = static_cast<M6502ELFStreamer &>(Streamer);
  MEF.EmitM6502OptionRecords();

  emitM6502AbiFlags();
}

void M6502TargetELFStreamer::emitAssignment(MCSymbol *S, const MCExpr *Value) {
}

MCELFStreamer &M6502TargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void M6502TargetELFStreamer::emitDirectiveSetMicroM6502() {
  MicroM6502Enabled = true;
  forbidModuleDirective();
}

void M6502TargetELFStreamer::emitDirectiveSetNoMicroM6502() {
  MicroM6502Enabled = false;
  forbidModuleDirective();
}

void M6502TargetELFStreamer::setUsesMicroM6502() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_MICROMIPS;
  MCA.setELFHeaderEFlags(Flags);
}

void M6502TargetELFStreamer::emitDirectiveSetM650216() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_ARCH_ASE_M16;
  MCA.setELFHeaderEFlags(Flags);
  forbidModuleDirective();
}

void M6502TargetELFStreamer::emitDirectiveSetNoReorder() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_NOREORDER;
  MCA.setELFHeaderEFlags(Flags);
  forbidModuleDirective();
}

void M6502TargetELFStreamer::emitDirectiveEnd(StringRef Name) {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();

  MCSectionELF *Sec = Context.getELFSection(".pdr", ELF::SHT_PROGBITS, 0);

  MCSymbol *Sym = Context.getOrCreateSymbol(Name);
  const MCSymbolRefExpr *ExprRef =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Context);

  MCA.registerSection(*Sec);
  Sec->setAlignment(4);

  OS.PushSection();

  OS.SwitchSection(Sec);

  OS.EmitValueImpl(ExprRef, 4);

  OS.EmitIntValue(GPRInfoSet ? GPRBitMask : 0, 4); // reg_mask
  OS.EmitIntValue(GPRInfoSet ? GPROffset : 0, 4);  // reg_offset

  OS.EmitIntValue(FPRInfoSet ? FPRBitMask : 0, 4); // fpreg_mask
  OS.EmitIntValue(FPRInfoSet ? FPROffset : 0, 4);  // fpreg_offset

  OS.EmitIntValue(FrameInfoSet ? FrameOffset : 0, 4); // frame_offset
  OS.EmitIntValue(FrameInfoSet ? FrameReg : 0, 4);    // frame_reg
  OS.EmitIntValue(FrameInfoSet ? ReturnReg : 0, 4);   // return_reg

  // The .end directive marks the end of a procedure. Invalidate
  // the information gathered up until this point.
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  OS.PopSection();

  // .end also implicitly sets the size.
  MCSymbol *CurPCSym = Context.createTempSymbol();
  OS.EmitLabel(CurPCSym);
  const MCExpr *Size = MCBinaryExpr::createSub(
      MCSymbolRefExpr::create(CurPCSym, MCSymbolRefExpr::VK_None, Context),
      ExprRef, Context);

  // The ELFObjectWriter can determine the absolute size as it has access to
  // the layout information of the assembly file, so a size expression rather
  // than an absolute value is ok here.
  static_cast<MCSymbolELF *>(Sym)->setSize(Size);
}

void M6502TargetELFStreamer::emitDirectiveEnt(const MCSymbol &Symbol) {
  GPRInfoSet = FPRInfoSet = FrameInfoSet = false;

  // .ent also acts like an implicit '.type symbol, STT_FUNC'
  static_cast<const MCSymbolELF &>(Symbol).setType(ELF::STT_FUNC);
}

void M6502TargetELFStreamer::emitDirectiveAbiCalls() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_CPIC | ELF::EF_MIPS_PIC;
  MCA.setELFHeaderEFlags(Flags);
}

void M6502TargetELFStreamer::emitDirectiveNaN2008() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags |= ELF::EF_MIPS_NAN2008;
  MCA.setELFHeaderEFlags(Flags);
}

void M6502TargetELFStreamer::emitDirectiveNaNLegacy() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Flags &= ~ELF::EF_MIPS_NAN2008;
  MCA.setELFHeaderEFlags(Flags);
}

void M6502TargetELFStreamer::emitDirectiveOptionPic0() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  // This option overrides other PIC options like -KPIC.
  Pic = false;
  Flags &= ~ELF::EF_MIPS_PIC;
  MCA.setELFHeaderEFlags(Flags);
}

void M6502TargetELFStreamer::emitDirectiveOptionPic2() {
  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned Flags = MCA.getELFHeaderEFlags();
  Pic = true;
  // NOTE: We are following the GAS behaviour here which means the directive
  // 'pic2' also sets the CPIC bit in the ELF header. This is different from
  // what is stated in the SYSV ABI which consider the bits EF_MIPS_PIC and
  // EF_MIPS_CPIC to be mutually exclusive.
  Flags |= ELF::EF_MIPS_PIC | ELF::EF_MIPS_CPIC;
  MCA.setELFHeaderEFlags(Flags);
}

void M6502TargetELFStreamer::emitDirectiveInsn() {
  M6502TargetStreamer::emitDirectiveInsn();
  M6502ELFStreamer &MEF = static_cast<M6502ELFStreamer &>(Streamer);
  MEF.createPendingLabelRelocs();
}

void M6502TargetELFStreamer::emitFrame(unsigned StackReg, unsigned StackSize,
                                      unsigned ReturnReg_) {
  MCContext &Context = getStreamer().getAssembler().getContext();
  const MCRegisterInfo *RegInfo = Context.getRegisterInfo();

  FrameInfoSet = true;
  FrameReg = RegInfo->getEncodingValue(StackReg);
  FrameOffset = StackSize;
  ReturnReg = RegInfo->getEncodingValue(ReturnReg_);
}

void M6502TargetELFStreamer::emitMask(unsigned CPUBitmask,
                                     int CPUTopSavedRegOff) {
  GPRInfoSet = true;
  GPRBitMask = CPUBitmask;
  GPROffset = CPUTopSavedRegOff;
}

void M6502TargetELFStreamer::emitFMask(unsigned FPUBitmask,
                                      int FPUTopSavedRegOff) {
  FPRInfoSet = true;
  FPRBitMask = FPUBitmask;
  FPROffset = FPUTopSavedRegOff;
}

void M6502TargetELFStreamer::emitDirectiveCpLoad(unsigned RegNo) {
  // .cpload $reg
  // This directive expands to:
  // lui   $gp, %hi(_gp_disp)
  // addui $gp, $gp, %lo(_gp_disp)
  // addu  $gp, $gp, $reg
  // when support for position independent code is enabled.
  if (!Pic || (getABI().IsN32() || getABI().IsN64()))
    return;

  // There's a GNU extension controlled by -mno-shared that allows
  // locally-binding symbols to be accessed using absolute addresses.
  // This is currently not supported. When supported -mno-shared makes
  // .cpload expand to:
  //   lui     $gp, %hi(__gnu_local_gp)
  //   addiu   $gp, $gp, %lo(__gnu_local_gp)

  StringRef SymName("_gp_disp");
  MCAssembler &MCA = getStreamer().getAssembler();
  MCSymbol *GP_Disp = MCA.getContext().getOrCreateSymbol(SymName);
  MCA.registerSymbol(*GP_Disp);

  MCInst TmpInst;
  TmpInst.setOpcode(M6502::LUi);
  TmpInst.addOperand(MCOperand::createReg(M6502::GP));
  const MCExpr *HiSym = M6502MCExpr::create(
      M6502MCExpr::MEK_HI,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(HiSym));
  getStreamer().EmitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(M6502::ADDiu);
  TmpInst.addOperand(MCOperand::createReg(M6502::GP));
  TmpInst.addOperand(MCOperand::createReg(M6502::GP));
  const MCExpr *LoSym = M6502MCExpr::create(
      M6502MCExpr::MEK_LO,
      MCSymbolRefExpr::create("_gp_disp", MCSymbolRefExpr::VK_None,
                              MCA.getContext()),
      MCA.getContext());
  TmpInst.addOperand(MCOperand::createExpr(LoSym));
  getStreamer().EmitInstruction(TmpInst, STI);

  TmpInst.clear();

  TmpInst.setOpcode(M6502::ADDu);
  TmpInst.addOperand(MCOperand::createReg(M6502::GP));
  TmpInst.addOperand(MCOperand::createReg(M6502::GP));
  TmpInst.addOperand(MCOperand::createReg(RegNo));
  getStreamer().EmitInstruction(TmpInst, STI);

  forbidModuleDirective();
}

bool M6502TargetELFStreamer::emitDirectiveCpRestore(
    int Offset, function_ref<unsigned()> GetATReg, SMLoc IDLoc,
    const MCSubtargetInfo *STI) {
  M6502TargetStreamer::emitDirectiveCpRestore(Offset, GetATReg, IDLoc, STI);
  // .cprestore offset
  // When PIC mode is enabled and the O32 ABI is used, this directive expands
  // to:
  //    sw $gp, offset($sp)
  // and adds a corresponding LW after every JAL.

  // Note that .cprestore is ignored if used with the N32 and N64 ABIs or if it
  // is used in non-PIC mode.
  if (!Pic || (getABI().IsN32() || getABI().IsN64()))
    return true;

  // Store the $gp on the stack.
  emitStoreWithImmOffset(M6502::SW, M6502::GP, M6502::SP, Offset, GetATReg, IDLoc,
                         STI);
  return true;
}

void M6502TargetELFStreamer::emitDirectiveCpsetup(unsigned RegNo,
                                                 int RegOrOffset,
                                                 const MCSymbol &Sym,
                                                 bool IsReg) {
  // Only N32 and N64 emit anything for .cpsetup iff PIC is set.
  if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;

  forbidModuleDirective();

  MCAssembler &MCA = getStreamer().getAssembler();
  MCInst Inst;

  // Either store the old $gp in a register or on the stack
  if (IsReg) {
    // move $save, $gpreg
    emitRRR(M6502::OR64, RegOrOffset, M6502::GP, M6502::ZERO, SMLoc(), &STI);
  } else {
    // sd $gpreg, offset($sp)
    emitRRI(M6502::SD, M6502::GP, M6502::SP, RegOrOffset, SMLoc(), &STI);
  }

  if (getABI().IsN32()) {
    MCSymbol *GPSym = MCA.getContext().getOrCreateSymbol("__gnu_local_gp");
    const M6502MCExpr *HiExpr = M6502MCExpr::create(
        M6502MCExpr::MEK_HI, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
        MCA.getContext());
    const M6502MCExpr *LoExpr = M6502MCExpr::create(
        M6502MCExpr::MEK_LO, MCSymbolRefExpr::create(GPSym, MCA.getContext()),
        MCA.getContext());

    // lui $gp, %hi(__gnu_local_gp)
    emitRX(M6502::LUi, M6502::GP, MCOperand::createExpr(HiExpr), SMLoc(), &STI);

    // addiu  $gp, $gp, %lo(__gnu_local_gp)
    emitRRX(M6502::ADDiu, M6502::GP, M6502::GP, MCOperand::createExpr(LoExpr),
            SMLoc(), &STI);

    return;
  }

  const M6502MCExpr *HiExpr = M6502MCExpr::createGpOff(
      M6502MCExpr::MEK_HI, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
      MCA.getContext());
  const M6502MCExpr *LoExpr = M6502MCExpr::createGpOff(
      M6502MCExpr::MEK_LO, MCSymbolRefExpr::create(&Sym, MCA.getContext()),
      MCA.getContext());

  // lui $gp, %hi(%neg(%gp_rel(funcSym)))
  emitRX(M6502::LUi, M6502::GP, MCOperand::createExpr(HiExpr), SMLoc(), &STI);

  // addiu  $gp, $gp, %lo(%neg(%gp_rel(funcSym)))
  emitRRX(M6502::ADDiu, M6502::GP, M6502::GP, MCOperand::createExpr(LoExpr),
          SMLoc(), &STI);

  // daddu  $gp, $gp, $funcreg
  emitRRR(M6502::DADDu, M6502::GP, M6502::GP, RegNo, SMLoc(), &STI);
}

void M6502TargetELFStreamer::emitDirectiveCpreturn(unsigned SaveLocation,
                                                  bool SaveLocationIsRegister) {
  // Only N32 and N64 emit anything for .cpreturn iff PIC is set.
  if (!Pic || !(getABI().IsN32() || getABI().IsN64()))
    return;

  MCInst Inst;
  // Either restore the old $gp from a register or on the stack
  if (SaveLocationIsRegister) {
    Inst.setOpcode(M6502::OR);
    Inst.addOperand(MCOperand::createReg(M6502::GP));
    Inst.addOperand(MCOperand::createReg(SaveLocation));
    Inst.addOperand(MCOperand::createReg(M6502::ZERO));
  } else {
    Inst.setOpcode(M6502::LD);
    Inst.addOperand(MCOperand::createReg(M6502::GP));
    Inst.addOperand(MCOperand::createReg(M6502::SP));
    Inst.addOperand(MCOperand::createImm(SaveLocation));
  }
  getStreamer().EmitInstruction(Inst, STI);

  forbidModuleDirective();
}

void M6502TargetELFStreamer::emitM6502AbiFlags() {
  MCAssembler &MCA = getStreamer().getAssembler();
  MCContext &Context = MCA.getContext();
  MCStreamer &OS = getStreamer();
  MCSectionELF *Sec = Context.getELFSection(
      ".M6502.abiflags", ELF::SHT_MIPS_ABIFLAGS, ELF::SHF_ALLOC, 24, "");
  MCA.registerSection(*Sec);
  Sec->setAlignment(8);
  OS.SwitchSection(Sec);

  OS << ABIFlagsSection;
}
