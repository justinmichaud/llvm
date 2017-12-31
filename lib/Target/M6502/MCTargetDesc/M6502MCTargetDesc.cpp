//===-- M6502MCTargetDesc.cpp - M6502 Target Descriptions -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides M6502 specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "M6502MCTargetDesc.h"
#include "InstPrinter/M6502InstPrinter.h"
#include "M6502AsmBackend.h"
#include "M6502ELFStreamer.h"
#include "M6502MCAsmInfo.h"
#include "M6502MCNaCl.h"
#include "M6502TargetStreamer.h"
#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MachineLocation.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "M6502GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "M6502GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "M6502GenRegisterInfo.inc"

/// Select the M6502 CPU for the given triple and cpu name.
/// FIXME: Merge with the copy in M6502Subtarget.cpp
StringRef M6502_MC::selectM6502CPU(const Triple &TT, StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    if (TT.getArch() == Triple::mips || TT.getArch() == Triple::mipsel)
      CPU = "mips32";
    else
      CPU = "mips64";
  }
  return CPU;
}

static MCInstrInfo *createM6502MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitM6502MCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createM6502MCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitM6502MCRegisterInfo(X, M6502::RA);
  return X;
}

static MCSubtargetInfo *createM6502MCSubtargetInfo(const Triple &TT,
                                                  StringRef CPU, StringRef FS) {
  CPU = M6502_MC::selectM6502CPU(TT, CPU);
  return createM6502MCSubtargetInfoImpl(TT, CPU, FS);
}

static MCAsmInfo *createM6502MCAsmInfo(const MCRegisterInfo &MRI,
                                      const Triple &TT) {
  MCAsmInfo *MAI = new M6502MCAsmInfo(TT);

  unsigned SP = MRI.getDwarfRegNum(M6502::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCInstPrinter *createM6502MCInstPrinter(const Triple &T,
                                              unsigned SyntaxVariant,
                                              const MCAsmInfo &MAI,
                                              const MCInstrInfo &MII,
                                              const MCRegisterInfo &MRI) {
  return new M6502InstPrinter(MAI, MII, MRI);
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    raw_pwrite_stream &OS,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  MCStreamer *S;
  if (!T.isOSNaCl())
    S = createM6502ELFStreamer(Context, std::move(MAB), OS, std::move(Emitter),
                              RelaxAll);
  else
    S = createM6502ELFStreamer(Context, std::move(MAB), OS,
                                  std::move(Emitter), RelaxAll);
  return S;
}

static MCTargetStreamer *createM6502AsmTargetStreamer(MCStreamer &S,
                                                     formatted_raw_ostream &OS,
                                                     MCInstPrinter *InstPrint,
                                                     bool isVerboseAsm) {
  return new M6502TargetAsmStreamer(S, OS);
}

static MCTargetStreamer *createM6502NullTargetStreamer(MCStreamer &S) {
  return new M6502TargetStreamer(S);
}

static MCTargetStreamer *
createM6502ObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new M6502TargetELFStreamer(S, STI);
}

namespace {

class M6502MCInstrAnalysis : public MCInstrAnalysis {
public:
  M6502MCInstrAnalysis(const MCInstrInfo *Info) : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override {
    unsigned NumOps = Inst.getNumOperands();
    if (NumOps == 0)
      return false;
    switch (Info->get(Inst.getOpcode()).OpInfo[NumOps - 1].OperandType) {
    case MCOI::OPERAND_UNKNOWN:
    case MCOI::OPERAND_IMMEDIATE:
      // jal, bal ...
      Target = Inst.getOperand(NumOps - 1).getImm();
      return true;
    case MCOI::OPERAND_PCREL:
      // b, j, beq ...
      Target = Addr + Inst.getOperand(NumOps - 1).getImm();
      return true;
    default:
      return false;
    }
  }
};
}

static MCInstrAnalysis *createM6502MCInstrAnalysis(const MCInstrInfo *Info) {
  return new M6502MCInstrAnalysis(Info);
}

extern "C" void LLVMInitializeM6502TargetMC() {
  for (Target *T : {&getTheM6502Target(), &getTheM6502elTarget(),
                    &getTheM650264Target(), &getTheM650264elTarget()}) {
    // Register the MC asm info.
    RegisterMCAsmInfoFn X(*T, createM6502MCAsmInfo);

    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(*T, createM6502MCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(*T, createM6502MCRegisterInfo);

    // Register the elf streamer.
    TargetRegistry::RegisterELFStreamer(*T, createMCStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createM6502AsmTargetStreamer);

    TargetRegistry::RegisterNullTargetStreamer(*T,
                                               createM6502NullTargetStreamer);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(*T, createM6502MCSubtargetInfo);

    // Register the MC instruction analyzer.
    TargetRegistry::RegisterMCInstrAnalysis(*T, createM6502MCInstrAnalysis);

    // Register the MCInstPrinter.
    TargetRegistry::RegisterMCInstPrinter(*T, createM6502MCInstPrinter);

    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createM6502ObjectTargetStreamer);

    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(*T, createM6502AsmBackend);
  }

  // Register the MC Code Emitter
  for (Target *T : {&getTheM6502Target(), &getTheM650264Target()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createM6502MCCodeEmitterEB);

  for (Target *T : {&getTheM6502elTarget(), &getTheM650264elTarget()})
    TargetRegistry::RegisterMCCodeEmitter(*T, createM6502MCCodeEmitterEL);
}
