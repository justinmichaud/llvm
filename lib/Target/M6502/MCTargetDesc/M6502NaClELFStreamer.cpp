//===-- M6502NaClELFStreamer.cpp - ELF Object Output for M6502 NaCl ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements MCELFStreamer for M6502 NaCl.  It emits .o object files
// as required by NaCl's SFI sandbox.  It inserts address-masking instructions
// before dangerous control-flow and memory access instructions.  It inserts
// address-masking instructions after instructions that change the stack
// pointer.  It ensures that the mask and the dangerous instruction are always
// emitted in the same bundle.  It aligns call + branch delay to the bundle end,
// so that return address is always aligned to the start of next bundle.
//
//===----------------------------------------------------------------------===//

#include "M6502.h"
#include "M6502ELFStreamer.h"
#include "M6502MCNaCl.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>

using namespace llvm;

#define DEBUG_TYPE "m6502-mc-nacl"

namespace {

const unsigned IndirectBranchMaskReg = M6502::T6;
const unsigned LoadStoreStackMaskReg = M6502::T7;

/// Extend the generic MCELFStreamer class so that it can mask dangerous
/// instructions.

class M6502NaClELFStreamer : public M6502ELFStreamer {
public:
  M6502NaClELFStreamer(MCContext &Context, std::unique_ptr<MCAsmBackend> TAB,
                      raw_pwrite_stream &OS,
                      std::unique_ptr<MCCodeEmitter> Emitter)
      : M6502ELFStreamer(Context, std::move(TAB), OS, std::move(Emitter)) {}

  ~M6502NaClELFStreamer() override = default;

private:
  // Whether we started the sandboxing sequence for calls.  Calls are bundled
  // with branch delays and aligned to the bundle end.
  bool PendingCall = false;

  bool isIndirectJump(const MCInst &MI) {
    if (MI.getOpcode() == M6502::JALR) {
      // M650232r6/M650264r6 doesn't have a JR instruction and uses JALR instead.
      // JALR is an indirect branch if the link register is $0.
      assert(MI.getOperand(0).isReg());
      return MI.getOperand(0).getReg() == M6502::ZERO;
    }
    return MI.getOpcode() == M6502::JR;
  }

  bool isStackPointerFirstOperand(const MCInst &MI) {
    return (MI.getNumOperands() > 0 && MI.getOperand(0).isReg()
            && MI.getOperand(0).getReg() == M6502::SP);
  }

  bool isCall(const MCInst &MI, bool *IsIndirectCall) {
    unsigned Opcode = MI.getOpcode();

    *IsIndirectCall = false;

    switch (Opcode) {
    default:
      return false;

    case M6502::JAL:
    case M6502::BAL:
    case M6502::BAL_BR:
    case M6502::BLTZAL:
    case M6502::BGEZAL:
      return true;

    case M6502::JALR:
      // JALR is only a call if the link register is not $0. Otherwise it's an
      // indirect branch.
      assert(MI.getOperand(0).isReg());
      if (MI.getOperand(0).getReg() == M6502::ZERO)
        return false;

      *IsIndirectCall = true;
      return true;
    }
  }

  void emitMask(unsigned AddrReg, unsigned MaskReg,
                const MCSubtargetInfo &STI) {
    MCInst MaskInst;
    MaskInst.setOpcode(M6502::AND);
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(AddrReg));
    MaskInst.addOperand(MCOperand::createReg(MaskReg));
    M6502ELFStreamer::EmitInstruction(MaskInst, STI);
  }

  // Sandbox indirect branch or return instruction by inserting mask operation
  // before it.
  void sandboxIndirectJump(const MCInst &MI, const MCSubtargetInfo &STI) {
    unsigned AddrReg = MI.getOperand(0).getReg();

    EmitBundleLock(false);
    emitMask(AddrReg, IndirectBranchMaskReg, STI);
    M6502ELFStreamer::EmitInstruction(MI, STI);
    EmitBundleUnlock();
  }

  // Sandbox memory access or SP change.  Insert mask operation before and/or
  // after the instruction.
  void sandboxLoadStoreStackChange(const MCInst &MI, unsigned AddrIdx,
                                   const MCSubtargetInfo &STI, bool MaskBefore,
                                   bool MaskAfter) {
    EmitBundleLock(false);
    if (MaskBefore) {
      // Sandbox memory access.
      unsigned BaseReg = MI.getOperand(AddrIdx).getReg();
      emitMask(BaseReg, LoadStoreStackMaskReg, STI);
    }
    M6502ELFStreamer::EmitInstruction(MI, STI);
    if (MaskAfter) {
      // Sandbox SP change.
      unsigned SPReg = MI.getOperand(0).getReg();
      assert((M6502::SP == SPReg) && "Unexpected stack-pointer register.");
      emitMask(SPReg, LoadStoreStackMaskReg, STI);
    }
    EmitBundleUnlock();
  }

public:
  /// This function is the one used to emit instruction data into the ELF
  /// streamer.  We override it to mask dangerous instructions.
  void EmitInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                       bool) override {
    // Sandbox indirect jumps.
    if (isIndirectJump(Inst)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");
      sandboxIndirectJump(Inst, STI);
      return;
    }

    // Sandbox loads, stores and SP changes.
    unsigned AddrIdx;
    bool IsStore;
    bool IsMemAccess = isBasePlusOffsetMemoryAccess(Inst.getOpcode(), &AddrIdx,
                                                    &IsStore);
    bool IsSPFirstOperand = isStackPointerFirstOperand(Inst);
    if (IsMemAccess || IsSPFirstOperand) {
      bool MaskBefore = (IsMemAccess
                         && baseRegNeedsLoadStoreMask(Inst.getOperand(AddrIdx)
                                                          .getReg()));
      bool MaskAfter = IsSPFirstOperand && !IsStore;
      if (MaskBefore || MaskAfter) {
        if (PendingCall)
          report_fatal_error("Dangerous instruction in branch delay slot!");
        sandboxLoadStoreStackChange(Inst, AddrIdx, STI, MaskBefore, MaskAfter);
        return;
      }
      // fallthrough
    }

    // Sandbox calls by aligning call and branch delay to the bundle end.
    // For indirect calls, emit the mask before the call.
    bool IsIndirectCall;
    if (isCall(Inst, &IsIndirectCall)) {
      if (PendingCall)
        report_fatal_error("Dangerous instruction in branch delay slot!");

      // Start the sandboxing sequence by emitting call.
      EmitBundleLock(true);
      if (IsIndirectCall) {
        unsigned TargetReg = Inst.getOperand(1).getReg();
        emitMask(TargetReg, IndirectBranchMaskReg, STI);
      }
      M6502ELFStreamer::EmitInstruction(Inst, STI);
      PendingCall = true;
      return;
    }
    if (PendingCall) {
      // Finish the sandboxing sequence by emitting branch delay.
      M6502ELFStreamer::EmitInstruction(Inst, STI);
      EmitBundleUnlock();
      PendingCall = false;
      return;
    }

    // None of the sandboxing applies, just emit the instruction.
    M6502ELFStreamer::EmitInstruction(Inst, STI);
  }
};

} // end anonymous namespace

