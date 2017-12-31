//===-- M6502SEInstrInfo.cpp - M650232/64 Instruction Information -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M650232/64 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "M6502SEInstrInfo.h"
#include "InstPrinter/M6502InstPrinter.h"
#include "M6502AnalyzeImmediate.h"
#include "M6502MachineFunction.h"
#include "M6502TargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

M6502SEInstrInfo::M6502SEInstrInfo(const M6502Subtarget &STI)
    : M6502InstrInfo(STI, STI.isPositionIndependent() ? M6502::B : M6502::J),
      RI() {}

const M6502RegisterInfo &M6502SEInstrInfo::getRegisterInfo() const {
  return RI;
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned M6502SEInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                              int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == M6502::LW)   || (Opc == M6502::LD)   ||
      (Opc == M6502::LWC1) || (Opc == M6502::LDC1) || (Opc == M6502::LDC164)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }

  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned M6502SEInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  unsigned Opc = MI.getOpcode();

  if ((Opc == M6502::SW)   || (Opc == M6502::SD)   ||
      (Opc == M6502::SWC1) || (Opc == M6502::SDC1) || (Opc == M6502::SDC164)) {
    if ((MI.getOperand(1).isFI()) &&  // is a stack slot
        (MI.getOperand(2).isImm()) && // the imm is zero
        (isZeroImm(MI.getOperand(2)))) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

void M6502SEInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, unsigned DestReg,
                                  unsigned SrcReg, bool KillSrc) const {
  unsigned Opc = 0, ZeroReg = 0;
  bool isMicroM6502 = Subtarget.inMicroM6502Mode();

  if (M6502::GPR32RegClass.contains(DestReg)) { // Copy to CPU Reg.
    if (M6502::GPR32RegClass.contains(SrcReg)) {
      if (isMicroM6502)
        Opc = M6502::MOVE16_MM;
      else
        Opc = M6502::OR, ZeroReg = M6502::ZERO;
    } else if (M6502::CCRRegClass.contains(SrcReg))
      Opc = M6502::CFC1;
    else if (M6502::FGR32RegClass.contains(SrcReg))
      Opc = M6502::MFC1;
    else if (M6502::HI32RegClass.contains(SrcReg)) {
      Opc = isMicroM6502 ? M6502::MFHI16_MM : M6502::MFHI;
      SrcReg = 0;
    } else if (M6502::LO32RegClass.contains(SrcReg)) {
      Opc = isMicroM6502 ? M6502::MFLO16_MM : M6502::MFLO;
      SrcReg = 0;
    } else if (M6502::HI32DSPRegClass.contains(SrcReg))
      Opc = M6502::MFHI_DSP;
    else if (M6502::LO32DSPRegClass.contains(SrcReg))
      Opc = M6502::MFLO_DSP;
    else if (M6502::DSPCCRegClass.contains(SrcReg)) {
      BuildMI(MBB, I, DL, get(M6502::RDDSP), DestReg).addImm(1 << 4)
        .addReg(SrcReg, RegState::Implicit | getKillRegState(KillSrc));
      return;
    }
    else if (M6502::MSACtrlRegClass.contains(SrcReg))
      Opc = M6502::CFCMSA;
  }
  else if (M6502::GPR32RegClass.contains(SrcReg)) { // Copy from CPU Reg.
    if (M6502::CCRRegClass.contains(DestReg))
      Opc = M6502::CTC1;
    else if (M6502::FGR32RegClass.contains(DestReg))
      Opc = M6502::MTC1;
    else if (M6502::HI32RegClass.contains(DestReg))
      Opc = M6502::MTHI, DestReg = 0;
    else if (M6502::LO32RegClass.contains(DestReg))
      Opc = M6502::MTLO, DestReg = 0;
    else if (M6502::HI32DSPRegClass.contains(DestReg))
      Opc = M6502::MTHI_DSP;
    else if (M6502::LO32DSPRegClass.contains(DestReg))
      Opc = M6502::MTLO_DSP;
    else if (M6502::DSPCCRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(M6502::WRDSP))
        .addReg(SrcReg, getKillRegState(KillSrc)).addImm(1 << 4)
        .addReg(DestReg, RegState::ImplicitDefine);
      return;
    } else if (M6502::MSACtrlRegClass.contains(DestReg)) {
      BuildMI(MBB, I, DL, get(M6502::CTCMSA))
          .addReg(DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
      return;
    }
  }
  else if (M6502::FGR32RegClass.contains(DestReg, SrcReg))
    Opc = M6502::FMOV_S;
  else if (M6502::AFGR64RegClass.contains(DestReg, SrcReg))
    Opc = M6502::FMOV_D32;
  else if (M6502::FGR64RegClass.contains(DestReg, SrcReg))
    Opc = M6502::FMOV_D64;
  else if (M6502::GPR64RegClass.contains(DestReg)) { // Copy to CPU64 Reg.
    if (M6502::GPR64RegClass.contains(SrcReg))
      Opc = M6502::OR64, ZeroReg = M6502::ZERO_64;
    else if (M6502::HI64RegClass.contains(SrcReg))
      Opc = M6502::MFHI64, SrcReg = 0;
    else if (M6502::LO64RegClass.contains(SrcReg))
      Opc = M6502::MFLO64, SrcReg = 0;
    else if (M6502::FGR64RegClass.contains(SrcReg))
      Opc = M6502::DMFC1;
  }
  else if (M6502::GPR64RegClass.contains(SrcReg)) { // Copy from CPU64 Reg.
    if (M6502::HI64RegClass.contains(DestReg))
      Opc = M6502::MTHI64, DestReg = 0;
    else if (M6502::LO64RegClass.contains(DestReg))
      Opc = M6502::MTLO64, DestReg = 0;
    else if (M6502::FGR64RegClass.contains(DestReg))
      Opc = M6502::DMTC1;
  }
  else if (M6502::MSA128BRegClass.contains(DestReg)) { // Copy to MSA reg
    if (M6502::MSA128BRegClass.contains(SrcReg))
      Opc = M6502::MOVE_V;
  }

  assert(Opc && "Cannot copy registers");

  MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc));

  if (DestReg)
    MIB.addReg(DestReg, RegState::Define);

  if (SrcReg)
    MIB.addReg(SrcReg, getKillRegState(KillSrc));

  if (ZeroReg)
    MIB.addReg(ZeroReg);
}

void M6502SEInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                unsigned SrcReg, bool isKill, int FI,
                const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
                int64_t Offset) const {
  DebugLoc DL;
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);

  unsigned Opc = 0;

  if (M6502::GPR32RegClass.hasSubClassEq(RC))
    Opc = M6502::SW;
  else if (M6502::GPR64RegClass.hasSubClassEq(RC))
    Opc = M6502::SD;
  else if (M6502::ACC64RegClass.hasSubClassEq(RC))
    Opc = M6502::STORE_ACC64;
  else if (M6502::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = M6502::STORE_ACC64DSP;
  else if (M6502::ACC128RegClass.hasSubClassEq(RC))
    Opc = M6502::STORE_ACC128;
  else if (M6502::DSPCCRegClass.hasSubClassEq(RC))
    Opc = M6502::STORE_CCOND_DSP;
  else if (M6502::FGR32RegClass.hasSubClassEq(RC))
    Opc = M6502::SWC1;
  else if (M6502::AFGR64RegClass.hasSubClassEq(RC))
    Opc = M6502::SDC1;
  else if (M6502::FGR64RegClass.hasSubClassEq(RC))
    Opc = M6502::SDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = M6502::ST_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = M6502::ST_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = M6502::ST_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = M6502::ST_D;
  else if (M6502::LO32RegClass.hasSubClassEq(RC))
    Opc = M6502::SW;
  else if (M6502::LO64RegClass.hasSubClassEq(RC))
    Opc = M6502::SD;
  else if (M6502::HI32RegClass.hasSubClassEq(RC))
    Opc = M6502::SW;
  else if (M6502::HI64RegClass.hasSubClassEq(RC))
    Opc = M6502::SD;
  else if (M6502::DSPRRegClass.hasSubClassEq(RC))
    Opc = M6502::SWDSP;

  // Hi, Lo are normally caller save but they are callee save
  // for interrupt handling.
  const Function *Func = MBB.getParent()->getFunction();
  if (Func->hasFnAttribute("interrupt")) {
    if (M6502::HI32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(M6502::MFHI), M6502::K0);
      SrcReg = M6502::K0;
    } else if (M6502::HI64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(M6502::MFHI64), M6502::K0_64);
      SrcReg = M6502::K0_64;
    } else if (M6502::LO32RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(M6502::MFLO), M6502::K0);
      SrcReg = M6502::K0;
    } else if (M6502::LO64RegClass.hasSubClassEq(RC)) {
      BuildMI(MBB, I, DL, get(M6502::MFLO64), M6502::K0_64);
      SrcReg = M6502::K0_64;
    }
  }

  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill))
    .addFrameIndex(FI).addImm(Offset).addMemOperand(MMO);
}

void M6502SEInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 unsigned DestReg, int FI, const TargetRegisterClass *RC,
                 const TargetRegisterInfo *TRI, int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;

  const Function *Func = MBB.getParent()->getFunction();
  bool ReqIndirectLoad = Func->hasFnAttribute("interrupt") &&
                         (DestReg == M6502::LO0 || DestReg == M6502::LO0_64 ||
                          DestReg == M6502::HI0 || DestReg == M6502::HI0_64);

  if (M6502::GPR32RegClass.hasSubClassEq(RC))
    Opc = M6502::LW;
  else if (M6502::GPR64RegClass.hasSubClassEq(RC))
    Opc = M6502::LD;
  else if (M6502::ACC64RegClass.hasSubClassEq(RC))
    Opc = M6502::LOAD_ACC64;
  else if (M6502::ACC64DSPRegClass.hasSubClassEq(RC))
    Opc = M6502::LOAD_ACC64DSP;
  else if (M6502::ACC128RegClass.hasSubClassEq(RC))
    Opc = M6502::LOAD_ACC128;
  else if (M6502::DSPCCRegClass.hasSubClassEq(RC))
    Opc = M6502::LOAD_CCOND_DSP;
  else if (M6502::FGR32RegClass.hasSubClassEq(RC))
    Opc = M6502::LWC1;
  else if (M6502::AFGR64RegClass.hasSubClassEq(RC))
    Opc = M6502::LDC1;
  else if (M6502::FGR64RegClass.hasSubClassEq(RC))
    Opc = M6502::LDC164;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v16i8))
    Opc = M6502::LD_B;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v8i16) ||
           TRI->isTypeLegalForClass(*RC, MVT::v8f16))
    Opc = M6502::LD_H;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v4i32) ||
           TRI->isTypeLegalForClass(*RC, MVT::v4f32))
    Opc = M6502::LD_W;
  else if (TRI->isTypeLegalForClass(*RC, MVT::v2i64) ||
           TRI->isTypeLegalForClass(*RC, MVT::v2f64))
    Opc = M6502::LD_D;
  else if (M6502::HI32RegClass.hasSubClassEq(RC))
    Opc = M6502::LW;
  else if (M6502::HI64RegClass.hasSubClassEq(RC))
    Opc = M6502::LD;
  else if (M6502::LO32RegClass.hasSubClassEq(RC))
    Opc = M6502::LW;
  else if (M6502::LO64RegClass.hasSubClassEq(RC))
    Opc = M6502::LD;
  else if (M6502::DSPRRegClass.hasSubClassEq(RC))
    Opc = M6502::LWDSP;

  assert(Opc && "Register class not handled!");

  if (!ReqIndirectLoad)
    BuildMI(MBB, I, DL, get(Opc), DestReg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
  else {
    // Load HI/LO through K0. Notably the DestReg is encoded into the
    // instruction itself.
    unsigned Reg = M6502::K0;
    unsigned LdOp = M6502::MTLO;
    if (DestReg == M6502::HI0)
      LdOp = M6502::MTHI;

    if (Subtarget.getABI().ArePtrs64bit()) {
      Reg = M6502::K0_64;
      if (DestReg == M6502::HI0_64)
        LdOp = M6502::MTHI64;
      else
        LdOp = M6502::MTLO64;
    }

    BuildMI(MBB, I, DL, get(Opc), Reg)
        .addFrameIndex(FI)
        .addImm(Offset)
        .addMemOperand(MMO);
    BuildMI(MBB, I, DL, get(LdOp)).addReg(Reg);
  }
}

bool M6502SEInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  bool isMicroM6502 = Subtarget.inMicroM6502Mode();
  unsigned Opc;

  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case M6502::RetRA:
    expandRetRA(MBB, MI);
    break;
  case M6502::ERet:
    expandERet(MBB, MI);
    break;
  case M6502::PseudoMFHI:
    Opc = isMicroM6502 ? M6502::MFHI16_MM : M6502::MFHI;
    expandPseudoMFHiLo(MBB, MI, Opc);
    break;
  case M6502::PseudoMFLO:
    Opc = isMicroM6502 ? M6502::MFLO16_MM : M6502::MFLO;
    expandPseudoMFHiLo(MBB, MI, Opc);
    break;
  case M6502::PseudoMFHI64:
    expandPseudoMFHiLo(MBB, MI, M6502::MFHI64);
    break;
  case M6502::PseudoMFLO64:
    expandPseudoMFHiLo(MBB, MI, M6502::MFLO64);
    break;
  case M6502::PseudoMTLOHI:
    expandPseudoMTLoHi(MBB, MI, M6502::MTLO, M6502::MTHI, false);
    break;
  case M6502::PseudoMTLOHI64:
    expandPseudoMTLoHi(MBB, MI, M6502::MTLO64, M6502::MTHI64, false);
    break;
  case M6502::PseudoMTLOHI_DSP:
    expandPseudoMTLoHi(MBB, MI, M6502::MTLO_DSP, M6502::MTHI_DSP, true);
    break;
  case M6502::PseudoCVT_S_W:
    expandCvtFPInt(MBB, MI, M6502::CVT_S_W, M6502::MTC1, false);
    break;
  case M6502::PseudoCVT_D32_W:
    expandCvtFPInt(MBB, MI, M6502::CVT_D32_W, M6502::MTC1, false);
    break;
  case M6502::PseudoCVT_S_L:
    expandCvtFPInt(MBB, MI, M6502::CVT_S_L, M6502::DMTC1, true);
    break;
  case M6502::PseudoCVT_D64_W:
    expandCvtFPInt(MBB, MI, M6502::CVT_D64_W, M6502::MTC1, true);
    break;
  case M6502::PseudoCVT_D64_L:
    expandCvtFPInt(MBB, MI, M6502::CVT_D64_L, M6502::DMTC1, true);
    break;
  case M6502::BuildPairF64:
    expandBuildPairF64(MBB, MI, false);
    break;
  case M6502::BuildPairF64_64:
    expandBuildPairF64(MBB, MI, true);
    break;
  case M6502::ExtractElementF64:
    expandExtractElementF64(MBB, MI, false);
    break;
  case M6502::ExtractElementF64_64:
    expandExtractElementF64(MBB, MI, true);
    break;
  case M6502::M6502eh_return32:
  case M6502::M6502eh_return64:
    expandEhReturn(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}

/// getOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned M6502SEInstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:           llvm_unreachable("Illegal opcode!");
  case M6502::BEQ:    return M6502::BNE;
  case M6502::BEQ_MM: return M6502::BNE_MM;
  case M6502::BNE:    return M6502::BEQ;
  case M6502::BNE_MM: return M6502::BEQ_MM;
  case M6502::BGTZ:   return M6502::BLEZ;
  case M6502::BGEZ:   return M6502::BLTZ;
  case M6502::BLTZ:   return M6502::BGEZ;
  case M6502::BLEZ:   return M6502::BGTZ;
  case M6502::BEQ64:  return M6502::BNE64;
  case M6502::BNE64:  return M6502::BEQ64;
  case M6502::BGTZ64: return M6502::BLEZ64;
  case M6502::BGEZ64: return M6502::BLTZ64;
  case M6502::BLTZ64: return M6502::BGEZ64;
  case M6502::BLEZ64: return M6502::BGTZ64;
  case M6502::BC1T:   return M6502::BC1F;
  case M6502::BC1F:   return M6502::BC1T;
  case M6502::BEQZC_MM: return M6502::BNEZC_MM;
  case M6502::BNEZC_MM: return M6502::BEQZC_MM;
  case M6502::BEQZC:  return M6502::BNEZC;
  case M6502::BNEZC:  return M6502::BEQZC;
  case M6502::BEQC:   return M6502::BNEC;
  case M6502::BNEC:   return M6502::BEQC;
  case M6502::BGTZC:  return M6502::BLEZC;
  case M6502::BGEZC:  return M6502::BLTZC;
  case M6502::BLTZC:  return M6502::BGEZC;
  case M6502::BLEZC:  return M6502::BGTZC;
  case M6502::BEQZC64:  return M6502::BNEZC64;
  case M6502::BNEZC64:  return M6502::BEQZC64;
  case M6502::BEQC64:   return M6502::BNEC64;
  case M6502::BNEC64:   return M6502::BEQC64;
  case M6502::BGEC64:   return M6502::BLTC64;
  case M6502::BGEUC64:  return M6502::BLTUC64;
  case M6502::BLTC64:   return M6502::BGEC64;
  case M6502::BLTUC64:  return M6502::BGEUC64;
  case M6502::BGTZC64:  return M6502::BLEZC64;
  case M6502::BGEZC64:  return M6502::BLTZC64;
  case M6502::BLTZC64:  return M6502::BGEZC64;
  case M6502::BLEZC64:  return M6502::BGTZC64;
  case M6502::BBIT0:  return M6502::BBIT1;
  case M6502::BBIT1:  return M6502::BBIT0;
  case M6502::BBIT032:  return M6502::BBIT132;
  case M6502::BBIT132:  return M6502::BBIT032;
  }
}

/// Adjust SP by Amount bytes.
void M6502SEInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  M6502ABIInfo ABI = Subtarget.getABI();
  DebugLoc DL;
  unsigned ADDiu = ABI.GetPtrAddiuOp();

  if (Amount == 0)
    return;

  if (isInt<16>(Amount)) {
    // addi sp, sp, amount
    BuildMI(MBB, I, DL, get(ADDiu), SP).addReg(SP).addImm(Amount);
  } else {
    // For numbers which are not 16bit integers we synthesize Amount inline
    // then add or subtract it from sp.
    unsigned Opc = ABI.GetPtrAdduOp();
    if (Amount < 0) {
      Opc = ABI.GetPtrSubuOp();
      Amount = -Amount;
    }
    unsigned Reg = loadImmediate(Amount, MBB, I, DL, nullptr);
    BuildMI(MBB, I, DL, get(Opc), SP).addReg(SP).addReg(Reg, RegState::Kill);
  }
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned M6502SEInstrInfo::loadImmediate(int64_t Imm, MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator II,
                                        const DebugLoc &DL,
                                        unsigned *NewImm) const {
  M6502AnalyzeImmediate AnalyzeImm;
  const M6502Subtarget &STI = Subtarget;
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  unsigned Size = STI.isABI_N64() ? 64 : 32;
  unsigned LUi = STI.isABI_N64() ? M6502::LUi64 : M6502::LUi;
  unsigned ZEROReg = STI.isABI_N64() ? M6502::ZERO_64 : M6502::ZERO;
  const TargetRegisterClass *RC = STI.isABI_N64() ?
    &M6502::GPR64RegClass : &M6502::GPR32RegClass;
  bool LastInstrIsADDiu = NewImm;

  const M6502AnalyzeImmediate::InstSeq &Seq =
    AnalyzeImm.Analyze(Imm, Size, LastInstrIsADDiu);
  M6502AnalyzeImmediate::InstSeq::const_iterator Inst = Seq.begin();

  assert(Seq.size() && (!LastInstrIsADDiu || (Seq.size() > 1)));

  // The first instruction can be a LUi, which is different from other
  // instructions (ADDiu, ORI and SLL) in that it does not have a register
  // operand.
  unsigned Reg = RegInfo.createVirtualRegister(RC);

  if (Inst->Opc == LUi)
    BuildMI(MBB, II, DL, get(LUi), Reg).addImm(SignExtend64<16>(Inst->ImmOpnd));
  else
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(ZEROReg)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  // Build the remaining instructions in Seq.
  for (++Inst; Inst != Seq.end() - LastInstrIsADDiu; ++Inst)
    BuildMI(MBB, II, DL, get(Inst->Opc), Reg).addReg(Reg, RegState::Kill)
      .addImm(SignExtend64<16>(Inst->ImmOpnd));

  if (LastInstrIsADDiu)
    *NewImm = Inst->ImmOpnd;

  return Reg;
}

unsigned M6502SEInstrInfo::getAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == M6502::BEQ    || Opc == M6502::BEQ_MM || Opc == M6502::BNE    ||
          Opc == M6502::BNE_MM || Opc == M6502::BGTZ   || Opc == M6502::BGEZ   ||
          Opc == M6502::BLTZ   || Opc == M6502::BLEZ   || Opc == M6502::BEQ64  ||
          Opc == M6502::BNE64  || Opc == M6502::BGTZ64 || Opc == M6502::BGEZ64 ||
          Opc == M6502::BLTZ64 || Opc == M6502::BLEZ64 || Opc == M6502::BC1T   ||
          Opc == M6502::BC1F   || Opc == M6502::B      || Opc == M6502::J      ||
          Opc == M6502::BEQZC_MM || Opc == M6502::BNEZC_MM || Opc == M6502::BEQC ||
          Opc == M6502::BNEC   || Opc == M6502::BLTC   || Opc == M6502::BGEC   ||
          Opc == M6502::BLTUC  || Opc == M6502::BGEUC  || Opc == M6502::BGTZC  ||
          Opc == M6502::BLEZC  || Opc == M6502::BGEZC  || Opc == M6502::BLTZC  ||
          Opc == M6502::BEQZC  || Opc == M6502::BNEZC  || Opc == M6502::BEQZC64 ||
          Opc == M6502::BNEZC64 || Opc == M6502::BEQC64 || Opc == M6502::BNEC64 ||
          Opc == M6502::BGEC64 || Opc == M6502::BGEUC64 || Opc == M6502::BLTC64 ||
          Opc == M6502::BLTUC64 || Opc == M6502::BGTZC64 ||
          Opc == M6502::BGEZC64 || Opc == M6502::BLTZC64 ||
          Opc == M6502::BLEZC64 || Opc == M6502::BC || Opc == M6502::BBIT0 ||
          Opc == M6502::BBIT1 || Opc == M6502::BBIT032 ||
          Opc == M6502::BBIT132) ? Opc : 0;
}

void M6502SEInstrInfo::expandRetRA(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I) const {

  MachineInstrBuilder MIB;
  if (Subtarget.isGP64bit())
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(M6502::PseudoReturn64))
              .addReg(M6502::RA_64, RegState::Undef);
  else
    MIB = BuildMI(MBB, I, I->getDebugLoc(), get(M6502::PseudoReturn))
              .addReg(M6502::RA, RegState::Undef);

  // Retain any imp-use flags.
  for (auto & MO : I->operands()) {
    if (MO.isImplicit())
      MIB.add(MO);
  }
}

void M6502SEInstrInfo::expandERet(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(M6502::ERET));
}

std::pair<bool, bool>
M6502SEInstrInfo::compareOpndSize(unsigned Opc,
                                 const MachineFunction &MF) const {
  const MCInstrDesc &Desc = get(Opc);
  assert(Desc.NumOperands == 2 && "Unary instruction expected.");
  const M6502RegisterInfo *RI = &getRegisterInfo();
  unsigned DstRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 0, RI, MF));
  unsigned SrcRegSize = RI->getRegSizeInBits(*getRegClass(Desc, 1, RI, MF));

  return std::make_pair(DstRegSize > SrcRegSize, DstRegSize < SrcRegSize);
}

void M6502SEInstrInfo::expandPseudoMFHiLo(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned NewOpc) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(NewOpc), I->getOperand(0).getReg());
}

void M6502SEInstrInfo::expandPseudoMTLoHi(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned LoOpc,
                                         unsigned HiOpc,
                                         bool HasExplicitDef) const {
  // Expand
  //  lo_hi pseudomtlohi $gpr0, $gpr1
  // to these two instructions:
  //  mtlo $gpr0
  //  mthi $gpr1

  DebugLoc DL = I->getDebugLoc();
  const MachineOperand &SrcLo = I->getOperand(1), &SrcHi = I->getOperand(2);
  MachineInstrBuilder LoInst = BuildMI(MBB, I, DL, get(LoOpc));
  MachineInstrBuilder HiInst = BuildMI(MBB, I, DL, get(HiOpc));

  // Add lo/hi registers if the mtlo/hi instructions created have explicit
  // def registers.
  if (HasExplicitDef) {
    unsigned DstReg = I->getOperand(0).getReg();
    unsigned DstLo = getRegisterInfo().getSubReg(DstReg, M6502::sub_lo);
    unsigned DstHi = getRegisterInfo().getSubReg(DstReg, M6502::sub_hi);
    LoInst.addReg(DstLo, RegState::Define);
    HiInst.addReg(DstHi, RegState::Define);
  }

  LoInst.addReg(SrcLo.getReg(), getKillRegState(SrcLo.isKill()));
  HiInst.addReg(SrcHi.getReg(), getKillRegState(SrcHi.isKill()));
}

void M6502SEInstrInfo::expandCvtFPInt(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I,
                                     unsigned CvtOpc, unsigned MovOpc,
                                     bool IsI64) const {
  const MCInstrDesc &CvtDesc = get(CvtOpc), &MovDesc = get(MovOpc);
  const MachineOperand &Dst = I->getOperand(0), &Src = I->getOperand(1);
  unsigned DstReg = Dst.getReg(), SrcReg = Src.getReg(), TmpReg = DstReg;
  unsigned KillSrc =  getKillRegState(Src.isKill());
  DebugLoc DL = I->getDebugLoc();
  bool DstIsLarger, SrcIsLarger;

  std::tie(DstIsLarger, SrcIsLarger) =
      compareOpndSize(CvtOpc, *MBB.getParent());

  if (DstIsLarger)
    TmpReg = getRegisterInfo().getSubReg(DstReg, M6502::sub_lo);

  if (SrcIsLarger)
    DstReg = getRegisterInfo().getSubReg(DstReg, M6502::sub_lo);

  BuildMI(MBB, I, DL, MovDesc, TmpReg).addReg(SrcReg, KillSrc);
  BuildMI(MBB, I, DL, CvtDesc, DstReg).addReg(TmpReg, RegState::Kill);
}

void M6502SEInstrInfo::expandExtractElementF64(MachineBasicBlock &MBB,
                                              MachineBasicBlock::iterator I,
                                              bool FP64) const {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned SrcReg = I->getOperand(1).getReg();
  unsigned N = I->getOperand(2).getImm();
  DebugLoc dl = I->getDebugLoc();

  assert(N < 2 && "Invalid immediate");
  unsigned SubIdx = N ? M6502::sub_hi : M6502::sub_lo;
  unsigned SubReg = getRegisterInfo().getSubReg(SrcReg, SubIdx);

  // FPXX on M6502-II or M650232r1 should have been handled with a spill/reload
  // in M6502SEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasM650232r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in M6502SEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  if (SubIdx == M6502::sub_hi && Subtarget.hasMTHC1()) {
    // FIXME: Strictly speaking MFHC1 only reads the top 32-bits however, we
    //        claim to read the whole 64-bits as part of a white lie used to
    //        temporarily work around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MFHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl, get(FP64 ? M6502::MFHC1_D64 : M6502::MFHC1_D32), DstReg)
        .addReg(SrcReg);
  } else
    BuildMI(MBB, I, dl, get(M6502::MFC1), DstReg).addReg(SubReg);
}

void M6502SEInstrInfo::expandBuildPairF64(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         bool FP64) const {
  unsigned DstReg = I->getOperand(0).getReg();
  unsigned LoReg = I->getOperand(1).getReg(), HiReg = I->getOperand(2).getReg();
  const MCInstrDesc& Mtc1Tdd = get(M6502::MTC1);
  DebugLoc dl = I->getDebugLoc();
  const TargetRegisterInfo &TRI = getRegisterInfo();

  // When mthc1 is available, use:
  //   mtc1 Lo, $fp
  //   mthc1 Hi, $fp
  //
  // Otherwise, for O32 FPXX ABI:
  //   spill + reload via ldc1
  // This case is handled by the frame lowering code.
  //
  // Otherwise, for FP32:
  //   mtc1 Lo, $fp
  //   mtc1 Hi, $fp + 1
  //
  // The case where dmtc1 is available doesn't need to be handled here
  // because it never creates a BuildPairF64 node.

  // FPXX on M6502-II or M650232r1 should have been handled with a spill/reload
  // in M6502SEFrameLowering.cpp.
  assert(!(Subtarget.isABI_FPXX() && !Subtarget.hasM650232r2()));

  // FP64A (FP64 with nooddspreg) should have been handled with a spill/reload
  // in M6502SEFrameLowering.cpp.
  assert(!(Subtarget.isFP64bit() && !Subtarget.useOddSPReg()));

  BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, M6502::sub_lo))
    .addReg(LoReg);

  if (Subtarget.hasMTHC1()) {
    // FIXME: The .addReg(DstReg) is a white lie used to temporarily work
    //        around a widespread bug in the -mfp64 support.
    //        The problem is that none of the 32-bit fpu ops mention the fact
    //        that they clobber the upper 32-bits of the 64-bit FPR. Fixing that
    //        requires a major overhaul of the FPU implementation which can't
    //        be done right now due to time constraints.
    //        MTHC1 is one of two instructions that are affected since they are
    //        the only instructions that don't read the lower 32-bits.
    //        We therefore pretend that it reads the bottom 32-bits to
    //        artificially create a dependency and prevent the scheduler
    //        changing the behaviour of the code.
    BuildMI(MBB, I, dl, get(FP64 ? M6502::MTHC1_D64 : M6502::MTHC1_D32), DstReg)
        .addReg(DstReg)
        .addReg(HiReg);
  } else if (Subtarget.isABI_FPXX())
    llvm_unreachable("BuildPairF64 not expanded in frame lowering code!");
  else
    BuildMI(MBB, I, dl, Mtc1Tdd, TRI.getSubReg(DstReg, M6502::sub_hi))
      .addReg(HiReg);
}

void M6502SEInstrInfo::expandEhReturn(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  // This pseudo instruction is generated as part of the lowering of
  // ISD::EH_RETURN. We convert it to a stack increment by OffsetReg, and
  // indirect jump to TargetReg
  M6502ABIInfo ABI = Subtarget.getABI();
  unsigned ADDU = ABI.GetPtrAdduOp();
  unsigned SP = Subtarget.isGP64bit() ? M6502::SP_64 : M6502::SP;
  unsigned RA = Subtarget.isGP64bit() ? M6502::RA_64 : M6502::RA;
  unsigned T9 = Subtarget.isGP64bit() ? M6502::T9_64 : M6502::T9;
  unsigned ZERO = Subtarget.isGP64bit() ? M6502::ZERO_64 : M6502::ZERO;
  unsigned OffsetReg = I->getOperand(0).getReg();
  unsigned TargetReg = I->getOperand(1).getReg();

  // addu $ra, $v0, $zero
  // addu $sp, $sp, $v1
  // jr   $ra (via RetRA)
  const TargetMachine &TM = MBB.getParent()->getTarget();
  if (TM.isPositionIndependent())
    BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), T9)
        .addReg(TargetReg)
        .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), RA)
      .addReg(TargetReg)
      .addReg(ZERO);
  BuildMI(MBB, I, I->getDebugLoc(), get(ADDU), SP).addReg(SP).addReg(OffsetReg);
  expandRetRA(MBB, I);
}

const M6502InstrInfo *llvm::createM6502SEInstrInfo(const M6502Subtarget &STI) {
  return new M6502SEInstrInfo(STI);
}
