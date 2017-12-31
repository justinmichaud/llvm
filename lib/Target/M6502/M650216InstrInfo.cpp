//===- M650216InstrInfo.cpp - M650216 Instruction Information ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M650216 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "M650216InstrInfo.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <vector>

using namespace llvm;

#define DEBUG_TYPE "m650216-instrinfo"

M650216InstrInfo::M650216InstrInfo(const M6502Subtarget &STI)
    : M6502InstrInfo(STI, M6502::Bimm16) {}

const M6502RegisterInfo &M650216InstrInfo::getRegisterInfo() const {
  return RI;
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned M650216InstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                              int &FrameIndex) const {
  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned M650216InstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  return 0;
}

void M650216InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, unsigned DestReg,
                                  unsigned SrcReg, bool KillSrc) const {
  unsigned Opc = 0;

  if (M6502::CPU16RegsRegClass.contains(DestReg) &&
      M6502::GPR32RegClass.contains(SrcReg))
    Opc = M6502::MoveR3216;
  else if (M6502::GPR32RegClass.contains(DestReg) &&
           M6502::CPU16RegsRegClass.contains(SrcReg))
    Opc = M6502::Move32R16;
  else if ((SrcReg == M6502::HI0) &&
           (M6502::CPU16RegsRegClass.contains(DestReg)))
    Opc = M6502::Mfhi16, SrcReg = 0;
  else if ((SrcReg == M6502::LO0) &&
           (M6502::CPU16RegsRegClass.contains(DestReg)))
    Opc = M6502::Mflo16, SrcReg = 0;

  assert(Opc && "Cannot copy registers");

  MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc));

  if (DestReg)
    MIB.addReg(DestReg, RegState::Define);

  if (SrcReg)
    MIB.addReg(SrcReg, getKillRegState(KillSrc));
}

void M650216InstrInfo::storeRegToStack(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator I,
                                      unsigned SrcReg, bool isKill, int FI,
                                      const TargetRegisterClass *RC,
                                      const TargetRegisterInfo *TRI,
                                      int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);
  unsigned Opc = 0;
  if (M6502::CPU16RegsRegClass.hasSubClassEq(RC))
    Opc = M6502::SwRxSpImmX16;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc)).addReg(SrcReg, getKillRegState(isKill)).
      addFrameIndex(FI).addImm(Offset)
      .addMemOperand(MMO);
}

void M650216InstrInfo::loadRegFromStack(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator I,
                                       unsigned DestReg, int FI,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI,
                                       int64_t Offset) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);
  unsigned Opc = 0;

  if (M6502::CPU16RegsRegClass.hasSubClassEq(RC))
    Opc = M6502::LwRxSpImmX16;
  assert(Opc && "Register class not handled!");
  BuildMI(MBB, I, DL, get(Opc), DestReg).addFrameIndex(FI).addImm(Offset)
    .addMemOperand(MMO);
}

bool M650216InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case M6502::RetRA16:
    ExpandRetRA16(MBB, MI, M6502::JrcRa16);
    break;
  }

  MBB.erase(MI.getIterator());
  return true;
}

/// GetOppositeBranchOpc - Return the inverse of the specified
/// opcode, e.g. turning BEQ to BNE.
unsigned M650216InstrInfo::getOppositeBranchOpc(unsigned Opc) const {
  switch (Opc) {
  case M6502::BeqzRxImmX16: return M6502::BnezRxImmX16;
  case M6502::BnezRxImmX16: return M6502::BeqzRxImmX16;
  case M6502::BeqzRxImm16: return M6502::BnezRxImm16;
  case M6502::BnezRxImm16: return M6502::BeqzRxImm16;
  case M6502::BteqzT8CmpX16: return M6502::BtnezT8CmpX16;
  case M6502::BteqzT8SltX16: return M6502::BtnezT8SltX16;
  case M6502::BteqzT8SltiX16: return M6502::BtnezT8SltiX16;
  case M6502::Btnez16: return M6502::Bteqz16;
  case M6502::BtnezX16: return M6502::BteqzX16;
  case M6502::BtnezT8CmpiX16: return M6502::BteqzT8CmpiX16;
  case M6502::BtnezT8SltuX16: return M6502::BteqzT8SltuX16;
  case M6502::BtnezT8SltiuX16: return M6502::BteqzT8SltiuX16;
  case M6502::Bteqz16: return M6502::Btnez16;
  case M6502::BteqzX16: return M6502::BtnezX16;
  case M6502::BteqzT8CmpiX16: return M6502::BtnezT8CmpiX16;
  case M6502::BteqzT8SltuX16: return M6502::BtnezT8SltuX16;
  case M6502::BteqzT8SltiuX16: return M6502::BtnezT8SltiuX16;
  case M6502::BtnezT8CmpX16: return M6502::BteqzT8CmpX16;
  case M6502::BtnezT8SltX16: return M6502::BteqzT8SltX16;
  case M6502::BtnezT8SltiX16: return M6502::BteqzT8SltiX16;
  }
  llvm_unreachable("Illegal opcode!");
}

static void addSaveRestoreRegs(MachineInstrBuilder &MIB,
                               const std::vector<CalleeSavedInfo> &CSI,
                               unsigned Flags = 0) {
  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    // Add the callee-saved register as live-in. Do not add if the register is
    // RA and return address is taken, because it has already been added in
    // method M6502TargetLowering::lowerRETURNADDR.
    // It's killed at the spill, unless the register is RA and return address
    // is taken.
    unsigned Reg = CSI[e-i-1].getReg();
    switch (Reg) {
    case M6502::RA:
    case M6502::S0:
    case M6502::S1:
      MIB.addReg(Reg, Flags);
      break;
    case M6502::S2:
      break;
    default:
      llvm_unreachable("unexpected mips16 callee saved register");

    }
  }
}

// Adjust SP by FrameSize bytes. Save RA, S0, S1
void M650216InstrInfo::makeFrame(unsigned SP, int64_t FrameSize,
                                MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const {
  DebugLoc DL;
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI    = MF.getFrameInfo();
  const BitVector Reserved = RI.getReservedRegs(MF);
  bool SaveS2 = Reserved[M6502::S2];
  MachineInstrBuilder MIB;
  unsigned Opc = ((FrameSize <= 128) && !SaveS2)? M6502::Save16:M6502::SaveX16;
  MIB = BuildMI(MBB, I, DL, get(Opc));
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  addSaveRestoreRegs(MIB, CSI);
  if (SaveS2)
    MIB.addReg(M6502::S2);
  if (isUInt<11>(FrameSize))
    MIB.addImm(FrameSize);
  else {
    int Base = 2040; // should create template function like isUInt that
                     // returns largest possible n bit unsigned integer
    int64_t Remainder = FrameSize - Base;
    MIB.addImm(Base);
    if (isInt<16>(-Remainder))
      BuildAddiuSpImm(MBB, I, -Remainder);
    else
      adjustStackPtrBig(SP, -Remainder, MBB, I, M6502::V0, M6502::V1);
  }
}

// Adjust SP by FrameSize bytes. Restore RA, S0, S1
void M650216InstrInfo::restoreFrame(unsigned SP, int64_t FrameSize,
                                   MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator I) const {
  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();
  MachineFunction *MF = MBB.getParent();
  MachineFrameInfo &MFI    = MF->getFrameInfo();
  const BitVector Reserved = RI.getReservedRegs(*MF);
  bool SaveS2 = Reserved[M6502::S2];
  MachineInstrBuilder MIB;
  unsigned Opc = ((FrameSize <= 128) && !SaveS2)?
    M6502::Restore16:M6502::RestoreX16;

  if (!isUInt<11>(FrameSize)) {
    unsigned Base = 2040;
    int64_t Remainder = FrameSize - Base;
    FrameSize = Base; // should create template function like isUInt that
                     // returns largest possible n bit unsigned integer

    if (isInt<16>(Remainder))
      BuildAddiuSpImm(MBB, I, Remainder);
    else
      adjustStackPtrBig(SP, Remainder, MBB, I, M6502::A0, M6502::A1);
  }
  MIB = BuildMI(MBB, I, DL, get(Opc));
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  addSaveRestoreRegs(MIB, CSI, RegState::Define);
  if (SaveS2)
    MIB.addReg(M6502::S2, RegState::Define);
  MIB.addImm(FrameSize);
}

// Adjust SP by Amount bytes where bytes can be up to 32bit number.
// This can only be called at times that we know that there is at least one free
// register.
// This is clearly safe at prologue and epilogue.
void M650216InstrInfo::adjustStackPtrBig(unsigned SP, int64_t Amount,
                                        MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator I,
                                        unsigned Reg1, unsigned Reg2) const {
  DebugLoc DL;
  //
  // li reg1, constant
  // move reg2, sp
  // add reg1, reg1, reg2
  // move sp, reg1
  //
  //
  MachineInstrBuilder MIB1 = BuildMI(MBB, I, DL, get(M6502::LwConstant32), Reg1);
  MIB1.addImm(Amount).addImm(-1);
  MachineInstrBuilder MIB2 = BuildMI(MBB, I, DL, get(M6502::MoveR3216), Reg2);
  MIB2.addReg(M6502::SP, RegState::Kill);
  MachineInstrBuilder MIB3 = BuildMI(MBB, I, DL, get(M6502::AdduRxRyRz16), Reg1);
  MIB3.addReg(Reg1);
  MIB3.addReg(Reg2, RegState::Kill);
  MachineInstrBuilder MIB4 = BuildMI(MBB, I, DL, get(M6502::Move32R16),
                                                     M6502::SP);
  MIB4.addReg(Reg1, RegState::Kill);
}

void M650216InstrInfo::adjustStackPtrBigUnrestricted(
    unsigned SP, int64_t Amount, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator I) const {
   llvm_unreachable("adjust stack pointer amount exceeded");
}

/// Adjust SP by Amount bytes.
void M650216InstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const {
  if (Amount == 0)
    return;

  if (isInt<16>(Amount))  // need to change to addiu sp, ....and isInt<16>
    BuildAddiuSpImm(MBB, I, Amount);
  else
    adjustStackPtrBigUnrestricted(SP, Amount, MBB, I);
}

/// This function generates the sequence of instructions needed to get the
/// result of adding register REG and immediate IMM.
unsigned M650216InstrInfo::loadImmediate(unsigned FrameReg, int64_t Imm,
                                        MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator II,
                                        const DebugLoc &DL,
                                        unsigned &NewImm) const {
  //
  // given original instruction is:
  // Instr rx, T[offset] where offset is too big.
  //
  // lo = offset & 0xFFFF
  // hi = ((offset >> 16) + (lo >> 15)) & 0xFFFF;
  //
  // let T = temporary register
  // li T, hi
  // shl T, 16
  // add T, Rx, T
  //
  RegScavenger rs;
  int32_t lo = Imm & 0xFFFF;
  NewImm = lo;
  int Reg =0;
  int SpReg = 0;

  rs.enterBasicBlock(MBB);
  rs.forward(II);
  //
  // We need to know which registers can be used, in the case where there
  // are not enough free registers. We exclude all registers that
  // are used in the instruction that we are helping.
  //  // Consider all allocatable registers in the register class initially
  BitVector Candidates =
      RI.getAllocatableSet
      (*II->getParent()->getParent(), &M6502::CPU16RegsRegClass);
  // Exclude all the registers being used by the instruction.
  for (unsigned i = 0, e = II->getNumOperands(); i != e; ++i) {
    MachineOperand &MO = II->getOperand(i);
    if (MO.isReg() && MO.getReg() != 0 && !MO.isDef() &&
        !TargetRegisterInfo::isVirtualRegister(MO.getReg()))
      Candidates.reset(MO.getReg());
  }

  // If the same register was used and defined in an instruction, then
  // it will not be in the list of candidates.
  //
  // we need to analyze the instruction that we are helping.
  // we need to know if it defines register x but register x is not
  // present as an operand of the instruction. this tells
  // whether the register is live before the instruction. if it's not
  // then we don't need to save it in case there are no free registers.
  int DefReg = 0;
  for (unsigned i = 0, e = II->getNumOperands(); i != e; ++i) {
    MachineOperand &MO = II->getOperand(i);
    if (MO.isReg() && MO.isDef()) {
      DefReg = MO.getReg();
      break;
    }
  }

  BitVector Available = rs.getRegsAvailable(&M6502::CPU16RegsRegClass);
  Available &= Candidates;
  //
  // we use T0 for the first register, if we need to save something away.
  // we use T1 for the second register, if we need to save something away.
  //
  unsigned FirstRegSaved =0, SecondRegSaved=0;
  unsigned FirstRegSavedTo = 0, SecondRegSavedTo = 0;

  Reg = Available.find_first();

  if (Reg == -1) {
    Reg = Candidates.find_first();
    Candidates.reset(Reg);
    if (DefReg != Reg) {
      FirstRegSaved = Reg;
      FirstRegSavedTo = M6502::T0;
      copyPhysReg(MBB, II, DL, FirstRegSavedTo, FirstRegSaved, true);
    }
  }
  else
    Available.reset(Reg);
  BuildMI(MBB, II, DL, get(M6502::LwConstant32), Reg).addImm(Imm).addImm(-1);
  NewImm = 0;
  if (FrameReg == M6502::SP) {
    SpReg = Available.find_first();
    if (SpReg == -1) {
      SpReg = Candidates.find_first();
      // Candidates.reset(SpReg); // not really needed
      if (DefReg!= SpReg) {
        SecondRegSaved = SpReg;
        SecondRegSavedTo = M6502::T1;
      }
      if (SecondRegSaved)
        copyPhysReg(MBB, II, DL, SecondRegSavedTo, SecondRegSaved, true);
    }
   else
     Available.reset(SpReg);
    copyPhysReg(MBB, II, DL, SpReg, M6502::SP, false);
    BuildMI(MBB, II, DL, get(M6502::  AdduRxRyRz16), Reg).addReg(SpReg, RegState::Kill)
      .addReg(Reg);
  }
  else
    BuildMI(MBB, II, DL, get(M6502::  AdduRxRyRz16), Reg).addReg(FrameReg)
      .addReg(Reg, RegState::Kill);
  if (FirstRegSaved || SecondRegSaved) {
    II = std::next(II);
    if (FirstRegSaved)
      copyPhysReg(MBB, II, DL, FirstRegSaved, FirstRegSavedTo, true);
    if (SecondRegSaved)
      copyPhysReg(MBB, II, DL, SecondRegSaved, SecondRegSavedTo, true);
  }
  return Reg;
}

unsigned M650216InstrInfo::getAnalyzableBrOpc(unsigned Opc) const {
  return (Opc == M6502::BeqzRxImmX16   || Opc == M6502::BimmX16  ||
          Opc == M6502::Bimm16  ||
          Opc == M6502::Bteqz16        || Opc == M6502::Btnez16 ||
          Opc == M6502::BeqzRxImm16    || Opc == M6502::BnezRxImm16   ||
          Opc == M6502::BnezRxImmX16   || Opc == M6502::BteqzX16 ||
          Opc == M6502::BteqzT8CmpX16  || Opc == M6502::BteqzT8CmpiX16 ||
          Opc == M6502::BteqzT8SltX16  || Opc == M6502::BteqzT8SltuX16  ||
          Opc == M6502::BteqzT8SltiX16 || Opc == M6502::BteqzT8SltiuX16 ||
          Opc == M6502::BtnezX16       || Opc == M6502::BtnezT8CmpX16 ||
          Opc == M6502::BtnezT8CmpiX16 || Opc == M6502::BtnezT8SltX16 ||
          Opc == M6502::BtnezT8SltuX16 || Opc == M6502::BtnezT8SltiX16 ||
          Opc == M6502::BtnezT8SltiuX16 ) ? Opc : 0;
}

void M650216InstrInfo::ExpandRetRA16(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  unsigned Opc) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(Opc));
}

const MCInstrDesc &M650216InstrInfo::AddiuSpImm(int64_t Imm) const {
  if (validSpImm8(Imm))
    return get(M6502::AddiuSpImm16);
  else
    return get(M6502::AddiuSpImmX16);
}

void M650216InstrInfo::BuildAddiuSpImm
  (MachineBasicBlock &MBB, MachineBasicBlock::iterator I, int64_t Imm) const {
  DebugLoc DL;
  BuildMI(MBB, I, DL, AddiuSpImm(Imm)).addImm(Imm);
}

const M6502InstrInfo *llvm::createM650216InstrInfo(const M6502Subtarget &STI) {
  return new M650216InstrInfo(STI);
}

bool M650216InstrInfo::validImmediate(unsigned Opcode, unsigned Reg,
                                     int64_t Amount) {
  switch (Opcode) {
  case M6502::LbRxRyOffMemX16:
  case M6502::LbuRxRyOffMemX16:
  case M6502::LhRxRyOffMemX16:
  case M6502::LhuRxRyOffMemX16:
  case M6502::SbRxRyOffMemX16:
  case M6502::ShRxRyOffMemX16:
  case M6502::LwRxRyOffMemX16:
  case M6502::SwRxRyOffMemX16:
  case M6502::SwRxSpImmX16:
  case M6502::LwRxSpImmX16:
    return isInt<16>(Amount);
  case M6502::AddiuRxRyOffMemX16:
    if ((Reg == M6502::PC) || (Reg == M6502::SP))
      return isInt<16>(Amount);
    return isInt<15>(Amount);
  }
  llvm_unreachable("unexpected Opcode in validImmediate");
}
