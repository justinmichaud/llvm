//===- M6502RegisterInfo.cpp - M6502 Register Information -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M6502 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "M6502RegisterInfo.h"
#include "MCTargetDesc/M6502ABIInfo.h"
#include "M6502.h"
#include "M6502MachineFunction.h"
#include "M6502Subtarget.h"
#include "M6502TargetMachine.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "m6502-reg-info"

#define GET_REGINFO_TARGET_DESC
#include "M6502GenRegisterInfo.inc"

M6502RegisterInfo::M6502RegisterInfo() : M6502GenRegisterInfo(M6502::RA) {}

unsigned M6502RegisterInfo::getPICCallReg() { return M6502::T9; }

const TargetRegisterClass *
M6502RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                     unsigned Kind) const {
  M6502ABIInfo ABI = MF.getSubtarget<M6502Subtarget>().getABI();
  M6502PtrClass PtrClassKind = static_cast<M6502PtrClass>(Kind);

  switch (PtrClassKind) {
  case M6502PtrClass::Default:
    return ABI.ArePtrs64bit() ? &M6502::GPR64RegClass : &M6502::GPR32RegClass;
  case M6502PtrClass::GPR16MM:
    return ABI.ArePtrs64bit() ? &M6502::GPRMM16_64RegClass
                              : &M6502::GPRMM16RegClass;
  case M6502PtrClass::StackPointer:
    return ABI.ArePtrs64bit() ? &M6502::SP64RegClass : &M6502::SP32RegClass;
  case M6502PtrClass::GlobalPointer:
    return ABI.ArePtrs64bit() ? &M6502::GP64RegClass : &M6502::GP32RegClass;
  }

  llvm_unreachable("Unknown pointer kind");
}

unsigned
M6502RegisterInfo::getRegPressureLimit(const TargetRegisterClass *RC,
                                      MachineFunction &MF) const {
  switch (RC->getID()) {
  default:
    return 0;
  case M6502::GPR32RegClassID:
  case M6502::GPR64RegClassID:
  case M6502::DSPRRegClassID: {
    const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
    return 28 - TFI->hasFP(MF);
  }
  case M6502::FGR32RegClassID:
    return 32;
  case M6502::AFGR64RegClassID:
    return 16;
  case M6502::FGR64RegClassID:
    return 32;
  }
}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//

/// M6502 Callee Saved Registers
const MCPhysReg *
M6502RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const M6502Subtarget &Subtarget = MF->getSubtarget<M6502Subtarget>();
  const Function *F = MF->getFunction();
  if (F->hasFnAttribute("interrupt")) {
    if (Subtarget.hasM650264())
      return Subtarget.hasM650264r6() ? CSR_Interrupt_64R6_SaveList
                                     : CSR_Interrupt_64_SaveList;
    else
      return Subtarget.hasM650232r6() ? CSR_Interrupt_32R6_SaveList
                                     : CSR_Interrupt_32_SaveList;
  }

  if (Subtarget.isSingleFloat())
    return CSR_SingleFloatOnly_SaveList;

  if (Subtarget.isABI_N64())
    return CSR_N64_SaveList;

  if (Subtarget.isABI_N32())
    return CSR_N32_SaveList;

  if (Subtarget.isFP64bit())
    return CSR_O32_FP64_SaveList;

  if (Subtarget.isFPXX())
    return CSR_O32_FPXX_SaveList;

  return CSR_O32_SaveList;
}

const uint32_t *
M6502RegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const {
  const M6502Subtarget &Subtarget = MF.getSubtarget<M6502Subtarget>();
  if (Subtarget.isSingleFloat())
    return CSR_SingleFloatOnly_RegMask;

  if (Subtarget.isABI_N64())
    return CSR_N64_RegMask;

  if (Subtarget.isABI_N32())
    return CSR_N32_RegMask;

  if (Subtarget.isFP64bit())
    return CSR_O32_FP64_RegMask;

  if (Subtarget.isFPXX())
    return CSR_O32_FPXX_RegMask;

  return CSR_O32_RegMask;
}

const uint32_t *M6502RegisterInfo::getM650216RetHelperMask() {
  return CSR_M650216RetHelper_RegMask;
}

BitVector M6502RegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  static const MCPhysReg ReservedGPR32[] = {
    M6502::ZERO, M6502::K0, M6502::K1, M6502::SP
  };

  static const MCPhysReg ReservedGPR64[] = {
    M6502::ZERO_64, M6502::K0_64, M6502::K1_64, M6502::SP_64
  };

  BitVector Reserved(getNumRegs());
  const M6502Subtarget &Subtarget = MF.getSubtarget<M6502Subtarget>();

  using RegIter = TargetRegisterClass::const_iterator;

  for (unsigned I = 0; I < array_lengthof(ReservedGPR32); ++I)
    Reserved.set(ReservedGPR32[I]);

  // Reserve registers for the NaCl sandbox.
  if (Subtarget.isTargetNaCl()) {
    Reserved.set(M6502::T6);   // Reserved for control flow mask.
    Reserved.set(M6502::T7);   // Reserved for memory access mask.
    Reserved.set(M6502::T8);   // Reserved for thread pointer.
  }

  for (unsigned I = 0; I < array_lengthof(ReservedGPR64); ++I)
    Reserved.set(ReservedGPR64[I]);

  // For mno-abicalls, GP is a program invariant!
  if (!Subtarget.isABICalls()) {
    Reserved.set(M6502::GP);
    Reserved.set(M6502::GP_64);
  }

  if (Subtarget.isFP64bit()) {
    // Reserve all registers in AFGR64.
    for (RegIter Reg = M6502::AFGR64RegClass.begin(),
         EReg = M6502::AFGR64RegClass.end(); Reg != EReg; ++Reg)
      Reserved.set(*Reg);
  } else {
    // Reserve all registers in FGR64.
    for (RegIter Reg = M6502::FGR64RegClass.begin(),
         EReg = M6502::FGR64RegClass.end(); Reg != EReg; ++Reg)
      Reserved.set(*Reg);
  }
  // Reserve FP if this function should have a dedicated frame pointer register.
  if (Subtarget.getFrameLowering()->hasFP(MF)) {
    if (Subtarget.inM650216Mode())
      Reserved.set(M6502::S0);
    else {
      Reserved.set(M6502::FP);
      Reserved.set(M6502::FP_64);

      // Reserve the base register if we need to both realign the stack and
      // allocate variable-sized objects at runtime. This should test the
      // same conditions as M6502FrameLowering::hasBP().
      if (needsStackRealignment(MF) &&
          MF.getFrameInfo().hasVarSizedObjects()) {
        Reserved.set(M6502::S7);
        Reserved.set(M6502::S7_64);
      }
    }
  }

  // Reserve hardware registers.
  Reserved.set(M6502::HWR29);

  // Reserve DSP control register.
  Reserved.set(M6502::DSPPos);
  Reserved.set(M6502::DSPSCount);
  Reserved.set(M6502::DSPCarry);
  Reserved.set(M6502::DSPEFI);
  Reserved.set(M6502::DSPOutFlag);

  // Reserve MSA control registers.
  Reserved.set(M6502::MSAIR);
  Reserved.set(M6502::MSACSR);
  Reserved.set(M6502::MSAAccess);
  Reserved.set(M6502::MSASave);
  Reserved.set(M6502::MSAModify);
  Reserved.set(M6502::MSARequest);
  Reserved.set(M6502::MSAMap);
  Reserved.set(M6502::MSAUnmap);

  // Reserve RA if in mips16 mode.
  if (Subtarget.inM650216Mode()) {
    const M6502FunctionInfo *M6502FI = MF.getInfo<M6502FunctionInfo>();
    Reserved.set(M6502::RA);
    Reserved.set(M6502::RA_64);
    Reserved.set(M6502::T0);
    Reserved.set(M6502::T1);
    if (MF.getFunction()->hasFnAttribute("saveS2") || M6502FI->hasSaveS2())
      Reserved.set(M6502::S2);
  }

  // Reserve GP if small section is used.
  if (Subtarget.useSmallSection()) {
    Reserved.set(M6502::GP);
    Reserved.set(M6502::GP_64);
  }

  if (Subtarget.isABI_O32() && !Subtarget.useOddSPReg()) {
    for (const auto &Reg : M6502::OddSPRegClass)
      Reserved.set(Reg);
  }

  return Reserved;
}

bool
M6502RegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool
M6502RegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void M6502RegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
                    unsigned FIOperandNum, RegScavenger *RS) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();

  DEBUG(errs() << "\nFunction : " << MF.getName() << "\n";
        errs() << "<--------->\n" << MI);

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  uint64_t stackSize = MF.getFrameInfo().getStackSize();
  int64_t spOffset = MF.getFrameInfo().getObjectOffset(FrameIndex);

  DEBUG(errs() << "FrameIndex : " << FrameIndex << "\n"
               << "spOffset   : " << spOffset << "\n"
               << "stackSize  : " << stackSize << "\n"
               << "alignment  : "
               << MF.getFrameInfo().getObjectAlignment(FrameIndex) << "\n");

  eliminateFI(MI, FIOperandNum, FrameIndex, stackSize, spOffset);
}

unsigned M6502RegisterInfo::
getFrameRegister(const MachineFunction &MF) const {
  const M6502Subtarget &Subtarget = MF.getSubtarget<M6502Subtarget>();
  const TargetFrameLowering *TFI = Subtarget.getFrameLowering();
  bool IsN64 =
      static_cast<const M6502TargetMachine &>(MF.getTarget()).getABI().IsN64();

  if (Subtarget.inM650216Mode())
    return TFI->hasFP(MF) ? M6502::S0 : M6502::SP;
  else
    return TFI->hasFP(MF) ? (IsN64 ? M6502::FP_64 : M6502::FP) :
                            (IsN64 ? M6502::SP_64 : M6502::SP);
}

bool M6502RegisterInfo::canRealignStack(const MachineFunction &MF) const {
  // Avoid realigning functions that explicitly do not want to be realigned.
  // Normally, we should report an error when a function should be dynamically
  // realigned but also has the attribute no-realign-stack. Unfortunately,
  // with this attribute, MachineFrameInfo clamps each new object's alignment
  // to that of the stack's alignment as specified by the ABI. As a result,
  // the information of whether we have objects with larger alignment
  // requirement than the stack's alignment is already lost at this point.
  if (!TargetRegisterInfo::canRealignStack(MF))
    return false;

  const M6502Subtarget &Subtarget = MF.getSubtarget<M6502Subtarget>();
  unsigned FP = Subtarget.isGP32bit() ? M6502::FP : M6502::FP_64;
  unsigned BP = Subtarget.isGP32bit() ? M6502::S7 : M6502::S7_64;

  // Support dynamic stack realignment only for targets with standard encoding.
  if (!Subtarget.hasStandardEncoding())
    return false;

  // We can't perform dynamic stack realignment if we can't reserve the
  // frame pointer register.
  if (!MF.getRegInfo().canReserveReg(FP))
    return false;

  // We can realign the stack if we know the maximum call frame size and we
  // don't have variable sized objects.
  if (Subtarget.getFrameLowering()->hasReservedCallFrame(MF))
    return true;

  // We have to reserve the base pointer register in the presence of variable
  // sized objects.
  return MF.getRegInfo().canReserveReg(BP);
}
