//===-- M6502MachineFunctionInfo.cpp - Private data used for M6502 ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "M6502MachineFunction.h"
#include "MCTargetDesc/M6502ABIInfo.h"
#include "M6502Subtarget.h"
#include "M6502TargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetRegisterInfo.h"

using namespace llvm;

static cl::opt<bool>
FixGlobalBaseReg("m6502-fix-global-base-reg", cl::Hidden, cl::init(true),
                 cl::desc("Always use $gp as the global base register."));

M6502FunctionInfo::~M6502FunctionInfo() = default;

bool M6502FunctionInfo::globalBaseRegSet() const {
  return GlobalBaseReg;
}

unsigned M6502FunctionInfo::getGlobalBaseReg() {
  // Return if it has already been initialized.
  if (GlobalBaseReg)
    return GlobalBaseReg;

  M6502Subtarget const &STI =
      static_cast<const M6502Subtarget &>(MF.getSubtarget());

  const TargetRegisterClass *RC =
      STI.inM650216Mode()
          ? &M6502::CPU16RegsRegClass
          : STI.inMicroM6502Mode()
                ? STI.hasM650264()
                      ? &M6502::GPRMM16_64RegClass
                      : &M6502::GPRMM16RegClass
                : static_cast<const M6502TargetMachine &>(MF.getTarget())
                          .getABI()
                          .IsN64()
                      ? &M6502::GPR64RegClass
                      : &M6502::GPR32RegClass;
  return GlobalBaseReg = MF.getRegInfo().createVirtualRegister(RC);
}

void M6502FunctionInfo::createEhDataRegsFI() {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  for (int I = 0; I < 4; ++I) {
    const TargetRegisterClass &RC =
        static_cast<const M6502TargetMachine &>(MF.getTarget()).getABI().IsN64()
            ? M6502::GPR64RegClass
            : M6502::GPR32RegClass;

    EhDataRegFI[I] = MF.getFrameInfo().CreateStackObject(TRI.getSpillSize(RC),
        TRI.getSpillAlignment(RC), false);
  }
}

void M6502FunctionInfo::createISRRegFI() {
  // ISRs require spill slots for Status & ErrorPC Coprocessor 0 registers.
  // The current implementation only supports M650232r2+ not M650264rX. Status
  // is always 32 bits, ErrorPC is 32 or 64 bits dependent on architecture,
  // however M650232r2+ is the supported architecture.
  const TargetRegisterClass &RC = M6502::GPR32RegClass;
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  for (int I = 0; I < 2; ++I)
    ISRDataRegFI[I] = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(RC), TRI.getSpillAlignment(RC), false);
}

bool M6502FunctionInfo::isEhDataRegFI(int FI) const {
  return CallsEhReturn && (FI == EhDataRegFI[0] || FI == EhDataRegFI[1]
                        || FI == EhDataRegFI[2] || FI == EhDataRegFI[3]);
}

bool M6502FunctionInfo::isISRRegFI(int FI) const {
  return IsISR && (FI == ISRDataRegFI[0] || FI == ISRDataRegFI[1]);
}
MachinePointerInfo M6502FunctionInfo::callPtrInfo(const char *ES) {
  return MachinePointerInfo(MF.getPSVManager().getExternalSymbolCallEntry(ES));
}

MachinePointerInfo M6502FunctionInfo::callPtrInfo(const GlobalValue *GV) {
  return MachinePointerInfo(MF.getPSVManager().getGlobalValueCallEntry(GV));
}

int M6502FunctionInfo::getMoveF64ViaSpillFI(const TargetRegisterClass *RC) {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  if (MoveF64ViaSpillFI == -1) {
    MoveF64ViaSpillFI = MF.getFrameInfo().CreateStackObject(
        TRI.getSpillSize(*RC), TRI.getSpillAlignment(*RC), false);
  }
  return MoveF64ViaSpillFI;
}

void M6502FunctionInfo::anchor() {}
