//===---- M6502ABIInfo.cpp - Information about M6502 ABI's ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "M6502ABIInfo.h"
#include "M6502RegisterInfo.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCTargetOptions.h"

using namespace llvm;

namespace {
static const MCPhysReg O32IntRegs[4] = {M6502::A0, M6502::A1, M6502::A2, M6502::A3};

static const MCPhysReg M650264IntRegs[8] = {
    M6502::A0_64, M6502::A1_64, M6502::A2_64, M6502::A3_64,
    M6502::T0_64, M6502::T1_64, M6502::T2_64, M6502::T3_64};
}

ArrayRef<MCPhysReg> M6502ABIInfo::GetByValArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(M650264IntRegs);
  llvm_unreachable("Unhandled ABI");
}

ArrayRef<MCPhysReg> M6502ABIInfo::GetVarArgRegs() const {
  if (IsO32())
    return makeArrayRef(O32IntRegs);
  if (IsN32() || IsN64())
    return makeArrayRef(M650264IntRegs);
  llvm_unreachable("Unhandled ABI");
}

unsigned M6502ABIInfo::GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const {
  if (IsO32())
    return CC != CallingConv::Fast ? 16 : 0;
  if (IsN32() || IsN64())
    return 0;
  llvm_unreachable("Unhandled ABI");
}

M6502ABIInfo M6502ABIInfo::computeTargetABI(const Triple &TT, StringRef CPU,
                                          const MCTargetOptions &Options) {
  return M6502ABIInfo::O32();
}

unsigned M6502ABIInfo::GetStackPtr() const {
  return ArePtrs64bit() ? M6502::SP_64 : M6502::SP;
}

unsigned M6502ABIInfo::GetFramePtr() const {
  return ArePtrs64bit() ? M6502::FP_64 : M6502::FP;
}

unsigned M6502ABIInfo::GetBasePtr() const {
  return ArePtrs64bit() ? M6502::S7_64 : M6502::S7;
}

unsigned M6502ABIInfo::GetGlobalPtr() const {
  return ArePtrs64bit() ? M6502::GP_64 : M6502::GP;
}

unsigned M6502ABIInfo::GetNullPtr() const {
  return ArePtrs64bit() ? M6502::ZERO_64 : M6502::ZERO;
}

unsigned M6502ABIInfo::GetZeroReg() const {
  return AreGprs64bit() ? M6502::ZERO_64 : M6502::ZERO;
}

unsigned M6502ABIInfo::GetPtrAdduOp() const {
  return ArePtrs64bit() ? M6502::DADDu : M6502::ADDu;
}

unsigned M6502ABIInfo::GetPtrAddiuOp() const {
  return ArePtrs64bit() ? M6502::DADDiu : M6502::ADDiu;
}

unsigned M6502ABIInfo::GetPtrSubuOp() const {
  return ArePtrs64bit() ? M6502::DSUBu : M6502::SUBu;
}

unsigned M6502ABIInfo::GetPtrAndOp() const {
  return ArePtrs64bit() ? M6502::AND64 : M6502::AND;
}

unsigned M6502ABIInfo::GetGPRMoveOp() const {
  return ArePtrs64bit() ? M6502::OR64 : M6502::OR;
}

unsigned M6502ABIInfo::GetEhDataReg(unsigned I) const {
  static const unsigned EhDataReg[] = {
    M6502::A0, M6502::A1, M6502::A2, M6502::A3
  };
  static const unsigned EhDataReg64[] = {
    M6502::A0_64, M6502::A1_64, M6502::A2_64, M6502::A3_64
  };

  return IsN64() ? EhDataReg64[I] : EhDataReg[I];
}

