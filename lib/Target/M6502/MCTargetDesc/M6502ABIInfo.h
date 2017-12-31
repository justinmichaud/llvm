//===---- M6502ABIInfo.h - Information about M6502 ABI's --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502ABIINFO_H
#define LLVM_LIB_TARGET_M6502_MCTARGETDESC_M6502ABIINFO_H

#include "llvm/ADT/Triple.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/MCRegisterInfo.h"

namespace llvm {

template <typename T> class ArrayRef;
class MCTargetOptions;
class StringRef;
class TargetRegisterClass;

class M6502ABIInfo {
public:
  enum class ABI { Unknown, O32, N32, N64 };

protected:
  ABI ThisABI;

public:
  M6502ABIInfo(ABI ThisABI) : ThisABI(ThisABI) {}

  static M6502ABIInfo Unknown() { return M6502ABIInfo(ABI::Unknown); }
  static M6502ABIInfo O32() { return M6502ABIInfo(ABI::O32); }
  static M6502ABIInfo N32() { return M6502ABIInfo(ABI::N32); }
  static M6502ABIInfo N64() { return M6502ABIInfo(ABI::N64); }
  static M6502ABIInfo computeTargetABI(const Triple &TT, StringRef CPU,
                                      const MCTargetOptions &Options);

  bool IsKnown() const { return ThisABI != ABI::Unknown; }
  bool IsO32() const { return ThisABI == ABI::O32; }
  bool IsN32() const { return ThisABI == ABI::N32; }
  bool IsN64() const { return ThisABI == ABI::N64; }
  ABI GetEnumValue() const { return ThisABI; }

  /// The registers to use for byval arguments.
  ArrayRef<MCPhysReg> GetByValArgRegs() const;

  /// The registers to use for the variable argument list.
  ArrayRef<MCPhysReg> GetVarArgRegs() const;

  /// Obtain the size of the area allocated by the callee for arguments.
  /// CallingConv::FastCall affects the value for O32.
  unsigned GetCalleeAllocdArgSizeInBytes(CallingConv::ID CC) const;

  /// Ordering of ABI's
  /// M6502GenSubtargetInfo.inc will use this to resolve conflicts when given
  /// multiple ABI options.
  bool operator<(const M6502ABIInfo Other) const {
    return ThisABI < Other.GetEnumValue();
  }

  unsigned GetStackPtr() const;
  unsigned GetFramePtr() const;
  unsigned GetBasePtr() const;
  unsigned GetGlobalPtr() const;
  unsigned GetNullPtr() const;
  unsigned GetZeroReg() const;
  unsigned GetPtrAdduOp() const;
  unsigned GetPtrAddiuOp() const;
  unsigned GetPtrSubuOp() const;
  unsigned GetPtrAndOp() const;
  unsigned GetGPRMoveOp() const;
  inline bool ArePtrs64bit() const { return IsN64(); }
  inline bool AreGprs64bit() const { return IsN32() || IsN64(); }

  unsigned GetEhDataReg(unsigned I) const;
};
}

#endif
