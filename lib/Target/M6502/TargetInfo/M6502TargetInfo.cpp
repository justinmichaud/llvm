//===-- M6502TargetInfo.cpp - M6502 Target Implementation -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "M6502.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheM6502Target() {
  static Target TheM6502Target;
  return TheM6502Target;
}

extern "C" void LLVMInitializeM6502TargetInfo() {
  RegisterTarget<Triple::m6502,
                 /*HasJIT=*/false>
      X(getTheM6502Target(), "m6502", "M6502");
}
