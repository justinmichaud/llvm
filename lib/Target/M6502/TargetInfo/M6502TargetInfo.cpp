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
Target &llvm::getTheM6502elTarget() {
  static Target TheM6502elTarget;
  return TheM6502elTarget;
}
Target &llvm::getTheM650264Target() {
  static Target TheM650264Target;
  return TheM650264Target;
}
Target &llvm::getTheM650264elTarget() {
  static Target TheM650264elTarget;
  return TheM650264elTarget;
}

extern "C" void LLVMInitializeM6502TargetInfo() {
  RegisterTarget<Triple::mips,
                 /*HasJIT=*/true>
      X(getTheM6502Target(), "mips", "M6502");

  RegisterTarget<Triple::mipsel,
                 /*HasJIT=*/true>
      Y(getTheM6502elTarget(), "mipsel", "M6502el");

  RegisterTarget<Triple::mips64,
                 /*HasJIT=*/true>
      A(getTheM650264Target(), "mips64", "M650264 [experimental]");

  RegisterTarget<Triple::mips64el,
                 /*HasJIT=*/true>
      B(getTheM650264elTarget(), "mips64el", "M650264el [experimental]");
}
