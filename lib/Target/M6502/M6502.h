//===-- M6502.h - Top-level interface for M6502 representation ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM M6502 back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502_H
#define LLVM_LIB_TARGET_M6502_M6502_H

#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class M6502TargetMachine;
  class ModulePass;
  class FunctionPass;

  ModulePass *createM6502Os16Pass();
  ModulePass *createM650216HardFloatPass();

  FunctionPass *createM6502ModuleISelDagPass();
  FunctionPass *createM6502OptimizePICCallPass();
  FunctionPass *createM6502DelaySlotFillerPass();
  FunctionPass *createM6502HazardSchedule();
  FunctionPass *createM6502LongBranchPass();
  FunctionPass *createM6502ConstantIslandPass();
  FunctionPass *createMicroM6502SizeReductionPass();
} // end namespace llvm;

#endif
