//===---- M6502Os16.cpp for M6502 Option -Os16                       --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an optimization phase for the M6502 target.
//
//===----------------------------------------------------------------------===//

#include "M6502.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "m6502-os16"

static cl::opt<std::string> M650232FunctionMask(
  "m650232-function-mask",
  cl::init(""),
  cl::desc("Force function to be m650232"),
  cl::Hidden);

namespace {
  class M6502Os16 : public ModulePass {
  public:
    static char ID;

    M6502Os16() : ModulePass(ID) {}

    StringRef getPassName() const override { return "M6502 Os16 Optimization"; }

    bool runOnModule(Module &M) override;
  };

  char M6502Os16::ID = 0;
}

// Figure out if we need float point based on the function signature.
// We need to move variables in and/or out of floating point
// registers because of the ABI
//
static  bool needsFPFromSig(Function &F) {
  Type* RetType = F.getReturnType();
  switch (RetType->getTypeID()) {
  case Type::FloatTyID:
  case Type::DoubleTyID:
    return true;
  default:
    ;
  }
  if (F.arg_size() >=1) {
    Argument &Arg = *F.arg_begin();
    switch (Arg.getType()->getTypeID()) {
    case Type::FloatTyID:
    case Type::DoubleTyID:
      return true;
    default:
      ;
    }
  }
  return false;
}

// Figure out if the function will need floating point operations
//
static bool needsFP(Function &F) {
  if (needsFPFromSig(F))
    return true;
  for (Function::const_iterator BB = F.begin(), E = F.end(); BB != E; ++BB)
    for (BasicBlock::const_iterator I = BB->begin(), E = BB->end();
         I != E; ++I) {
      const Instruction &Inst = *I;
      switch (Inst.getOpcode()) {
      case Instruction::FAdd:
      case Instruction::FSub:
      case Instruction::FMul:
      case Instruction::FDiv:
      case Instruction::FRem:
      case Instruction::FPToUI:
      case Instruction::FPToSI:
      case Instruction::UIToFP:
      case Instruction::SIToFP:
      case Instruction::FPTrunc:
      case Instruction::FPExt:
      case Instruction::FCmp:
        return true;
      default:
        ;
      }
      if (const CallInst *CI = dyn_cast<CallInst>(I)) {
        DEBUG(dbgs() << "Working on call" << "\n");
        Function &F_ =  *CI->getCalledFunction();
        if (needsFPFromSig(F_))
          return true;
      }
    }
  return false;
}


bool M6502Os16::runOnModule(Module &M) {
  bool usingMask = M650232FunctionMask.length() > 0;
  bool doneUsingMask = false; // this will make it stop repeating

  DEBUG(dbgs() << "Run on Module M6502Os16 \n" << M650232FunctionMask << "\n");
  if (usingMask)
    DEBUG(dbgs() << "using mask \n" << M650232FunctionMask << "\n");

  unsigned int functionIndex = 0;
  bool modified = false;

  for (auto &F : M) {
    if (F.isDeclaration())
      continue;

    DEBUG(dbgs() << "Working on " << F.getName() << "\n");
    if (usingMask) {
      if (!doneUsingMask) {
        if (functionIndex == M650232FunctionMask.length())
          functionIndex = 0;
        switch (M650232FunctionMask[functionIndex]) {
        case '1':
          DEBUG(dbgs() << "mask forced m650232: " << F.getName() << "\n");
          F.addFnAttr("nom650216");
          break;
        case '.':
          doneUsingMask = true;
          break;
        default:
          break;
        }
        functionIndex++;
      }
    }
    else {
      if (needsFP(F)) {
        DEBUG(dbgs() << "os16 forced m650232: " << F.getName() << "\n");
        F.addFnAttr("nom650216");
      }
      else {
        DEBUG(dbgs() << "os16 forced m650216: " << F.getName() << "\n");
        F.addFnAttr("m650216");
      }
    }
  }

  return modified;
}

ModulePass *llvm::createM6502Os16Pass() { return new M6502Os16(); }
