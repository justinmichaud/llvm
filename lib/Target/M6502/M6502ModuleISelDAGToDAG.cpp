//===----------------------------------------------------------------------===//
// Instruction Selector Subtarget Control
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// This file defines a pass used to change the subtarget for the
// M6502 Instruction selector.
//
//===----------------------------------------------------------------------===//

#include "M6502.h"
#include "M6502TargetMachine.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "m6502-isel"

namespace {
  class M6502ModuleDAGToDAGISel : public MachineFunctionPass {
  public:
    static char ID;

    M6502ModuleDAGToDAGISel() : MachineFunctionPass(ID) {}

    // Pass Name
    StringRef getPassName() const override {
      return "M6502 DAG->DAG Pattern Instruction Selection";
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.addRequired<TargetPassConfig>();
      MachineFunctionPass::getAnalysisUsage(AU);
    }

    bool runOnMachineFunction(MachineFunction &MF) override;
  };

  char M6502ModuleDAGToDAGISel::ID = 0;
}

bool M6502ModuleDAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  DEBUG(errs() << "In M6502ModuleDAGToDAGISel::runMachineFunction\n");
  auto &TPC = getAnalysis<TargetPassConfig>();
  auto &TM = TPC.getTM<M6502TargetMachine>();
  TM.resetSubtarget(&MF);
  return false;
}

llvm::FunctionPass *llvm::createM6502ModuleISelDagPass() {
  return new M6502ModuleDAGToDAGISel();
}
