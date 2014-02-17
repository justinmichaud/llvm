#define DEBUG_TYPE "asm-printer"
#include "Simple.h"
#include "SimpleTargetStreamer.h"
#include "SimpleInstrInfo.h"
#include "SimpleSubtarget.h"
#include "SimpleTargetMachine.h"
#include "SimpleMCInstLower.h"
#include "InstPrinter/SimpleInstPrinter.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/DebugInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Mangler.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include <algorithm>
#include <cctype>
using namespace llvm;

namespace {
  class SimpleAsmPrinter : public AsmPrinter {
    const SimpleSubtarget &Subtarget;
    SimpleTargetStreamer &getTargetStreamer();
    SimpleMCInstLower MCInstLowering;

  public:
    explicit SimpleAsmPrinter(TargetMachine &TM, MCStreamer &Streamer)
      : AsmPrinter(TM, Streamer), Subtarget(TM.getSubtarget<SimpleSubtarget>()),
        MCInstLowering(*this) 
    {}

    virtual const char *getPassName() const {
      return "Simple Assembly Printer";
    }

    void printOperand(const MachineInstr *MI, int opNum, raw_ostream &O);
    void EmitInstruction(const MachineInstr *MI);
    void EmitFunctionBodyStart();
  };
} // end of anonymous namespace

SimpleTargetStreamer &SimpleAsmPrinter::getTargetStreamer() {
  return static_cast<SimpleTargetStreamer&>(*OutStreamer.getTargetStreamer());
}


void SimpleAsmPrinter::EmitFunctionBodyStart() {
  MCInstLowering.Initialize(Mang, &MF->getContext());
}

void SimpleAsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                   raw_ostream &O) {
  const DataLayout *DL = TM.getDataLayout();
  const MachineOperand &MO = MI->getOperand(opNum);
  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << SimpleInstPrinter::getRegisterName(MO.getReg());
    break;
  case MachineOperand::MO_Immediate:
    O << MO.getImm();
    break;
  default:
    llvm_unreachable("not implemented");
  }
}

void SimpleAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  MCInst TmpInst;
  MCInstLowering.Lower(MI, TmpInst);
  EmitToStreamer(OutStreamer, TmpInst);
}

// Force static initialization.
extern "C" void LLVMInitializeSimpleAsmPrinter() { 
  RegisterAsmPrinter<SimpleAsmPrinter> X(TheSimpleTarget);
}
