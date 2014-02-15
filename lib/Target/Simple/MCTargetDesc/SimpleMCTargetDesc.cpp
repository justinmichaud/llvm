#include "SimpleMCTargetDesc.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_SUBTARGETINFO_MC_DESC
#include "SimpleGenSubtargetInfo.inc"

// Force static initialization.
extern "C" void LLVMInitializeSimpleTargetMC() {
  ;
}
