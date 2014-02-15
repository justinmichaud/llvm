#include "SimpleInstrInfo.h"
#include "Simple.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "SimpleGenInstrInfo.inc"

using namespace llvm;

void SimpleInstrInfo::anchor() {}

SimpleInstrInfo::SimpleInstrInfo()
  : SimpleGenInstrInfo(Simple::ADJCALLSTACKDOWN, Simple::ADJCALLSTACKUP),
    RI() 
{
}
