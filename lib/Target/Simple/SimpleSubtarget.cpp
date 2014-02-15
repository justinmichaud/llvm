#include "SimpleSubtarget.h"
#include "Simple.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "SimpleGenSubtargetInfo.inc"

using namespace llvm;

void SimpleSubtarget::anchor() { }

SimpleSubtarget::SimpleSubtarget(const std::string &TT,
                                 const std::string &CPU, const std::string &FS)
  : SimpleGenSubtargetInfo(TT, CPU, FS)
{
}
