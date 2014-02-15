#ifndef SIMPLE_MCTARGETDESC_H
#define SIMPLE_MCTARGETDESC_H

namespace llvm {
class Target;

extern Target TheSimpleTarget;

} // End llvm namespace

#define GET_SUBTARGETINFO_ENUM
#include "SimpleGenSubtargetInfo.inc"

#endif // SIMPLE_MCTARGETDESC_H
