#ifndef SIMPLE_MCTARGETDESC_H
#define SIMPLE_MCTARGETDESC_H

namespace llvm {
class Target;

extern Target TheSimpleTarget;

} // End llvm namespace


// Defines symbolic names for Simple registers.  This defines a mapping from
// register name to register number.
//
#define GET_REGINFO_ENUM
#include "SimpleGenRegisterInfo.inc"


#define GET_SUBTARGETINFO_ENUM
#include "SimpleGenSubtargetInfo.inc"

#endif // SIMPLE_MCTARGETDESC_H
