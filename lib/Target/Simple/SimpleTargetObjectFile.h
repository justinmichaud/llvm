#ifndef SIMPLE_TARGETOBJECTFILE_H
#define SIMPLE_TARGETOBJECTFILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {

  class SimpleTargetObjectFile : public TargetLoweringObjectFileELF {
   const MCSection *SmallDataSection;
   const MCSection *SmallBSSSection;
  public:
    void Initialize(MCContext &Ctx, const TargetMachine &TM);
  };
} // end namespace llvm

#endif // SIMPLE_TARGETOBJECTFILE_H
