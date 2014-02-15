#include "SimpleTargetObjectFile.h"
#include "SimpleSubtarget.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/Support/ELF.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

void SimpleTargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM)
{
  TargetLoweringObjectFileELF::Initialize(Ctx, TM);

  SmallDataSection =
    getContext().getELFSection(".sdata", ELF::SHT_PROGBITS,
    ELF::SHF_WRITE |ELF::SHF_ALLOC,
    SectionKind::getDataRel());

  SmallBSSSection =
    getContext().getELFSection(".sbss", ELF::SHT_NOBITS,
    ELF::SHF_WRITE |ELF::SHF_ALLOC,
    SectionKind::getBSS());
}
