#ifndef SIMPLE_TARGETASMINFO_H
#define SIMPLE_TARGETASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
  class StringRef;
  class Target;

  class SimpleMCAsmInfo : public MCAsmInfoELF {
    virtual void anchor();
  public:
    explicit SimpleMCAsmInfo(StringRef TT);
  };

} // namespace llvm

#endif // SIMPLE_TARGETASMINFO_H
