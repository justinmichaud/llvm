#ifndef SIMPLE_TARGETSTREAMER_H
#define SIMPLE_TARGETSTREAMER_H

#include "llvm/MC/MCStreamer.h"

namespace llvm {
class SimpleTargetStreamer : public MCTargetStreamer {
public:
  SimpleTargetStreamer(MCStreamer &S);
  virtual ~SimpleTargetStreamer();
  virtual void emitCCTopData(StringRef Name) = 0;
  virtual void emitCCTopFunction(StringRef Name) = 0;
  virtual void emitCCBottomData(StringRef Name) = 0;
  virtual void emitCCBottomFunction(StringRef Name) = 0;
};
}

#endif // SIMPLE_TARGETSTREAMER_H
