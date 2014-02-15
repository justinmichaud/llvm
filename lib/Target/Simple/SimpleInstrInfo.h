#ifndef SIMPLE_INSTRUCTIONINFO_H
#define SIMPLE_INSTRUCTIONINFO_H

#include "SimpleRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "SimpleGenInstrInfo.inc"

namespace llvm {

class SimpleInstrInfo : public SimpleGenInstrInfo {
  const SimpleRegisterInfo RI;
  virtual void anchor();
public:
  SimpleInstrInfo();

  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  ///
  virtual const TargetRegisterInfo &getRegisterInfo() const { return RI; }
};

} // end of namespace llvm

#endif // SIMPLE_INSTRUCTIONINFO_H
