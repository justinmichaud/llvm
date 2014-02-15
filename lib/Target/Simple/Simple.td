include "llvm/Target/Target.td"

include "SimpleRegisterInfo.td"
include "SimpleInstrInfo.td"

def SimpleInstrInfo : InstrInfo;

class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, NoItineraries, Features>;

def : Proc<"generic",      []>;

def Simple : Target {
  let InstructionSet = SimpleInstrInfo;
}
