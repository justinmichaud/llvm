//===- M6502OptionRecord.h - Abstraction for storing information -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// M6502OptionRecord - Abstraction for storing arbitrary information in
// ELF files. Arbitrary information (e.g. register usage) can be stored in M6502
// specific ELF sections like .M6502.options. Specific records should subclass
// M6502OptionRecord and provide an implementation to EmitM6502OptionRecord which
// basically just dumps the information into an ELF section. More information
// about .M6502.option can be found in the SysV ABI and the 64-bit ELF Object
// specification.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502OPTIONRECORD_H
#define LLVM_LIB_TARGET_M6502_M6502OPTIONRECORD_H

#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCRegisterInfo.h"
#include <cstdint>

namespace llvm {

class M6502ELFStreamer;

class M6502OptionRecord {
public:
  virtual ~M6502OptionRecord() = default;

  virtual void EmitM6502OptionRecord() = 0;
};

class M6502RegInfoRecord : public M6502OptionRecord {
public:
  M6502RegInfoRecord(M6502ELFStreamer *S, MCContext &Context)
      : Streamer(S), Context(Context) {
    ri_gprmask = 0;
    ri_cprmask[0] = ri_cprmask[1] = ri_cprmask[2] = ri_cprmask[3] = 0;
    ri_gp_value = 0;

    const MCRegisterInfo *TRI = Context.getRegisterInfo();
    GPR32RegClass = &(TRI->getRegClass(M6502::GPR32RegClassID));
    GPR64RegClass = &(TRI->getRegClass(M6502::GPR64RegClassID));
    FGR32RegClass = &(TRI->getRegClass(M6502::FGR32RegClassID));
    FGR64RegClass = &(TRI->getRegClass(M6502::FGR64RegClassID));
    AFGR64RegClass = &(TRI->getRegClass(M6502::AFGR64RegClassID));
    MSA128BRegClass = &(TRI->getRegClass(M6502::MSA128BRegClassID));
    COP0RegClass = &(TRI->getRegClass(M6502::COP0RegClassID));
    COP2RegClass = &(TRI->getRegClass(M6502::COP2RegClassID));
    COP3RegClass = &(TRI->getRegClass(M6502::COP3RegClassID));
  }

  ~M6502RegInfoRecord() override = default;

  void EmitM6502OptionRecord() override;
  void SetPhysRegUsed(unsigned Reg, const MCRegisterInfo *MCRegInfo);

private:
  M6502ELFStreamer *Streamer;
  MCContext &Context;
  const MCRegisterClass *GPR32RegClass;
  const MCRegisterClass *GPR64RegClass;
  const MCRegisterClass *FGR32RegClass;
  const MCRegisterClass *FGR64RegClass;
  const MCRegisterClass *AFGR64RegClass;
  const MCRegisterClass *MSA128BRegClass;
  const MCRegisterClass *COP0RegClass;
  const MCRegisterClass *COP2RegClass;
  const MCRegisterClass *COP3RegClass;
  uint32_t ri_gprmask;
  uint32_t ri_cprmask[4];
  int64_t ri_gp_value;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_M6502_M6502OPTIONRECORD_H
