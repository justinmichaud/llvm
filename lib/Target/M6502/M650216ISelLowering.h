//===-- M650216ISelLowering.h - M650216 DAG Lowering Interface ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of M6502TargetLowering specialized for mips16.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M650216ISELLOWERING_H
#define LLVM_LIB_TARGET_M6502_M650216ISELLOWERING_H

#include "M6502ISelLowering.h"

namespace llvm {
  class M650216TargetLowering : public M6502TargetLowering  {
  public:
    explicit M650216TargetLowering(const M6502TargetMachine &TM,
                                  const M6502Subtarget &STI);

    bool allowsMisalignedMemoryAccesses(EVT VT, unsigned AddrSpace,
                                        unsigned Align,
                                        bool *Fast) const override;

    MachineBasicBlock *
    EmitInstrWithCustomInserter(MachineInstr &MI,
                                MachineBasicBlock *MBB) const override;

  private:
    bool isEligibleForTailCallOptimization(
        const CCState &CCInfo, unsigned NextStackOffset,
        const M6502FunctionInfo &FI) const override;

    void setM650216HardFloatLibCalls();

    unsigned int
      getM650216HelperFunctionStubNumber(ArgListTy &Args) const;

    const char *getM650216HelperFunction
      (Type* RetTy, ArgListTy &Args, bool &needHelper) const;

    void
    getOpndList(SmallVectorImpl<SDValue> &Ops,
                std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
                bool IsPICCall, bool GlobalOrExternal, bool InternalLinkage,
                bool IsCallReloc, CallLoweringInfo &CLI, SDValue Callee,
                SDValue Chain) const override;

    MachineBasicBlock *emitSel16(unsigned Opc, MachineInstr &MI,
                                 MachineBasicBlock *BB) const;

    MachineBasicBlock *emitSeliT16(unsigned Opc1, unsigned Opc2,
                                   MachineInstr &MI,
                                   MachineBasicBlock *BB) const;

    MachineBasicBlock *emitSelT16(unsigned Opc1, unsigned Opc2,
                                  MachineInstr &MI,
                                  MachineBasicBlock *BB) const;

    MachineBasicBlock *emitFEXT_T8I816_ins(unsigned BtOpc, unsigned CmpOpc,
                                           MachineInstr &MI,
                                           MachineBasicBlock *BB) const;

    MachineBasicBlock *emitFEXT_T8I8I16_ins(unsigned BtOpc, unsigned CmpiOpc,
                                            unsigned CmpiXOpc, bool ImmSigned,
                                            MachineInstr &MI,
                                            MachineBasicBlock *BB) const;

    MachineBasicBlock *emitFEXT_CCRX16_ins(unsigned SltOpc, MachineInstr &MI,
                                           MachineBasicBlock *BB) const;

    MachineBasicBlock *emitFEXT_CCRXI16_ins(unsigned SltiOpc, unsigned SltiXOpc,
                                            MachineInstr &MI,
                                            MachineBasicBlock *BB) const;
  };
}

#endif
