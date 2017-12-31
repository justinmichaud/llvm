//===---- M650216ISelDAGToDAG.h - A Dag to Dag Inst Selector for M6502 ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Subclass of M6502DAGToDAGISel specialized for m650216.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M650216ISELDAGTODAG_H
#define LLVM_LIB_TARGET_M6502_M650216ISELDAGTODAG_H

#include "M6502ISelDAGToDAG.h"

namespace llvm {

class M650216DAGToDAGISel : public M6502DAGToDAGISel {
public:
  explicit M650216DAGToDAGISel(M6502TargetMachine &TM, CodeGenOpt::Level OL)
      : M6502DAGToDAGISel(TM, OL) {}

private:
  std::pair<SDNode *, SDNode *> selectMULT(SDNode *N, unsigned Opc,
                                           const SDLoc &DL, EVT Ty, bool HasLo,
                                           bool HasHi);

  bool runOnMachineFunction(MachineFunction &MF) override;

  bool selectAddr(bool SPAllowed, SDValue Addr, SDValue &Base,
                  SDValue &Offset);
  bool selectAddr16(SDValue Addr, SDValue &Base,
                    SDValue &Offset) override;
  bool selectAddr16SP(SDValue Addr, SDValue &Base,
                      SDValue &Offset) override;

  bool trySelect(SDNode *Node) override;

  void processFunctionAfterISel(MachineFunction &MF) override;

  // Insert instructions to initialize the global base register in the
  // first MBB of the function.
  void initGlobalBaseReg(MachineFunction &MF);

  void initM650216SPAliasReg(MachineFunction &MF);
};

FunctionPass *createM650216ISelDag(M6502TargetMachine &TM,
                                  CodeGenOpt::Level OptLevel);
}

#endif
