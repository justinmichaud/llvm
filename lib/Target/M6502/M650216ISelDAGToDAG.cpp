//===-- M650216ISelDAGToDAG.cpp - A Dag to Dag Inst Selector for M650216 ----===//
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

#include "M650216ISelDAGToDAG.h"
#include "MCTargetDesc/M6502BaseInfo.h"
#include "M6502.h"
#include "M6502MachineFunction.h"
#include "M6502RegisterInfo.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;

#define DEBUG_TYPE "m6502-isel"

bool M650216DAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const M6502Subtarget &>(MF.getSubtarget());
  if (!Subtarget->inM650216Mode())
    return false;
  return M6502DAGToDAGISel::runOnMachineFunction(MF);
}
/// Select multiply instructions.
std::pair<SDNode *, SDNode *>
M650216DAGToDAGISel::selectMULT(SDNode *N, unsigned Opc, const SDLoc &DL, EVT Ty,
                               bool HasLo, bool HasHi) {
  SDNode *Lo = nullptr, *Hi = nullptr;
  SDNode *Mul = CurDAG->getMachineNode(Opc, DL, MVT::Glue, N->getOperand(0),
                                       N->getOperand(1));
  SDValue InFlag = SDValue(Mul, 0);

  if (HasLo) {
    unsigned Opcode = M6502::Mflo16;
    Lo = CurDAG->getMachineNode(Opcode, DL, Ty, MVT::Glue, InFlag);
    InFlag = SDValue(Lo, 1);
  }
  if (HasHi) {
    unsigned Opcode = M6502::Mfhi16;
    Hi = CurDAG->getMachineNode(Opcode, DL, Ty, InFlag);
  }
  return std::make_pair(Lo, Hi);
}

void M650216DAGToDAGISel::initGlobalBaseReg(MachineFunction &MF) {
  M6502FunctionInfo *M6502FI = MF.getInfo<M6502FunctionInfo>();

  if (!M6502FI->globalBaseRegSet())
    return;

  MachineBasicBlock &MBB = MF.front();
  MachineBasicBlock::iterator I = MBB.begin();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  const TargetInstrInfo &TII = *Subtarget->getInstrInfo();
  DebugLoc DL;
  unsigned V0, V1, V2, GlobalBaseReg = M6502FI->getGlobalBaseReg();
  const TargetRegisterClass *RC = &M6502::CPU16RegsRegClass;

  V0 = RegInfo.createVirtualRegister(RC);
  V1 = RegInfo.createVirtualRegister(RC);
  V2 = RegInfo.createVirtualRegister(RC);


  BuildMI(MBB, I, DL, TII.get(M6502::LiRxImmX16), V0)
      .addExternalSymbol("_gp_disp", M6502II::MO_ABS_HI);
  BuildMI(MBB, I, DL, TII.get(M6502::AddiuRxPcImmX16), V1)
      .addExternalSymbol("_gp_disp", M6502II::MO_ABS_LO);

  BuildMI(MBB, I, DL, TII.get(M6502::SllX16), V2).addReg(V0).addImm(16);
  BuildMI(MBB, I, DL, TII.get(M6502::AdduRxRyRz16), GlobalBaseReg)
      .addReg(V1)
      .addReg(V2);
}

void M650216DAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
  initGlobalBaseReg(MF);
}

bool M650216DAGToDAGISel::selectAddr(bool SPAllowed, SDValue Addr, SDValue &Base,
                                    SDValue &Offset) {
  SDLoc DL(Addr);
  EVT ValTy = Addr.getValueType();

  // if Address is FI, get the TargetFrameIndex.
  if (SPAllowed) {
    if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
      Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
      Offset = CurDAG->getTargetConstant(0, DL, ValTy);
      return true;
    }
  }
  // on PIC code Load GA
  if (Addr.getOpcode() == M6502ISD::Wrapper) {
    Base = Addr.getOperand(0);
    Offset = Addr.getOperand(1);
    return true;
  }
  if (!TM.isPositionIndependent()) {
    if ((Addr.getOpcode() == ISD::TargetExternalSymbol ||
         Addr.getOpcode() == ISD::TargetGlobalAddress))
      return false;
  }
  // Addresses of the form FI+const or FI|const
  if (CurDAG->isBaseWithConstantOffset(Addr)) {
    ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1));
    if (isInt<16>(CN->getSExtValue())) {
      // If the first operand is a FI, get the TargetFI Node
      if (SPAllowed) {
        if (FrameIndexSDNode *FIN =
                dyn_cast<FrameIndexSDNode>(Addr.getOperand(0))) {
          Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), ValTy);
          Offset = CurDAG->getTargetConstant(CN->getZExtValue(), DL, ValTy);
          return true;
        }
      }

      Base = Addr.getOperand(0);
      Offset = CurDAG->getTargetConstant(CN->getZExtValue(), DL, ValTy);
      return true;
    }
  }
  // Operand is a result from an ADD.
  if (Addr.getOpcode() == ISD::ADD) {
    // When loading from constant pools, load the lower address part in
    // the instruction itself. Example, instead of:
    //  lui $2, %hi($CPI1_0)
    //  addiu $2, $2, %lo($CPI1_0)
    //  lwc1 $f0, 0($2)
    // Generate:
    //  lui $2, %hi($CPI1_0)
    //  lwc1 $f0, %lo($CPI1_0)($2)
    if (Addr.getOperand(1).getOpcode() == M6502ISD::Lo ||
        Addr.getOperand(1).getOpcode() == M6502ISD::GPRel) {
      SDValue Opnd0 = Addr.getOperand(1).getOperand(0);
      if (isa<ConstantPoolSDNode>(Opnd0) || isa<GlobalAddressSDNode>(Opnd0) ||
          isa<JumpTableSDNode>(Opnd0)) {
        Base = Addr.getOperand(0);
        Offset = Opnd0;
        return true;
      }
    }
  }
  Base = Addr;
  Offset = CurDAG->getTargetConstant(0, DL, ValTy);
  return true;
}

bool M650216DAGToDAGISel::selectAddr16(SDValue Addr, SDValue &Base,
                                      SDValue &Offset) {
  return selectAddr(false, Addr, Base, Offset);
}

bool M650216DAGToDAGISel::selectAddr16SP(SDValue Addr, SDValue &Base,
                                        SDValue &Offset) {
  return selectAddr(true, Addr, Base, Offset);
}

/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
bool M650216DAGToDAGISel::trySelect(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();
  SDLoc DL(Node);

  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///
  EVT NodeTy = Node->getValueType(0);
  unsigned MultOpc;

  switch (Opcode) {
  default:
    break;

  case ISD::SUBE:
  case ISD::ADDE: {
    SDValue InFlag = Node->getOperand(2), CmpLHS;
    unsigned Opc = InFlag.getOpcode();
    (void)Opc;
    assert(((Opc == ISD::ADDC || Opc == ISD::ADDE) ||
            (Opc == ISD::SUBC || Opc == ISD::SUBE)) &&
           "(ADD|SUB)E flag operand must come from (ADD|SUB)C/E insn");

    unsigned MOp;
    if (Opcode == ISD::ADDE) {
      CmpLHS = InFlag.getValue(0);
      MOp = M6502::AdduRxRyRz16;
    } else {
      CmpLHS = InFlag.getOperand(0);
      MOp = M6502::SubuRxRyRz16;
    }

    SDValue Ops[] = {CmpLHS, InFlag.getOperand(1)};

    SDValue LHS = Node->getOperand(0);
    SDValue RHS = Node->getOperand(1);

    EVT VT = LHS.getValueType();

    unsigned Sltu_op = M6502::SltuRxRyRz16;
    SDNode *Carry = CurDAG->getMachineNode(Sltu_op, DL, VT, Ops);
    unsigned Addu_op = M6502::AdduRxRyRz16;
    SDNode *AddCarry =
        CurDAG->getMachineNode(Addu_op, DL, VT, SDValue(Carry, 0), RHS);

    CurDAG->SelectNodeTo(Node, MOp, VT, MVT::Glue, LHS, SDValue(AddCarry, 0));
    return true;
  }

  /// Mul with two results
  case ISD::SMUL_LOHI:
  case ISD::UMUL_LOHI: {
    MultOpc = (Opcode == ISD::UMUL_LOHI ? M6502::MultuRxRy16 : M6502::MultRxRy16);
    std::pair<SDNode *, SDNode *> LoHi =
        selectMULT(Node, MultOpc, DL, NodeTy, true, true);
    if (!SDValue(Node, 0).use_empty())
      ReplaceUses(SDValue(Node, 0), SDValue(LoHi.first, 0));

    if (!SDValue(Node, 1).use_empty())
      ReplaceUses(SDValue(Node, 1), SDValue(LoHi.second, 0));

    CurDAG->RemoveDeadNode(Node);
    return true;
  }

  case ISD::MULHS:
  case ISD::MULHU: {
    MultOpc = (Opcode == ISD::MULHU ? M6502::MultuRxRy16 : M6502::MultRxRy16);
    auto LoHi = selectMULT(Node, MultOpc, DL, NodeTy, false, true);
    ReplaceNode(Node, LoHi.second);
    return true;
  }
  }

  return false;
}

FunctionPass *llvm::createM650216ISelDag(M6502TargetMachine &TM,
                                        CodeGenOpt::Level OptLevel) {
  return new M650216DAGToDAGISel(TM, OptLevel);
}
