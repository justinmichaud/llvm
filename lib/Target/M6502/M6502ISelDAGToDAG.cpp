//===-- M6502ISelDAGToDAG.cpp - A Dag to Dag Inst Selector for M6502 --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the M6502 target.
//
//===----------------------------------------------------------------------===//

#include "M6502ISelDAGToDAG.h"
#include "MCTargetDesc/M6502BaseInfo.h"
#include "M6502.h"
#include "M650216ISelDAGToDAG.h"
#include "M6502MachineFunction.h"
#include "M6502RegisterInfo.h"
#include "M6502SEISelDAGToDAG.h"
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

//===----------------------------------------------------------------------===//
// Instruction Selector Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// M6502DAGToDAGISel - M6502 specific code to select M6502 machine
// instructions for SelectionDAG operations.
//===----------------------------------------------------------------------===//

bool M6502DAGToDAGISel::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &static_cast<const M6502Subtarget &>(MF.getSubtarget());
  bool Ret = SelectionDAGISel::runOnMachineFunction(MF);

  processFunctionAfterISel(MF);

  return Ret;
}

/// getGlobalBaseReg - Output the instructions required to put the
/// GOT address into a register.
SDNode *M6502DAGToDAGISel::getGlobalBaseReg() {
  unsigned GlobalBaseReg = MF->getInfo<M6502FunctionInfo>()->getGlobalBaseReg();
  return CurDAG->getRegister(GlobalBaseReg, getTargetLowering()->getPointerTy(
                                                CurDAG->getDataLayout()))
      .getNode();
}

/// ComplexPattern used on M6502InstrInfo
/// Used on M6502 Load/Store instructions
bool M6502DAGToDAGISel::selectAddrRegImm(SDValue Addr, SDValue &Base,
                                        SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectAddrDefault(SDValue Addr, SDValue &Base,
                                         SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddr(SDValue Addr, SDValue &Base,
                                     SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddr11MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddr12MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddr16MM(SDValue Addr, SDValue &Base,
                                       SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddrLSL2MM(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddrSImm10(SDValue Addr, SDValue &Base,
                                           SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddrSImm10Lsl1(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddrSImm10Lsl2(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectIntAddrSImm10Lsl3(SDValue Addr, SDValue &Base,
                                               SDValue &Offset) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectAddr16(SDValue Addr, SDValue &Base,
                                    SDValue &Offset) {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectAddr16SP(SDValue Addr, SDValue &Base,
                                      SDValue &Offset) {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplat(SDNode *N, APInt &Imm,
                                    unsigned MinSizeInBits) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimm1(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimm2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimm3(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimm4(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimm5(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimm6(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimm8(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatSimm5(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimmPow2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatUimmInvPow2(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatMaskL(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

bool M6502DAGToDAGISel::selectVSplatMaskR(SDValue N, SDValue &Imm) const {
  llvm_unreachable("Unimplemented function.");
  return false;
}

/// Select instructions not customized! Used for
/// expanded, promoted and normal instructions
void M6502DAGToDAGISel::Select(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();

  // Dump information about the Node being selected
  DEBUG(errs() << "Selecting: "; Node->dump(CurDAG); errs() << "\n");

  // If we have a custom node, we already have selected!
  if (Node->isMachineOpcode()) {
    DEBUG(errs() << "== "; Node->dump(CurDAG); errs() << "\n");
    Node->setNodeId(-1);
    return;
  }

  // See if subclasses can handle this node.
  if (trySelect(Node))
    return;

  switch(Opcode) {
  default: break;

  // Get target GOT address.
  case ISD::GLOBAL_OFFSET_TABLE:
    ReplaceNode(Node, getGlobalBaseReg());
    return;

#ifndef NDEBUG
  case ISD::LOAD:
  case ISD::STORE:
    assert((Subtarget->systemSupportsUnalignedAccess() ||
            cast<MemSDNode>(Node)->getMemoryVT().getSizeInBits() / 8 <=
            cast<MemSDNode>(Node)->getAlignment()) &&
           "Unexpected unaligned loads/stores.");
    break;
#endif
  }

  // Select the default instruction
  SelectCode(Node);
}

bool M6502DAGToDAGISel::
SelectInlineAsmMemoryOperand(const SDValue &Op, unsigned ConstraintID,
                             std::vector<SDValue> &OutOps) {
  // All memory constraints can at least accept raw pointers.
  switch(ConstraintID) {
  default:
    llvm_unreachable("Unexpected asm memory constraint");
  case InlineAsm::Constraint_i:
  case InlineAsm::Constraint_m:
  case InlineAsm::Constraint_R:
  case InlineAsm::Constraint_ZC:
    OutOps.push_back(Op);
    return false;
  }
  return true;
}
