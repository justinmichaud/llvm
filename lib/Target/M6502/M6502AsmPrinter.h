//===- M6502AsmPrinter.h - M6502 LLVM Assembly Printer -----------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// M6502 Assembly printer class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_M6502_M6502ASMPRINTER_H
#define LLVM_LIB_TARGET_M6502_M6502ASMPRINTER_H

#include "M650216HardFloatInfo.h"
#include "M6502MCInstLower.h"
#include "M6502Subtarget.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/Compiler.h"
#include <algorithm>
#include <map>
#include <memory>

namespace llvm {

class MCOperand;
class MCSubtargetInfo;
class MCSymbol;
class MachineBasicBlock;
class MachineConstantPool;
class MachineFunction;
class MachineInstr;
class MachineOperand;
class M6502FunctionInfo;
class M6502TargetStreamer;
class Module;
class raw_ostream;
class TargetMachine;

class LLVM_LIBRARY_VISIBILITY M6502AsmPrinter : public AsmPrinter {
  M6502TargetStreamer &getTargetStreamer() const;

  void EmitInstrWithMacroNoAT(const MachineInstr *MI);

  //===------------------------------------------------------------------===//
  // XRay implementation
  //===------------------------------------------------------------------===//

public:
  // XRay-specific lowering for M6502.
  void LowerPATCHABLE_FUNCTION_ENTER(const MachineInstr &MI);
  void LowerPATCHABLE_FUNCTION_EXIT(const MachineInstr &MI);
  void LowerPATCHABLE_TAIL_CALL(const MachineInstr &MI);
  // Helper function that emits the XRay sleds we've collected for a particular
  // function.
  void EmitXRayTable();

private:
  /// MCP - Keep a pointer to constantpool entries of the current
  /// MachineFunction.
  const MachineConstantPool *MCP = nullptr;

  /// InConstantPool - Maintain state when emitting a sequence of constant
  /// pool entries so we can properly mark them as data regions.
  bool InConstantPool = false;

  std::map<const char *, const M650216HardFloatInfo::FuncSignature *>
      StubsNeeded;

  void EmitSled(const MachineInstr &MI, SledKind Kind);

  // tblgen'erated function.
  bool emitPseudoExpansionLowering(MCStreamer &OutStreamer,
                                   const MachineInstr *MI);

  // Emit PseudoReturn, PseudoReturn64, PseudoIndirectBranch,
  // and PseudoIndirectBranch64 as a JR, JR_MM, JALR, or JALR64 as appropriate
  // for the target.
  void emitPseudoIndirectBranch(MCStreamer &OutStreamer,
                                const MachineInstr *MI);

  // lowerOperand - Convert a MachineOperand into the equivalent MCOperand.
  bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp);

  void emitInlineAsmStart() const override;

  void emitInlineAsmEnd(const MCSubtargetInfo &StartInfo,
                        const MCSubtargetInfo *EndInfo) const override;

  void EmitJal(const MCSubtargetInfo &STI, MCSymbol *Symbol);

  void EmitInstrReg(const MCSubtargetInfo &STI, unsigned Opcode, unsigned Reg);

  void EmitInstrRegReg(const MCSubtargetInfo &STI, unsigned Opcode,
                       unsigned Reg1, unsigned Reg2);

  void EmitInstrRegRegReg(const MCSubtargetInfo &STI, unsigned Opcode,
                          unsigned Reg1, unsigned Reg2, unsigned Reg3);

  void EmitMovFPIntPair(const MCSubtargetInfo &STI, unsigned MovOpc,
                        unsigned Reg1, unsigned Reg2, unsigned FPReg1,
                        unsigned FPReg2, bool LE);

  void EmitSwapFPIntParams(const MCSubtargetInfo &STI,
                           M650216HardFloatInfo::FPParamVariant, bool LE,
                           bool ToFP);

  void EmitSwapFPIntRetval(const MCSubtargetInfo &STI,
                           M650216HardFloatInfo::FPReturnVariant, bool LE);

  void EmitFPCallStub(const char *, const M650216HardFloatInfo::FuncSignature *);

  void NaClAlignIndirectJumpTargets(MachineFunction &MF);

  bool isLongBranchPseudo(int Opcode) const;

public:
  const M6502Subtarget *Subtarget;
  const M6502FunctionInfo *M6502FI;
  M6502MCInstLower MCInstLowering;

  explicit M6502AsmPrinter(TargetMachine &TM,
                          std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), MCInstLowering(*this) {}

  StringRef getPassName() const override { return "M6502 Assembly Printer"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void EmitConstantPool() override {
    bool UsingConstantPools =
      (Subtarget->inM650216Mode() && Subtarget->useConstantIslands());
    if (!UsingConstantPools)
      AsmPrinter::EmitConstantPool();
    // we emit constant pools customly!
  }

  void EmitInstruction(const MachineInstr *MI) override;
  void printSavedRegsBitmask();
  void emitFrameDirective();
  const char *getCurrentABIString() const;
  void EmitFunctionEntryLabel() override;
  void EmitFunctionBodyStart() override;
  void EmitFunctionBodyEnd() override;
  void EmitBasicBlockEnd(const MachineBasicBlock &MBB) override;
  bool isBlockOnlyReachableByFallthrough(
                                   const MachineBasicBlock* MBB) const override;
  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                       unsigned AsmVariant, const char *ExtraCode,
                       raw_ostream &O) override;
  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNum,
                             unsigned AsmVariant, const char *ExtraCode,
                             raw_ostream &O) override;
  void printOperand(const MachineInstr *MI, int opNum, raw_ostream &O);
  void printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &O);
  void printMemOperandEA(const MachineInstr *MI, int opNum, raw_ostream &O);
  void printFCCOperand(const MachineInstr *MI, int opNum, raw_ostream &O,
                       const char *Modifier = nullptr);
  void printRegisterList(const MachineInstr *MI, int opNum, raw_ostream &O);
  void EmitStartOfAsmFile(Module &M) override;
  void EmitEndOfAsmFile(Module &M) override;
  void PrintDebugValueComment(const MachineInstr *MI, raw_ostream &OS);
  void EmitDebugThreadLocal(const MCExpr *Value, unsigned Size) const override;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_M6502_M6502ASMPRINTER_H
