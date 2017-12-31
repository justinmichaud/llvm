//===- M6502AsmPrinter.cpp - M6502 LLVM Assembly Printer --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format M6502 assembly language.
//
//===----------------------------------------------------------------------===//

#include "M6502AsmPrinter.h"
#include "InstPrinter/M6502InstPrinter.h"
#include "MCTargetDesc/M6502ABIInfo.h"
#include "MCTargetDesc/M6502BaseInfo.h"
#include "MCTargetDesc/M6502MCNaCl.h"
#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "M6502.h"
#include "M6502MCInstLower.h"
#include "M6502MachineFunction.h"
#include "M6502Subtarget.h"
#include "M6502TargetMachine.h"
#include "M6502TargetStreamer.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/ADT/Twine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/IR/Instructions.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/Target/TargetSubtargetInfo.h"
#include <cassert>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace llvm;

#define DEBUG_TYPE "m6502-asm-printer"

M6502TargetStreamer &M6502AsmPrinter::getTargetStreamer() const {
  return static_cast<M6502TargetStreamer &>(*OutStreamer->getTargetStreamer());
}

bool M6502AsmPrinter::runOnMachineFunction(MachineFunction &MF) {
  Subtarget = &MF.getSubtarget<M6502Subtarget>();

  M6502FI = MF.getInfo<M6502FunctionInfo>();
  if (Subtarget->inM650216Mode())
    for (std::map<
             const char *,
             const M650216HardFloatInfo::FuncSignature *>::const_iterator
             it = M6502FI->StubsNeeded.begin();
         it != M6502FI->StubsNeeded.end(); ++it) {
      const char *Symbol = it->first;
      const M650216HardFloatInfo::FuncSignature *Signature = it->second;
      if (StubsNeeded.find(Symbol) == StubsNeeded.end())
        StubsNeeded[Symbol] = Signature;
    }
  MCP = MF.getConstantPool();

  // In NaCl, all indirect jump targets must be aligned to bundle size.
  if (Subtarget->isTargetNaCl())
    NaClAlignIndirectJumpTargets(MF);

  AsmPrinter::runOnMachineFunction(MF);

  emitXRayTable();

  return true;
}

bool M6502AsmPrinter::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  MCOp = MCInstLowering.LowerOperand(MO);
  return MCOp.isValid();
}

#include "M6502GenMCPseudoLowering.inc"

// Lower PseudoReturn/PseudoIndirectBranch/PseudoIndirectBranch64 to JR, JR_MM,
// JALR, or JALR64 as appropriate for the target.
void M6502AsmPrinter::emitPseudoIndirectBranch(MCStreamer &OutStreamer,
                                              const MachineInstr *MI) {
  bool HasLinkReg = false;
  bool InMicroM6502Mode = Subtarget->inMicroM6502Mode();
  MCInst TmpInst0;

  if (Subtarget->hasM650264r6()) {
    // M650264r6 should use (JALR64 ZERO_64, $rs)
    TmpInst0.setOpcode(M6502::JALR64);
    HasLinkReg = true;
  } else if (Subtarget->hasM650232r6()) {
    // M650232r6 should use (JALR ZERO, $rs)
    if (InMicroM6502Mode)
      TmpInst0.setOpcode(M6502::JRC16_MMR6);
    else {
      TmpInst0.setOpcode(M6502::JALR);
      HasLinkReg = true;
    }
  } else if (Subtarget->inMicroM6502Mode())
    // microM6502 should use (JR_MM $rs)
    TmpInst0.setOpcode(M6502::JR_MM);
  else {
    // Everything else should use (JR $rs)
    TmpInst0.setOpcode(M6502::JR);
  }

  MCOperand MCOp;

  if (HasLinkReg) {
    unsigned ZeroReg = Subtarget->isGP64bit() ? M6502::ZERO_64 : M6502::ZERO;
    TmpInst0.addOperand(MCOperand::createReg(ZeroReg));
  }

  lowerOperand(MI->getOperand(0), MCOp);
  TmpInst0.addOperand(MCOp);

  EmitToStreamer(OutStreamer, TmpInst0);
}

void M6502AsmPrinter::EmitInstruction(const MachineInstr *MI) {
  M6502TargetStreamer &TS = getTargetStreamer();
  unsigned Opc = MI->getOpcode();
  TS.forbidModuleDirective();

  if (MI->isDebugValue()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);

    PrintDebugValueComment(MI, OS);
    return;
  }

  // If we just ended a constant pool, mark it as such.
  if (InConstantPool && Opc != M6502::CONSTPOOL_ENTRY) {
    OutStreamer->EmitDataRegion(MCDR_DataRegionEnd);
    InConstantPool = false;
  }
  if (Opc == M6502::CONSTPOOL_ENTRY) {
    // CONSTPOOL_ENTRY - This instruction represents a floating
    // constant pool in the function.  The first operand is the ID#
    // for this instruction, the second is the index into the
    // MachineConstantPool that this is, the third is the size in
    // bytes of this constant pool entry.
    // The required alignment is specified on the basic block holding this MI.
    //
    unsigned LabelId = (unsigned)MI->getOperand(0).getImm();
    unsigned CPIdx = (unsigned)MI->getOperand(1).getIndex();

    // If this is the first entry of the pool, mark it.
    if (!InConstantPool) {
      OutStreamer->EmitDataRegion(MCDR_DataRegion);
      InConstantPool = true;
    }

    OutStreamer->EmitLabel(GetCPISymbol(LabelId));

    const MachineConstantPoolEntry &MCPE = MCP->getConstants()[CPIdx];
    if (MCPE.isMachineConstantPoolEntry())
      EmitMachineConstantPoolValue(MCPE.Val.MachineCPVal);
    else
      EmitGlobalConstant(MF->getDataLayout(), MCPE.Val.ConstVal);
    return;
  }

  switch (Opc) {
  case M6502::PATCHABLE_FUNCTION_ENTER:
    LowerPATCHABLE_FUNCTION_ENTER(*MI);
    return;
  case M6502::PATCHABLE_FUNCTION_EXIT:
    LowerPATCHABLE_FUNCTION_EXIT(*MI);
    return;
  case M6502::PATCHABLE_TAIL_CALL:
    LowerPATCHABLE_TAIL_CALL(*MI);
    return;
  }

  MachineBasicBlock::const_instr_iterator I = MI->getIterator();
  MachineBasicBlock::const_instr_iterator E = MI->getParent()->instr_end();

  do {
    // Do any auto-generated pseudo lowerings.
    if (emitPseudoExpansionLowering(*OutStreamer, &*I))
      continue;

    if (I->getOpcode() == M6502::PseudoReturn ||
        I->getOpcode() == M6502::PseudoReturn64 ||
        I->getOpcode() == M6502::PseudoIndirectBranch ||
        I->getOpcode() == M6502::PseudoIndirectBranch64 ||
        I->getOpcode() == M6502::TAILCALLREG ||
        I->getOpcode() == M6502::TAILCALLREG64) {
      emitPseudoIndirectBranch(*OutStreamer, &*I);
      continue;
    }

    // The inM650216Mode() test is not permanent.
    // Some instructions are marked as pseudo right now which
    // would make the test fail for the wrong reason but
    // that will be fixed soon. We need this here because we are
    // removing another test for this situation downstream in the
    // callchain.
    //
    if (I->isPseudo() && !Subtarget->inM650216Mode()
        && !isLongBranchPseudo(I->getOpcode()))
      llvm_unreachable("Pseudo opcode found in EmitInstruction()");

    MCInst TmpInst0;
    MCInstLowering.Lower(&*I, TmpInst0);
    EmitToStreamer(*OutStreamer, TmpInst0);
  } while ((++I != E) && I->isInsideBundle()); // Delay slot check
}

//===----------------------------------------------------------------------===//
//
//  M6502 Asm Directives
//
//  -- Frame directive "frame Stackpointer, Stacksize, RARegister"
//  Describe the stack frame.
//
//  -- Mask directives "(f)mask  bitmask, offset"
//  Tells the assembler which registers are saved and where.
//  bitmask - contain a little endian bitset indicating which registers are
//            saved on function prologue (e.g. with a 0x80000000 mask, the
//            assembler knows the register 31 (RA) is saved at prologue.
//  offset  - the position before stack pointer subtraction indicating where
//            the first saved register on prologue is located. (e.g. with a
//
//  Consider the following function prologue:
//
//    .frame  $fp,48,$ra
//    .mask   0xc0000000,-8
//       addiu $sp, $sp, -48
//       sw $ra, 40($sp)
//       sw $fp, 36($sp)
//
//    With a 0xc0000000 mask, the assembler knows the register 31 (RA) and
//    30 (FP) are saved at prologue. As the save order on prologue is from
//    left to right, RA is saved first. A -8 offset means that after the
//    stack pointer subtration, the first register in the mask (RA) will be
//    saved at address 48-8=40.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Mask directives
//===----------------------------------------------------------------------===//

// Create a bitmask with all callee saved registers for CPU or Floating Point
// registers. For CPU registers consider RA, GP and FP for saving if necessary.
void M6502AsmPrinter::printSavedRegsBitmask() {
  // CPU and FPU Saved Registers Bitmasks
  unsigned CPUBitmask = 0, FPUBitmask = 0;
  int CPUTopSavedRegOff, FPUTopSavedRegOff;

  // Set the CPU and FPU Bitmasks
  const MachineFrameInfo &MFI = MF->getFrameInfo();
  const TargetRegisterInfo *TRI = MF->getSubtarget().getRegisterInfo();
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  // size of stack area to which FP callee-saved regs are saved.
  unsigned CPURegSize = TRI->getRegSizeInBits(M6502::GPR32RegClass) / 8;
  unsigned FGR32RegSize = TRI->getRegSizeInBits(M6502::FGR32RegClass) / 8;
  unsigned AFGR64RegSize = TRI->getRegSizeInBits(M6502::AFGR64RegClass) / 8;
  bool HasAFGR64Reg = false;
  unsigned CSFPRegsSize = 0;

  for (const auto &I : CSI) {
    unsigned Reg = I.getReg();
    unsigned RegNum = TRI->getEncodingValue(Reg);

    // If it's a floating point register, set the FPU Bitmask.
    // If it's a general purpose register, set the CPU Bitmask.
    if (M6502::FGR32RegClass.contains(Reg)) {
      FPUBitmask |= (1 << RegNum);
      CSFPRegsSize += FGR32RegSize;
    } else if (M6502::AFGR64RegClass.contains(Reg)) {
      FPUBitmask |= (3 << RegNum);
      CSFPRegsSize += AFGR64RegSize;
      HasAFGR64Reg = true;
    } else if (M6502::GPR32RegClass.contains(Reg))
      CPUBitmask |= (1 << RegNum);
  }

  // FP Regs are saved right below where the virtual frame pointer points to.
  FPUTopSavedRegOff = FPUBitmask ?
    (HasAFGR64Reg ? -AFGR64RegSize : -FGR32RegSize) : 0;

  // CPU Regs are saved below FP Regs.
  CPUTopSavedRegOff = CPUBitmask ? -CSFPRegsSize - CPURegSize : 0;

  M6502TargetStreamer &TS = getTargetStreamer();
  // Print CPUBitmask
  TS.emitMask(CPUBitmask, CPUTopSavedRegOff);

  // Print FPUBitmask
  TS.emitFMask(FPUBitmask, FPUTopSavedRegOff);
}

//===----------------------------------------------------------------------===//
// Frame and Set directives
//===----------------------------------------------------------------------===//

/// Frame Directive
void M6502AsmPrinter::emitFrameDirective() {
  const TargetRegisterInfo &RI = *MF->getSubtarget().getRegisterInfo();

  unsigned stackReg  = RI.getFrameRegister(*MF);
  unsigned returnReg = RI.getRARegister();
  unsigned stackSize = MF->getFrameInfo().getStackSize();

  getTargetStreamer().emitFrame(stackReg, stackSize, returnReg);
}

/// Emit Set directives.
const char *M6502AsmPrinter::getCurrentABIString() const {
  switch (static_cast<M6502TargetMachine &>(TM).getABI().GetEnumValue()) {
  case M6502ABIInfo::ABI::O32:  return "abi32";
  case M6502ABIInfo::ABI::N32:  return "abiN32";
  case M6502ABIInfo::ABI::N64:  return "abi64";
  default: llvm_unreachable("Unknown M6502 ABI");
  }
}

void M6502AsmPrinter::EmitFunctionEntryLabel() {
  M6502TargetStreamer &TS = getTargetStreamer();

  // NaCl sandboxing requires that indirect call instructions are masked.
  // This means that function entry points should be bundle-aligned.
  if (Subtarget->isTargetNaCl())
    EmitAlignment(std::max(MF->getAlignment(), M6502_NACL_BUNDLE_ALIGN));

  if (Subtarget->inMicroM6502Mode()) {
    TS.emitDirectiveSetMicroM6502();
    TS.setUsesMicroM6502();
  } else
    TS.emitDirectiveSetNoMicroM6502();

  if (Subtarget->inM650216Mode())
    TS.emitDirectiveSetM650216();
  else
    TS.emitDirectiveSetNoM650216();

  TS.emitDirectiveEnt(*CurrentFnSym);
  OutStreamer->EmitLabel(CurrentFnSym);
}

/// EmitFunctionBodyStart - Targets can override this to emit stuff before
/// the first basic block in the function.
void M6502AsmPrinter::EmitFunctionBodyStart() {
  M6502TargetStreamer &TS = getTargetStreamer();

  MCInstLowering.Initialize(&MF->getContext());

  bool IsNakedFunction = MF->getFunction()->hasFnAttribute(Attribute::Naked);
  if (!IsNakedFunction)
    emitFrameDirective();

  if (!IsNakedFunction)
    printSavedRegsBitmask();

  if (!Subtarget->inM650216Mode()) {
    TS.emitDirectiveSetNoReorder();
    TS.emitDirectiveSetNoMacro();
    TS.emitDirectiveSetNoAt();
  }
}

/// EmitFunctionBodyEnd - Targets can override this to emit stuff after
/// the last basic block in the function.
void M6502AsmPrinter::EmitFunctionBodyEnd() {
  M6502TargetStreamer &TS = getTargetStreamer();

  // There are instruction for this macros, but they must
  // always be at the function end, and we can't emit and
  // break with BB logic.
  if (!Subtarget->inM650216Mode()) {
    TS.emitDirectiveSetAt();
    TS.emitDirectiveSetMacro();
    TS.emitDirectiveSetReorder();
  }
  TS.emitDirectiveEnd(CurrentFnSym->getName());
  // Make sure to terminate any constant pools that were at the end
  // of the function.
  if (!InConstantPool)
    return;
  InConstantPool = false;
  OutStreamer->EmitDataRegion(MCDR_DataRegionEnd);
}

void M6502AsmPrinter::EmitBasicBlockEnd(const MachineBasicBlock &MBB) {
  AsmPrinter::EmitBasicBlockEnd(MBB);
  M6502TargetStreamer &TS = getTargetStreamer();
  if (MBB.empty())
    TS.emitDirectiveInsn();
}

/// isBlockOnlyReachableByFallthough - Return true if the basic block has
/// exactly one predecessor and the control transfer mechanism between
/// the predecessor and this block is a fall-through.
bool M6502AsmPrinter::isBlockOnlyReachableByFallthrough(const MachineBasicBlock*
                                                       MBB) const {
  // The predecessor has to be immediately before this block.
  const MachineBasicBlock *Pred = *MBB->pred_begin();

  // If the predecessor is a switch statement, assume a jump table
  // implementation, so it is not a fall through.
  if (const BasicBlock *bb = Pred->getBasicBlock())
    if (isa<SwitchInst>(bb->getTerminator()))
      return false;

  // If this is a landing pad, it isn't a fall through.  If it has no preds,
  // then nothing falls through to it.
  if (MBB->isEHPad() || MBB->pred_empty())
    return false;

  // If there isn't exactly one predecessor, it can't be a fall through.
  MachineBasicBlock::const_pred_iterator PI = MBB->pred_begin(), PI2 = PI;
  ++PI2;

  if (PI2 != MBB->pred_end())
    return false;

  // The predecessor has to be immediately before this block.
  if (!Pred->isLayoutSuccessor(MBB))
    return false;

  // If the block is completely empty, then it definitely does fall through.
  if (Pred->empty())
    return true;

  // Otherwise, check the last instruction.
  // Check if the last terminator is an unconditional branch.
  MachineBasicBlock::const_iterator I = Pred->end();
  while (I != Pred->begin() && !(--I)->isTerminator()) ;

  return !I->isBarrier();
}

// Print out an operand for an inline asm expression.
bool M6502AsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                                     unsigned AsmVariant, const char *ExtraCode,
                                     raw_ostream &O) {
  // Does this asm operand have a single letter operand modifier?
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0) return true; // Unknown modifier.

    const MachineOperand &MO = MI->getOperand(OpNum);
    switch (ExtraCode[0]) {
    default:
      // See if this is a generic print operand
      return AsmPrinter::PrintAsmOperand(MI,OpNum,AsmVariant,ExtraCode,O);
    case 'X': // hex const int
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << "0x" << Twine::utohexstr(MO.getImm());
      return false;
    case 'x': // hex const int (low 16 bits)
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << "0x" << Twine::utohexstr(MO.getImm() & 0xffff);
      return false;
    case 'd': // decimal const int
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << MO.getImm();
      return false;
    case 'm': // decimal const int minus 1
      if ((MO.getType()) != MachineOperand::MO_Immediate)
        return true;
      O << MO.getImm() - 1;
      return false;
    case 'z':
      // $0 if zero, regular printing otherwise
      if (MO.getType() == MachineOperand::MO_Immediate && MO.getImm() == 0) {
        O << "$0";
        return false;
      }
      // If not, call printOperand as normal.
      break;
    case 'D': // Second part of a double word register operand
    case 'L': // Low order register of a double word register operand
    case 'M': // High order register of a double word register operand
    {
      if (OpNum == 0)
        return true;
      const MachineOperand &FlagsOP = MI->getOperand(OpNum - 1);
      if (!FlagsOP.isImm())
        return true;
      unsigned Flags = FlagsOP.getImm();
      unsigned NumVals = InlineAsm::getNumOperandRegisters(Flags);
      // Number of registers represented by this operand. We are looking
      // for 2 for 32 bit mode and 1 for 64 bit mode.
      if (NumVals != 2) {
        if (Subtarget->isGP64bit() && NumVals == 1 && MO.isReg()) {
          unsigned Reg = MO.getReg();
          O << '$' << M6502InstPrinter::getRegisterName(Reg);
          return false;
        }
        return true;
      }

      unsigned RegOp = OpNum;
      if (!Subtarget->isGP64bit()){
        // Endianness reverses which register holds the high or low value
        // between M and L.
        switch(ExtraCode[0]) {
        case 'M':
          RegOp = (Subtarget->isLittle()) ? OpNum + 1 : OpNum;
          break;
        case 'L':
          RegOp = (Subtarget->isLittle()) ? OpNum : OpNum + 1;
          break;
        case 'D': // Always the second part
          RegOp = OpNum + 1;
        }
        if (RegOp >= MI->getNumOperands())
          return true;
        const MachineOperand &MO = MI->getOperand(RegOp);
        if (!MO.isReg())
          return true;
        unsigned Reg = MO.getReg();
        O << '$' << M6502InstPrinter::getRegisterName(Reg);
        return false;
      }
    }
    case 'w':
      // Print MSA registers for the 'f' constraint
      // In LLVM, the 'w' modifier doesn't need to do anything.
      // We can just call printOperand as normal.
      break;
    }
  }

  printOperand(MI, OpNum, O);
  return false;
}

bool M6502AsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                           unsigned OpNum, unsigned AsmVariant,
                                           const char *ExtraCode,
                                           raw_ostream &O) {
  assert(OpNum + 1 < MI->getNumOperands() && "Insufficient operands");
  const MachineOperand &BaseMO = MI->getOperand(OpNum);
  const MachineOperand &OffsetMO = MI->getOperand(OpNum + 1);
  assert(BaseMO.isReg() && "Unexpected base pointer for inline asm memory operand.");
  assert(OffsetMO.isImm() && "Unexpected offset for inline asm memory operand.");
  int Offset = OffsetMO.getImm();

  // Currently we are expecting either no ExtraCode or 'D'
  if (ExtraCode) {
    if (ExtraCode[0] == 'D')
      Offset += 4;
    else
      return true; // Unknown modifier.
    // FIXME: M = high order bits
    // FIXME: L = low order bits
  }

  O << Offset << "($" << M6502InstPrinter::getRegisterName(BaseMO.getReg()) << ")";

  return false;
}

void M6502AsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                  raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(opNum);
  bool closeP = false;

  if (MO.getTargetFlags())
    closeP = true;

  switch(MO.getTargetFlags()) {
  case M6502II::MO_GPREL:    O << "%gp_rel("; break;
  case M6502II::MO_GOT_CALL: O << "%call16("; break;
  case M6502II::MO_GOT:      O << "%got(";    break;
  case M6502II::MO_ABS_HI:   O << "%hi(";     break;
  case M6502II::MO_ABS_LO:   O << "%lo(";     break;
  case M6502II::MO_HIGHER:   O << "%higher("; break;
  case M6502II::MO_HIGHEST:  O << "%highest(("; break;
  case M6502II::MO_TLSGD:    O << "%tlsgd(";  break;
  case M6502II::MO_GOTTPREL: O << "%gottprel("; break;
  case M6502II::MO_TPREL_HI: O << "%tprel_hi("; break;
  case M6502II::MO_TPREL_LO: O << "%tprel_lo("; break;
  case M6502II::MO_GPOFF_HI: O << "%hi(%neg(%gp_rel("; break;
  case M6502II::MO_GPOFF_LO: O << "%lo(%neg(%gp_rel("; break;
  case M6502II::MO_GOT_DISP: O << "%got_disp("; break;
  case M6502II::MO_GOT_PAGE: O << "%got_page("; break;
  case M6502II::MO_GOT_OFST: O << "%got_ofst("; break;
  }

  switch (MO.getType()) {
    case MachineOperand::MO_Register:
      O << '$'
        << StringRef(M6502InstPrinter::getRegisterName(MO.getReg())).lower();
      break;

    case MachineOperand::MO_Immediate:
      O << MO.getImm();
      break;

    case MachineOperand::MO_MachineBasicBlock:
      MO.getMBB()->getSymbol()->print(O, MAI);
      return;

    case MachineOperand::MO_GlobalAddress:
      getSymbol(MO.getGlobal())->print(O, MAI);
      break;

    case MachineOperand::MO_BlockAddress: {
      MCSymbol *BA = GetBlockAddressSymbol(MO.getBlockAddress());
      O << BA->getName();
      break;
    }

    case MachineOperand::MO_ConstantPoolIndex:
      O << getDataLayout().getPrivateGlobalPrefix() << "CPI"
        << getFunctionNumber() << "_" << MO.getIndex();
      if (MO.getOffset())
        O << "+" << MO.getOffset();
      break;

    default:
      llvm_unreachable("<unknown operand type>");
  }

  if (closeP) O << ")";
}

void M6502AsmPrinter::
printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &O) {
  // Load/Store memory operands -- imm($reg)
  // If PIC target the target is loaded as the
  // pattern lw $25,%call16($28)

  // opNum can be invalid if instruction has reglist as operand.
  // MemOperand is always last operand of instruction (base + offset).
  switch (MI->getOpcode()) {
  default:
    break;
  case M6502::SWM32_MM:
  case M6502::LWM32_MM:
    opNum = MI->getNumOperands() - 2;
    break;
  }

  printOperand(MI, opNum+1, O);
  O << "(";
  printOperand(MI, opNum, O);
  O << ")";
}

void M6502AsmPrinter::
printMemOperandEA(const MachineInstr *MI, int opNum, raw_ostream &O) {
  // when using stack locations for not load/store instructions
  // print the same way as all normal 3 operand instructions.
  printOperand(MI, opNum, O);
  O << ", ";
  printOperand(MI, opNum+1, O);
}

void M6502AsmPrinter::
printFCCOperand(const MachineInstr *MI, int opNum, raw_ostream &O,
                const char *Modifier) {
  const MachineOperand &MO = MI->getOperand(opNum);
  O << M6502::M6502FCCToString((M6502::CondCode)MO.getImm());
}

void M6502AsmPrinter::
printRegisterList(const MachineInstr *MI, int opNum, raw_ostream &O) {
  for (int i = opNum, e = MI->getNumOperands(); i != e; ++i) {
    if (i != opNum) O << ", ";
    printOperand(MI, i, O);
  }
}

void M6502AsmPrinter::EmitStartOfAsmFile(Module &M) {
  M6502TargetStreamer &TS = getTargetStreamer();

  // M6502TargetStreamer has an initialization order problem when emitting an
  // object file directly (see M6502TargetELFStreamer for full details). Work
  // around it by re-initializing the PIC state here.
  TS.setPic(OutContext.getObjectFileInfo()->isPositionIndependent());

  // Compute M6502 architecture attributes based on the default subtarget
  // that we'd have constructed. Module level directives aren't LTO
  // clean anyhow.
  // FIXME: For ifunc related functions we could iterate over and look
  // for a feature string that doesn't match the default one.
  const Triple &TT = TM.getTargetTriple();
  StringRef CPU = TM.getTargetCPU();
  StringRef FS = TM.getTargetFeatureString();
  const M6502TargetMachine &MTM = static_cast<const M6502TargetMachine &>(TM);
  const M6502Subtarget STI(TT, CPU, FS, MTM.isLittleEndian(), MTM, 0);

  bool IsABICalls = STI.isABICalls();
  const M6502ABIInfo &ABI = MTM.getABI();
  if (IsABICalls) {
    TS.emitDirectiveAbiCalls();
    // FIXME: This condition should be a lot more complicated that it is here.
    //        Ideally it should test for properties of the ABI and not the ABI
    //        itself.
    //        For the moment, I'm only correcting enough to make M6502-IV work.
    if (!isPositionIndependent() && STI.hasSym32())
      TS.emitDirectiveOptionPic0();
  }

  // Tell the assembler which ABI we are using
  std::string SectionName = std::string(".mdebug.") + getCurrentABIString();
  OutStreamer->SwitchSection(
      OutContext.getELFSection(SectionName, ELF::SHT_PROGBITS, 0));

  // NaN: At the moment we only support:
  // 1. .nan legacy (default)
  // 2. .nan 2008
  STI.isNaN2008() ? TS.emitDirectiveNaN2008()
                  : TS.emitDirectiveNaNLegacy();

  // TODO: handle O64 ABI

  TS.updateABIInfo(STI);

  // We should always emit a '.module fp=...' but binutils 2.24 does not accept
  // it. We therefore emit it when it contradicts the ABI defaults (-mfpxx or
  // -mfp64) and omit it otherwise.
  if (ABI.IsO32() && (STI.isABI_FPXX() || STI.isFP64bit()))
    TS.emitDirectiveModuleFP();

  // We should always emit a '.module [no]oddspreg' but binutils 2.24 does not
  // accept it. We therefore emit it when it contradicts the default or an
  // option has changed the default (i.e. FPXX) and omit it otherwise.
  if (ABI.IsO32() && (!STI.useOddSPReg() || STI.isABI_FPXX()))
    TS.emitDirectiveModuleOddSPReg();
}

void M6502AsmPrinter::emitInlineAsmStart() const {
  M6502TargetStreamer &TS = getTargetStreamer();

  // GCC's choice of assembler options for inline assembly code ('at', 'macro'
  // and 'reorder') is different from LLVM's choice for generated code ('noat',
  // 'nomacro' and 'noreorder').
  // In order to maintain compatibility with inline assembly code which depends
  // on GCC's assembler options being used, we have to switch to those options
  // for the duration of the inline assembly block and then switch back.
  TS.emitDirectiveSetPush();
  TS.emitDirectiveSetAt();
  TS.emitDirectiveSetMacro();
  TS.emitDirectiveSetReorder();
  OutStreamer->AddBlankLine();
}

void M6502AsmPrinter::emitInlineAsmEnd(const MCSubtargetInfo &StartInfo,
                                      const MCSubtargetInfo *EndInfo) const {
  OutStreamer->AddBlankLine();
  getTargetStreamer().emitDirectiveSetPop();
}

void M6502AsmPrinter::EmitJal(const MCSubtargetInfo &STI, MCSymbol *Symbol) {
  MCInst I;
  I.setOpcode(M6502::JAL);
  I.addOperand(
      MCOperand::createExpr(MCSymbolRefExpr::create(Symbol, OutContext)));
  OutStreamer->EmitInstruction(I, STI);
}

void M6502AsmPrinter::EmitInstrReg(const MCSubtargetInfo &STI, unsigned Opcode,
                                  unsigned Reg) {
  MCInst I;
  I.setOpcode(Opcode);
  I.addOperand(MCOperand::createReg(Reg));
  OutStreamer->EmitInstruction(I, STI);
}

void M6502AsmPrinter::EmitInstrRegReg(const MCSubtargetInfo &STI,
                                     unsigned Opcode, unsigned Reg1,
                                     unsigned Reg2) {
  MCInst I;
  //
  // Because of the current td files for M650232, the operands for MTC1
  // appear backwards from their normal assembly order. It's not a trivial
  // change to fix this in the td file so we adjust for it here.
  //
  if (Opcode == M6502::MTC1) {
    unsigned Temp = Reg1;
    Reg1 = Reg2;
    Reg2 = Temp;
  }
  I.setOpcode(Opcode);
  I.addOperand(MCOperand::createReg(Reg1));
  I.addOperand(MCOperand::createReg(Reg2));
  OutStreamer->EmitInstruction(I, STI);
}

void M6502AsmPrinter::EmitInstrRegRegReg(const MCSubtargetInfo &STI,
                                        unsigned Opcode, unsigned Reg1,
                                        unsigned Reg2, unsigned Reg3) {
  MCInst I;
  I.setOpcode(Opcode);
  I.addOperand(MCOperand::createReg(Reg1));
  I.addOperand(MCOperand::createReg(Reg2));
  I.addOperand(MCOperand::createReg(Reg3));
  OutStreamer->EmitInstruction(I, STI);
}

void M6502AsmPrinter::EmitMovFPIntPair(const MCSubtargetInfo &STI,
                                      unsigned MovOpc, unsigned Reg1,
                                      unsigned Reg2, unsigned FPReg1,
                                      unsigned FPReg2, bool LE) {
  if (!LE) {
    unsigned temp = Reg1;
    Reg1 = Reg2;
    Reg2 = temp;
  }
  EmitInstrRegReg(STI, MovOpc, Reg1, FPReg1);
  EmitInstrRegReg(STI, MovOpc, Reg2, FPReg2);
}

void M6502AsmPrinter::EmitSwapFPIntParams(const MCSubtargetInfo &STI,
                                         M650216HardFloatInfo::FPParamVariant PV,
                                         bool LE, bool ToFP) {
  using namespace M650216HardFloatInfo;

  unsigned MovOpc = ToFP ? M6502::MTC1 : M6502::MFC1;
  switch (PV) {
  case FSig:
    EmitInstrRegReg(STI, MovOpc, M6502::A0, M6502::F12);
    break;
  case FFSig:
    EmitMovFPIntPair(STI, MovOpc, M6502::A0, M6502::A1, M6502::F12, M6502::F14, LE);
    break;
  case FDSig:
    EmitInstrRegReg(STI, MovOpc, M6502::A0, M6502::F12);
    EmitMovFPIntPair(STI, MovOpc, M6502::A2, M6502::A3, M6502::F14, M6502::F15, LE);
    break;
  case DSig:
    EmitMovFPIntPair(STI, MovOpc, M6502::A0, M6502::A1, M6502::F12, M6502::F13, LE);
    break;
  case DDSig:
    EmitMovFPIntPair(STI, MovOpc, M6502::A0, M6502::A1, M6502::F12, M6502::F13, LE);
    EmitMovFPIntPair(STI, MovOpc, M6502::A2, M6502::A3, M6502::F14, M6502::F15, LE);
    break;
  case DFSig:
    EmitMovFPIntPair(STI, MovOpc, M6502::A0, M6502::A1, M6502::F12, M6502::F13, LE);
    EmitInstrRegReg(STI, MovOpc, M6502::A2, M6502::F14);
    break;
  case NoSig:
    return;
  }
}

void M6502AsmPrinter::EmitSwapFPIntRetval(
    const MCSubtargetInfo &STI, M650216HardFloatInfo::FPReturnVariant RV,
    bool LE) {
  using namespace M650216HardFloatInfo;

  unsigned MovOpc = M6502::MFC1;
  switch (RV) {
  case FRet:
    EmitInstrRegReg(STI, MovOpc, M6502::V0, M6502::F0);
    break;
  case DRet:
    EmitMovFPIntPair(STI, MovOpc, M6502::V0, M6502::V1, M6502::F0, M6502::F1, LE);
    break;
  case CFRet:
    EmitMovFPIntPair(STI, MovOpc, M6502::V0, M6502::V1, M6502::F0, M6502::F1, LE);
    break;
  case CDRet:
    EmitMovFPIntPair(STI, MovOpc, M6502::V0, M6502::V1, M6502::F0, M6502::F1, LE);
    EmitMovFPIntPair(STI, MovOpc, M6502::A0, M6502::A1, M6502::F2, M6502::F3, LE);
    break;
  case NoFPRet:
    break;
  }
}

void M6502AsmPrinter::EmitFPCallStub(
    const char *Symbol, const M650216HardFloatInfo::FuncSignature *Signature) {
  using namespace M650216HardFloatInfo;

  MCSymbol *MSymbol = OutContext.getOrCreateSymbol(StringRef(Symbol));
  bool LE = getDataLayout().isLittleEndian();
  // Construct a local MCSubtargetInfo here.
  // This is because the MachineFunction won't exist (but have not yet been
  // freed) and since we're at the global level we can use the default
  // constructed subtarget.
  std::unique_ptr<MCSubtargetInfo> STI(TM.getTarget().createMCSubtargetInfo(
      TM.getTargetTriple().str(), TM.getTargetCPU(),
      TM.getTargetFeatureString()));

  //
  // .global xxxx
  //
  OutStreamer->EmitSymbolAttribute(MSymbol, MCSA_Global);
  const char *RetType;
  //
  // make the comment field identifying the return and parameter
  // types of the floating point stub
  // # Stub function to call rettype xxxx (params)
  //
  switch (Signature->RetSig) {
  case FRet:
    RetType = "float";
    break;
  case DRet:
    RetType = "double";
    break;
  case CFRet:
    RetType = "complex";
    break;
  case CDRet:
    RetType = "double complex";
    break;
  case NoFPRet:
    RetType = "";
    break;
  }
  const char *Parms;
  switch (Signature->ParamSig) {
  case FSig:
    Parms = "float";
    break;
  case FFSig:
    Parms = "float, float";
    break;
  case FDSig:
    Parms = "float, double";
    break;
  case DSig:
    Parms = "double";
    break;
  case DDSig:
    Parms = "double, double";
    break;
  case DFSig:
    Parms = "double, float";
    break;
  case NoSig:
    Parms = "";
    break;
  }
  OutStreamer->AddComment("\t# Stub function to call " + Twine(RetType) + " " +
                          Twine(Symbol) + " (" + Twine(Parms) + ")");
  //
  // probably not necessary but we save and restore the current section state
  //
  OutStreamer->PushSection();
  //
  // .section m650216.call.fpxxxx,"ax",@progbits
  //
  MCSectionELF *M = OutContext.getELFSection(
      ".m650216.call.fp." + std::string(Symbol), ELF::SHT_PROGBITS,
      ELF::SHF_ALLOC | ELF::SHF_EXECINSTR);
  OutStreamer->SwitchSection(M, nullptr);
  //
  // .align 2
  //
  OutStreamer->EmitValueToAlignment(4);
  M6502TargetStreamer &TS = getTargetStreamer();
  //
  // .set nom650216
  // .set nomicrom6502
  //
  TS.emitDirectiveSetNoM650216();
  TS.emitDirectiveSetNoMicroM6502();
  //
  // .ent __call_stub_fp_xxxx
  // .type  __call_stub_fp_xxxx,@function
  //  __call_stub_fp_xxxx:
  //
  std::string x = "__call_stub_fp_" + std::string(Symbol);
  MCSymbolELF *Stub =
      cast<MCSymbolELF>(OutContext.getOrCreateSymbol(StringRef(x)));
  TS.emitDirectiveEnt(*Stub);
  MCSymbol *MType =
      OutContext.getOrCreateSymbol("__call_stub_fp_" + Twine(Symbol));
  OutStreamer->EmitSymbolAttribute(MType, MCSA_ELF_TypeFunction);
  OutStreamer->EmitLabel(Stub);

  // Only handle non-pic for now.
  assert(!isPositionIndependent() &&
         "should not be here if we are compiling pic");
  TS.emitDirectiveSetReorder();
  //
  // We need to add a M6502MCExpr class to MCTargetDesc to fully implement
  // stubs without raw text but this current patch is for compiler generated
  // functions and they all return some value.
  // The calling sequence for non pic is different in that case and we need
  // to implement %lo and %hi in order to handle the case of no return value
  // See the corresponding method in M650216HardFloat for details.
  //
  // mov the return address to S2.
  // we have no stack space to store it and we are about to make another call.
  // We need to make sure that the enclosing function knows to save S2
  // This should have already been handled.
  //
  // Mov $18, $31

  EmitInstrRegRegReg(*STI, M6502::OR, M6502::S2, M6502::RA, M6502::ZERO);

  EmitSwapFPIntParams(*STI, Signature->ParamSig, LE, true);

  // Jal xxxx
  //
  EmitJal(*STI, MSymbol);

  // fix return values
  EmitSwapFPIntRetval(*STI, Signature->RetSig, LE);
  //
  // do the return
  // if (Signature->RetSig == NoFPRet)
  //  llvm_unreachable("should not be any stubs here with no return value");
  // else
  EmitInstrReg(*STI, M6502::JR, M6502::S2);

  MCSymbol *Tmp = OutContext.createTempSymbol();
  OutStreamer->EmitLabel(Tmp);
  const MCSymbolRefExpr *E = MCSymbolRefExpr::create(Stub, OutContext);
  const MCSymbolRefExpr *T = MCSymbolRefExpr::create(Tmp, OutContext);
  const MCExpr *T_min_E = MCBinaryExpr::createSub(T, E, OutContext);
  OutStreamer->emitELFSize(Stub, T_min_E);
  TS.emitDirectiveEnd(x);
  OutStreamer->PopSection();
}

void M6502AsmPrinter::EmitEndOfAsmFile(Module &M) {
  // Emit needed stubs
  //
  for (std::map<
           const char *,
           const M650216HardFloatInfo::FuncSignature *>::const_iterator
           it = StubsNeeded.begin();
       it != StubsNeeded.end(); ++it) {
    const char *Symbol = it->first;
    const M650216HardFloatInfo::FuncSignature *Signature = it->second;
    EmitFPCallStub(Symbol, Signature);
  }
  // return to the text section
  OutStreamer->SwitchSection(OutContext.getObjectFileInfo()->getTextSection());
}

void M6502AsmPrinter::EmitSled(const MachineInstr &MI, SledKind Kind) {
  const uint8_t NoopsInSledCount = Subtarget->isGP64bit() ? 15 : 11;
  // For m650232 we want to emit the following pattern:
  //
  // .Lxray_sled_N:
  //   ALIGN
  //   B .tmpN
  //   11 NOP instructions (44 bytes)
  //   ADDIU T9, T9, 52 
  // .tmpN
  //
  // We need the 44 bytes (11 instructions) because at runtime, we'd
  // be patching over the full 48 bytes (12 instructions) with the following
  // pattern:
  //
  //   ADDIU    SP, SP, -8
  //   NOP
  //   SW       RA, 4(SP)
  //   SW       T9, 0(SP)
  //   LUI      T9, %hi(__xray_FunctionEntry/Exit)
  //   ORI      T9, T9, %lo(__xray_FunctionEntry/Exit)
  //   LUI      T0, %hi(function_id)
  //   JALR     T9
  //   ORI      T0, T0, %lo(function_id)
  //   LW       T9, 0(SP)
  //   LW       RA, 4(SP)
  //   ADDIU    SP, SP, 8
  //
  // We add 52 bytes to t9 because we want to adjust the function pointer to
  // the actual start of function i.e. the address just after the noop sled.
  // We do this because gp displacement relocation is emitted at the start of
  // of the function i.e after the nop sled and to correctly calculate the
  // global offset table address, t9 must hold the address of the instruction
  // containing the gp displacement relocation.
  // FIXME: Is this correct for the static relocation model?
  //
  // For m650264 we want to emit the following pattern:
  //
  // .Lxray_sled_N:
  //   ALIGN
  //   B .tmpN
  //   15 NOP instructions (60 bytes)
  // .tmpN
  //
  // We need the 60 bytes (15 instructions) because at runtime, we'd
  // be patching over the full 64 bytes (16 instructions) with the following
  // pattern:
  //
  //   DADDIU   SP, SP, -16
  //   NOP
  //   SD       RA, 8(SP)
  //   SD       T9, 0(SP)
  //   LUI      T9, %highest(__xray_FunctionEntry/Exit)
  //   ORI      T9, T9, %higher(__xray_FunctionEntry/Exit)
  //   DSLL     T9, T9, 16
  //   ORI      T9, T9, %hi(__xray_FunctionEntry/Exit)
  //   DSLL     T9, T9, 16
  //   ORI      T9, T9, %lo(__xray_FunctionEntry/Exit)
  //   LUI      T0, %hi(function_id)
  //   JALR     T9
  //   ADDIU    T0, T0, %lo(function_id)
  //   LD       T9, 0(SP)
  //   LD       RA, 8(SP)
  //   DADDIU   SP, SP, 16
  //
  OutStreamer->EmitCodeAlignment(4);
  auto CurSled = OutContext.createTempSymbol("xray_sled_", true);
  OutStreamer->EmitLabel(CurSled);
  auto Target = OutContext.createTempSymbol();

  // Emit "B .tmpN" instruction, which jumps over the nop sled to the actual
  // start of function
  const MCExpr *TargetExpr = MCSymbolRefExpr::create(
      Target, MCSymbolRefExpr::VariantKind::VK_None, OutContext);
  EmitToStreamer(*OutStreamer, MCInstBuilder(M6502::BEQ)
                                   .addReg(M6502::ZERO)
                                   .addReg(M6502::ZERO)
                                   .addExpr(TargetExpr));

  for (int8_t I = 0; I < NoopsInSledCount; I++)
    EmitToStreamer(*OutStreamer, MCInstBuilder(M6502::SLL)
                                     .addReg(M6502::ZERO)
                                     .addReg(M6502::ZERO)
                                     .addImm(0));

  OutStreamer->EmitLabel(Target);

  if (!Subtarget->isGP64bit()) {
    EmitToStreamer(*OutStreamer,
                   MCInstBuilder(M6502::ADDiu)
                       .addReg(M6502::T9)
                       .addReg(M6502::T9)
                       .addImm(0x34));
  }

  recordSled(CurSled, MI, Kind);
}

void M6502AsmPrinter::LowerPATCHABLE_FUNCTION_ENTER(const MachineInstr &MI) {
  EmitSled(MI, SledKind::FUNCTION_ENTER);
}

void M6502AsmPrinter::LowerPATCHABLE_FUNCTION_EXIT(const MachineInstr &MI) {
  EmitSled(MI, SledKind::FUNCTION_EXIT);
}

void M6502AsmPrinter::LowerPATCHABLE_TAIL_CALL(const MachineInstr &MI) {
  EmitSled(MI, SledKind::TAIL_CALL);
}

void M6502AsmPrinter::PrintDebugValueComment(const MachineInstr *MI,
                                           raw_ostream &OS) {
  // TODO: implement
}

// Emit .dtprelword or .dtpreldword directive
// and value for debug thread local expression.
void M6502AsmPrinter::EmitDebugThreadLocal(const MCExpr *Value,
                                          unsigned Size) const {
  switch (Size) {
  case 4:
    OutStreamer->EmitDTPRel32Value(Value);
    break;
  case 8:
    OutStreamer->EmitDTPRel64Value(Value);
    break;
  default:
    llvm_unreachable("Unexpected size of expression value.");
  }
}

// Align all targets of indirect branches on bundle size.  Used only if target
// is NaCl.
void M6502AsmPrinter::NaClAlignIndirectJumpTargets(MachineFunction &MF) {
  // Align all blocks that are jumped to through jump table.
  if (MachineJumpTableInfo *JtInfo = MF.getJumpTableInfo()) {
    const std::vector<MachineJumpTableEntry> &JT = JtInfo->getJumpTables();
    for (unsigned I = 0; I < JT.size(); ++I) {
      const std::vector<MachineBasicBlock*> &MBBs = JT[I].MBBs;

      for (unsigned J = 0; J < MBBs.size(); ++J)
        MBBs[J]->setAlignment(M6502_NACL_BUNDLE_ALIGN);
    }
  }

  // If basic block address is taken, block can be target of indirect branch.
  for (auto &MBB : MF) {
    if (MBB.hasAddressTaken())
      MBB.setAlignment(M6502_NACL_BUNDLE_ALIGN);
  }
}

bool M6502AsmPrinter::isLongBranchPseudo(int Opcode) const {
  return (Opcode == M6502::LONG_BRANCH_LUi
          || Opcode == M6502::LONG_BRANCH_ADDiu
          || Opcode == M6502::LONG_BRANCH_DADDiu);
}

// Force static initialization.
extern "C" void LLVMInitializeM6502AsmPrinter() {
  RegisterAsmPrinter<M6502AsmPrinter> X(getTheM6502Target());
}
