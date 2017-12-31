//=== MicroM6502SizeReduction.cpp - MicroM6502 size reduction pass --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///\file
/// This pass is used to reduce the size of instructions where applicable.
///
/// TODO: Implement microM650264 support.
/// TODO: Implement support for reducing into lwp/swp instruction.
//===----------------------------------------------------------------------===//
#include "M6502.h"
#include "M6502InstrInfo.h"
#include "M6502Subtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "microm6502-reduce-size"

STATISTIC(NumReduced, "Number of 32-bit instructions reduced to 16-bit ones");

namespace {

/// Order of operands to transfer
// TODO: Will be extended when additional optimizations are added
enum OperandTransfer {
  OT_NA,          ///< Not applicable
  OT_OperandsAll, ///< Transfer all operands
  OT_Operands02,  ///< Transfer operands 0 and 2
  OT_Operand2,    ///< Transfer just operand 2
  OT_OperandsXOR, ///< Transfer operands for XOR16
};

/// Reduction type
// TODO: Will be extended when additional optimizations are added
enum ReduceType {
  RT_OneInstr ///< Reduce one instruction into a smaller instruction
};

// Information about immediate field restrictions
struct ImmField {
  ImmField() : ImmFieldOperand(-1), Shift(0), LBound(0), HBound(0) {}
  ImmField(uint8_t Shift, int16_t LBound, int16_t HBound,
           int8_t ImmFieldOperand)
      : ImmFieldOperand(ImmFieldOperand), Shift(Shift), LBound(LBound),
        HBound(HBound) {}
  int8_t ImmFieldOperand; // Immediate operand, -1 if it does not exist
  uint8_t Shift;          // Shift value
  int16_t LBound;         // Low bound of the immediate operand
  int16_t HBound;         // High bound of the immediate operand
};

/// Information about operands
// TODO: Will be extended when additional optimizations are added
struct OpInfo {
  OpInfo(enum OperandTransfer TransferOperands)
      : TransferOperands(TransferOperands) {}
  OpInfo() : TransferOperands(OT_NA) {}

  enum OperandTransfer
      TransferOperands; ///< Operands to transfer to the new instruction
};

// Information about opcodes
struct OpCodes {
  OpCodes(unsigned WideOpc, unsigned NarrowOpc)
      : WideOpc(WideOpc), NarrowOpc(NarrowOpc) {}

  unsigned WideOpc;   ///< Wide opcode
  unsigned NarrowOpc; ///< Narrow opcode
};

/// ReduceTable - A static table with information on mapping from wide
/// opcodes to narrow
struct ReduceEntry {

  enum ReduceType eRType; ///< Reduction type
  bool (*ReduceFunction)(
      MachineInstr *MI,
      const ReduceEntry &Entry); ///< Pointer to reduce function
  struct OpCodes Ops;            ///< All relevant OpCodes
  struct OpInfo OpInf;           ///< Characteristics of operands
  struct ImmField Imm;           ///< Characteristics of immediate field

  ReduceEntry(enum ReduceType RType, struct OpCodes Op,
              bool (*F)(MachineInstr *MI, const ReduceEntry &Entry),
              struct OpInfo OpInf, struct ImmField Imm)
      : eRType(RType), ReduceFunction(F), Ops(Op), OpInf(OpInf), Imm(Imm) {}

  unsigned NarrowOpc() const { return Ops.NarrowOpc; }
  unsigned WideOpc() const { return Ops.WideOpc; }
  int16_t LBound() const { return Imm.LBound; }
  int16_t HBound() const { return Imm.HBound; }
  uint8_t Shift() const { return Imm.Shift; }
  int8_t ImmField() const { return Imm.ImmFieldOperand; }
  enum OperandTransfer TransferOperands() const {
    return OpInf.TransferOperands;
  }
  enum ReduceType RType() const { return eRType; }

  // operator used by std::equal_range
  bool operator<(const unsigned int r) const { return (WideOpc() < r); }

  // operator used by std::equal_range
  friend bool operator<(const unsigned int r, const struct ReduceEntry &re) {
    return (r < re.WideOpc());
  }
};

class MicroM6502SizeReduce : public MachineFunctionPass {
public:
  static char ID;
  MicroM6502SizeReduce();

  static const M6502InstrInfo *M6502II;
  const M6502Subtarget *Subtarget;

  bool runOnMachineFunction(MachineFunction &MF) override;

  llvm::StringRef getPassName() const override {
    return "microM6502 instruction size reduction pass";
  }

private:
  /// Reduces width of instructions in the specified basic block.
  bool ReduceMBB(MachineBasicBlock &MBB);

  /// Attempts to reduce MI, returns true on success.
  bool ReduceMI(const MachineBasicBlock::instr_iterator &MII);

  // Attempts to reduce LW/SW instruction into LWSP/SWSP,
  // returns true on success.
  static bool ReduceXWtoXWSP(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce LBU/LHU instruction into LBU16/LHU16,
  // returns true on success.
  static bool ReduceLXUtoLXU16(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce SB/SH instruction into SB16/SH16,
  // returns true on success.
  static bool ReduceSXtoSX16(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce arithmetic instructions, returns true on success.
  static bool ReduceArithmeticInstructions(MachineInstr *MI,
                                           const ReduceEntry &Entry);

  // Attempts to reduce ADDIU into ADDIUSP instruction,
  // returns true on success.
  static bool ReduceADDIUToADDIUSP(MachineInstr *MI, const ReduceEntry &Entry);

  // Attempts to reduce ADDIU into ADDIUR1SP instruction,
  // returns true on success.
  static bool ReduceADDIUToADDIUR1SP(MachineInstr *MI,
                                     const ReduceEntry &Entry);

  // Attempts to reduce XOR into XOR16 instruction,
  // returns true on success.
  static bool ReduceXORtoXOR16(MachineInstr *MI, const ReduceEntry &Entry);

  // Changes opcode of an instruction.
  static bool ReplaceInstruction(MachineInstr *MI, const ReduceEntry &Entry);

  // Table with transformation rules for each instruction.
  static llvm::SmallVector<ReduceEntry, 16> ReduceTable;
};

char MicroM6502SizeReduce::ID = 0;
const M6502InstrInfo *MicroM6502SizeReduce::M6502II;

// This table must be sorted by WideOpc as a main criterion and
// ReduceType as a sub-criterion (when wide opcodes are the same).
llvm::SmallVector<ReduceEntry, 16> MicroM6502SizeReduce::ReduceTable = {

    // ReduceType, OpCodes, ReduceFunction,
    // OpInfo(TransferOperands),
    // ImmField(Shift, LBound, HBound, ImmFieldPosition)
    {RT_OneInstr, OpCodes(M6502::ADDiu, M6502::ADDIUR1SP_MM),
     ReduceADDIUToADDIUR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(M6502::ADDiu, M6502::ADDIUSP_MM), ReduceADDIUToADDIUSP,
     OpInfo(OT_Operand2), ImmField(0, 0, 0, 2)},
    {RT_OneInstr, OpCodes(M6502::ADDiu_MM, M6502::ADDIUR1SP_MM),
     ReduceADDIUToADDIUR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(M6502::ADDiu_MM, M6502::ADDIUSP_MM),
     ReduceADDIUToADDIUSP, OpInfo(OT_Operand2), ImmField(0, 0, 0, 2)},
    {RT_OneInstr, OpCodes(M6502::ADDu, M6502::ADDU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(M6502::ADDu_MM, M6502::ADDU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(M6502::LBu, M6502::LBU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(0, -1, 15, 2)},
    {RT_OneInstr, OpCodes(M6502::LBu_MM, M6502::LBU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(0, -1, 15, 2)},
    {RT_OneInstr, OpCodes(M6502::LEA_ADDiu, M6502::ADDIUR1SP_MM),
     ReduceADDIUToADDIUR1SP, OpInfo(OT_Operands02), ImmField(2, 0, 64, 2)},
    {RT_OneInstr, OpCodes(M6502::LHu, M6502::LHU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(M6502::LHu_MM, M6502::LHU16_MM), ReduceLXUtoLXU16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(M6502::LW, M6502::LWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(M6502::LW_MM, M6502::LWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(M6502::SB, M6502::SB16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(0, 0, 16, 2)},
    {RT_OneInstr, OpCodes(M6502::SB_MM, M6502::SB16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(0, 0, 16, 2)},
    {RT_OneInstr, OpCodes(M6502::SH, M6502::SH16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(M6502::SH_MM, M6502::SH16_MM), ReduceSXtoSX16,
     OpInfo(OT_OperandsAll), ImmField(1, 0, 16, 2)},
    {RT_OneInstr, OpCodes(M6502::SUBu, M6502::SUBU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(M6502::SUBu_MM, M6502::SUBU16_MM),
     ReduceArithmeticInstructions, OpInfo(OT_OperandsAll),
     ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(M6502::SW, M6502::SWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(M6502::SW_MM, M6502::SWSP_MM), ReduceXWtoXWSP,
     OpInfo(OT_OperandsAll), ImmField(2, 0, 32, 2)},
    {RT_OneInstr, OpCodes(M6502::XOR, M6502::XOR16_MM), ReduceXORtoXOR16,
     OpInfo(OT_OperandsXOR), ImmField(0, 0, 0, -1)},
    {RT_OneInstr, OpCodes(M6502::XOR_MM, M6502::XOR16_MM), ReduceXORtoXOR16,
     OpInfo(OT_OperandsXOR), ImmField(0, 0, 0, -1)}};
} // namespace

// Returns true if the machine operand MO is register SP.
static bool IsSP(const MachineOperand &MO) {
  if (MO.isReg() && ((MO.getReg() == M6502::SP)))
    return true;
  return false;
}

// Returns true if the machine operand MO is register $16, $17, or $2-$7.
static bool isMMThreeBitGPRegister(const MachineOperand &MO) {
  if (MO.isReg() && M6502::GPRMM16RegClass.contains(MO.getReg()))
    return true;
  return false;
}

// Returns true if the machine operand MO is register $0, $17, or $2-$7.
static bool isMMSourceRegister(const MachineOperand &MO) {
  if (MO.isReg() && M6502::GPRMM16ZeroRegClass.contains(MO.getReg()))
    return true;
  return false;
}

// Returns true if the operand Op is an immediate value
// and writes the immediate value into variable Imm.
static bool GetImm(MachineInstr *MI, unsigned Op, int64_t &Imm) {

  if (!MI->getOperand(Op).isImm())
    return false;
  Imm = MI->getOperand(Op).getImm();
  return true;
}

// Returns true if the value is a valid immediate for ADDIUSP.
static bool AddiuspImmValue(int64_t Value) {
  int64_t Value2 = Value >> 2;
  if (((Value & (int64_t)maskTrailingZeros<uint64_t>(2)) == Value) &&
      ((Value2 >= 2 && Value2 <= 257) || (Value2 >= -258 && Value2 <= -3)))
    return true;
  return false;
}

// Returns true if the variable Value has the number of least-significant zero
// bits equal to Shift and if the shifted value is between the bounds.
static bool InRange(int64_t Value, unsigned short Shift, int LBound,
                    int HBound) {
  int64_t Value2 = Value >> Shift;
  if (((Value & (int64_t)maskTrailingZeros<uint64_t>(Shift)) == Value) &&
      (Value2 >= LBound) && (Value2 < HBound))
    return true;
  return false;
}

// Returns true if immediate operand is in range.
static bool ImmInRange(MachineInstr *MI, const ReduceEntry &Entry) {

  int64_t offset;

  if (!GetImm(MI, Entry.ImmField(), offset))
    return false;

  if (!InRange(offset, Entry.Shift(), Entry.LBound(), Entry.HBound()))
    return false;

  return true;
}

MicroM6502SizeReduce::MicroM6502SizeReduce() : MachineFunctionPass(ID) {}

bool MicroM6502SizeReduce::ReduceMI(
    const MachineBasicBlock::instr_iterator &MII) {

  MachineInstr *MI = &*MII;
  unsigned Opcode = MI->getOpcode();

  // Search the table.
  llvm::SmallVector<ReduceEntry, 16>::const_iterator Start =
      std::begin(ReduceTable);
  llvm::SmallVector<ReduceEntry, 16>::const_iterator End =
      std::end(ReduceTable);

  std::pair<llvm::SmallVector<ReduceEntry, 16>::const_iterator,
            llvm::SmallVector<ReduceEntry, 16>::const_iterator>
      Range = std::equal_range(Start, End, Opcode);

  if (Range.first == Range.second)
    return false;

  for (llvm::SmallVector<ReduceEntry, 16>::const_iterator Entry = Range.first;
       Entry != Range.second; ++Entry)
    if (((*Entry).ReduceFunction)(&(*MII), *Entry))
      return true;

  return false;
}

bool MicroM6502SizeReduce::ReduceXWtoXWSP(MachineInstr *MI,
                                         const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroM6502SizeReduce::ReduceArithmeticInstructions(
    MachineInstr *MI, const ReduceEntry &Entry) {

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)) ||
      !isMMThreeBitGPRegister(MI->getOperand(2)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroM6502SizeReduce::ReduceADDIUToADDIUR1SP(MachineInstr *MI,
                                                 const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) || !IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroM6502SizeReduce::ReduceADDIUToADDIUSP(MachineInstr *MI,
                                               const ReduceEntry &Entry) {

  int64_t ImmValue;
  if (!GetImm(MI, Entry.ImmField(), ImmValue))
    return false;

  if (!AddiuspImmValue(ImmValue))
    return false;

  if (!IsSP(MI->getOperand(0)) || !IsSP(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroM6502SizeReduce::ReduceLXUtoLXU16(MachineInstr *MI,
                                           const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroM6502SizeReduce::ReduceSXtoSX16(MachineInstr *MI,
                                         const ReduceEntry &Entry) {

  if (!ImmInRange(MI, Entry))
    return false;

  if (!isMMSourceRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroM6502SizeReduce::ReduceXORtoXOR16(MachineInstr *MI,
                                           const ReduceEntry &Entry) {
  if (!isMMThreeBitGPRegister(MI->getOperand(0)) ||
      !isMMThreeBitGPRegister(MI->getOperand(1)) ||
      !isMMThreeBitGPRegister(MI->getOperand(2)))
    return false;

  if (!(MI->getOperand(0).getReg() == MI->getOperand(2).getReg()) &&
      !(MI->getOperand(0).getReg() == MI->getOperand(1).getReg()))
    return false;

  return ReplaceInstruction(MI, Entry);
}

bool MicroM6502SizeReduce::ReduceMBB(MachineBasicBlock &MBB) {
  bool Modified = false;
  MachineBasicBlock::instr_iterator MII = MBB.instr_begin(),
                                    E = MBB.instr_end();
  MachineBasicBlock::instr_iterator NextMII;

  // Iterate through the instructions in the basic block
  for (; MII != E; MII = NextMII) {
    NextMII = std::next(MII);
    MachineInstr *MI = &*MII;

    // Don't reduce bundled instructions or pseudo operations
    if (MI->isBundle() || MI->isTransient())
      continue;

    // Try to reduce 32-bit instruction into 16-bit instruction
    Modified |= ReduceMI(MII);
  }

  return Modified;
}

bool MicroM6502SizeReduce::ReplaceInstruction(MachineInstr *MI,
                                             const ReduceEntry &Entry) {

  enum OperandTransfer OpTransfer = Entry.TransferOperands();

  DEBUG(dbgs() << "Converting 32-bit: " << *MI);
  ++NumReduced;

  if (OpTransfer == OT_OperandsAll) {
    MI->setDesc(M6502II->get(Entry.NarrowOpc()));
    DEBUG(dbgs() << "       to 16-bit: " << *MI);
    return true;
  } else {
    MachineBasicBlock &MBB = *MI->getParent();
    const MCInstrDesc &NewMCID = M6502II->get(Entry.NarrowOpc());
    DebugLoc dl = MI->getDebugLoc();
    MachineInstrBuilder MIB = BuildMI(MBB, MI, dl, NewMCID);
    switch (OpTransfer) {
    case OT_Operand2:
      MIB.add(MI->getOperand(2));
      break;
    case OT_Operands02: {
      MIB.add(MI->getOperand(0));
      MIB.add(MI->getOperand(2));
      break;
    }
    case OT_OperandsXOR: {
      if (MI->getOperand(0).getReg() == MI->getOperand(2).getReg()) {
        MIB.add(MI->getOperand(0));
        MIB.add(MI->getOperand(1));
        MIB.add(MI->getOperand(2));
      } else {
        MIB.add(MI->getOperand(0));
        MIB.add(MI->getOperand(2));
        MIB.add(MI->getOperand(1));
      }
      break;
    }
    default:
      llvm_unreachable("Unknown operand transfer!");
    }

    // Transfer MI flags.
    MIB.setMIFlags(MI->getFlags());

    DEBUG(dbgs() << "       to 16-bit: " << *MIB);
    MBB.erase_instr(MI);
    return true;
  }
  return false;
}

bool MicroM6502SizeReduce::runOnMachineFunction(MachineFunction &MF) {

  Subtarget = &static_cast<const M6502Subtarget &>(MF.getSubtarget());

  // TODO: Add support for other subtargets:
  // microM650232r6 and microM650264r6
  if (!Subtarget->inMicroM6502Mode() || !Subtarget->hasM650232r2() ||
      Subtarget->hasM650232r6())
    return false;

  M6502II = static_cast<const M6502InstrInfo *>(Subtarget->getInstrInfo());

  bool Modified = false;
  MachineFunction::iterator I = MF.begin(), E = MF.end();

  for (; I != E; ++I)
    Modified |= ReduceMBB(*I);
  return Modified;
}

/// Returns an instance of the MicroM6502 size reduction pass.
FunctionPass *llvm::createMicroM6502SizeReductionPass() {
  return new MicroM6502SizeReduce();
}
