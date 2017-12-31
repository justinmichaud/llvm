//===-- M6502AsmParser.cpp - Parse M6502 assembly to MCInst instructions ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/M6502ABIFlagsSection.h"
#include "MCTargetDesc/M6502ABIInfo.h"
#include "MCTargetDesc/M6502BaseInfo.h"
#include "MCTargetDesc/M6502MCExpr.h"
#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "M6502TargetStreamer.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/ADT/Triple.h"
#include "llvm/ADT/Twine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCAsmParser.h"
#include "llvm/MC/MCParser/MCAsmParserExtension.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/MC/MCValue.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/SMLoc.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

using namespace llvm;

#define DEBUG_TYPE "m6502-asm-parser"

namespace llvm {

class MCInstrInfo;

} // end namespace llvm

namespace {

class M6502AssemblerOptions {
public:
  M6502AssemblerOptions(const FeatureBitset &Features_) : Features(Features_) {}

  M6502AssemblerOptions(const M6502AssemblerOptions *Opts) {
    ATReg = Opts->getATRegIndex();
    Reorder = Opts->isReorder();
    Macro = Opts->isMacro();
    Features = Opts->getFeatures();
  }

  unsigned getATRegIndex() const { return ATReg; }
  bool setATRegIndex(unsigned Reg) {
    if (Reg > 31)
      return false;

    ATReg = Reg;
    return true;
  }

  bool isReorder() const { return Reorder; }
  void setReorder() { Reorder = true; }
  void setNoReorder() { Reorder = false; }

  bool isMacro() const { return Macro; }
  void setMacro() { Macro = true; }
  void setNoMacro() { Macro = false; }

  const FeatureBitset &getFeatures() const { return Features; }
  void setFeatures(const FeatureBitset &Features_) { Features = Features_; }

  // Set of features that are either architecture features or referenced
  // by them (e.g.: FeatureNaN2008 implied by FeatureM650232r6).
  // The full table can be found in M6502GenSubtargetInfo.inc (M6502FeatureKV[]).
  // The reason we need this mask is explained in the selectArch function.
  // FIXME: Ideally we would like TableGen to generate this information.
  static const FeatureBitset AllArchRelatedMask;

private:
  unsigned ATReg = 1;
  bool Reorder = true;
  bool Macro = true;
  FeatureBitset Features;
};

} // end anonymous namespace

const FeatureBitset M6502AssemblerOptions::AllArchRelatedMask = {
    M6502::FeatureM65021, M6502::FeatureM65022, M6502::FeatureM65023,
    M6502::FeatureM65023_32, M6502::FeatureM65023_32r2, M6502::FeatureM65024,
    M6502::FeatureM65024_32, M6502::FeatureM65024_32r2, M6502::FeatureM65025,
    M6502::FeatureM65025_32r2, M6502::FeatureM650232, M6502::FeatureM650232r2,
    M6502::FeatureM650232r3, M6502::FeatureM650232r5, M6502::FeatureM650232r6,
    M6502::FeatureM650264, M6502::FeatureM650264r2, M6502::FeatureM650264r3,
    M6502::FeatureM650264r5, M6502::FeatureM650264r6, M6502::FeatureCnM6502,
    M6502::FeatureFP64Bit, M6502::FeatureGP64Bit, M6502::FeatureNaN2008
};

namespace {

class M6502AsmParser : public MCTargetAsmParser {
  M6502TargetStreamer &getTargetStreamer() {
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<M6502TargetStreamer &>(TS);
  }

  M6502ABIInfo ABI;
  SmallVector<std::unique_ptr<M6502AssemblerOptions>, 2> AssemblerOptions;
  MCSymbol *CurrentFn; // Pointer to the function being parsed. It may be a
                       // nullptr, which indicates that no function is currently
                       // selected. This usually happens after an '.end func'
                       // directive.
  bool IsLittleEndian;
  bool IsPicEnabled;
  bool IsCpRestoreSet;
  int CpRestoreOffset;
  unsigned CpSaveLocation;
  /// If true, then CpSaveLocation is a register, otherwise it's an offset.
  bool     CpSaveLocationIsRegister;

  // Print a warning along with its fix-it message at the given range.
  void printWarningWithFixIt(const Twine &Msg, const Twine &FixMsg,
                             SMRange Range, bool ShowColors = true);

#define GET_ASSEMBLER_HEADER
#include "M6502GenAsmMatcher.inc"

  unsigned
  checkEarlyTargetMatchPredicate(MCInst &Inst,
                                 const OperandVector &Operands) override;
  unsigned checkTargetMatchPredicate(MCInst &Inst) override;

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  /// Parse a register as used in CFI directives
  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;

  bool parseParenSuffix(StringRef Name, OperandVector &Operands);

  bool parseBracketSuffix(StringRef Name, OperandVector &Operands);

  bool mnemonicIsValid(StringRef Mnemonic, unsigned VariantID);

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  OperandMatchResultTy parseMemOperand(OperandVector &Operands);
  OperandMatchResultTy
  matchAnyRegisterNameWithoutDollar(OperandVector &Operands,
                                    StringRef Identifier, SMLoc S);
  OperandMatchResultTy matchAnyRegisterWithoutDollar(OperandVector &Operands,
                                                     SMLoc S);
  OperandMatchResultTy parseAnyRegister(OperandVector &Operands);
  OperandMatchResultTy parseImm(OperandVector &Operands);
  OperandMatchResultTy parseJumpTarget(OperandVector &Operands);
  OperandMatchResultTy parseInvNum(OperandVector &Operands);
  OperandMatchResultTy parseRegisterPair(OperandVector &Operands);
  OperandMatchResultTy parseMovePRegPair(OperandVector &Operands);
  OperandMatchResultTy parseRegisterList(OperandVector &Operands);

  bool searchSymbolAlias(OperandVector &Operands);

  bool parseOperand(OperandVector &, StringRef Mnemonic);

  enum MacroExpanderResultTy {
    MER_NotAMacro,
    MER_Success,
    MER_Fail,
  };

  // Expands assembly pseudo instructions.
  MacroExpanderResultTy tryExpandInstruction(MCInst &Inst, SMLoc IDLoc,
                                             MCStreamer &Out,
                                             const MCSubtargetInfo *STI);

  bool expandJalWithRegs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                         const MCSubtargetInfo *STI);

  bool loadImmediate(int64_t ImmValue, unsigned DstReg, unsigned SrcReg,
                     bool Is32BitImm, bool IsAddress, SMLoc IDLoc,
                     MCStreamer &Out, const MCSubtargetInfo *STI);

  bool loadAndAddSymbolAddress(const MCExpr *SymExpr, unsigned DstReg,
                               unsigned SrcReg, bool Is32BitSym, SMLoc IDLoc,
                               MCStreamer &Out, const MCSubtargetInfo *STI);

  bool emitPartialAddress(M6502TargetStreamer &TOut, SMLoc IDLoc, MCSymbol *Sym);

  bool expandLoadImm(MCInst &Inst, bool Is32BitImm, SMLoc IDLoc,
                     MCStreamer &Out, const MCSubtargetInfo *STI);

  bool expandLoadImmReal(MCInst &Inst, bool IsSingle, bool IsGPR, bool Is64FPU,
                         SMLoc IDLoc, MCStreamer &Out,
                         const MCSubtargetInfo *STI);

  bool expandLoadAddress(unsigned DstReg, unsigned BaseReg,
                         const MCOperand &Offset, bool Is32BitAddress,
                         SMLoc IDLoc, MCStreamer &Out,
                         const MCSubtargetInfo *STI);

  bool expandUncondBranchMMPseudo(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                  const MCSubtargetInfo *STI);

  void expandMemInst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                     const MCSubtargetInfo *STI, bool IsLoad, bool IsImmOpnd);

  void expandLoadInst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                      const MCSubtargetInfo *STI, bool IsImmOpnd);

  void expandStoreInst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI, bool IsImmOpnd);

  bool expandLoadStoreMultiple(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                               const MCSubtargetInfo *STI);

  bool expandAliasImmediate(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                            const MCSubtargetInfo *STI);

  bool expandBranchImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI);

  bool expandCondBranches(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                          const MCSubtargetInfo *STI);

  bool expandDiv(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI, const bool IsM650264,
                 const bool Signed);

  bool expandTrunc(MCInst &Inst, bool IsDouble, bool Is64FPU, SMLoc IDLoc,
                   MCStreamer &Out, const MCSubtargetInfo *STI);

  bool expandUlh(MCInst &Inst, bool Signed, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandUsh(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandUxw(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandRotation(MCInst &Inst, SMLoc IDLoc,
                      MCStreamer &Out, const MCSubtargetInfo *STI);
  bool expandRotationImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                         const MCSubtargetInfo *STI);
  bool expandDRotation(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI);
  bool expandDRotationImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                          const MCSubtargetInfo *STI);

  bool expandAbs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandMulImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                    const MCSubtargetInfo *STI);

  bool expandMulO(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                  const MCSubtargetInfo *STI);

  bool expandMulOU(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                   const MCSubtargetInfo *STI);

  bool expandDMULMacro(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                       const MCSubtargetInfo *STI);

  bool expandLoadStoreDMacro(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                             const MCSubtargetInfo *STI, bool IsLoad);

  bool expandSeq(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                 const MCSubtargetInfo *STI);

  bool expandSeqI(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                  const MCSubtargetInfo *STI);

  bool reportParseError(Twine ErrorMsg);
  bool reportParseError(SMLoc Loc, Twine ErrorMsg);

  bool parseMemOffset(const MCExpr *&Res, bool isParenExpr);

  bool isEvaluated(const MCExpr *Expr);
  bool parseSetM65020Directive();
  bool parseSetArchDirective();
  bool parseSetFeature(uint64_t Feature);
  bool isPicAndNotNxxAbi(); // Used by .cpload, .cprestore, and .cpsetup.
  bool parseDirectiveCpLoad(SMLoc Loc);
  bool parseDirectiveCpRestore(SMLoc Loc);
  bool parseDirectiveCPSetup();
  bool parseDirectiveCPReturn();
  bool parseDirectiveNaN();
  bool parseDirectiveSet();
  bool parseDirectiveOption();
  bool parseInsnDirective();
  bool parseRSectionDirective(StringRef Section);
  bool parseSSectionDirective(StringRef Section, unsigned Type);

  bool parseSetAtDirective();
  bool parseSetNoAtDirective();
  bool parseSetMacroDirective();
  bool parseSetNoMacroDirective();
  bool parseSetMsaDirective();
  bool parseSetNoMsaDirective();
  bool parseSetNoDspDirective();
  bool parseSetReorderDirective();
  bool parseSetNoReorderDirective();
  bool parseSetM650216Directive();
  bool parseSetNoM650216Directive();
  bool parseSetFpDirective();
  bool parseSetOddSPRegDirective();
  bool parseSetNoOddSPRegDirective();
  bool parseSetPopDirective();
  bool parseSetPushDirective();
  bool parseSetSoftFloatDirective();
  bool parseSetHardFloatDirective();
  bool parseSetMtDirective();
  bool parseSetNoMtDirective();

  bool parseSetAssignment();

  bool parseDataDirective(unsigned Size, SMLoc L);
  bool parseDirectiveGpWord();
  bool parseDirectiveGpDWord();
  bool parseDirectiveDtpRelWord();
  bool parseDirectiveDtpRelDWord();
  bool parseDirectiveTpRelWord();
  bool parseDirectiveTpRelDWord();
  bool parseDirectiveModule();
  bool parseDirectiveModuleFP();
  bool parseFpABIValue(M6502ABIFlagsSection::FpABIKind &FpABI,
                       StringRef Directive);

  bool parseInternalDirectiveReallowModule();

  bool eatComma(StringRef ErrorStr);

  int matchCPURegisterName(StringRef Symbol);

  int matchHWRegsRegisterName(StringRef Symbol);

  int matchFPURegisterName(StringRef Name);

  int matchFCCRegisterName(StringRef Name);

  int matchACRegisterName(StringRef Name);

  int matchMSA128RegisterName(StringRef Name);

  int matchMSA128CtrlRegisterName(StringRef Name);

  unsigned getReg(int RC, int RegNo);

  /// Returns the internal register number for the current AT. Also checks if
  /// the current AT is unavailable (set to $0) and gives an error if it is.
  /// This should be used in pseudo-instruction expansions which need AT.
  unsigned getATReg(SMLoc Loc);

  bool canUseATReg();

  bool processInstruction(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                          const MCSubtargetInfo *STI);

  // Helper function that checks if the value of a vector index is within the
  // boundaries of accepted values for each RegisterKind
  // Example: INSERT.B $w0[n], $1 => 16 > n >= 0
  bool validateMSAIndex(int Val, int RegKind);

  // Selects a new architecture by updating the FeatureBits with the necessary
  // info including implied dependencies.
  // Internally, it clears all the feature bits related to *any* architecture
  // and selects the new one using the ToggleFeature functionality of the
  // MCSubtargetInfo object that handles implied dependencies. The reason we
  // clear all the arch related bits manually is because ToggleFeature only
  // clears the features that imply the feature being cleared and not the
  // features implied by the feature being cleared. This is easier to see
  // with an example:
  //  --------------------------------------------------
  // | Feature         | Implies                        |
  // | -------------------------------------------------|
  // | FeatureM65021    | None                           |
  // | FeatureM65022    | FeatureM65021                   |
  // | FeatureM65023    | FeatureM65022 | FeatureM6502GP64 |
  // | FeatureM65024    | FeatureM65023                   |
  // | ...             |                                |
  //  --------------------------------------------------
  //
  // Setting M65023 is equivalent to set: (FeatureM65023 | FeatureM65022 |
  // FeatureM6502GP64 | FeatureM65021)
  // Clearing M65023 is equivalent to clear (FeatureM65023 | FeatureM65024).
  void selectArch(StringRef ArchFeature) {
    MCSubtargetInfo &STI = copySTI();
    FeatureBitset FeatureBits = STI.getFeatureBits();
    FeatureBits &= ~M6502AssemblerOptions::AllArchRelatedMask;
    STI.setFeatureBits(FeatureBits);
    setAvailableFeatures(
        ComputeAvailableFeatures(STI.ToggleFeature(ArchFeature)));
    AssemblerOptions.back()->setFeatures(STI.getFeatureBits());
  }

  void setFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (!(getSTI().getFeatureBits()[Feature])) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
      AssemblerOptions.back()->setFeatures(STI.getFeatureBits());
    }
  }

  void clearFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (getSTI().getFeatureBits()[Feature]) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
      AssemblerOptions.back()->setFeatures(STI.getFeatureBits());
    }
  }

  void setModuleFeatureBits(uint64_t Feature, StringRef FeatureString) {
    setFeatureBits(Feature, FeatureString);
    AssemblerOptions.front()->setFeatures(getSTI().getFeatureBits());
  }

  void clearModuleFeatureBits(uint64_t Feature, StringRef FeatureString) {
    clearFeatureBits(Feature, FeatureString);
    AssemblerOptions.front()->setFeatures(getSTI().getFeatureBits());
  }

public:
  enum M6502MatchResultTy {
    Match_RequiresDifferentSrcAndDst = FIRST_TARGET_MATCH_RESULT_TY,
    Match_RequiresDifferentOperands,
    Match_RequiresNoZeroRegister,
    Match_RequiresSameSrcAndDst,
    Match_NoFCCRegisterForCurrentISA,
    Match_NonZeroOperandForSync,
    Match_RequiresPosSizeRange0_32,
    Match_RequiresPosSizeRange33_64,
    Match_RequiresPosSizeUImm6,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "M6502GenAsmMatcher.inc"
#undef GET_OPERAND_DIAGNOSTIC_TYPES
  };

  M6502AsmParser(const MCSubtargetInfo &sti, MCAsmParser &parser,
                const MCInstrInfo &MII, const MCTargetOptions &Options)
    : MCTargetAsmParser(Options, sti, MII),
        ABI(M6502ABIInfo::computeTargetABI(Triple(sti.getTargetTriple()),
                                          sti.getCPU(), Options)) {
    MCAsmParserExtension::Initialize(parser);

    parser.addAliasForDirective(".asciiz", ".asciz");

    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));

    // Remember the initial assembler options. The user can not modify these.
    AssemblerOptions.push_back(
        llvm::make_unique<M6502AssemblerOptions>(getSTI().getFeatureBits()));

    // Create an assembler options environment for the user to modify.
    AssemblerOptions.push_back(
        llvm::make_unique<M6502AssemblerOptions>(getSTI().getFeatureBits()));

    getTargetStreamer().updateABIInfo(*this);

    if (!isABI_O32() && !useOddSPReg() != 0)
      report_fatal_error("-mno-odd-spreg requires the O32 ABI");

    CurrentFn = nullptr;

    IsPicEnabled = getContext().getObjectFileInfo()->isPositionIndependent();

    IsCpRestoreSet = false;
    CpRestoreOffset = -1;

    const Triple &TheTriple = sti.getTargetTriple();
    IsLittleEndian = false;
  }

  /// True if all of $fcc0 - $fcc7 exist for the current ISA.
  bool hasEightFccRegisters() const { return hasM65024() || hasM650232(); }

  bool isGP64bit() const {
    return getSTI().getFeatureBits()[M6502::FeatureGP64Bit];
  }

  bool isFP64bit() const {
    return getSTI().getFeatureBits()[M6502::FeatureFP64Bit];
  }

  const M6502ABIInfo &getABI() const { return ABI; }
  bool isABI_N32() const { return ABI.IsN32(); }
  bool isABI_N64() const { return ABI.IsN64(); }
  bool isABI_O32() const { return ABI.IsO32(); }
  bool isABI_FPXX() const {
    return getSTI().getFeatureBits()[M6502::FeatureFPXX];
  }

  bool useOddSPReg() const {
    return !(getSTI().getFeatureBits()[M6502::FeatureNoOddSPReg]);
  }

  bool inMicroM6502Mode() const {
    return getSTI().getFeatureBits()[M6502::FeatureMicroM6502];
  }

  bool hasM65021() const {
    return getSTI().getFeatureBits()[M6502::FeatureM65021];
  }

  bool hasM65022() const {
    return getSTI().getFeatureBits()[M6502::FeatureM65022];
  }

  bool hasM65023() const {
    return getSTI().getFeatureBits()[M6502::FeatureM65023];
  }

  bool hasM65024() const {
    return getSTI().getFeatureBits()[M6502::FeatureM65024];
  }

  bool hasM65025() const {
    return getSTI().getFeatureBits()[M6502::FeatureM65025];
  }

  bool hasM650232() const {
    return getSTI().getFeatureBits()[M6502::FeatureM650232];
  }

  bool hasM650264() const {
    return getSTI().getFeatureBits()[M6502::FeatureM650264];
  }

  bool hasM650232r2() const {
    return getSTI().getFeatureBits()[M6502::FeatureM650232r2];
  }

  bool hasM650264r2() const {
    return getSTI().getFeatureBits()[M6502::FeatureM650264r2];
  }

  bool hasM650232r3() const {
    return (getSTI().getFeatureBits()[M6502::FeatureM650232r3]);
  }

  bool hasM650264r3() const {
    return (getSTI().getFeatureBits()[M6502::FeatureM650264r3]);
  }

  bool hasM650232r5() const {
    return (getSTI().getFeatureBits()[M6502::FeatureM650232r5]);
  }

  bool hasM650264r5() const {
    return (getSTI().getFeatureBits()[M6502::FeatureM650264r5]);
  }

  bool hasM650232r6() const {
    return getSTI().getFeatureBits()[M6502::FeatureM650232r6];
  }

  bool hasM650264r6() const {
    return getSTI().getFeatureBits()[M6502::FeatureM650264r6];
  }

  bool hasDSP() const {
    return getSTI().getFeatureBits()[M6502::FeatureDSP];
  }

  bool hasDSPR2() const {
    return getSTI().getFeatureBits()[M6502::FeatureDSPR2];
  }

  bool hasDSPR3() const {
    return getSTI().getFeatureBits()[M6502::FeatureDSPR3];
  }

  bool hasMSA() const {
    return getSTI().getFeatureBits()[M6502::FeatureMSA];
  }

  bool hasCnM6502() const {
    return (getSTI().getFeatureBits()[M6502::FeatureCnM6502]);
  }

  bool inPicMode() {
    return IsPicEnabled;
  }

  bool inM650216Mode() const {
    return getSTI().getFeatureBits()[M6502::FeatureM650216];
  }

  bool useTraps() const {
    return getSTI().getFeatureBits()[M6502::FeatureUseTCCInDIV];
  }

  bool useSoftFloat() const {
    return getSTI().getFeatureBits()[M6502::FeatureSoftFloat];
  }
  bool hasMT() const {
    return getSTI().getFeatureBits()[M6502::FeatureMT];
  }

  /// Warn if RegIndex is the same as the current AT.
  void warnIfRegIndexIsAT(unsigned RegIndex, SMLoc Loc);

  void warnIfNoMacro(SMLoc Loc);

  bool isLittle() const { return IsLittleEndian; }

  const MCExpr *createTargetUnaryExpr(const MCExpr *E,
                                      AsmToken::TokenKind OperatorToken,
                                      MCContext &Ctx) override {
    switch(OperatorToken) {
    default:
      llvm_unreachable("Unknown token");
      return nullptr;
    case AsmToken::PercentCall16:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOT_CALL, E, Ctx);
    case AsmToken::PercentCall_Hi:
      return M6502MCExpr::create(M6502MCExpr::MEK_CALL_HI16, E, Ctx);
    case AsmToken::PercentCall_Lo:
      return M6502MCExpr::create(M6502MCExpr::MEK_CALL_LO16, E, Ctx);
    case AsmToken::PercentDtprel_Hi:
      return M6502MCExpr::create(M6502MCExpr::MEK_DTPREL_HI, E, Ctx);
    case AsmToken::PercentDtprel_Lo:
      return M6502MCExpr::create(M6502MCExpr::MEK_DTPREL_LO, E, Ctx);
    case AsmToken::PercentGot:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOT, E, Ctx);
    case AsmToken::PercentGot_Disp:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOT_DISP, E, Ctx);
    case AsmToken::PercentGot_Hi:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOT_HI16, E, Ctx);
    case AsmToken::PercentGot_Lo:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOT_LO16, E, Ctx);
    case AsmToken::PercentGot_Ofst:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOT_OFST, E, Ctx);
    case AsmToken::PercentGot_Page:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOT_PAGE, E, Ctx);
    case AsmToken::PercentGottprel:
      return M6502MCExpr::create(M6502MCExpr::MEK_GOTTPREL, E, Ctx);
    case AsmToken::PercentGp_Rel:
      return M6502MCExpr::create(M6502MCExpr::MEK_GPREL, E, Ctx);
    case AsmToken::PercentHi:
      return M6502MCExpr::create(M6502MCExpr::MEK_HI, E, Ctx);
    case AsmToken::PercentHigher:
      return M6502MCExpr::create(M6502MCExpr::MEK_HIGHER, E, Ctx);
    case AsmToken::PercentHighest:
      return M6502MCExpr::create(M6502MCExpr::MEK_HIGHEST, E, Ctx);
    case AsmToken::PercentLo:
      return M6502MCExpr::create(M6502MCExpr::MEK_LO, E, Ctx);
    case AsmToken::PercentNeg:
      return M6502MCExpr::create(M6502MCExpr::MEK_NEG, E, Ctx);
    case AsmToken::PercentPcrel_Hi:
      return M6502MCExpr::create(M6502MCExpr::MEK_PCREL_HI16, E, Ctx);
    case AsmToken::PercentPcrel_Lo:
      return M6502MCExpr::create(M6502MCExpr::MEK_PCREL_LO16, E, Ctx);
    case AsmToken::PercentTlsgd:
      return M6502MCExpr::create(M6502MCExpr::MEK_TLSGD, E, Ctx);
    case AsmToken::PercentTlsldm:
      return M6502MCExpr::create(M6502MCExpr::MEK_TLSLDM, E, Ctx);
    case AsmToken::PercentTprel_Hi:
      return M6502MCExpr::create(M6502MCExpr::MEK_TPREL_HI, E, Ctx);
    case AsmToken::PercentTprel_Lo:
      return M6502MCExpr::create(M6502MCExpr::MEK_TPREL_LO, E, Ctx);
    }
  }
};

/// M6502Operand - Instances of this class represent a parsed M6502 machine
/// instruction.
class M6502Operand : public MCParsedAsmOperand {
public:
  /// Broad categories of register classes
  /// The exact class is finalized by the render method.
  enum RegKind {
    RegKind_GPR = 1,      /// GPR32 and GPR64 (depending on isGP64bit())
    RegKind_FGR = 2,      /// FGR32, FGR64, AFGR64 (depending on context and
                          /// isFP64bit())
    RegKind_FCC = 4,      /// FCC
    RegKind_MSA128 = 8,   /// MSA128[BHWD] (makes no difference which)
    RegKind_MSACtrl = 16, /// MSA control registers
    RegKind_COP2 = 32,    /// COP2
    RegKind_ACC = 64,     /// HI32DSP, LO32DSP, and ACC64DSP (depending on
                          /// context).
    RegKind_CCR = 128,    /// CCR
    RegKind_HWRegs = 256, /// HWRegs
    RegKind_COP3 = 512,   /// COP3
    RegKind_COP0 = 1024,  /// COP0
    /// Potentially any (e.g. $1)
    RegKind_Numeric = RegKind_GPR | RegKind_FGR | RegKind_FCC | RegKind_MSA128 |
                      RegKind_MSACtrl | RegKind_COP2 | RegKind_ACC |
                      RegKind_CCR | RegKind_HWRegs | RegKind_COP3 | RegKind_COP0
  };

private:
  enum KindTy {
    k_Immediate,     /// An immediate (possibly involving symbol references)
    k_Memory,        /// Base + Offset Memory Address
    k_RegisterIndex, /// A register index in one or more RegKind.
    k_Token,         /// A simple token
    k_RegList,       /// A physical register list
    k_RegPair        /// A pair of physical register
  } Kind;

public:
  M6502Operand(KindTy K, M6502AsmParser &Parser)
      : MCParsedAsmOperand(), Kind(K), AsmParser(Parser) {}

  ~M6502Operand() override {
    switch (Kind) {
    case k_Immediate:
      break;
    case k_Memory:
      delete Mem.Base;
      break;
    case k_RegList:
      delete RegList.List;
    case k_RegisterIndex:
    case k_Token:
    case k_RegPair:
      break;
    }
  }

private:
  /// For diagnostics, and checking the assembler temporary
  M6502AsmParser &AsmParser;

  struct Token {
    const char *Data;
    unsigned Length;
  };

  struct RegIdxOp {
    unsigned Index; /// Index into the register class
    RegKind Kind;   /// Bitfield of the kinds it could possibly be
    struct Token Tok; /// The input token this operand originated from.
    const MCRegisterInfo *RegInfo;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  struct MemOp {
    M6502Operand *Base;
    const MCExpr *Off;
  };

  struct RegListOp {
    SmallVector<unsigned, 10> *List;
  };

  union {
    struct Token Tok;
    struct RegIdxOp RegIdx;
    struct ImmOp Imm;
    struct MemOp Mem;
    struct RegListOp RegList;
  };

  SMLoc StartLoc, EndLoc;

  /// Internal constructor for register kinds
  static std::unique_ptr<M6502Operand> CreateReg(unsigned Index, StringRef Str,
                                                RegKind RegKind,
                                                const MCRegisterInfo *RegInfo,
                                                SMLoc S, SMLoc E,
                                                M6502AsmParser &Parser) {
    auto Op = llvm::make_unique<M6502Operand>(k_RegisterIndex, Parser);
    Op->RegIdx.Index = Index;
    Op->RegIdx.RegInfo = RegInfo;
    Op->RegIdx.Kind = RegKind;
    Op->RegIdx.Tok.Data = Str.data();
    Op->RegIdx.Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

public:
  /// Coerce the register to GPR32 and return the real register for the current
  /// target.
  unsigned getGPR32Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    AsmParser.warnIfRegIndexIsAT(RegIdx.Index, StartLoc);
    unsigned ClassID = M6502::GPR32RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to GPR32 and return the real register for the current
  /// target.
  unsigned getGPRMM16Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    unsigned ClassID = M6502::GPR32RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to GPR64 and return the real register for the current
  /// target.
  unsigned getGPR64Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    unsigned ClassID = M6502::GPR64RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

private:
  /// Coerce the register to AFGR64 and return the real register for the current
  /// target.
  unsigned getAFGR64Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FGR) && "Invalid access!");
    if (RegIdx.Index % 2 != 0)
      AsmParser.Warning(StartLoc, "Float register should be even.");
    return RegIdx.RegInfo->getRegClass(M6502::AFGR64RegClassID)
        .getRegister(RegIdx.Index / 2);
  }

  /// Coerce the register to FGR64 and return the real register for the current
  /// target.
  unsigned getFGR64Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FGR) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(M6502::FGR64RegClassID)
        .getRegister(RegIdx.Index);
  }

  /// Coerce the register to FGR32 and return the real register for the current
  /// target.
  unsigned getFGR32Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FGR) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(M6502::FGR32RegClassID)
        .getRegister(RegIdx.Index);
  }

  /// Coerce the register to FGRH32 and return the real register for the current
  /// target.
  unsigned getFGRH32Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FGR) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(M6502::FGRH32RegClassID)
        .getRegister(RegIdx.Index);
  }

  /// Coerce the register to FCC and return the real register for the current
  /// target.
  unsigned getFCCReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_FCC) && "Invalid access!");
    return RegIdx.RegInfo->getRegClass(M6502::FCCRegClassID)
        .getRegister(RegIdx.Index);
  }

  /// Coerce the register to MSA128 and return the real register for the current
  /// target.
  unsigned getMSA128Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_MSA128) && "Invalid access!");
    // It doesn't matter which of the MSA128[BHWD] classes we use. They are all
    // identical
    unsigned ClassID = M6502::MSA128BRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to MSACtrl and return the real register for the
  /// current target.
  unsigned getMSACtrlReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_MSACtrl) && "Invalid access!");
    unsigned ClassID = M6502::MSACtrlRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to COP0 and return the real register for the
  /// current target.
  unsigned getCOP0Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_COP0) && "Invalid access!");
    unsigned ClassID = M6502::COP0RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to COP2 and return the real register for the
  /// current target.
  unsigned getCOP2Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_COP2) && "Invalid access!");
    unsigned ClassID = M6502::COP2RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to COP3 and return the real register for the
  /// current target.
  unsigned getCOP3Reg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_COP3) && "Invalid access!");
    unsigned ClassID = M6502::COP3RegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to ACC64DSP and return the real register for the
  /// current target.
  unsigned getACC64DSPReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_ACC) && "Invalid access!");
    unsigned ClassID = M6502::ACC64DSPRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to HI32DSP and return the real register for the
  /// current target.
  unsigned getHI32DSPReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_ACC) && "Invalid access!");
    unsigned ClassID = M6502::HI32DSPRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to LO32DSP and return the real register for the
  /// current target.
  unsigned getLO32DSPReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_ACC) && "Invalid access!");
    unsigned ClassID = M6502::LO32DSPRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to CCR and return the real register for the
  /// current target.
  unsigned getCCRReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_CCR) && "Invalid access!");
    unsigned ClassID = M6502::CCRRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

  /// Coerce the register to HWRegs and return the real register for the
  /// current target.
  unsigned getHWRegsReg() const {
    assert(isRegIdx() && (RegIdx.Kind & RegKind_HWRegs) && "Invalid access!");
    unsigned ClassID = M6502::HWRegsRegClassID;
    return RegIdx.RegInfo->getRegClass(ClassID).getRegister(RegIdx.Index);
  }

public:
  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediate when possible.  Null MCExpr = 0.
    if (!Expr)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addRegOperands(MCInst &Inst, unsigned N) const {
    llvm_unreachable("Use a custom parser instead");
  }

  /// Render the operand to an MCInst as a GPR32
  /// Asserts if the wrong number of operands are requested, or the operand
  /// is not a k_RegisterIndex compatible with RegKind_GPR
  void addGPR32ZeroAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR32Reg()));
  }

  void addGPR32NonZeroAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR32Reg()));
  }

  void addGPR32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR32Reg()));
  }

  void addGPRMM16AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  void addGPRMM16AsmRegZeroOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  void addGPRMM16AsmRegMovePOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPRMM16Reg()));
  }

  /// Render the operand to an MCInst as a GPR64
  /// Asserts if the wrong number of operands are requested, or the operand
  /// is not a k_RegisterIndex compatible with RegKind_GPR
  void addGPR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getGPR64Reg()));
  }

  void addAFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getAFGR64Reg()));
  }

  void addStrictlyAFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getAFGR64Reg()));
  }

  void addStrictlyFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR64Reg()));
  }

  void addFGR64AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR64Reg()));
  }

  void addFGR32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR32Reg()));
    // FIXME: We ought to do this for -integrated-as without -via-file-asm too.
    // FIXME: This should propagate failure up to parseStatement.
    if (!AsmParser.useOddSPReg() && RegIdx.Index & 1)
      AsmParser.getParser().printError(
          StartLoc, "-mno-odd-spreg prohibits the use of odd FPU "
                    "registers");
  }

  void addStrictlyFGR32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGR32Reg()));
    // FIXME: We ought to do this for -integrated-as without -via-file-asm too.
    if (!AsmParser.useOddSPReg() && RegIdx.Index & 1)
      AsmParser.Error(StartLoc, "-mno-odd-spreg prohibits the use of odd FPU "
                                "registers");
  }

  void addFGRH32AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFGRH32Reg()));
  }

  void addFCCAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getFCCReg()));
  }

  void addMSA128AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getMSA128Reg()));
  }

  void addMSACtrlAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getMSACtrlReg()));
  }

  void addCOP0AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCOP0Reg()));
  }

  void addCOP2AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCOP2Reg()));
  }

  void addCOP3AsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCOP3Reg()));
  }

  void addACC64DSPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getACC64DSPReg()));
  }

  void addHI32DSPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getHI32DSPReg()));
  }

  void addLO32DSPAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getLO32DSPReg()));
  }

  void addCCRAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getCCRReg()));
  }

  void addHWRegsAsmRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getHWRegsReg()));
  }

  template <unsigned Bits, int Offset = 0, int AdjustOffset = 0>
  void addConstantUImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    uint64_t Imm = getConstantImm() - Offset;
    Imm &= (1ULL << Bits) - 1;
    Imm += Offset;
    Imm += AdjustOffset;
    Inst.addOperand(MCOperand::createImm(Imm));
  }

  template <unsigned Bits>
  void addSImmOperands(MCInst &Inst, unsigned N) const {
    if (isImm() && !isConstantImm()) {
      addExpr(Inst, getImm());
      return;
    }
    addConstantSImmOperands<Bits, 0, 0>(Inst, N);
  }

  template <unsigned Bits>
  void addUImmOperands(MCInst &Inst, unsigned N) const {
    if (isImm() && !isConstantImm()) {
      addExpr(Inst, getImm());
      return;
    }
    addConstantUImmOperands<Bits, 0, 0>(Inst, N);
  }

  template <unsigned Bits, int Offset = 0, int AdjustOffset = 0>
  void addConstantSImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    int64_t Imm = getConstantImm() - Offset;
    Imm = SignExtend64<Bits>(Imm);
    Imm += Offset;
    Imm += AdjustOffset;
    Inst.addOperand(MCOperand::createImm(Imm));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addMemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(AsmParser.getABI().ArePtrs64bit()
                                             ? getMemBase()->getGPR64Reg()
                                             : getMemBase()->getGPR32Reg()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst, Expr);
  }

  void addMicroM6502MemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase()->getGPRMM16Reg()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst, Expr);
  }

  void addRegListOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");

    for (auto RegNo : getRegList())
      Inst.addOperand(MCOperand::createReg(RegNo));
  }

  void addRegPairOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");
    assert((RegIdx.Kind & RegKind_GPR) && "Invalid access!");
    unsigned RegNo = getRegPair();
    AsmParser.warnIfRegIndexIsAT(RegNo, StartLoc);
    Inst.addOperand(MCOperand::createReg(
      RegIdx.RegInfo->getRegClass(
        AsmParser.getABI().AreGprs64bit()
          ? M6502::GPR64RegClassID
          : M6502::GPR32RegClassID).getRegister(RegNo++)));
    Inst.addOperand(MCOperand::createReg(
      RegIdx.RegInfo->getRegClass(
        AsmParser.getABI().AreGprs64bit()
          ? M6502::GPR64RegClassID
          : M6502::GPR32RegClassID).getRegister(RegNo)));
  }

  void addMovePRegPairOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");
    for (auto RegNo : getRegList())
      Inst.addOperand(MCOperand::createReg(RegNo));
  }

  bool isReg() const override {
    // As a special case until we sort out the definition of div/divu, accept
    // $0/$zero here so that MCK_ZERO works correctly.
    return isGPRAsmReg() && RegIdx.Index == 0;
  }

  bool isRegIdx() const { return Kind == k_RegisterIndex; }
  bool isImm() const override { return Kind == k_Immediate; }

  bool isConstantImm() const {
    int64_t Res;
    return isImm() && getImm()->evaluateAsAbsolute(Res);
  }

  bool isConstantImmz() const {
    return isConstantImm() && getConstantImm() == 0;
  }

  template <unsigned Bits, int Offset = 0> bool isConstantUImm() const {
    return isConstantImm() && isUInt<Bits>(getConstantImm() - Offset);
  }

  template <unsigned Bits> bool isSImm() const {
    return isConstantImm() ? isInt<Bits>(getConstantImm()) : isImm();
  }

  template <unsigned Bits> bool isUImm() const {
    return isConstantImm() ? isUInt<Bits>(getConstantImm()) : isImm();
  }

  template <unsigned Bits> bool isAnyImm() const {
    return isConstantImm() ? (isInt<Bits>(getConstantImm()) ||
                              isUInt<Bits>(getConstantImm()))
                           : isImm();
  }

  template <unsigned Bits, int Offset = 0> bool isConstantSImm() const {
    return isConstantImm() && isInt<Bits>(getConstantImm() - Offset);
  }

  template <unsigned Bottom, unsigned Top> bool isConstantUImmRange() const {
    return isConstantImm() && getConstantImm() >= Bottom &&
           getConstantImm() <= Top;
  }

  bool isToken() const override {
    // Note: It's not possible to pretend that other operand kinds are tokens.
    // The matcher emitter checks tokens first.
    return Kind == k_Token;
  }

  bool isMem() const override { return Kind == k_Memory; }

  bool isConstantMemOff() const {
    return isMem() && isa<MCConstantExpr>(getMemOff());
  }

  // Allow relocation operators.
  // FIXME: This predicate and others need to look through binary expressions
  //        and determine whether a Value is a constant or not.
  template <unsigned Bits, unsigned ShiftAmount = 0>
  bool isMemWithSimmOffset() const {
    if (!isMem())
      return false;
    if (!getMemBase()->isGPRAsmReg())
      return false;
    if (isa<MCTargetExpr>(getMemOff()) ||
        (isConstantMemOff() &&
         isShiftedInt<Bits, ShiftAmount>(getConstantMemOff())))
      return true;
    MCValue Res;
    bool IsReloc = getMemOff()->evaluateAsRelocatable(Res, nullptr, nullptr);
    return IsReloc && isShiftedInt<Bits, ShiftAmount>(Res.getConstant());
  }

  bool isMemWithGRPMM16Base() const {
    return isMem() && getMemBase()->isMM16AsmReg();
  }

  template <unsigned Bits> bool isMemWithUimmOffsetSP() const {
    return isMem() && isConstantMemOff() && isUInt<Bits>(getConstantMemOff())
      && getMemBase()->isRegIdx() && (getMemBase()->getGPR32Reg() == M6502::SP);
  }

  template <unsigned Bits> bool isMemWithUimmWordAlignedOffsetSP() const {
    return isMem() && isConstantMemOff() && isUInt<Bits>(getConstantMemOff())
      && (getConstantMemOff() % 4 == 0) && getMemBase()->isRegIdx()
      && (getMemBase()->getGPR32Reg() == M6502::SP);
  }

  template <unsigned Bits> bool isMemWithSimmWordAlignedOffsetGP() const {
    return isMem() && isConstantMemOff() && isInt<Bits>(getConstantMemOff())
      && (getConstantMemOff() % 4 == 0) && getMemBase()->isRegIdx()
      && (getMemBase()->getGPR32Reg() == M6502::GP);
  }

  template <unsigned Bits, unsigned ShiftLeftAmount>
  bool isScaledUImm() const {
    return isConstantImm() &&
           isShiftedUInt<Bits, ShiftLeftAmount>(getConstantImm());
  }

  template <unsigned Bits, unsigned ShiftLeftAmount>
  bool isScaledSImm() const {
    if (isConstantImm() && isShiftedInt<Bits, ShiftLeftAmount>(getConstantImm()))
      return true;
    // Operand can also be a symbol or symbol plus offset in case of relocations.
    if (Kind != k_Immediate)
      return false;
    MCValue Res;
    bool Success = getImm()->evaluateAsRelocatable(Res, nullptr, nullptr);
    return Success && isShiftedInt<Bits, ShiftLeftAmount>(Res.getConstant());
  }

  bool isRegList16() const {
    if (!isRegList())
      return false;

    int Size = RegList.List->size();
    if (Size < 2 || Size > 5)
      return false;

    unsigned R0 = RegList.List->front();
    unsigned R1 = RegList.List->back();
    if (!((R0 == M6502::S0 && R1 == M6502::RA) ||
          (R0 == M6502::S0_64 && R1 == M6502::RA_64)))
      return false;

    int PrevReg = *RegList.List->begin();
    for (int i = 1; i < Size - 1; i++) {
      int Reg = (*(RegList.List))[i];
      if ( Reg != PrevReg + 1)
        return false;
      PrevReg = Reg;
    }

    return true;
  }

  bool isInvNum() const { return Kind == k_Immediate; }

  bool isLSAImm() const {
    if (!isConstantImm())
      return false;
    int64_t Val = getConstantImm();
    return 1 <= Val && Val <= 4;
  }

  bool isRegList() const { return Kind == k_RegList; }

  bool isMovePRegPair() const {
    if (Kind != k_RegList || RegList.List->size() != 2)
      return false;

    unsigned R0 = RegList.List->front();
    unsigned R1 = RegList.List->back();

    if ((R0 == M6502::A1 && R1 == M6502::A2) ||
        (R0 == M6502::A1 && R1 == M6502::A3) ||
        (R0 == M6502::A2 && R1 == M6502::A3) ||
        (R0 == M6502::A0 && R1 == M6502::S5) ||
        (R0 == M6502::A0 && R1 == M6502::S6) ||
        (R0 == M6502::A0 && R1 == M6502::A1) ||
        (R0 == M6502::A0 && R1 == M6502::A2) ||
        (R0 == M6502::A0 && R1 == M6502::A3) ||
        (R0 == M6502::A1_64 && R1 == M6502::A2_64) ||
        (R0 == M6502::A1_64 && R1 == M6502::A3_64) ||
        (R0 == M6502::A2_64 && R1 == M6502::A3_64) ||
        (R0 == M6502::A0_64 && R1 == M6502::S5_64) ||
        (R0 == M6502::A0_64 && R1 == M6502::S6_64) ||
        (R0 == M6502::A0_64 && R1 == M6502::A1_64) ||
        (R0 == M6502::A0_64 && R1 == M6502::A2_64) ||
        (R0 == M6502::A0_64 && R1 == M6502::A3_64))
      return true;

    return false;
  }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }

  bool isRegPair() const {
    return Kind == k_RegPair && RegIdx.Index <= 30;
  }

  unsigned getReg() const override {
    // As a special case until we sort out the definition of div/divu, accept
    // $0/$zero here so that MCK_ZERO works correctly.
    if (Kind == k_RegisterIndex && RegIdx.Index == 0 &&
        RegIdx.Kind & RegKind_GPR)
      return getGPR32Reg(); // FIXME: GPR64 too

    llvm_unreachable("Invalid access!");
    return 0;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate) && "Invalid access!");
    return Imm.Val;
  }

  int64_t getConstantImm() const {
    const MCExpr *Val = getImm();
    int64_t Value = 0;
    (void)Val->evaluateAsAbsolute(Value);
    return Value;
  }

  M6502Operand *getMemBase() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Base;
  }

  const MCExpr *getMemOff() const {
    assert((Kind == k_Memory) && "Invalid access!");
    return Mem.Off;
  }

  int64_t getConstantMemOff() const {
    return static_cast<const MCConstantExpr *>(getMemOff())->getValue();
  }

  const SmallVectorImpl<unsigned> &getRegList() const {
    assert((Kind == k_RegList) && "Invalid access!");
    return *(RegList.List);
  }

  unsigned getRegPair() const {
    assert((Kind == k_RegPair) && "Invalid access!");
    return RegIdx.Index;
  }

  static std::unique_ptr<M6502Operand> CreateToken(StringRef Str, SMLoc S,
                                                  M6502AsmParser &Parser) {
    auto Op = llvm::make_unique<M6502Operand>(k_Token, Parser);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  /// Create a numeric register (e.g. $1). The exact register remains
  /// unresolved until an instruction successfully matches
  static std::unique_ptr<M6502Operand>
  createNumericReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                   SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    DEBUG(dbgs() << "createNumericReg(" << Index << ", ...)\n");
    return CreateReg(Index, Str, RegKind_Numeric, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely a GPR.
  /// This is typically only used for named registers such as $gp.
  static std::unique_ptr<M6502Operand>
  createGPRReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_GPR, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely a FGR.
  /// This is typically only used for named registers such as $f0.
  static std::unique_ptr<M6502Operand>
  createFGRReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_FGR, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely a HWReg.
  /// This is typically only used for named registers such as $hwr_cpunum.
  static std::unique_ptr<M6502Operand>
  createHWRegsReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                  SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_HWRegs, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an FCC.
  /// This is typically only used for named registers such as $fcc0.
  static std::unique_ptr<M6502Operand>
  createFCCReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_FCC, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an ACC.
  /// This is typically only used for named registers such as $ac0.
  static std::unique_ptr<M6502Operand>
  createACCReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
               SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_ACC, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an MSA128.
  /// This is typically only used for named registers such as $w0.
  static std::unique_ptr<M6502Operand>
  createMSA128Reg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                  SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_MSA128, RegInfo, S, E, Parser);
  }

  /// Create a register that is definitely an MSACtrl.
  /// This is typically only used for named registers such as $msaaccess.
  static std::unique_ptr<M6502Operand>
  createMSACtrlReg(unsigned Index, StringRef Str, const MCRegisterInfo *RegInfo,
                   SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    return CreateReg(Index, Str, RegKind_MSACtrl, RegInfo, S, E, Parser);
  }

  static std::unique_ptr<M6502Operand>
  CreateImm(const MCExpr *Val, SMLoc S, SMLoc E, M6502AsmParser &Parser) {
    auto Op = llvm::make_unique<M6502Operand>(k_Immediate, Parser);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<M6502Operand>
  CreateMem(std::unique_ptr<M6502Operand> Base, const MCExpr *Off, SMLoc S,
            SMLoc E, M6502AsmParser &Parser) {
    auto Op = llvm::make_unique<M6502Operand>(k_Memory, Parser);
    Op->Mem.Base = Base.release();
    Op->Mem.Off = Off;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<M6502Operand>
  CreateRegList(SmallVectorImpl<unsigned> &Regs, SMLoc StartLoc, SMLoc EndLoc,
                M6502AsmParser &Parser) {
    assert(Regs.size() > 0 && "Empty list not allowed");

    auto Op = llvm::make_unique<M6502Operand>(k_RegList, Parser);
    Op->RegList.List = new SmallVector<unsigned, 10>(Regs.begin(), Regs.end());
    Op->StartLoc = StartLoc;
    Op->EndLoc = EndLoc;
    return Op;
  }

  static std::unique_ptr<M6502Operand> CreateRegPair(const M6502Operand &MOP,
                                                    SMLoc S, SMLoc E,
                                                    M6502AsmParser &Parser) {
    auto Op = llvm::make_unique<M6502Operand>(k_RegPair, Parser);
    Op->RegIdx.Index = MOP.RegIdx.Index;
    Op->RegIdx.RegInfo = MOP.RegIdx.RegInfo;
    Op->RegIdx.Kind = MOP.RegIdx.Kind;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

 bool isGPRZeroAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_GPR && RegIdx.Index == 0;
  }

 bool isGPRNonZeroAsmReg() const {
   return isRegIdx() && RegIdx.Kind & RegKind_GPR && RegIdx.Index > 0 &&
          RegIdx.Index <= 31;
  }

  bool isGPRAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_GPR && RegIdx.Index <= 31;
  }

  bool isMM16AsmReg() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return ((RegIdx.Index >= 2 && RegIdx.Index <= 7)
            || RegIdx.Index == 16 || RegIdx.Index == 17);

  }
  bool isMM16AsmRegZero() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return (RegIdx.Index == 0 ||
            (RegIdx.Index >= 2 && RegIdx.Index <= 7) ||
            RegIdx.Index == 17);
  }

  bool isMM16AsmRegMoveP() const {
    if (!(isRegIdx() && RegIdx.Kind))
      return false;
    return (RegIdx.Index == 0 || (RegIdx.Index >= 2 && RegIdx.Index <= 3) ||
      (RegIdx.Index >= 16 && RegIdx.Index <= 20));
  }

  bool isFGRAsmReg() const {
    // AFGR64 is $0-$15 but we handle this in getAFGR64()
    return isRegIdx() && RegIdx.Kind & RegKind_FGR && RegIdx.Index <= 31;
  }

  bool isStrictlyFGRAsmReg() const {
    // AFGR64 is $0-$15 but we handle this in getAFGR64()
    return isRegIdx() && RegIdx.Kind == RegKind_FGR && RegIdx.Index <= 31;
  }

  bool isHWRegsAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_HWRegs && RegIdx.Index <= 31;
  }

  bool isCCRAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_CCR && RegIdx.Index <= 31;
  }

  bool isFCCAsmReg() const {
    if (!(isRegIdx() && RegIdx.Kind & RegKind_FCC))
      return false;
    return RegIdx.Index <= 7;
  }

  bool isACCAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_ACC && RegIdx.Index <= 3;
  }

  bool isCOP0AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_COP0 && RegIdx.Index <= 31;
  }

  bool isCOP2AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_COP2 && RegIdx.Index <= 31;
  }

  bool isCOP3AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_COP3 && RegIdx.Index <= 31;
  }

  bool isMSA128AsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_MSA128 && RegIdx.Index <= 31;
  }

  bool isMSACtrlAsmReg() const {
    return isRegIdx() && RegIdx.Kind & RegKind_MSACtrl && RegIdx.Index <= 7;
  }

  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const override { return EndLoc; }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case k_Immediate:
      OS << "Imm<";
      OS << *Imm.Val;
      OS << ">";
      break;
    case k_Memory:
      OS << "Mem<";
      Mem.Base->print(OS);
      OS << ", ";
      OS << *Mem.Off;
      OS << ">";
      break;
    case k_RegisterIndex:
      OS << "RegIdx<" << RegIdx.Index << ":" << RegIdx.Kind << ", "
         << StringRef(RegIdx.Tok.Data, RegIdx.Tok.Length) << ">";
      break;
    case k_Token:
      OS << getToken();
      break;
    case k_RegList:
      OS << "RegList< ";
      for (auto Reg : (*RegList.List))
        OS << Reg << " ";
      OS <<  ">";
      break;
    case k_RegPair:
      OS << "RegPair<" << RegIdx.Index << "," << RegIdx.Index + 1 << ">";
      break;
    }
  }

  bool isValidForTie(const M6502Operand &Other) const {
    if (Kind != Other.Kind)
      return false;

    switch (Kind) {
    default:
      llvm_unreachable("Unexpected kind");
      return false;
    case k_RegisterIndex: {
      StringRef Token(RegIdx.Tok.Data, RegIdx.Tok.Length);
      StringRef OtherToken(Other.RegIdx.Tok.Data, Other.RegIdx.Tok.Length);
      return Token == OtherToken;
    }
    }
  }
}; // class M6502Operand

} // end anonymous namespace

namespace llvm {

extern const MCInstrDesc M6502Insts[];

} // end namespace llvm

static const MCInstrDesc &getInstDesc(unsigned Opcode) {
  return M6502Insts[Opcode];
}

static bool hasShortDelaySlot(unsigned Opcode) {
  switch (Opcode) {
    case M6502::JALS_MM:
    case M6502::JALRS_MM:
    case M6502::JALRS16_MM:
    case M6502::BGEZALS_MM:
    case M6502::BLTZALS_MM:
      return true;
    default:
      return false;
  }
}

static const MCSymbol *getSingleMCSymbol(const MCExpr *Expr) {
  if (const MCSymbolRefExpr *SRExpr = dyn_cast<MCSymbolRefExpr>(Expr)) {
    return &SRExpr->getSymbol();
  }

  if (const MCBinaryExpr *BExpr = dyn_cast<MCBinaryExpr>(Expr)) {
    const MCSymbol *LHSSym = getSingleMCSymbol(BExpr->getLHS());
    const MCSymbol *RHSSym = getSingleMCSymbol(BExpr->getRHS());

    if (LHSSym)
      return LHSSym;

    if (RHSSym)
      return RHSSym;

    return nullptr;
  }

  if (const MCUnaryExpr *UExpr = dyn_cast<MCUnaryExpr>(Expr))
    return getSingleMCSymbol(UExpr->getSubExpr());

  return nullptr;
}

static unsigned countMCSymbolRefExpr(const MCExpr *Expr) {
  if (isa<MCSymbolRefExpr>(Expr))
    return 1;

  if (const MCBinaryExpr *BExpr = dyn_cast<MCBinaryExpr>(Expr))
    return countMCSymbolRefExpr(BExpr->getLHS()) +
           countMCSymbolRefExpr(BExpr->getRHS());

  if (const MCUnaryExpr *UExpr = dyn_cast<MCUnaryExpr>(Expr))
    return countMCSymbolRefExpr(UExpr->getSubExpr());

  return 0;
}

bool M6502AsmParser::processInstruction(MCInst &Inst, SMLoc IDLoc,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  const MCInstrDesc &MCID = getInstDesc(Inst.getOpcode());
  bool ExpandedJalSym = false;

  Inst.setLoc(IDLoc);

  if (MCID.isBranch() || MCID.isCall()) {
    const unsigned Opcode = Inst.getOpcode();
    MCOperand Offset;

    switch (Opcode) {
    default:
      break;
    case M6502::BBIT0:
    case M6502::BBIT032:
    case M6502::BBIT1:
    case M6502::BBIT132:
      assert(hasCnM6502() && "instruction only valid for octeon cpus");
      LLVM_FALLTHROUGH;

    case M6502::BEQ:
    case M6502::BNE:
    case M6502::BEQ_MM:
    case M6502::BNE_MM:
      assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
      Offset = Inst.getOperand(2);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(inMicroM6502Mode() ? 17 : 18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (OffsetToAlignment(Offset.getImm(),
                            1LL << (inMicroM6502Mode() ? 1 : 2)))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case M6502::BGEZ:
    case M6502::BGTZ:
    case M6502::BLEZ:
    case M6502::BLTZ:
    case M6502::BGEZAL:
    case M6502::BLTZAL:
    case M6502::BC1F:
    case M6502::BC1T:
    case M6502::BGEZ_MM:
    case M6502::BGTZ_MM:
    case M6502::BLEZ_MM:
    case M6502::BLTZ_MM:
    case M6502::BGEZAL_MM:
    case M6502::BLTZAL_MM:
    case M6502::BC1F_MM:
    case M6502::BC1T_MM:
    case M6502::BC1EQZC_MMR6:
    case M6502::BC1NEZC_MMR6:
    case M6502::BC2EQZC_MMR6:
    case M6502::BC2NEZC_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(inMicroM6502Mode() ? 17 : 18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (OffsetToAlignment(Offset.getImm(),
                            1LL << (inMicroM6502Mode() ? 1 : 2)))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case M6502::BGEC:    case M6502::BGEC_MMR6:
    case M6502::BLTC:    case M6502::BLTC_MMR6:
    case M6502::BGEUC:   case M6502::BGEUC_MMR6:
    case M6502::BLTUC:   case M6502::BLTUC_MMR6:
    case M6502::BEQC:    case M6502::BEQC_MMR6:
    case M6502::BNEC:    case M6502::BNEC_MMR6:
      assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
      Offset = Inst.getOperand(2);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (OffsetToAlignment(Offset.getImm(), 1LL << 2))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case M6502::BLEZC:   case M6502::BLEZC_MMR6:
    case M6502::BGEZC:   case M6502::BGEZC_MMR6:
    case M6502::BGTZC:   case M6502::BGTZC_MMR6:
    case M6502::BLTZC:   case M6502::BLTZC_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(18, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (OffsetToAlignment(Offset.getImm(), 1LL << 2))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case M6502::BEQZC:   case M6502::BEQZC_MMR6:
    case M6502::BNEZC:   case M6502::BNEZC_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isIntN(23, Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (OffsetToAlignment(Offset.getImm(), 1LL << 2))
        return Error(IDLoc, "branch to misaligned address");
      break;
    case M6502::BEQZ16_MM:
    case M6502::BEQZC16_MMR6:
    case M6502::BNEZ16_MM:
    case M6502::BNEZC16_MMR6:
      assert(MCID.getNumOperands() == 2 && "unexpected number of operands");
      Offset = Inst.getOperand(1);
      if (!Offset.isImm())
        break; // We'll deal with this situation later on when applying fixups.
      if (!isInt<8>(Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (OffsetToAlignment(Offset.getImm(), 2LL))
        return Error(IDLoc, "branch to misaligned address");
      break;
    }
  }

  // SSNOP is deprecated on M650232r6/M650264r6
  // We still accept it but it is a normal nop.
  if (hasM650232r6() && Inst.getOpcode() == M6502::SSNOP) {
    std::string ISA = hasM650264r6() ? "M650264r6" : "M650232r6";
    Warning(IDLoc, "ssnop is deprecated for " + ISA + " and is equivalent to a "
                                                      "nop instruction");
  }

  if (hasCnM6502()) {
    const unsigned Opcode = Inst.getOpcode();
    MCOperand Opnd;
    int Imm;

    switch (Opcode) {
      default:
        break;

      case M6502::BBIT0:
      case M6502::BBIT032:
      case M6502::BBIT1:
      case M6502::BBIT132:
        assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
        // The offset is handled above
        Opnd = Inst.getOperand(1);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > (Opcode == M6502::BBIT0 ||
                              Opcode == M6502::BBIT1 ? 63 : 31))
          return Error(IDLoc, "immediate operand value out of range");
        if (Imm > 31) {
          Inst.setOpcode(Opcode == M6502::BBIT0 ? M6502::BBIT032
                                               : M6502::BBIT132);
          Inst.getOperand(1).setImm(Imm - 32);
        }
        break;

      case M6502::SEQi:
      case M6502::SNEi:
        assert(MCID.getNumOperands() == 3 && "unexpected number of operands");
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (!isInt<10>(Imm))
          return Error(IDLoc, "immediate operand value out of range");
        break;
    }
  }

  // Warn on division by zero. We're checking here as all instructions get
  // processed here, not just the macros that need expansion.
  //
  // The M6502 backend models most of the divison instructions and macros as
  // three operand instructions. The pre-R6 divide instructions however have
  // two operands and explicitly define HI/LO as part of the instruction,
  // not in the operands.
  unsigned FirstOp = 1;
  unsigned SecondOp = 2;
  switch (Inst.getOpcode()) {
  default:
    break;
  case M6502::SDivIMacro:
  case M6502::UDivIMacro:
  case M6502::DSDivIMacro:
  case M6502::DUDivIMacro:
    if (Inst.getOperand(2).getImm() == 0) {
      if (Inst.getOperand(1).getReg() == M6502::ZERO ||
          Inst.getOperand(1).getReg() == M6502::ZERO_64)
        Warning(IDLoc, "dividing zero by zero");
      else
        Warning(IDLoc, "division by zero");
    }
    break;
  case M6502::DSDIV:
  case M6502::SDIV:
  case M6502::UDIV:
  case M6502::DUDIV:
  case M6502::UDIV_MM:
  case M6502::SDIV_MM:
    FirstOp = 0;
    SecondOp = 1;
    LLVM_FALLTHROUGH;
  case M6502::SDivMacro:
  case M6502::DSDivMacro:
  case M6502::UDivMacro:
  case M6502::DUDivMacro:
  case M6502::DIV:
  case M6502::DIVU:
  case M6502::DDIV:
  case M6502::DDIVU:
  case M6502::DIVU_MMR6:
  case M6502::DDIVU_MM64R6:
  case M6502::DIV_MMR6:
  case M6502::DDIV_MM64R6:
    if (Inst.getOperand(SecondOp).getReg() == M6502::ZERO ||
        Inst.getOperand(SecondOp).getReg() == M6502::ZERO_64) {
      if (Inst.getOperand(FirstOp).getReg() == M6502::ZERO ||
          Inst.getOperand(FirstOp).getReg() == M6502::ZERO_64)
        Warning(IDLoc, "dividing zero by zero");
      else
        Warning(IDLoc, "division by zero");
    }
    break;
  }

  // For PIC code convert unconditional jump to unconditional branch.
  if ((Inst.getOpcode() == M6502::J || Inst.getOpcode() == M6502::J_MM) &&
      inPicMode()) {
    MCInst BInst;
    BInst.setOpcode(inMicroM6502Mode() ? M6502::BEQ_MM : M6502::BEQ);
    BInst.addOperand(MCOperand::createReg(M6502::ZERO));
    BInst.addOperand(MCOperand::createReg(M6502::ZERO));
    BInst.addOperand(Inst.getOperand(0));
    Inst = BInst;
  }

  // This expansion is not in a function called by tryExpandInstruction()
  // because the pseudo-instruction doesn't have a distinct opcode.
  if ((Inst.getOpcode() == M6502::JAL || Inst.getOpcode() == M6502::JAL_MM) &&
      inPicMode()) {
    warnIfNoMacro(IDLoc);

    const MCExpr *JalExpr = Inst.getOperand(0).getExpr();

    // We can do this expansion if there's only 1 symbol in the argument
    // expression.
    if (countMCSymbolRefExpr(JalExpr) > 1)
      return Error(IDLoc, "jal doesn't support multiple symbols in PIC mode");

    // FIXME: This is checking the expression can be handled by the later stages
    //        of the assembler. We ought to leave it to those later stages.
    const MCSymbol *JalSym = getSingleMCSymbol(JalExpr);

    // FIXME: Add support for label+offset operands (currently causes an error).
    // FIXME: Add support for forward-declared local symbols.
    // FIXME: Add expansion for when the LargeGOT option is enabled.
    if (JalSym->isInSection() || JalSym->isTemporary() ||
        (JalSym->isELF() && cast<MCSymbolELF>(JalSym)->getBinding() == ELF::STB_LOCAL)) {
      if (isABI_O32()) {
        // If it's a local symbol and the O32 ABI is being used, we expand to:
        //  lw $25, 0($gp)
        //    R_(MICRO)MIPS_GOT16  label
        //  addiu $25, $25, 0
        //    R_(MICRO)MIPS_LO16   label
        //  jalr  $25
        const MCExpr *Got16RelocExpr =
            M6502MCExpr::create(M6502MCExpr::MEK_GOT, JalExpr, getContext());
        const MCExpr *Lo16RelocExpr =
            M6502MCExpr::create(M6502MCExpr::MEK_LO, JalExpr, getContext());

        TOut.emitRRX(M6502::LW, M6502::T9, M6502::GP,
                     MCOperand::createExpr(Got16RelocExpr), IDLoc, STI);
        TOut.emitRRX(M6502::ADDiu, M6502::T9, M6502::T9,
                     MCOperand::createExpr(Lo16RelocExpr), IDLoc, STI);
      } else if (isABI_N32() || isABI_N64()) {
        // If it's a local symbol and the N32/N64 ABIs are being used,
        // we expand to:
        //  lw/ld $25, 0($gp)
        //    R_(MICRO)MIPS_GOT_DISP  label
        //  jalr  $25
        const MCExpr *GotDispRelocExpr =
            M6502MCExpr::create(M6502MCExpr::MEK_GOT_DISP, JalExpr, getContext());

        TOut.emitRRX(ABI.ArePtrs64bit() ? M6502::LD : M6502::LW, M6502::T9,
                     M6502::GP, MCOperand::createExpr(GotDispRelocExpr), IDLoc,
                     STI);
      }
    } else {
      // If it's an external/weak symbol, we expand to:
      //  lw/ld    $25, 0($gp)
      //    R_(MICRO)MIPS_CALL16  label
      //  jalr  $25
      const MCExpr *Call16RelocExpr =
          M6502MCExpr::create(M6502MCExpr::MEK_GOT_CALL, JalExpr, getContext());

      TOut.emitRRX(ABI.ArePtrs64bit() ? M6502::LD : M6502::LW, M6502::T9, M6502::GP,
                   MCOperand::createExpr(Call16RelocExpr), IDLoc, STI);
    }

    MCInst JalrInst;
    if (IsCpRestoreSet && inMicroM6502Mode())
      JalrInst.setOpcode(M6502::JALRS_MM);
    else
      JalrInst.setOpcode(inMicroM6502Mode() ? M6502::JALR_MM : M6502::JALR);
    JalrInst.addOperand(MCOperand::createReg(M6502::RA));
    JalrInst.addOperand(MCOperand::createReg(M6502::T9));

    // FIXME: Add an R_(MICRO)MIPS_JALR relocation after the JALR.
    // This relocation is supposed to be an optimization hint for the linker
    // and is not necessary for correctness.

    Inst = JalrInst;
    ExpandedJalSym = true;
  }

  bool IsPCRelativeLoad = (MCID.TSFlags & M6502II::IsPCRelativeLoad) != 0;
  if ((MCID.mayLoad() || MCID.mayStore()) && !IsPCRelativeLoad) {
    // Check the offset of memory operand, if it is a symbol
    // reference or immediate we may have to expand instructions.
    for (unsigned i = 0; i < MCID.getNumOperands(); i++) {
      const MCOperandInfo &OpInfo = MCID.OpInfo[i];
      if ((OpInfo.OperandType == MCOI::OPERAND_MEMORY) ||
          (OpInfo.OperandType == MCOI::OPERAND_UNKNOWN)) {
        MCOperand &Op = Inst.getOperand(i);
        if (Op.isImm()) {
          int MemOffset = Op.getImm();
          if (MemOffset < -32768 || MemOffset > 32767) {
            // Offset can't exceed 16bit value.
            expandMemInst(Inst, IDLoc, Out, STI, MCID.mayLoad(), true);
            return getParser().hasPendingError();
          }
        } else if (Op.isExpr()) {
          const MCExpr *Expr = Op.getExpr();
          if (Expr->getKind() == MCExpr::SymbolRef) {
            const MCSymbolRefExpr *SR =
                static_cast<const MCSymbolRefExpr *>(Expr);
            if (SR->getKind() == MCSymbolRefExpr::VK_None) {
              // Expand symbol.
              expandMemInst(Inst, IDLoc, Out, STI, MCID.mayLoad(), false);
              return getParser().hasPendingError();
            }
          } else if (!isEvaluated(Expr)) {
            expandMemInst(Inst, IDLoc, Out, STI, MCID.mayLoad(), false);
            return getParser().hasPendingError();
          }
        }
      }
    } // for
  }   // if load/store

  if (inMicroM6502Mode()) {
    if (MCID.mayLoad()) {
      // Try to create 16-bit GP relative load instruction.
      for (unsigned i = 0; i < MCID.getNumOperands(); i++) {
        const MCOperandInfo &OpInfo = MCID.OpInfo[i];
        if ((OpInfo.OperandType == MCOI::OPERAND_MEMORY) ||
            (OpInfo.OperandType == MCOI::OPERAND_UNKNOWN)) {
          MCOperand &Op = Inst.getOperand(i);
          if (Op.isImm()) {
            int MemOffset = Op.getImm();
            MCOperand &DstReg = Inst.getOperand(0);
            MCOperand &BaseReg = Inst.getOperand(1);
            if (isInt<9>(MemOffset) && (MemOffset % 4 == 0) &&
                getContext().getRegisterInfo()->getRegClass(
                  M6502::GPRMM16RegClassID).contains(DstReg.getReg()) &&
                (BaseReg.getReg() == M6502::GP ||
                BaseReg.getReg() == M6502::GP_64)) {

              TOut.emitRRI(M6502::LWGP_MM, DstReg.getReg(), M6502::GP, MemOffset,
                           IDLoc, STI);
              return false;
            }
          }
        }
      } // for
    }   // if load

    // TODO: Handle this with the AsmOperandClass.PredicateMethod.

    MCOperand Opnd;
    int Imm;

    switch (Inst.getOpcode()) {
      default:
        break;
      case M6502::ADDIUSP_MM:
        Opnd = Inst.getOperand(0);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < -1032 || Imm > 1028 || (Imm < 8 && Imm > -12) ||
            Imm % 4 != 0)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::SLL16_MM:
      case M6502::SRL16_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 1 || Imm > 8)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::LI16_MM:
        Opnd = Inst.getOperand(1);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < -1 || Imm > 126)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::ADDIUR2_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (!(Imm == 1 || Imm == -1 ||
              ((Imm % 4 == 0) && Imm < 28 && Imm > 0)))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::ANDI16_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (!(Imm == 128 || (Imm >= 1 && Imm <= 4) || Imm == 7 || Imm == 8 ||
              Imm == 15 || Imm == 16 || Imm == 31 || Imm == 32 || Imm == 63 ||
              Imm == 64 || Imm == 255 || Imm == 32768 || Imm == 65535))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::LBU16_MM:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < -1 || Imm > 14)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::SB16_MM:
      case M6502::SB16_MMR6:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > 15)
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::LHU16_MM:
      case M6502::SH16_MM:
      case M6502::SH16_MMR6:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > 30 || (Imm % 2 != 0))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::LW16_MM:
      case M6502::SW16_MM:
      case M6502::SW16_MMR6:
        Opnd = Inst.getOperand(2);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        Imm = Opnd.getImm();
        if (Imm < 0 || Imm > 60 || (Imm % 4 != 0))
          return Error(IDLoc, "immediate operand value out of range");
        break;
      case M6502::ADDIUPC_MM:
        MCOperand Opnd = Inst.getOperand(1);
        if (!Opnd.isImm())
          return Error(IDLoc, "expected immediate operand kind");
        int Imm = Opnd.getImm();
        if ((Imm % 4 != 0) || !isInt<25>(Imm))
          return Error(IDLoc, "immediate operand value out of range");
        break;
    }
  }

  bool FillDelaySlot =
      MCID.hasDelaySlot() && AssemblerOptions.back()->isReorder();
  if (FillDelaySlot)
    TOut.emitDirectiveSetNoReorder();

  MacroExpanderResultTy ExpandResult =
      tryExpandInstruction(Inst, IDLoc, Out, STI);
  switch (ExpandResult) {
  case MER_NotAMacro:
    Out.EmitInstruction(Inst, *STI);
    break;
  case MER_Success:
    break;
  case MER_Fail:
    return true;
  }

  // We know we emitted an instruction on the MER_NotAMacro or MER_Success path.
  // If we're in microM6502 mode then we must also set EF_MIPS_MICROM6502.
  if (inMicroM6502Mode())
    TOut.setUsesMicroM6502();

  // If this instruction has a delay slot and .set reorder is active,
  // emit a NOP after it.
  if (FillDelaySlot) {
    TOut.emitEmptyDelaySlot(hasShortDelaySlot(Inst.getOpcode()), IDLoc, STI);
    TOut.emitDirectiveSetReorder();
  }

  if ((Inst.getOpcode() == M6502::JalOneReg ||
       Inst.getOpcode() == M6502::JalTwoReg || ExpandedJalSym) &&
      isPicAndNotNxxAbi()) {
    if (IsCpRestoreSet) {
      // We need a NOP between the JALR and the LW:
      // If .set reorder has been used, we've already emitted a NOP.
      // If .set noreorder has been used, we need to emit a NOP at this point.
      if (!AssemblerOptions.back()->isReorder())
        TOut.emitEmptyDelaySlot(hasShortDelaySlot(Inst.getOpcode()), IDLoc,
                                STI);

      // Load the $gp from the stack.
      TOut.emitGPRestore(CpRestoreOffset, IDLoc, STI);
    } else
      Warning(IDLoc, "no .cprestore used in PIC mode");
  }

  return false;
}

M6502AsmParser::MacroExpanderResultTy
M6502AsmParser::tryExpandInstruction(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  switch (Inst.getOpcode()) {
  default:
    return MER_NotAMacro;
  case M6502::LoadImm32:
    return expandLoadImm(Inst, true, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::LoadImm64:
    return expandLoadImm(Inst, false, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::LoadAddrImm32:
  case M6502::LoadAddrImm64:
    assert(Inst.getOperand(0).isReg() && "expected register operand kind");
    assert((Inst.getOperand(1).isImm() || Inst.getOperand(1).isExpr()) &&
           "expected immediate operand kind");

    return expandLoadAddress(Inst.getOperand(0).getReg(), M6502::NoRegister,
                             Inst.getOperand(1),
                             Inst.getOpcode() == M6502::LoadAddrImm32, IDLoc,
                             Out, STI)
               ? MER_Fail
               : MER_Success;
  case M6502::LoadAddrReg32:
  case M6502::LoadAddrReg64:
    assert(Inst.getOperand(0).isReg() && "expected register operand kind");
    assert(Inst.getOperand(1).isReg() && "expected register operand kind");
    assert((Inst.getOperand(2).isImm() || Inst.getOperand(2).isExpr()) &&
           "expected immediate operand kind");

    return expandLoadAddress(Inst.getOperand(0).getReg(),
                             Inst.getOperand(1).getReg(), Inst.getOperand(2),
                             Inst.getOpcode() == M6502::LoadAddrReg32, IDLoc,
                             Out, STI)
               ? MER_Fail
               : MER_Success;
  case M6502::B_MM_Pseudo:
  case M6502::B_MMR6_Pseudo:
    return expandUncondBranchMMPseudo(Inst, IDLoc, Out, STI) ? MER_Fail
                                                             : MER_Success;
  case M6502::SWM_MM:
  case M6502::LWM_MM:
    return expandLoadStoreMultiple(Inst, IDLoc, Out, STI) ? MER_Fail
                                                          : MER_Success;
  case M6502::JalOneReg:
  case M6502::JalTwoReg:
    return expandJalWithRegs(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::BneImm:
  case M6502::BeqImm:
  case M6502::BEQLImmMacro:
  case M6502::BNELImmMacro:
    return expandBranchImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::BLT:
  case M6502::BLE:
  case M6502::BGE:
  case M6502::BGT:
  case M6502::BLTU:
  case M6502::BLEU:
  case M6502::BGEU:
  case M6502::BGTU:
  case M6502::BLTL:
  case M6502::BLEL:
  case M6502::BGEL:
  case M6502::BGTL:
  case M6502::BLTUL:
  case M6502::BLEUL:
  case M6502::BGEUL:
  case M6502::BGTUL:
  case M6502::BLTImmMacro:
  case M6502::BLEImmMacro:
  case M6502::BGEImmMacro:
  case M6502::BGTImmMacro:
  case M6502::BLTUImmMacro:
  case M6502::BLEUImmMacro:
  case M6502::BGEUImmMacro:
  case M6502::BGTUImmMacro:
  case M6502::BLTLImmMacro:
  case M6502::BLELImmMacro:
  case M6502::BGELImmMacro:
  case M6502::BGTLImmMacro:
  case M6502::BLTULImmMacro:
  case M6502::BLEULImmMacro:
  case M6502::BGEULImmMacro:
  case M6502::BGTULImmMacro:
    return expandCondBranches(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::SDivMacro:
  case M6502::SDivIMacro:
    return expandDiv(Inst, IDLoc, Out, STI, false, true) ? MER_Fail
                                                         : MER_Success;
  case M6502::DSDivMacro:
  case M6502::DSDivIMacro:
    return expandDiv(Inst, IDLoc, Out, STI, true, true) ? MER_Fail
                                                        : MER_Success;
  case M6502::UDivMacro:
  case M6502::UDivIMacro:
    return expandDiv(Inst, IDLoc, Out, STI, false, false) ? MER_Fail
                                                          : MER_Success;
  case M6502::DUDivMacro:
  case M6502::DUDivIMacro:
    return expandDiv(Inst, IDLoc, Out, STI, true, false) ? MER_Fail
                                                         : MER_Success;
  case M6502::PseudoTRUNC_W_S:
    return expandTrunc(Inst, false, false, IDLoc, Out, STI) ? MER_Fail
                                                            : MER_Success;
  case M6502::PseudoTRUNC_W_D32:
    return expandTrunc(Inst, true, false, IDLoc, Out, STI) ? MER_Fail
                                                           : MER_Success;
  case M6502::PseudoTRUNC_W_D:
    return expandTrunc(Inst, true, true, IDLoc, Out, STI) ? MER_Fail
                                                          : MER_Success;

  case M6502::LoadImmSingleGPR:
    return expandLoadImmReal(Inst, true, true, false, IDLoc, Out, STI)
               ? MER_Fail
               : MER_Success;
  case M6502::LoadImmSingleFGR:
    return expandLoadImmReal(Inst, true, false, false, IDLoc, Out, STI)
               ? MER_Fail
               : MER_Success;
  case M6502::LoadImmDoubleGPR:
    return expandLoadImmReal(Inst, false, true, false, IDLoc, Out, STI)
               ? MER_Fail
               : MER_Success;
  case M6502::LoadImmDoubleFGR:
      return expandLoadImmReal(Inst, false, false, true, IDLoc, Out, STI)
               ? MER_Fail
               : MER_Success;
  case M6502::LoadImmDoubleFGR_32:
    return expandLoadImmReal(Inst, false, false, false, IDLoc, Out, STI)
               ? MER_Fail
               : MER_Success;
  case M6502::Ulh:
    return expandUlh(Inst, true, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::Ulhu:
    return expandUlh(Inst, false, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::Ush:
    return expandUsh(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::Ulw:
  case M6502::Usw:
    return expandUxw(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::NORImm:
  case M6502::NORImm64:
    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::SLTImm64:
    if (isInt<16>(Inst.getOperand(2).getImm())) {
      Inst.setOpcode(M6502::SLTi64);
      return MER_NotAMacro;
    }
    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::SLTUImm64:
    if (isInt<16>(Inst.getOperand(2).getImm())) {
      Inst.setOpcode(M6502::SLTiu64);
      return MER_NotAMacro;
    }
    return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::ADDi:   case M6502::ADDi_MM:
  case M6502::ADDiu:  case M6502::ADDiu_MM:
  case M6502::SLTi:   case M6502::SLTi_MM:
  case M6502::SLTiu:  case M6502::SLTiu_MM:
    if ((Inst.getNumOperands() == 3) && Inst.getOperand(0).isReg() &&
        Inst.getOperand(1).isReg() && Inst.getOperand(2).isImm()) {
      int64_t ImmValue = Inst.getOperand(2).getImm();
      if (isInt<16>(ImmValue))
        return MER_NotAMacro;
      return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail
                                                         : MER_Success;
    }
    return MER_NotAMacro;
  case M6502::ANDi:  case M6502::ANDi_MM:  case M6502::ANDi64:
  case M6502::ORi:   case M6502::ORi_MM:   case M6502::ORi64:
  case M6502::XORi:  case M6502::XORi_MM:  case M6502::XORi64:
    if ((Inst.getNumOperands() == 3) && Inst.getOperand(0).isReg() &&
        Inst.getOperand(1).isReg() && Inst.getOperand(2).isImm()) {
      int64_t ImmValue = Inst.getOperand(2).getImm();
      if (isUInt<16>(ImmValue))
        return MER_NotAMacro;
      return expandAliasImmediate(Inst, IDLoc, Out, STI) ? MER_Fail
                                                         : MER_Success;
    }
    return MER_NotAMacro;
  case M6502::ROL:
  case M6502::ROR:
    return expandRotation(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::ROLImm:
  case M6502::RORImm:
    return expandRotationImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::DROL:
  case M6502::DROR:
    return expandDRotation(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::DROLImm:
  case M6502::DRORImm:
    return expandDRotationImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::ABSMacro:
    return expandAbs(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::MULImmMacro:
  case M6502::DMULImmMacro:
    return expandMulImm(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::MULOMacro:
  case M6502::DMULOMacro:
    return expandMulO(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::MULOUMacro:
  case M6502::DMULOUMacro:
    return expandMulOU(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::DMULMacro:
    return expandDMULMacro(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::LDMacro:
  case M6502::SDMacro:
    return expandLoadStoreDMacro(Inst, IDLoc, Out, STI,
                                 Inst.getOpcode() == M6502::LDMacro)
               ? MER_Fail
               : MER_Success;
  case M6502::SEQMacro:
    return expandSeq(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  case M6502::SEQIMacro:
    return expandSeqI(Inst, IDLoc, Out, STI) ? MER_Fail : MER_Success;
  }
}

bool M6502AsmParser::expandJalWithRegs(MCInst &Inst, SMLoc IDLoc,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  // Create a JALR instruction which is going to replace the pseudo-JAL.
  MCInst JalrInst;
  JalrInst.setLoc(IDLoc);
  const MCOperand FirstRegOp = Inst.getOperand(0);
  const unsigned Opcode = Inst.getOpcode();

  if (Opcode == M6502::JalOneReg) {
    // jal $rs => jalr $rs
    if (IsCpRestoreSet && inMicroM6502Mode()) {
      JalrInst.setOpcode(M6502::JALRS16_MM);
      JalrInst.addOperand(FirstRegOp);
    } else if (inMicroM6502Mode()) {
      JalrInst.setOpcode(hasM650232r6() ? M6502::JALRC16_MMR6 : M6502::JALR16_MM);
      JalrInst.addOperand(FirstRegOp);
    } else {
      JalrInst.setOpcode(M6502::JALR);
      JalrInst.addOperand(MCOperand::createReg(M6502::RA));
      JalrInst.addOperand(FirstRegOp);
    }
  } else if (Opcode == M6502::JalTwoReg) {
    // jal $rd, $rs => jalr $rd, $rs
    if (IsCpRestoreSet && inMicroM6502Mode())
      JalrInst.setOpcode(M6502::JALRS_MM);
    else
      JalrInst.setOpcode(inMicroM6502Mode() ? M6502::JALR_MM : M6502::JALR);
    JalrInst.addOperand(FirstRegOp);
    const MCOperand SecondRegOp = Inst.getOperand(1);
    JalrInst.addOperand(SecondRegOp);
  }
  Out.EmitInstruction(JalrInst, *STI);

  // If .set reorder is active and branch instruction has a delay slot,
  // emit a NOP after it.
  const MCInstrDesc &MCID = getInstDesc(JalrInst.getOpcode());
  if (MCID.hasDelaySlot() && AssemblerOptions.back()->isReorder())
    TOut.emitEmptyDelaySlot(hasShortDelaySlot(JalrInst.getOpcode()), IDLoc,
                            STI);

  return false;
}

/// Can the value be represented by a unsigned N-bit value and a shift left?
template <unsigned N> static bool isShiftedUIntAtAnyPosition(uint64_t x) {
  unsigned BitNum = findFirstSet(x);

  return (x == x >> BitNum << BitNum) && isUInt<N>(x >> BitNum);
}

/// Load (or add) an immediate into a register.
///
/// @param ImmValue     The immediate to load.
/// @param DstReg       The register that will hold the immediate.
/// @param SrcReg       A register to add to the immediate or M6502::NoRegister
///                     for a simple initialization.
/// @param Is32BitImm   Is ImmValue 32-bit or 64-bit?
/// @param IsAddress    True if the immediate represents an address. False if it
///                     is an integer.
/// @param IDLoc        Location of the immediate in the source file.
bool M6502AsmParser::loadImmediate(int64_t ImmValue, unsigned DstReg,
                                  unsigned SrcReg, bool Is32BitImm,
                                  bool IsAddress, SMLoc IDLoc, MCStreamer &Out,
                                  const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  if (!Is32BitImm && !isGP64bit()) {
    Error(IDLoc, "instruction requires a 64-bit architecture");
    return true;
  }

  if (Is32BitImm) {
    if (isInt<32>(ImmValue) || isUInt<32>(ImmValue)) {
      // Sign extend up to 64-bit so that the predicates match the hardware
      // behaviour. In particular, isInt<16>(0xffff8000) and similar should be
      // true.
      ImmValue = SignExtend64<32>(ImmValue);
    } else {
      Error(IDLoc, "instruction requires a 32-bit immediate");
      return true;
    }
  }

  unsigned ZeroReg = IsAddress ? ABI.GetNullPtr() : ABI.GetZeroReg();
  unsigned AdduOp = !Is32BitImm ? M6502::DADDu : M6502::ADDu;

  bool UseSrcReg = false;
  if (SrcReg != M6502::NoRegister)
    UseSrcReg = true;

  unsigned TmpReg = DstReg;
  if (UseSrcReg &&
      getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, SrcReg)) {
    // At this point we need AT to perform the expansions and we exit if it is
    // not available.
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;
    TmpReg = ATReg;
  }

  if (isInt<16>(ImmValue)) {
    if (!UseSrcReg)
      SrcReg = ZeroReg;

    // This doesn't quite follow the usual ABI expectations for N32 but matches
    // traditional assembler behaviour. N32 would normally use addiu for both
    // integers and addresses.
    if (IsAddress && !Is32BitImm) {
      TOut.emitRRI(M6502::DADDiu, DstReg, SrcReg, ImmValue, IDLoc, STI);
      return false;
    }

    TOut.emitRRI(M6502::ADDiu, DstReg, SrcReg, ImmValue, IDLoc, STI);
    return false;
  }

  if (isUInt<16>(ImmValue)) {
    unsigned TmpReg = DstReg;
    if (SrcReg == DstReg) {
      TmpReg = getATReg(IDLoc);
      if (!TmpReg)
        return true;
    }

    TOut.emitRRI(M6502::ORi, TmpReg, ZeroReg, ImmValue, IDLoc, STI);
    if (UseSrcReg)
      TOut.emitRRR(ABI.GetPtrAdduOp(), DstReg, TmpReg, SrcReg, IDLoc, STI);
    return false;
  }

  if (isInt<32>(ImmValue) || isUInt<32>(ImmValue)) {
    warnIfNoMacro(IDLoc);

    uint16_t Bits31To16 = (ImmValue >> 16) & 0xffff;
    uint16_t Bits15To0 = ImmValue & 0xffff;
    if (!Is32BitImm && !isInt<32>(ImmValue)) {
      // Traditional behaviour seems to special case this particular value. It's
      // not clear why other masks are handled differently.
      if (ImmValue == 0xffffffff) {
        TOut.emitRI(M6502::LUi, TmpReg, 0xffff, IDLoc, STI);
        TOut.emitRRI(M6502::DSRL32, TmpReg, TmpReg, 0, IDLoc, STI);
        if (UseSrcReg)
          TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);
        return false;
      }

      // Expand to an ORi instead of a LUi to avoid sign-extending into the
      // upper 32 bits.
      TOut.emitRRI(M6502::ORi, TmpReg, ZeroReg, Bits31To16, IDLoc, STI);
      TOut.emitRRI(M6502::DSLL, TmpReg, TmpReg, 16, IDLoc, STI);
      if (Bits15To0)
        TOut.emitRRI(M6502::ORi, TmpReg, TmpReg, Bits15To0, IDLoc, STI);
      if (UseSrcReg)
        TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);
      return false;
    }

    TOut.emitRI(M6502::LUi, TmpReg, Bits31To16, IDLoc, STI);
    if (Bits15To0)
      TOut.emitRRI(M6502::ORi, TmpReg, TmpReg, Bits15To0, IDLoc, STI);
    if (UseSrcReg)
      TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);
    return false;
  }

  if (isShiftedUIntAtAnyPosition<16>(ImmValue)) {
    if (Is32BitImm) {
      Error(IDLoc, "instruction requires a 32-bit immediate");
      return true;
    }

    // Traditionally, these immediates are shifted as little as possible and as
    // such we align the most significant bit to bit 15 of our temporary.
    unsigned FirstSet = findFirstSet((uint64_t)ImmValue);
    unsigned LastSet = findLastSet((uint64_t)ImmValue);
    unsigned ShiftAmount = FirstSet - (15 - (LastSet - FirstSet));
    uint16_t Bits = (ImmValue >> ShiftAmount) & 0xffff;
    TOut.emitRRI(M6502::ORi, TmpReg, ZeroReg, Bits, IDLoc, STI);
    TOut.emitRRI(M6502::DSLL, TmpReg, TmpReg, ShiftAmount, IDLoc, STI);

    if (UseSrcReg)
      TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);

    return false;
  }

  warnIfNoMacro(IDLoc);

  // The remaining case is packed with a sequence of dsll and ori with zeros
  // being omitted and any neighbouring dsll's being coalesced.
  // The highest 32-bit's are equivalent to a 32-bit immediate load.

  // Load bits 32-63 of ImmValue into bits 0-31 of the temporary register.
  if (loadImmediate(ImmValue >> 32, TmpReg, M6502::NoRegister, true, false,
                    IDLoc, Out, STI))
    return false;

  // Shift and accumulate into the register. If a 16-bit chunk is zero, then
  // skip it and defer the shift to the next chunk.
  unsigned ShiftCarriedForwards = 16;
  for (int BitNum = 16; BitNum >= 0; BitNum -= 16) {
    uint16_t ImmChunk = (ImmValue >> BitNum) & 0xffff;

    if (ImmChunk != 0) {
      TOut.emitDSLL(TmpReg, TmpReg, ShiftCarriedForwards, IDLoc, STI);
      TOut.emitRRI(M6502::ORi, TmpReg, TmpReg, ImmChunk, IDLoc, STI);
      ShiftCarriedForwards = 0;
    }

    ShiftCarriedForwards += 16;
  }
  ShiftCarriedForwards -= 16;

  // Finish any remaining shifts left by trailing zeros.
  if (ShiftCarriedForwards)
    TOut.emitDSLL(TmpReg, TmpReg, ShiftCarriedForwards, IDLoc, STI);

  if (UseSrcReg)
    TOut.emitRRR(AdduOp, DstReg, TmpReg, SrcReg, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandLoadImm(MCInst &Inst, bool Is32BitImm, SMLoc IDLoc,
                                  MCStreamer &Out, const MCSubtargetInfo *STI) {
  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");

  if (loadImmediate(ImmOp.getImm(), DstRegOp.getReg(), M6502::NoRegister,
                    Is32BitImm, false, IDLoc, Out, STI))
    return true;

  return false;
}

bool M6502AsmParser::expandLoadAddress(unsigned DstReg, unsigned BaseReg,
                                      const MCOperand &Offset,
                                      bool Is32BitAddress, SMLoc IDLoc,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo *STI) {
  // la can't produce a usable address when addresses are 64-bit.
  if (Is32BitAddress && ABI.ArePtrs64bit()) {
    // FIXME: Demote this to a warning and continue as if we had 'dla' instead.
    //        We currently can't do this because we depend on the equality
    //        operator and N64 can end up with a GPR32/GPR64 mismatch.
    Error(IDLoc, "la used to load 64-bit address");
    // Continue as if we had 'dla' instead.
    Is32BitAddress = false;
    return true;
  }

  // dla requires 64-bit addresses.
  if (!Is32BitAddress && !hasM65023()) {
    Error(IDLoc, "instruction requires a 64-bit architecture");
    return true;
  }

  if (!Offset.isImm())
    return loadAndAddSymbolAddress(Offset.getExpr(), DstReg, BaseReg,
                                   Is32BitAddress, IDLoc, Out, STI);

  if (!ABI.ArePtrs64bit()) {
    // Continue as if we had 'la' whether we had 'la' or 'dla'.
    Is32BitAddress = true;
  }

  return loadImmediate(Offset.getImm(), DstReg, BaseReg, Is32BitAddress, true,
                       IDLoc, Out, STI);
}

bool M6502AsmParser::loadAndAddSymbolAddress(const MCExpr *SymExpr,
                                            unsigned DstReg, unsigned SrcReg,
                                            bool Is32BitSym, SMLoc IDLoc,
                                            MCStreamer &Out,
                                            const MCSubtargetInfo *STI) {
  // FIXME: These expansions do not respect -mxgot.
  M6502TargetStreamer &TOut = getTargetStreamer();
  bool UseSrcReg = SrcReg != M6502::NoRegister;
  warnIfNoMacro(IDLoc);

  if (inPicMode() && ABI.IsO32()) {
    MCValue Res;
    if (!SymExpr->evaluateAsRelocatable(Res, nullptr, nullptr)) {
      Error(IDLoc, "expected relocatable expression");
      return true;
    }
    if (Res.getSymB() != nullptr) {
      Error(IDLoc, "expected relocatable expression with only one symbol");
      return true;
    }

    // The case where the result register is $25 is somewhat special. If the
    // symbol in the final relocation is external and not modified with a
    // constant then we must use R_MIPS_CALL16 instead of R_MIPS_GOT16.
    if ((DstReg == M6502::T9 || DstReg == M6502::T9_64) && !UseSrcReg &&
        Res.getConstant() == 0 &&
        !(Res.getSymA()->getSymbol().isInSection() ||
          Res.getSymA()->getSymbol().isTemporary() ||
          (Res.getSymA()->getSymbol().isELF() &&
           cast<MCSymbolELF>(Res.getSymA()->getSymbol()).getBinding() ==
               ELF::STB_LOCAL))) {
      const MCExpr *CallExpr =
          M6502MCExpr::create(M6502MCExpr::MEK_GOT_CALL, SymExpr, getContext());
      TOut.emitRRX(M6502::LW, DstReg, ABI.GetGlobalPtr(),
                   MCOperand::createExpr(CallExpr), IDLoc, STI);
      return false;
    }

    // The remaining cases are:
    //   External GOT: lw $tmp, %got(symbol+offset)($gp)
    //                >addiu $tmp, $tmp, %lo(offset)
    //                >addiu $rd, $tmp, $rs
    //   Local GOT:    lw $tmp, %got(symbol+offset)($gp)
    //                 addiu $tmp, $tmp, %lo(symbol+offset)($gp)
    //                >addiu $rd, $tmp, $rs
    // The addiu's marked with a '>' may be omitted if they are redundant. If
    // this happens then the last instruction must use $rd as the result
    // register.
    const M6502MCExpr *GotExpr =
        M6502MCExpr::create(M6502MCExpr::MEK_GOT, SymExpr, getContext());
    const MCExpr *LoExpr = nullptr;
    if (Res.getSymA()->getSymbol().isInSection() ||
        Res.getSymA()->getSymbol().isTemporary())
      LoExpr = M6502MCExpr::create(M6502MCExpr::MEK_LO, SymExpr, getContext());
    else if (Res.getConstant() != 0) {
      // External symbols fully resolve the symbol with just the %got(symbol)
      // but we must still account for any offset to the symbol for expressions
      // like symbol+8.
      LoExpr = MCConstantExpr::create(Res.getConstant(), getContext());
    }

    unsigned TmpReg = DstReg;
    if (UseSrcReg &&
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg,
                                                               SrcReg)) {
      // If $rs is the same as $rd, we need to use AT.
      // If it is not available we exit.
      unsigned ATReg = getATReg(IDLoc);
      if (!ATReg)
        return true;
      TmpReg = ATReg;
    }

    TOut.emitRRX(M6502::LW, TmpReg, ABI.GetGlobalPtr(),
                 MCOperand::createExpr(GotExpr), IDLoc, STI);

    if (LoExpr)
      TOut.emitRRX(M6502::ADDiu, TmpReg, TmpReg, MCOperand::createExpr(LoExpr),
                   IDLoc, STI);

    if (UseSrcReg)
      TOut.emitRRR(M6502::ADDu, DstReg, TmpReg, SrcReg, IDLoc, STI);

    return false;
  }

  if (inPicMode() && ABI.ArePtrs64bit()) {
    MCValue Res;
    if (!SymExpr->evaluateAsRelocatable(Res, nullptr, nullptr)) {
      Error(IDLoc, "expected relocatable expression");
      return true;
    }
    if (Res.getSymB() != nullptr) {
      Error(IDLoc, "expected relocatable expression with only one symbol");
      return true;
    }

    // The case where the result register is $25 is somewhat special. If the
    // symbol in the final relocation is external and not modified with a
    // constant then we must use R_MIPS_CALL16 instead of R_MIPS_GOT_DISP.
    if ((DstReg == M6502::T9 || DstReg == M6502::T9_64) && !UseSrcReg &&
        Res.getConstant() == 0 &&
        !(Res.getSymA()->getSymbol().isInSection() ||
          Res.getSymA()->getSymbol().isTemporary() ||
          (Res.getSymA()->getSymbol().isELF() &&
           cast<MCSymbolELF>(Res.getSymA()->getSymbol()).getBinding() ==
               ELF::STB_LOCAL))) {
      const MCExpr *CallExpr =
          M6502MCExpr::create(M6502MCExpr::MEK_GOT_CALL, SymExpr, getContext());
      TOut.emitRRX(M6502::LD, DstReg, ABI.GetGlobalPtr(),
                   MCOperand::createExpr(CallExpr), IDLoc, STI);
      return false;
    }

    // The remaining cases are:
    //   Small offset: ld $tmp, %got_disp(symbol)($gp)
    //                >daddiu $tmp, $tmp, offset
    //                >daddu $rd, $tmp, $rs
    // The daddiu's marked with a '>' may be omitted if they are redundant. If
    // this happens then the last instruction must use $rd as the result
    // register.
    const M6502MCExpr *GotExpr = M6502MCExpr::create(M6502MCExpr::MEK_GOT_DISP,
                                                   Res.getSymA(),
                                                   getContext());
    const MCExpr *LoExpr = nullptr;
    if (Res.getConstant() != 0) {
      // Symbols fully resolve with just the %got_disp(symbol) but we
      // must still account for any offset to the symbol for
      // expressions like symbol+8.
      LoExpr = MCConstantExpr::create(Res.getConstant(), getContext());

      // FIXME: Offsets greater than 16 bits are not yet implemented.
      // FIXME: The correct range is a 32-bit sign-extended number.
      if (Res.getConstant() < -0x8000 || Res.getConstant() > 0x7fff) {
        Error(IDLoc, "macro instruction uses large offset, which is not "
                     "currently supported");
        return true;
      }
    }

    unsigned TmpReg = DstReg;
    if (UseSrcReg &&
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg,
                                                               SrcReg)) {
      // If $rs is the same as $rd, we need to use AT.
      // If it is not available we exit.
      unsigned ATReg = getATReg(IDLoc);
      if (!ATReg)
        return true;
      TmpReg = ATReg;
    }

    TOut.emitRRX(M6502::LD, TmpReg, ABI.GetGlobalPtr(),
                 MCOperand::createExpr(GotExpr), IDLoc, STI);

    if (LoExpr)
      TOut.emitRRX(M6502::DADDiu, TmpReg, TmpReg, MCOperand::createExpr(LoExpr),
                   IDLoc, STI);

    if (UseSrcReg)
      TOut.emitRRR(M6502::DADDu, DstReg, TmpReg, SrcReg, IDLoc, STI);

    return false;
  }

  const M6502MCExpr *HiExpr =
      M6502MCExpr::create(M6502MCExpr::MEK_HI, SymExpr, getContext());
  const M6502MCExpr *LoExpr =
      M6502MCExpr::create(M6502MCExpr::MEK_LO, SymExpr, getContext());

  // This is the 64-bit symbol address expansion.
  if (ABI.ArePtrs64bit() && isGP64bit()) {
    // We need AT for the 64-bit expansion in the cases where the optional
    // source register is the destination register and for the superscalar
    // scheduled form.
    //
    // If it is not available we exit if the destination is the same as the
    // source register.

    const M6502MCExpr *HighestExpr =
        M6502MCExpr::create(M6502MCExpr::MEK_HIGHEST, SymExpr, getContext());
    const M6502MCExpr *HigherExpr =
        M6502MCExpr::create(M6502MCExpr::MEK_HIGHER, SymExpr, getContext());

    bool RdRegIsRsReg =
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, SrcReg);

    if (canUseATReg() && UseSrcReg && RdRegIsRsReg) {
      unsigned ATReg = getATReg(IDLoc);

      // If $rs is the same as $rd:
      // (d)la $rd, sym($rd) => lui    $at, %highest(sym)
      //                        daddiu $at, $at, %higher(sym)
      //                        dsll   $at, $at, 16
      //                        daddiu $at, $at, %hi(sym)
      //                        dsll   $at, $at, 16
      //                        daddiu $at, $at, %lo(sym)
      //                        daddu  $rd, $at, $rd
      TOut.emitRX(M6502::LUi, ATReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRRX(M6502::DADDiu, ATReg, ATReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRI(M6502::DSLL, ATReg, ATReg, 16, IDLoc, STI);
      TOut.emitRRX(M6502::DADDiu, ATReg, ATReg, MCOperand::createExpr(HiExpr),
                   IDLoc, STI);
      TOut.emitRRI(M6502::DSLL, ATReg, ATReg, 16, IDLoc, STI);
      TOut.emitRRX(M6502::DADDiu, ATReg, ATReg, MCOperand::createExpr(LoExpr),
                   IDLoc, STI);
      TOut.emitRRR(M6502::DADDu, DstReg, ATReg, SrcReg, IDLoc, STI);

      return false;
    } else if (canUseATReg() && !RdRegIsRsReg) {
      unsigned ATReg = getATReg(IDLoc);

      // If the $rs is different from $rd or if $rs isn't specified and we
      // have $at available:
      // (d)la $rd, sym/sym($rs) => lui    $rd, %highest(sym)
      //                            lui    $at, %hi(sym)
      //                            daddiu $rd, $rd, %higher(sym)
      //                            daddiu $at, $at, %lo(sym)
      //                            dsll32 $rd, $rd, 0
      //                            daddu  $rd, $rd, $at
      //                            (daddu  $rd, $rd, $rs)
      //
      // Which is preferred for superscalar issue.
      TOut.emitRX(M6502::LUi, DstReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRX(M6502::LUi, ATReg, MCOperand::createExpr(HiExpr), IDLoc, STI);
      TOut.emitRRX(M6502::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRX(M6502::DADDiu, ATReg, ATReg, MCOperand::createExpr(LoExpr),
                   IDLoc, STI);
      TOut.emitRRI(M6502::DSLL32, DstReg, DstReg, 0, IDLoc, STI);
      TOut.emitRRR(M6502::DADDu, DstReg, DstReg, ATReg, IDLoc, STI);
      if (UseSrcReg)
        TOut.emitRRR(M6502::DADDu, DstReg, DstReg, SrcReg, IDLoc, STI);

      return false;
    } else if (!canUseATReg() && !RdRegIsRsReg) {
      // Otherwise, synthesize the address in the destination register
      // serially:
      // (d)la $rd, sym/sym($rs) => lui    $rd, %highest(sym)
      //                            daddiu $rd, $rd, %higher(sym)
      //                            dsll   $rd, $rd, 16
      //                            daddiu $rd, $rd, %hi(sym)
      //                            dsll   $rd, $rd, 16
      //                            daddiu $rd, $rd, %lo(sym)
      TOut.emitRX(M6502::LUi, DstReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRRX(M6502::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRI(M6502::DSLL, DstReg, DstReg, 16, IDLoc, STI);
      TOut.emitRRX(M6502::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(HiExpr), IDLoc, STI);
      TOut.emitRRI(M6502::DSLL, DstReg, DstReg, 16, IDLoc, STI);
      TOut.emitRRX(M6502::DADDiu, DstReg, DstReg,
                   MCOperand::createExpr(LoExpr), IDLoc, STI);
      if (UseSrcReg)
        TOut.emitRRR(M6502::DADDu, DstReg, DstReg, SrcReg, IDLoc, STI);

      return false;
    } else {
      // We have a case where SrcReg == DstReg and we don't have $at
      // available. We can't expand this case, so error out appropriately.
      assert(SrcReg == DstReg && !canUseATReg() &&
             "Could have expanded dla but didn't?");
      reportParseError(IDLoc,
                     "pseudo-instruction requires $at, which is not available");
      return true;
    }
  }

  // And now, the 32-bit symbol address expansion:
  // If $rs is the same as $rd:
  // (d)la $rd, sym($rd)     => lui   $at, %hi(sym)
  //                            ori   $at, $at, %lo(sym)
  //                            addu  $rd, $at, $rd
  // Otherwise, if the $rs is different from $rd or if $rs isn't specified:
  // (d)la $rd, sym/sym($rs) => lui   $rd, %hi(sym)
  //                            ori   $rd, $rd, %lo(sym)
  //                            (addu $rd, $rd, $rs)
  unsigned TmpReg = DstReg;
  if (UseSrcReg &&
      getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, SrcReg)) {
    // If $rs is the same as $rd, we need to use AT.
    // If it is not available we exit.
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;
    TmpReg = ATReg;
  }

  TOut.emitRX(M6502::LUi, TmpReg, MCOperand::createExpr(HiExpr), IDLoc, STI);
  TOut.emitRRX(M6502::ADDiu, TmpReg, TmpReg, MCOperand::createExpr(LoExpr),
               IDLoc, STI);

  if (UseSrcReg)
    TOut.emitRRR(M6502::ADDu, DstReg, TmpReg, SrcReg, IDLoc, STI);
  else
    assert(
        getContext().getRegisterInfo()->isSuperOrSubRegisterEq(DstReg, TmpReg));

  return false;
}

// Each double-precision register DO-D15 overlaps with two of the single
// precision registers F0-F31. As an example, all of the following hold true:
// D0 + 1 == F1, F1 + 1 == D1, F1 + 1 == F2, depending on the context.
static unsigned nextReg(unsigned Reg) {
  if (M6502MCRegisterClasses[M6502::FGR32RegClassID].contains(Reg))
    return Reg == (unsigned)M6502::F31 ? (unsigned)M6502::F0 : Reg + 1;
  switch (Reg) {
  default: llvm_unreachable("Unknown register in assembly macro expansion!");
  case M6502::ZERO: return M6502::AT;
  case M6502::AT:   return M6502::V0;
  case M6502::V0:   return M6502::V1;
  case M6502::V1:   return M6502::A0;
  case M6502::A0:   return M6502::A1;
  case M6502::A1:   return M6502::A2;
  case M6502::A2:   return M6502::A3;
  case M6502::A3:   return M6502::T0;
  case M6502::T0:   return M6502::T1;
  case M6502::T1:   return M6502::T2;
  case M6502::T2:   return M6502::T3;
  case M6502::T3:   return M6502::T4;
  case M6502::T4:   return M6502::T5;
  case M6502::T5:   return M6502::T6;
  case M6502::T6:   return M6502::T7;
  case M6502::T7:   return M6502::S0;
  case M6502::S0:   return M6502::S1;
  case M6502::S1:   return M6502::S2;
  case M6502::S2:   return M6502::S3;
  case M6502::S3:   return M6502::S4;
  case M6502::S4:   return M6502::S5;
  case M6502::S5:   return M6502::S6;
  case M6502::S6:   return M6502::S7;
  case M6502::S7:   return M6502::T8;
  case M6502::T8:   return M6502::T9;
  case M6502::T9:   return M6502::K0;
  case M6502::K0:   return M6502::K1;
  case M6502::K1:   return M6502::GP;
  case M6502::GP:   return M6502::SP;
  case M6502::SP:   return M6502::FP;
  case M6502::FP:   return M6502::RA;
  case M6502::RA:   return M6502::ZERO;
  case M6502::D0:   return M6502::F1;
  case M6502::D1:   return M6502::F3;
  case M6502::D2:   return M6502::F5;
  case M6502::D3:   return M6502::F7;
  case M6502::D4:   return M6502::F9;
  case M6502::D5:   return M6502::F11;
  case M6502::D6:   return M6502::F13;
  case M6502::D7:   return M6502::F15;
  case M6502::D8:   return M6502::F17;
  case M6502::D9:   return M6502::F19;
  case M6502::D10:   return M6502::F21;
  case M6502::D11:   return M6502::F23;
  case M6502::D12:   return M6502::F25;
  case M6502::D13:   return M6502::F27;
  case M6502::D14:   return M6502::F29;
  case M6502::D15:   return M6502::F31;
  }
}

// FIXME: This method is too general. In principle we should compute the number
// of instructions required to synthesize the immediate inline compared to
// synthesizing the address inline and relying on non .text sections.
// For static O32 and N32 this may yield a small benefit, for static N64 this is
// likely to yield a much larger benefit as we have to synthesize a 64bit
// address to load a 64 bit value.
bool M6502AsmParser::emitPartialAddress(M6502TargetStreamer &TOut, SMLoc IDLoc,
                                       MCSymbol *Sym) {
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  if(IsPicEnabled) {
    const MCExpr *GotSym =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    const M6502MCExpr *GotExpr =
        M6502MCExpr::create(M6502MCExpr::MEK_GOT, GotSym, getContext());

    if(isABI_O32() || isABI_N32()) {
      TOut.emitRRX(M6502::LW, ATReg, M6502::GP, MCOperand::createExpr(GotExpr),
                   IDLoc, STI);
    } else { //isABI_N64()
      TOut.emitRRX(M6502::LD, ATReg, M6502::GP, MCOperand::createExpr(GotExpr),
                   IDLoc, STI);
    }
  } else { //!IsPicEnabled
    const MCExpr *HiSym =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    const M6502MCExpr *HiExpr =
        M6502MCExpr::create(M6502MCExpr::MEK_HI, HiSym, getContext());

    // FIXME: This is technically correct but gives a different result to gas,
    // but gas is incomplete there (it has a fixme noting it doesn't work with
    // 64-bit addresses).
    // FIXME: With -msym32 option, the address expansion for N64 should probably
    // use the O32 / N32 case. It's safe to use the 64 address expansion as the
    // symbol's value is considered sign extended.
    if(isABI_O32() || isABI_N32()) {
      TOut.emitRX(M6502::LUi, ATReg, MCOperand::createExpr(HiExpr), IDLoc, STI);
    } else { //isABI_N64()
      const MCExpr *HighestSym =
          MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
      const M6502MCExpr *HighestExpr =
          M6502MCExpr::create(M6502MCExpr::MEK_HIGHEST, HighestSym, getContext());
      const MCExpr *HigherSym =
          MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
      const M6502MCExpr *HigherExpr =
          M6502MCExpr::create(M6502MCExpr::MEK_HIGHER, HigherSym, getContext());

      TOut.emitRX(M6502::LUi, ATReg, MCOperand::createExpr(HighestExpr), IDLoc,
                  STI);
      TOut.emitRRX(M6502::DADDiu, ATReg, ATReg,
                   MCOperand::createExpr(HigherExpr), IDLoc, STI);
      TOut.emitRRI(M6502::DSLL, ATReg, ATReg, 16, IDLoc, STI);
      TOut.emitRRX(M6502::DADDiu, ATReg, ATReg, MCOperand::createExpr(HiExpr),
                   IDLoc, STI);
      TOut.emitRRI(M6502::DSLL, ATReg, ATReg, 16, IDLoc, STI);
    }
  }
  return false;
}

bool M6502AsmParser::expandLoadImmReal(MCInst &Inst, bool IsSingle, bool IsGPR,
                                      bool Is64FPU, SMLoc IDLoc,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  assert(Inst.getNumOperands() == 2 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(1).isImm() &&
         "Invalid instruction operand.");

  unsigned FirstReg = Inst.getOperand(0).getReg();
  uint64_t ImmOp64 = Inst.getOperand(1).getImm();

  uint32_t HiImmOp64 = (ImmOp64 & 0xffffffff00000000) >> 32;
  // If ImmOp64 is AsmToken::Integer type (all bits set to zero in the
  // exponent field), convert it to double (e.g. 1 to 1.0)
  if ((HiImmOp64 & 0x7ff00000) == 0) {
    APFloat RealVal(APFloat::IEEEdouble(), ImmOp64);
    ImmOp64 = RealVal.bitcastToAPInt().getZExtValue();
  }

  uint32_t LoImmOp64 = ImmOp64 & 0xffffffff;
  HiImmOp64 = (ImmOp64 & 0xffffffff00000000) >> 32;

  if (IsSingle) {
    // Conversion of a double in an uint64_t to a float in a uint32_t,
    // retaining the bit pattern of a float.
    uint32_t ImmOp32;
    double doubleImm = BitsToDouble(ImmOp64);
    float tmp_float = static_cast<float>(doubleImm);
    ImmOp32 = FloatToBits(tmp_float);

    if (IsGPR) {
      if (loadImmediate(ImmOp32, FirstReg, M6502::NoRegister, true, true, IDLoc,
                        Out, STI))
        return true;
      return false;
    } else {
      unsigned ATReg = getATReg(IDLoc);
      if (!ATReg)
        return true;
      if (LoImmOp64 == 0) {
        if (loadImmediate(ImmOp32, ATReg, M6502::NoRegister, true, true, IDLoc,
                          Out, STI))
          return true;
        TOut.emitRR(M6502::MTC1, FirstReg, ATReg, IDLoc, STI);
        return false;
      }

      MCSection *CS = getStreamer().getCurrentSectionOnly();
      // FIXME: Enhance this expansion to use the .lit4 & .lit8 sections
      // where appropriate.
      MCSection *ReadOnlySection = getContext().getELFSection(
          ".rodata", ELF::SHT_PROGBITS, ELF::SHF_ALLOC);

      MCSymbol *Sym = getContext().createTempSymbol();
      const MCExpr *LoSym =
          MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
      const M6502MCExpr *LoExpr =
          M6502MCExpr::create(M6502MCExpr::MEK_LO, LoSym, getContext());

      getStreamer().SwitchSection(ReadOnlySection);
      getStreamer().EmitLabel(Sym, IDLoc);
      getStreamer().EmitIntValue(ImmOp32, 4);
      getStreamer().SwitchSection(CS);

      if(emitPartialAddress(TOut, IDLoc, Sym))
        return true;
      TOut.emitRRX(M6502::LWC1, FirstReg, ATReg,
                   MCOperand::createExpr(LoExpr), IDLoc, STI);
    }
    return false;
  }

  // if(!IsSingle)
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  if (IsGPR) {
    if (LoImmOp64 == 0) {
      if(isABI_N32() || isABI_N64()) {
        if (loadImmediate(HiImmOp64, FirstReg, M6502::NoRegister, false, true,
                          IDLoc, Out, STI))
          return true;
        return false;
      } else {
        if (loadImmediate(HiImmOp64, FirstReg, M6502::NoRegister, true, true,
                        IDLoc, Out, STI))
          return true;

        if (loadImmediate(0, nextReg(FirstReg), M6502::NoRegister, true, true,
                        IDLoc, Out, STI))
          return true;
        return false;
      }
    }

    MCSection *CS = getStreamer().getCurrentSectionOnly();
    MCSection *ReadOnlySection = getContext().getELFSection(
        ".rodata", ELF::SHT_PROGBITS, ELF::SHF_ALLOC);

    MCSymbol *Sym = getContext().createTempSymbol();
    const MCExpr *LoSym =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    const M6502MCExpr *LoExpr =
        M6502MCExpr::create(M6502MCExpr::MEK_LO, LoSym, getContext());

    getStreamer().SwitchSection(ReadOnlySection);
    getStreamer().EmitLabel(Sym, IDLoc);
    getStreamer().EmitIntValue(HiImmOp64, 4);
    getStreamer().EmitIntValue(LoImmOp64, 4);
    getStreamer().SwitchSection(CS);

    if(emitPartialAddress(TOut, IDLoc, Sym))
      return true;
    if(isABI_N64())
      TOut.emitRRX(M6502::DADDiu, ATReg, ATReg,
                   MCOperand::createExpr(LoExpr), IDLoc, STI);
    else
      TOut.emitRRX(M6502::ADDiu, ATReg, ATReg,
                   MCOperand::createExpr(LoExpr), IDLoc, STI);

    if(isABI_N32() || isABI_N64())
      TOut.emitRRI(M6502::LD, FirstReg, ATReg, 0, IDLoc, STI);
    else {
      TOut.emitRRI(M6502::LW, FirstReg, ATReg, 0, IDLoc, STI);
      TOut.emitRRI(M6502::LW, nextReg(FirstReg), ATReg, 4, IDLoc, STI);
    }
    return false;
  } else { // if(!IsGPR && !IsSingle)
    if ((LoImmOp64 == 0) &&
        !((HiImmOp64 & 0xffff0000) && (HiImmOp64 & 0x0000ffff))) {
      // FIXME: In the case where the constant is zero, we can load the
      // register directly from the zero register.
      if (loadImmediate(HiImmOp64, ATReg, M6502::NoRegister, true, true, IDLoc,
                        Out, STI))
        return true;
      if (isABI_N32() || isABI_N64())
        TOut.emitRR(M6502::DMTC1, FirstReg, ATReg, IDLoc, STI);
      else if (hasM650232r2()) {
        TOut.emitRR(M6502::MTC1, FirstReg, M6502::ZERO, IDLoc, STI);
        TOut.emitRRR(M6502::MTHC1_D32, FirstReg, FirstReg, ATReg, IDLoc, STI);
      } else {
        TOut.emitRR(M6502::MTC1, nextReg(FirstReg), ATReg, IDLoc, STI);
        TOut.emitRR(M6502::MTC1, FirstReg, M6502::ZERO, IDLoc, STI);
      }
      return false;
    }

    MCSection *CS = getStreamer().getCurrentSectionOnly();
    // FIXME: Enhance this expansion to use the .lit4 & .lit8 sections
    // where appropriate.
    MCSection *ReadOnlySection = getContext().getELFSection(
        ".rodata", ELF::SHT_PROGBITS, ELF::SHF_ALLOC);

    MCSymbol *Sym = getContext().createTempSymbol();
    const MCExpr *LoSym =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    const M6502MCExpr *LoExpr =
        M6502MCExpr::create(M6502MCExpr::MEK_LO, LoSym, getContext());

    getStreamer().SwitchSection(ReadOnlySection);
    getStreamer().EmitLabel(Sym, IDLoc);
    getStreamer().EmitIntValue(HiImmOp64, 4);
    getStreamer().EmitIntValue(LoImmOp64, 4);
    getStreamer().SwitchSection(CS);

    if(emitPartialAddress(TOut, IDLoc, Sym))
      return true;
    TOut.emitRRX(Is64FPU ? M6502::LDC164 : M6502::LDC1, FirstReg, ATReg,
                 MCOperand::createExpr(LoExpr), IDLoc, STI);
  }
  return false;
}

bool M6502AsmParser::expandUncondBranchMMPseudo(MCInst &Inst, SMLoc IDLoc,
                                               MCStreamer &Out,
                                               const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  assert(getInstDesc(Inst.getOpcode()).getNumOperands() == 1 &&
         "unexpected number of operands");

  MCOperand Offset = Inst.getOperand(0);
  if (Offset.isExpr()) {
    Inst.clear();
    Inst.setOpcode(M6502::BEQ_MM);
    Inst.addOperand(MCOperand::createReg(M6502::ZERO));
    Inst.addOperand(MCOperand::createReg(M6502::ZERO));
    Inst.addOperand(MCOperand::createExpr(Offset.getExpr()));
  } else {
    assert(Offset.isImm() && "expected immediate operand kind");
    if (isInt<11>(Offset.getImm())) {
      // If offset fits into 11 bits then this instruction becomes microM6502
      // 16-bit unconditional branch instruction.
      if (inMicroM6502Mode())
        Inst.setOpcode(hasM650232r6() ? M6502::BC16_MMR6 : M6502::B16_MM);
    } else {
      if (!isInt<17>(Offset.getImm()))
        return Error(IDLoc, "branch target out of range");
      if (OffsetToAlignment(Offset.getImm(), 1LL << 1))
        return Error(IDLoc, "branch to misaligned address");
      Inst.clear();
      Inst.setOpcode(M6502::BEQ_MM);
      Inst.addOperand(MCOperand::createReg(M6502::ZERO));
      Inst.addOperand(MCOperand::createReg(M6502::ZERO));
      Inst.addOperand(MCOperand::createImm(Offset.getImm()));
    }
  }
  Out.EmitInstruction(Inst, *STI);

  // If .set reorder is active and branch instruction has a delay slot,
  // emit a NOP after it.
  const MCInstrDesc &MCID = getInstDesc(Inst.getOpcode());
  if (MCID.hasDelaySlot() && AssemblerOptions.back()->isReorder())
    TOut.emitEmptyDelaySlot(true, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandBranchImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");

  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");

  const MCOperand &MemOffsetOp = Inst.getOperand(2);
  assert((MemOffsetOp.isImm() || MemOffsetOp.isExpr()) &&
         "expected immediate or expression operand");

  bool IsLikely = false;

  unsigned OpCode = 0;
  switch(Inst.getOpcode()) {
    case M6502::BneImm:
      OpCode = M6502::BNE;
      break;
    case M6502::BeqImm:
      OpCode = M6502::BEQ;
      break;
    case M6502::BEQLImmMacro:
      OpCode = M6502::BEQL;
      IsLikely = true;
      break;
    case M6502::BNELImmMacro:
      OpCode = M6502::BNEL;
      IsLikely = true;
      break;
    default:
      llvm_unreachable("Unknown immediate branch pseudo-instruction.");
      break;
  }

  int64_t ImmValue = ImmOp.getImm();
  if (ImmValue == 0) {
    if (IsLikely) {
      TOut.emitRRX(OpCode, DstRegOp.getReg(), M6502::ZERO,
                   MCOperand::createExpr(MemOffsetOp.getExpr()), IDLoc, STI);
      TOut.emitRRI(M6502::SLL, M6502::ZERO, M6502::ZERO, 0, IDLoc, STI);
    } else
      TOut.emitRRX(OpCode, DstRegOp.getReg(), M6502::ZERO, MemOffsetOp, IDLoc,
              STI);
  } else {
    warnIfNoMacro(IDLoc);

    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;

    if (loadImmediate(ImmValue, ATReg, M6502::NoRegister, !isGP64bit(), true,
                      IDLoc, Out, STI))
      return true;

    if (IsLikely) {
      TOut.emitRRX(OpCode, DstRegOp.getReg(), ATReg,
              MCOperand::createExpr(MemOffsetOp.getExpr()), IDLoc, STI);
      TOut.emitRRI(M6502::SLL, M6502::ZERO, M6502::ZERO, 0, IDLoc, STI);
    } else
      TOut.emitRRX(OpCode, DstRegOp.getReg(), ATReg, MemOffsetOp, IDLoc, STI);
  }
  return false;
}

void M6502AsmParser::expandMemInst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                  const MCSubtargetInfo *STI, bool IsLoad,
                                  bool IsImmOpnd) {
  if (IsLoad) {
    expandLoadInst(Inst, IDLoc, Out, STI, IsImmOpnd);
    return;
  }
  expandStoreInst(Inst, IDLoc, Out, STI, IsImmOpnd);
}

void M6502AsmParser::expandLoadInst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                   const MCSubtargetInfo *STI, bool IsImmOpnd) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned BaseReg = Inst.getOperand(1).getReg();

  const MCInstrDesc &Desc = getInstDesc(Inst.getOpcode());
  int16_t DstRegClass = Desc.OpInfo[0].RegClass;
  unsigned DstRegClassID =
      getContext().getRegisterInfo()->getRegClass(DstRegClass).getID();
  bool IsGPR = (DstRegClassID == M6502::GPR32RegClassID) ||
               (DstRegClassID == M6502::GPR64RegClassID);

  if (IsImmOpnd) {
    // Try to use DstReg as the temporary.
    if (IsGPR && (BaseReg != DstReg)) {
      TOut.emitLoadWithImmOffset(Inst.getOpcode(), DstReg, BaseReg,
                                 Inst.getOperand(2).getImm(), DstReg, IDLoc,
                                 STI);
      return;
    }

    // At this point we need AT to perform the expansions and we exit if it is
    // not available.
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return;

    TOut.emitLoadWithImmOffset(Inst.getOpcode(), DstReg, BaseReg,
                               Inst.getOperand(2).getImm(), ATReg, IDLoc, STI);
    return;
  }

  const MCExpr *ExprOffset = Inst.getOperand(2).getExpr();
  MCOperand LoOperand = MCOperand::createExpr(
      M6502MCExpr::create(M6502MCExpr::MEK_LO, ExprOffset, getContext()));
  MCOperand HiOperand = MCOperand::createExpr(
      M6502MCExpr::create(M6502MCExpr::MEK_HI, ExprOffset, getContext()));

  // Try to use DstReg as the temporary.
  if (IsGPR && (BaseReg != DstReg)) {
    TOut.emitLoadWithSymOffset(Inst.getOpcode(), DstReg, BaseReg, HiOperand,
                               LoOperand, DstReg, IDLoc, STI);
    return;
  }

  // At this point we need AT to perform the expansions and we exit if it is
  // not available.
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return;

  TOut.emitLoadWithSymOffset(Inst.getOpcode(), DstReg, BaseReg, HiOperand,
                             LoOperand, ATReg, IDLoc, STI);
}

void M6502AsmParser::expandStoreInst(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI,
                                    bool IsImmOpnd) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  unsigned SrcReg = Inst.getOperand(0).getReg();
  unsigned BaseReg = Inst.getOperand(1).getReg();

  if (IsImmOpnd) {
    TOut.emitStoreWithImmOffset(Inst.getOpcode(), SrcReg, BaseReg,
                                Inst.getOperand(2).getImm(),
                                [&]() { return getATReg(IDLoc); }, IDLoc, STI);
    return;
  }

  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return;

  const MCExpr *ExprOffset = Inst.getOperand(2).getExpr();
  MCOperand LoOperand = MCOperand::createExpr(
      M6502MCExpr::create(M6502MCExpr::MEK_LO, ExprOffset, getContext()));
  MCOperand HiOperand = MCOperand::createExpr(
      M6502MCExpr::create(M6502MCExpr::MEK_HI, ExprOffset, getContext()));
  TOut.emitStoreWithSymOffset(Inst.getOpcode(), SrcReg, BaseReg, HiOperand,
                              LoOperand, ATReg, IDLoc, STI);
}

bool M6502AsmParser::expandLoadStoreMultiple(MCInst &Inst, SMLoc IDLoc,
                                            MCStreamer &Out,
                                            const MCSubtargetInfo *STI) {
  unsigned OpNum = Inst.getNumOperands();
  unsigned Opcode = Inst.getOpcode();
  unsigned NewOpcode = Opcode == M6502::SWM_MM ? M6502::SWM32_MM : M6502::LWM32_MM;

  assert(Inst.getOperand(OpNum - 1).isImm() &&
         Inst.getOperand(OpNum - 2).isReg() &&
         Inst.getOperand(OpNum - 3).isReg() && "Invalid instruction operand.");

  if (OpNum < 8 && Inst.getOperand(OpNum - 1).getImm() <= 60 &&
      Inst.getOperand(OpNum - 1).getImm() >= 0 &&
      (Inst.getOperand(OpNum - 2).getReg() == M6502::SP ||
       Inst.getOperand(OpNum - 2).getReg() == M6502::SP_64) &&
      (Inst.getOperand(OpNum - 3).getReg() == M6502::RA ||
       Inst.getOperand(OpNum - 3).getReg() == M6502::RA_64)) {
    // It can be implemented as SWM16 or LWM16 instruction.
    if (inMicroM6502Mode() && hasM650232r6())
      NewOpcode = Opcode == M6502::SWM_MM ? M6502::SWM16_MMR6 : M6502::LWM16_MMR6;
    else
      NewOpcode = Opcode == M6502::SWM_MM ? M6502::SWM16_MM : M6502::LWM16_MM;
  }

  Inst.setOpcode(NewOpcode);
  Out.EmitInstruction(Inst, *STI);
  return false;
}

bool M6502AsmParser::expandCondBranches(MCInst &Inst, SMLoc IDLoc,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  bool EmittedNoMacroWarning = false;
  unsigned PseudoOpcode = Inst.getOpcode();
  unsigned SrcReg = Inst.getOperand(0).getReg();
  const MCOperand &TrgOp = Inst.getOperand(1);
  const MCExpr *OffsetExpr = Inst.getOperand(2).getExpr();

  unsigned ZeroSrcOpcode, ZeroTrgOpcode;
  bool ReverseOrderSLT, IsUnsigned, IsLikely, AcceptsEquality;

  unsigned TrgReg;
  if (TrgOp.isReg())
    TrgReg = TrgOp.getReg();
  else if (TrgOp.isImm()) {
    warnIfNoMacro(IDLoc);
    EmittedNoMacroWarning = true;

    TrgReg = getATReg(IDLoc);
    if (!TrgReg)
      return true;

    switch(PseudoOpcode) {
    default:
      llvm_unreachable("unknown opcode for branch pseudo-instruction");
    case M6502::BLTImmMacro:
      PseudoOpcode = M6502::BLT;
      break;
    case M6502::BLEImmMacro:
      PseudoOpcode = M6502::BLE;
      break;
    case M6502::BGEImmMacro:
      PseudoOpcode = M6502::BGE;
      break;
    case M6502::BGTImmMacro:
      PseudoOpcode = M6502::BGT;
      break;
    case M6502::BLTUImmMacro:
      PseudoOpcode = M6502::BLTU;
      break;
    case M6502::BLEUImmMacro:
      PseudoOpcode = M6502::BLEU;
      break;
    case M6502::BGEUImmMacro:
      PseudoOpcode = M6502::BGEU;
      break;
    case M6502::BGTUImmMacro:
      PseudoOpcode = M6502::BGTU;
      break;
    case M6502::BLTLImmMacro:
      PseudoOpcode = M6502::BLTL;
      break;
    case M6502::BLELImmMacro:
      PseudoOpcode = M6502::BLEL;
      break;
    case M6502::BGELImmMacro:
      PseudoOpcode = M6502::BGEL;
      break;
    case M6502::BGTLImmMacro:
      PseudoOpcode = M6502::BGTL;
      break;
    case M6502::BLTULImmMacro:
      PseudoOpcode = M6502::BLTUL;
      break;
    case M6502::BLEULImmMacro:
      PseudoOpcode = M6502::BLEUL;
      break;
    case M6502::BGEULImmMacro:
      PseudoOpcode = M6502::BGEUL;
      break;
    case M6502::BGTULImmMacro:
      PseudoOpcode = M6502::BGTUL;
      break;
    }

    if (loadImmediate(TrgOp.getImm(), TrgReg, M6502::NoRegister, !isGP64bit(),
                      false, IDLoc, Out, STI))
      return true;
  }

  switch (PseudoOpcode) {
  case M6502::BLT:
  case M6502::BLTU:
  case M6502::BLTL:
  case M6502::BLTUL:
    AcceptsEquality = false;
    ReverseOrderSLT = false;
    IsUnsigned = ((PseudoOpcode == M6502::BLTU) || (PseudoOpcode == M6502::BLTUL));
    IsLikely = ((PseudoOpcode == M6502::BLTL) || (PseudoOpcode == M6502::BLTUL));
    ZeroSrcOpcode = M6502::BGTZ;
    ZeroTrgOpcode = M6502::BLTZ;
    break;
  case M6502::BLE:
  case M6502::BLEU:
  case M6502::BLEL:
  case M6502::BLEUL:
    AcceptsEquality = true;
    ReverseOrderSLT = true;
    IsUnsigned = ((PseudoOpcode == M6502::BLEU) || (PseudoOpcode == M6502::BLEUL));
    IsLikely = ((PseudoOpcode == M6502::BLEL) || (PseudoOpcode == M6502::BLEUL));
    ZeroSrcOpcode = M6502::BGEZ;
    ZeroTrgOpcode = M6502::BLEZ;
    break;
  case M6502::BGE:
  case M6502::BGEU:
  case M6502::BGEL:
  case M6502::BGEUL:
    AcceptsEquality = true;
    ReverseOrderSLT = false;
    IsUnsigned = ((PseudoOpcode == M6502::BGEU) || (PseudoOpcode == M6502::BGEUL));
    IsLikely = ((PseudoOpcode == M6502::BGEL) || (PseudoOpcode == M6502::BGEUL));
    ZeroSrcOpcode = M6502::BLEZ;
    ZeroTrgOpcode = M6502::BGEZ;
    break;
  case M6502::BGT:
  case M6502::BGTU:
  case M6502::BGTL:
  case M6502::BGTUL:
    AcceptsEquality = false;
    ReverseOrderSLT = true;
    IsUnsigned = ((PseudoOpcode == M6502::BGTU) || (PseudoOpcode == M6502::BGTUL));
    IsLikely = ((PseudoOpcode == M6502::BGTL) || (PseudoOpcode == M6502::BGTUL));
    ZeroSrcOpcode = M6502::BLTZ;
    ZeroTrgOpcode = M6502::BGTZ;
    break;
  default:
    llvm_unreachable("unknown opcode for branch pseudo-instruction");
  }

  bool IsTrgRegZero = (TrgReg == M6502::ZERO);
  bool IsSrcRegZero = (SrcReg == M6502::ZERO);
  if (IsSrcRegZero && IsTrgRegZero) {
    // FIXME: All of these Opcode-specific if's are needed for compatibility
    // with GAS' behaviour. However, they may not generate the most efficient
    // code in some circumstances.
    if (PseudoOpcode == M6502::BLT) {
      TOut.emitRX(M6502::BLTZ, M6502::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      return false;
    }
    if (PseudoOpcode == M6502::BLE) {
      TOut.emitRX(M6502::BLEZ, M6502::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    if (PseudoOpcode == M6502::BGE) {
      TOut.emitRX(M6502::BGEZ, M6502::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    if (PseudoOpcode == M6502::BGT) {
      TOut.emitRX(M6502::BGTZ, M6502::ZERO, MCOperand::createExpr(OffsetExpr),
                  IDLoc, STI);
      return false;
    }
    if (PseudoOpcode == M6502::BGTU) {
      TOut.emitRRX(M6502::BNE, M6502::ZERO, M6502::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      return false;
    }
    if (AcceptsEquality) {
      // If both registers are $0 and the pseudo-branch accepts equality, it
      // will always be taken, so we emit an unconditional branch.
      TOut.emitRRX(M6502::BEQ, M6502::ZERO, M6502::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    // If both registers are $0 and the pseudo-branch does not accept
    // equality, it will never be taken, so we don't have to emit anything.
    return false;
  }
  if (IsSrcRegZero || IsTrgRegZero) {
    if ((IsSrcRegZero && PseudoOpcode == M6502::BGTU) ||
        (IsTrgRegZero && PseudoOpcode == M6502::BLTU)) {
      // If the $rs is $0 and the pseudo-branch is BGTU (0 > x) or
      // if the $rt is $0 and the pseudo-branch is BLTU (x < 0),
      // the pseudo-branch will never be taken, so we don't emit anything.
      // This only applies to unsigned pseudo-branches.
      return false;
    }
    if ((IsSrcRegZero && PseudoOpcode == M6502::BLEU) ||
        (IsTrgRegZero && PseudoOpcode == M6502::BGEU)) {
      // If the $rs is $0 and the pseudo-branch is BLEU (0 <= x) or
      // if the $rt is $0 and the pseudo-branch is BGEU (x >= 0),
      // the pseudo-branch will always be taken, so we emit an unconditional
      // branch.
      // This only applies to unsigned pseudo-branches.
      TOut.emitRRX(M6502::BEQ, M6502::ZERO, M6502::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      Warning(IDLoc, "branch is always taken");
      return false;
    }
    if (IsUnsigned) {
      // If the $rs is $0 and the pseudo-branch is BLTU (0 < x) or
      // if the $rt is $0 and the pseudo-branch is BGTU (x > 0),
      // the pseudo-branch will be taken only when the non-zero register is
      // different from 0, so we emit a BNEZ.
      //
      // If the $rs is $0 and the pseudo-branch is BGEU (0 >= x) or
      // if the $rt is $0 and the pseudo-branch is BLEU (x <= 0),
      // the pseudo-branch will be taken only when the non-zero register is
      // equal to 0, so we emit a BEQZ.
      //
      // Because only BLEU and BGEU branch on equality, we can use the
      // AcceptsEquality variable to decide when to emit the BEQZ.
      TOut.emitRRX(AcceptsEquality ? M6502::BEQ : M6502::BNE,
                   IsSrcRegZero ? TrgReg : SrcReg, M6502::ZERO,
                   MCOperand::createExpr(OffsetExpr), IDLoc, STI);
      return false;
    }
    // If we have a signed pseudo-branch and one of the registers is $0,
    // we can use an appropriate compare-to-zero branch. We select which one
    // to use in the switch statement above.
    TOut.emitRX(IsSrcRegZero ? ZeroSrcOpcode : ZeroTrgOpcode,
                IsSrcRegZero ? TrgReg : SrcReg,
                MCOperand::createExpr(OffsetExpr), IDLoc, STI);
    return false;
  }

  // If neither the SrcReg nor the TrgReg are $0, we need AT to perform the
  // expansions. If it is not available, we return.
  unsigned ATRegNum = getATReg(IDLoc);
  if (!ATRegNum)
    return true;

  if (!EmittedNoMacroWarning)
    warnIfNoMacro(IDLoc);

  // SLT fits well with 2 of our 4 pseudo-branches:
  //   BLT, where $rs < $rt, translates into "slt $at, $rs, $rt" and
  //   BGT, where $rs > $rt, translates into "slt $at, $rt, $rs".
  // If the result of the SLT is 1, we branch, and if it's 0, we don't.
  // This is accomplished by using a BNEZ with the result of the SLT.
  //
  // The other 2 pseudo-branches are opposites of the above 2 (BGE with BLT
  // and BLE with BGT), so we change the BNEZ into a a BEQZ.
  // Because only BGE and BLE branch on equality, we can use the
  // AcceptsEquality variable to decide when to emit the BEQZ.
  // Note that the order of the SLT arguments doesn't change between
  // opposites.
  //
  // The same applies to the unsigned variants, except that SLTu is used
  // instead of SLT.
  TOut.emitRRR(IsUnsigned ? M6502::SLTu : M6502::SLT, ATRegNum,
               ReverseOrderSLT ? TrgReg : SrcReg,
               ReverseOrderSLT ? SrcReg : TrgReg, IDLoc, STI);

  TOut.emitRRX(IsLikely ? (AcceptsEquality ? M6502::BEQL : M6502::BNEL)
                        : (AcceptsEquality ? M6502::BEQ : M6502::BNE),
               ATRegNum, M6502::ZERO, MCOperand::createExpr(OffsetExpr), IDLoc,
               STI);
  return false;
}

// Expand a integer division macro.
//
// Notably we don't have to emit a warning when encountering $rt as the $zero
// register, or 0 as an immediate. processInstruction() has already done that.
//
// The destination register can only be $zero when expanding (S)DivIMacro or
// D(S)DivMacro.

bool M6502AsmParser::expandDiv(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI, const bool IsM650264,
                              const bool Signed) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  warnIfNoMacro(IDLoc);

  const MCOperand &RdRegOp = Inst.getOperand(0);
  assert(RdRegOp.isReg() && "expected register operand kind");
  unsigned RdReg = RdRegOp.getReg();

  const MCOperand &RsRegOp = Inst.getOperand(1);
  assert(RsRegOp.isReg() && "expected register operand kind");
  unsigned RsReg = RsRegOp.getReg();

  unsigned RtReg;
  int64_t ImmValue;

  const MCOperand &RtOp = Inst.getOperand(2);
  assert((RtOp.isReg() || RtOp.isImm()) &&
         "expected register or immediate operand kind");
  if (RtOp.isReg())
    RtReg = RtOp.getReg();
  else
    ImmValue = RtOp.getImm();

  unsigned DivOp;
  unsigned ZeroReg;
  unsigned SubOp;

  if (IsM650264) {
    DivOp = Signed ? M6502::DSDIV : M6502::DUDIV;
    ZeroReg = M6502::ZERO_64;
    SubOp = M6502::DSUB;
  } else {
    DivOp = Signed ? M6502::SDIV : M6502::UDIV;
    ZeroReg = M6502::ZERO;
    SubOp = M6502::SUB;
  }

  bool UseTraps = useTraps();

  if (RtOp.isImm()) {
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;

    if (ImmValue == 0) {
      if (UseTraps)
        TOut.emitRRI(M6502::TEQ, ZeroReg, ZeroReg, 0x7, IDLoc, STI);
      else
        TOut.emitII(M6502::BREAK, 0x7, 0, IDLoc, STI);
      return false;
    }

    if (ImmValue == 1) {
      TOut.emitRRR(M6502::OR, RdReg, RsReg, M6502::ZERO, IDLoc, STI);
      return false;
    } else if (Signed && ImmValue == -1) {
      TOut.emitRRR(SubOp, RdReg, ZeroReg, RsReg, IDLoc, STI);
      return false;
    } else {
      if (loadImmediate(ImmValue, ATReg, M6502::NoRegister, isInt<32>(ImmValue),
                        false, Inst.getLoc(), Out, STI))
        return true;
      TOut.emitRR(DivOp, RsReg, ATReg, IDLoc, STI);
      TOut.emitR(M6502::MFLO, RdReg, IDLoc, STI);
      return false;
    }
    return true;
  }

  // If the macro expansion of (d)div(u) would always trap or break, insert
  // the trap/break and exit. This gives a different result to GAS. GAS has
  // an inconsistency/missed optimization in that not all cases are handled
  // equivalently. As the observed behaviour is the same, we're ok.
  if (RtReg == M6502::ZERO || RtReg == M6502::ZERO_64) {
    if (UseTraps) {
      TOut.emitRRI(M6502::TEQ, ZeroReg, ZeroReg, 0x7, IDLoc, STI);
      return false;
    }
    TOut.emitII(M6502::BREAK, 0x7, 0, IDLoc, STI);
    return false;
  }

  // Temporary label for first branch traget
  MCContext &Context = TOut.getStreamer().getContext();
  MCSymbol *BrTarget;
  MCOperand LabelOp;

  if (UseTraps) {
    TOut.emitRRI(M6502::TEQ, RtReg, ZeroReg, 0x7, IDLoc, STI);
  } else {
    // Branch to the li instruction.
    BrTarget = Context.createTempSymbol();
    LabelOp = MCOperand::createExpr(MCSymbolRefExpr::create(BrTarget, Context));
    TOut.emitRRX(M6502::BNE, RtReg, ZeroReg, LabelOp, IDLoc, STI);
  }

  TOut.emitRR(DivOp, RsReg, RtReg, IDLoc, STI);

  if (!UseTraps)
    TOut.emitII(M6502::BREAK, 0x7, 0, IDLoc, STI);

  if (!Signed) {
    if (!UseTraps)
      TOut.getStreamer().EmitLabel(BrTarget);

    TOut.emitR(M6502::MFLO, RdReg, IDLoc, STI);
    return false;
  }

  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  if (!UseTraps)
    TOut.getStreamer().EmitLabel(BrTarget);

  TOut.emitRRI(M6502::ADDiu, ATReg, ZeroReg, -1, IDLoc, STI);

  // Temporary label for the second branch target.
  MCSymbol *BrTargetEnd = Context.createTempSymbol();
  MCOperand LabelOpEnd =
      MCOperand::createExpr(MCSymbolRefExpr::create(BrTargetEnd, Context));

  // Branch to the mflo instruction.
  TOut.emitRRX(M6502::BNE, RtReg, ATReg, LabelOpEnd, IDLoc, STI);

  if (IsM650264) {
    TOut.emitRRI(M6502::ADDiu, ATReg, ZeroReg, 1, IDLoc, STI);
    TOut.emitRRI(M6502::DSLL32, ATReg, ATReg, 0x1f, IDLoc, STI);
  } else {
    TOut.emitRI(M6502::LUi, ATReg, (uint16_t)0x8000, IDLoc, STI);
  }

  if (UseTraps)
    TOut.emitRRI(M6502::TEQ, RsReg, ATReg, 0x6, IDLoc, STI);
  else {
    // Branch to the mflo instruction.
    TOut.emitRRX(M6502::BNE, RsReg, ATReg, LabelOpEnd, IDLoc, STI);
    TOut.emitRRI(M6502::SLL, ZeroReg, ZeroReg, 0, IDLoc, STI);
    TOut.emitII(M6502::BREAK, 0x6, 0, IDLoc, STI);
  }

  TOut.getStreamer().EmitLabel(BrTargetEnd);
  TOut.emitR(M6502::MFLO, RdReg, IDLoc, STI);
  return false;
}

bool M6502AsmParser::expandTrunc(MCInst &Inst, bool IsDouble, bool Is64FPU,
                                SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() && Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isReg() && "Invalid instruction operand.");

  unsigned FirstReg = Inst.getOperand(0).getReg();
  unsigned SecondReg = Inst.getOperand(1).getReg();
  unsigned ThirdReg = Inst.getOperand(2).getReg();

  if (hasM65021() && !hasM65022()) {
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;
    TOut.emitRR(M6502::CFC1, ThirdReg, M6502::RA, IDLoc, STI);
    TOut.emitRR(M6502::CFC1, ThirdReg, M6502::RA, IDLoc, STI);
    TOut.emitNop(IDLoc, STI);
    TOut.emitRRI(M6502::ORi, ATReg, ThirdReg, 0x3, IDLoc, STI);
    TOut.emitRRI(M6502::XORi, ATReg, ATReg, 0x2, IDLoc, STI);
    TOut.emitRR(M6502::CTC1, M6502::RA, ATReg, IDLoc, STI);
    TOut.emitNop(IDLoc, STI);
    TOut.emitRR(IsDouble ? (Is64FPU ? M6502::CVT_W_D64 : M6502::CVT_W_D32)
                         : M6502::CVT_W_S,
                FirstReg, SecondReg, IDLoc, STI);
    TOut.emitRR(M6502::CTC1, M6502::RA, ThirdReg, IDLoc, STI);
    TOut.emitNop(IDLoc, STI);
    return false;
  }

  TOut.emitRR(IsDouble ? (Is64FPU ? M6502::TRUNC_W_D64 : M6502::TRUNC_W_D32)
                       : M6502::TRUNC_W_S,
              FirstReg, SecondReg, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandUlh(MCInst &Inst, bool Signed, SMLoc IDLoc,
                              MCStreamer &Out, const MCSubtargetInfo *STI) {
  if (hasM650232r6() || hasM650264r6()) {
    return Error(IDLoc, "instruction not supported on m650232r6 or m650264r6");
  }

  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &SrcRegOp = Inst.getOperand(1);
  assert(SrcRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetImmOp = Inst.getOperand(2);
  assert(OffsetImmOp.isImm() && "expected immediate operand kind");

  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = DstRegOp.getReg();
  unsigned SrcReg = SrcRegOp.getReg();
  int64_t OffsetValue = OffsetImmOp.getImm();

  // NOTE: We always need AT for ULHU, as it is always used as the source
  // register for one of the LBu's.
  warnIfNoMacro(IDLoc);
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  bool IsLargeOffset = !(isInt<16>(OffsetValue + 1) && isInt<16>(OffsetValue));
  if (IsLargeOffset) {
    if (loadImmediate(OffsetValue, ATReg, SrcReg, !ABI.ArePtrs64bit(), true,
                      IDLoc, Out, STI))
      return true;
  }

  int64_t FirstOffset = IsLargeOffset ? 0 : OffsetValue;
  int64_t SecondOffset = IsLargeOffset ? 1 : (OffsetValue + 1);
  if (isLittle())
    std::swap(FirstOffset, SecondOffset);

  unsigned FirstLbuDstReg = IsLargeOffset ? DstReg : ATReg;
  unsigned SecondLbuDstReg = IsLargeOffset ? ATReg : DstReg;

  unsigned LbuSrcReg = IsLargeOffset ? ATReg : SrcReg;
  unsigned SllReg = IsLargeOffset ? DstReg : ATReg;

  TOut.emitRRI(Signed ? M6502::LB : M6502::LBu, FirstLbuDstReg, LbuSrcReg,
               FirstOffset, IDLoc, STI);
  TOut.emitRRI(M6502::LBu, SecondLbuDstReg, LbuSrcReg, SecondOffset, IDLoc, STI);
  TOut.emitRRI(M6502::SLL, SllReg, SllReg, 8, IDLoc, STI);
  TOut.emitRRR(M6502::OR, DstReg, DstReg, ATReg, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandUsh(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  if (hasM650232r6() || hasM650264r6()) {
    return Error(IDLoc, "instruction not supported on m650232r6 or m650264r6");
  }

  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &SrcRegOp = Inst.getOperand(1);
  assert(SrcRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetImmOp = Inst.getOperand(2);
  assert(OffsetImmOp.isImm() && "expected immediate operand kind");

  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = DstRegOp.getReg();
  unsigned SrcReg = SrcRegOp.getReg();
  int64_t OffsetValue = OffsetImmOp.getImm();

  warnIfNoMacro(IDLoc);
  unsigned ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  bool IsLargeOffset = !(isInt<16>(OffsetValue + 1) && isInt<16>(OffsetValue));
  if (IsLargeOffset) {
    if (loadImmediate(OffsetValue, ATReg, SrcReg, !ABI.ArePtrs64bit(), true,
                      IDLoc, Out, STI))
      return true;
  }

  int64_t FirstOffset = IsLargeOffset ? 1 : (OffsetValue + 1);
  int64_t SecondOffset = IsLargeOffset ? 0 : OffsetValue;
  if (isLittle())
    std::swap(FirstOffset, SecondOffset);

  if (IsLargeOffset) {
    TOut.emitRRI(M6502::SB, DstReg, ATReg, FirstOffset, IDLoc, STI);
    TOut.emitRRI(M6502::SRL, DstReg, DstReg, 8, IDLoc, STI);
    TOut.emitRRI(M6502::SB, DstReg, ATReg, SecondOffset, IDLoc, STI);
    TOut.emitRRI(M6502::LBu, ATReg, ATReg, 0, IDLoc, STI);
    TOut.emitRRI(M6502::SLL, DstReg, DstReg, 8, IDLoc, STI);
    TOut.emitRRR(M6502::OR, DstReg, DstReg, ATReg, IDLoc, STI);
  } else {
    TOut.emitRRI(M6502::SB, DstReg, SrcReg, FirstOffset, IDLoc, STI);
    TOut.emitRRI(M6502::SRL, ATReg, DstReg, 8, IDLoc, STI);
    TOut.emitRRI(M6502::SB, ATReg, SrcReg, SecondOffset, IDLoc, STI);
  }

  return false;
}

bool M6502AsmParser::expandUxw(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  if (hasM650232r6() || hasM650264r6()) {
    return Error(IDLoc, "instruction not supported on m650232r6 or m650264r6");
  }

  const MCOperand &DstRegOp = Inst.getOperand(0);
  assert(DstRegOp.isReg() && "expected register operand kind");
  const MCOperand &SrcRegOp = Inst.getOperand(1);
  assert(SrcRegOp.isReg() && "expected register operand kind");
  const MCOperand &OffsetImmOp = Inst.getOperand(2);
  assert(OffsetImmOp.isImm() && "expected immediate operand kind");

  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = DstRegOp.getReg();
  unsigned SrcReg = SrcRegOp.getReg();
  int64_t OffsetValue = OffsetImmOp.getImm();

  // Compute left/right load/store offsets.
  bool IsLargeOffset = !(isInt<16>(OffsetValue + 3) && isInt<16>(OffsetValue));
  int64_t LxlOffset = IsLargeOffset ? 0 : OffsetValue;
  int64_t LxrOffset = IsLargeOffset ? 3 : (OffsetValue + 3);
  if (isLittle())
    std::swap(LxlOffset, LxrOffset);

  bool IsLoadInst = (Inst.getOpcode() == M6502::Ulw);
  bool DoMove = IsLoadInst && (SrcReg == DstReg) && !IsLargeOffset;
  unsigned TmpReg = SrcReg;
  if (IsLargeOffset || DoMove) {
    warnIfNoMacro(IDLoc);
    TmpReg = getATReg(IDLoc);
    if (!TmpReg)
      return true;
  }

  if (IsLargeOffset) {
    if (loadImmediate(OffsetValue, TmpReg, SrcReg, !ABI.ArePtrs64bit(), true,
                      IDLoc, Out, STI))
      return true;
  }

  if (DoMove)
    std::swap(DstReg, TmpReg);

  unsigned XWL = IsLoadInst ? M6502::LWL : M6502::SWL;
  unsigned XWR = IsLoadInst ? M6502::LWR : M6502::SWR;
  TOut.emitRRI(XWL, DstReg, TmpReg, LxlOffset, IDLoc, STI);
  TOut.emitRRI(XWR, DstReg, TmpReg, LxrOffset, IDLoc, STI);

  if (DoMove)
    TOut.emitRRR(M6502::OR, TmpReg, DstReg, M6502::ZERO, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandAliasImmediate(MCInst &Inst, SMLoc IDLoc,
                                         MCStreamer &Out,
                                         const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();

  assert(Inst.getNumOperands() == 3 && "Invalid operand count");
  assert(Inst.getOperand(0).isReg() &&
         Inst.getOperand(1).isReg() &&
         Inst.getOperand(2).isImm() && "Invalid instruction operand.");

  unsigned ATReg = M6502::NoRegister;
  unsigned FinalDstReg = M6502::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm();

  bool Is32Bit = isInt<32>(ImmValue) || (!isGP64bit() && isUInt<32>(ImmValue));

  unsigned FinalOpcode = Inst.getOpcode();

  if (DstReg == SrcReg) {
    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;
    FinalDstReg = DstReg;
    DstReg = ATReg;
  }

  if (!loadImmediate(ImmValue, DstReg, M6502::NoRegister, Is32Bit, false, Inst.getLoc(), Out, STI)) {
    switch (FinalOpcode) {
    default:
      llvm_unreachable("unimplemented expansion");
    case M6502::ADDi:
      FinalOpcode = M6502::ADD;
      break;
    case M6502::ADDiu:
      FinalOpcode = M6502::ADDu;
      break;
    case M6502::ANDi:
      FinalOpcode = M6502::AND;
      break;
    case M6502::NORImm:
      FinalOpcode = M6502::NOR;
      break;
    case M6502::ORi:
      FinalOpcode = M6502::OR;
      break;
    case M6502::SLTi:
      FinalOpcode = M6502::SLT;
      break;
    case M6502::SLTiu:
      FinalOpcode = M6502::SLTu;
      break;
    case M6502::XORi:
      FinalOpcode = M6502::XOR;
      break;
    case M6502::ADDi_MM:
      FinalOpcode = M6502::ADD_MM;
      break;
    case M6502::ADDiu_MM:
      FinalOpcode = M6502::ADDu_MM;
      break;
    case M6502::ANDi_MM:
      FinalOpcode = M6502::AND_MM;
      break;
    case M6502::ORi_MM:
      FinalOpcode = M6502::OR_MM;
      break;
    case M6502::SLTi_MM:
      FinalOpcode = M6502::SLT_MM;
      break;
    case M6502::SLTiu_MM:
      FinalOpcode = M6502::SLTu_MM;
      break;
    case M6502::XORi_MM:
      FinalOpcode = M6502::XOR_MM;
      break;
    case M6502::ANDi64:
      FinalOpcode = M6502::AND64;
      break;
    case M6502::NORImm64:
      FinalOpcode = M6502::NOR64;
      break;
    case M6502::ORi64:
      FinalOpcode = M6502::OR64;
      break;
    case M6502::SLTImm64:
      FinalOpcode = M6502::SLT64;
      break;
    case M6502::SLTUImm64:
      FinalOpcode = M6502::SLTu64;
      break;
    case M6502::XORi64:
      FinalOpcode = M6502::XOR64;
      break;
    }

    if (FinalDstReg == M6502::NoRegister)
      TOut.emitRRR(FinalOpcode, DstReg, DstReg, SrcReg, IDLoc, STI);
    else
      TOut.emitRRR(FinalOpcode, FinalDstReg, FinalDstReg, DstReg, IDLoc, STI);
    return false;
  }
  return true;
}

bool M6502AsmParser::expandRotation(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                   const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = M6502::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  unsigned TReg = Inst.getOperand(2).getReg();
  unsigned TmpReg = DReg;

  unsigned FirstShift = M6502::NOP;
  unsigned SecondShift = M6502::NOP;

  if (hasM650232r2()) {
    if (DReg == SReg) {
      TmpReg = getATReg(Inst.getLoc());
      if (!TmpReg)
        return true;
    }

    if (Inst.getOpcode() == M6502::ROL) {
      TOut.emitRRR(M6502::SUBu, TmpReg, M6502::ZERO, TReg, Inst.getLoc(), STI);
      TOut.emitRRR(M6502::ROTRV, DReg, SReg, TmpReg, Inst.getLoc(), STI);
      return false;
    }

    if (Inst.getOpcode() == M6502::ROR) {
      TOut.emitRRR(M6502::ROTRV, DReg, SReg, TReg, Inst.getLoc(), STI);
      return false;
    }

    return true;
  }

  if (hasM650232()) {
    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case M6502::ROL:
      FirstShift = M6502::SRLV;
      SecondShift = M6502::SLLV;
      break;
    case M6502::ROR:
      FirstShift = M6502::SLLV;
      SecondShift = M6502::SRLV;
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRR(M6502::SUBu, ATReg, M6502::ZERO, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(FirstShift, ATReg, SReg, ATReg, Inst.getLoc(), STI);
    TOut.emitRRR(SecondShift, DReg, SReg, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(M6502::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool M6502AsmParser::expandRotationImm(MCInst &Inst, SMLoc IDLoc,
                                      MCStreamer &Out,
                                      const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = M6502::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm();

  unsigned FirstShift = M6502::NOP;
  unsigned SecondShift = M6502::NOP;

  if (hasM650232r2()) {
    if (Inst.getOpcode() == M6502::ROLImm) {
      uint64_t MaxShift = 32;
      uint64_t ShiftValue = ImmValue;
      if (ImmValue != 0)
        ShiftValue = MaxShift - ImmValue;
      TOut.emitRRI(M6502::ROTR, DReg, SReg, ShiftValue, Inst.getLoc(), STI);
      return false;
    }

    if (Inst.getOpcode() == M6502::RORImm) {
      TOut.emitRRI(M6502::ROTR, DReg, SReg, ImmValue, Inst.getLoc(), STI);
      return false;
    }

    return true;
  }

  if (hasM650232()) {
    if (ImmValue == 0) {
      TOut.emitRRI(M6502::SRL, DReg, SReg, 0, Inst.getLoc(), STI);
      return false;
    }

    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case M6502::ROLImm:
      FirstShift = M6502::SLL;
      SecondShift = M6502::SRL;
      break;
    case M6502::RORImm:
      FirstShift = M6502::SRL;
      SecondShift = M6502::SLL;
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRI(FirstShift, ATReg, SReg, ImmValue, Inst.getLoc(), STI);
    TOut.emitRRI(SecondShift, DReg, SReg, 32 - ImmValue, Inst.getLoc(), STI);
    TOut.emitRRR(M6502::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool M6502AsmParser::expandDRotation(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = M6502::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  unsigned TReg = Inst.getOperand(2).getReg();
  unsigned TmpReg = DReg;

  unsigned FirstShift = M6502::NOP;
  unsigned SecondShift = M6502::NOP;

  if (hasM650264r2()) {
    if (TmpReg == SReg) {
      TmpReg = getATReg(Inst.getLoc());
      if (!TmpReg)
        return true;
    }

    if (Inst.getOpcode() == M6502::DROL) {
      TOut.emitRRR(M6502::DSUBu, TmpReg, M6502::ZERO, TReg, Inst.getLoc(), STI);
      TOut.emitRRR(M6502::DROTRV, DReg, SReg, TmpReg, Inst.getLoc(), STI);
      return false;
    }

    if (Inst.getOpcode() == M6502::DROR) {
      TOut.emitRRR(M6502::DROTRV, DReg, SReg, TReg, Inst.getLoc(), STI);
      return false;
    }

    return true;
  }

  if (hasM650264()) {
    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case M6502::DROL:
      FirstShift = M6502::DSRLV;
      SecondShift = M6502::DSLLV;
      break;
    case M6502::DROR:
      FirstShift = M6502::DSLLV;
      SecondShift = M6502::DSRLV;
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRR(M6502::DSUBu, ATReg, M6502::ZERO, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(FirstShift, ATReg, SReg, ATReg, Inst.getLoc(), STI);
    TOut.emitRRR(SecondShift, DReg, SReg, TReg, Inst.getLoc(), STI);
    TOut.emitRRR(M6502::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool M6502AsmParser::expandDRotationImm(MCInst &Inst, SMLoc IDLoc,
                                       MCStreamer &Out,
                                       const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = M6502::NoRegister;
  unsigned DReg = Inst.getOperand(0).getReg();
  unsigned SReg = Inst.getOperand(1).getReg();
  int64_t ImmValue = Inst.getOperand(2).getImm() % 64;

  unsigned FirstShift = M6502::NOP;
  unsigned SecondShift = M6502::NOP;

  MCInst TmpInst;

  if (hasM650264r2()) {
    unsigned FinalOpcode = M6502::NOP;
    if (ImmValue == 0)
      FinalOpcode = M6502::DROTR;
    else if (ImmValue % 32 == 0)
      FinalOpcode = M6502::DROTR32;
    else if ((ImmValue >= 1) && (ImmValue <= 32)) {
      if (Inst.getOpcode() == M6502::DROLImm)
        FinalOpcode = M6502::DROTR32;
      else
        FinalOpcode = M6502::DROTR;
    } else if (ImmValue >= 33) {
      if (Inst.getOpcode() == M6502::DROLImm)
        FinalOpcode = M6502::DROTR;
      else
        FinalOpcode = M6502::DROTR32;
    }

    uint64_t ShiftValue = ImmValue % 32;
    if (Inst.getOpcode() == M6502::DROLImm)
      ShiftValue = (32 - ImmValue % 32) % 32;

    TOut.emitRRI(FinalOpcode, DReg, SReg, ShiftValue, Inst.getLoc(), STI);

    return false;
  }

  if (hasM650264()) {
    if (ImmValue == 0) {
      TOut.emitRRI(M6502::DSRL, DReg, SReg, 0, Inst.getLoc(), STI);
      return false;
    }

    switch (Inst.getOpcode()) {
    default:
      llvm_unreachable("unexpected instruction opcode");
    case M6502::DROLImm:
      if ((ImmValue >= 1) && (ImmValue <= 31)) {
        FirstShift = M6502::DSLL;
        SecondShift = M6502::DSRL32;
      }
      if (ImmValue == 32) {
        FirstShift = M6502::DSLL32;
        SecondShift = M6502::DSRL32;
      }
      if ((ImmValue >= 33) && (ImmValue <= 63)) {
        FirstShift = M6502::DSLL32;
        SecondShift = M6502::DSRL;
      }
      break;
    case M6502::DRORImm:
      if ((ImmValue >= 1) && (ImmValue <= 31)) {
        FirstShift = M6502::DSRL;
        SecondShift = M6502::DSLL32;
      }
      if (ImmValue == 32) {
        FirstShift = M6502::DSRL32;
        SecondShift = M6502::DSLL32;
      }
      if ((ImmValue >= 33) && (ImmValue <= 63)) {
        FirstShift = M6502::DSRL32;
        SecondShift = M6502::DSLL;
      }
      break;
    }

    ATReg = getATReg(Inst.getLoc());
    if (!ATReg)
      return true;

    TOut.emitRRI(FirstShift, ATReg, SReg, ImmValue % 32, Inst.getLoc(), STI);
    TOut.emitRRI(SecondShift, DReg, SReg, (32 - ImmValue % 32) % 32,
                 Inst.getLoc(), STI);
    TOut.emitRRR(M6502::OR, DReg, DReg, ATReg, Inst.getLoc(), STI);

    return false;
  }

  return true;
}

bool M6502AsmParser::expandAbs(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned FirstRegOp = Inst.getOperand(0).getReg();
  unsigned SecondRegOp = Inst.getOperand(1).getReg();

  TOut.emitRI(M6502::BGEZ, SecondRegOp, 8, IDLoc, STI);
  if (FirstRegOp != SecondRegOp)
    TOut.emitRRR(M6502::ADDu, FirstRegOp, SecondRegOp, M6502::ZERO, IDLoc, STI);
  else
    TOut.emitEmptyDelaySlot(false, IDLoc, STI);
  TOut.emitRRR(M6502::SUB, FirstRegOp, M6502::ZERO, SecondRegOp, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandMulImm(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                 const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = M6502::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  int32_t ImmValue = Inst.getOperand(2).getImm();

  ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  loadImmediate(ImmValue, ATReg, M6502::NoRegister, true, false, IDLoc, Out, STI);

  TOut.emitRR(Inst.getOpcode() == M6502::MULImmMacro ? M6502::MULT : M6502::DMULT,
              SrcReg, ATReg, IDLoc, STI);

  TOut.emitR(M6502::MFLO, DstReg, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandMulO(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                               const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = M6502::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned TmpReg = Inst.getOperand(2).getReg();

  ATReg = getATReg(Inst.getLoc());
  if (!ATReg)
    return true;

  TOut.emitRR(Inst.getOpcode() == M6502::MULOMacro ? M6502::MULT : M6502::DMULT,
              SrcReg, TmpReg, IDLoc, STI);

  TOut.emitR(M6502::MFLO, DstReg, IDLoc, STI);

  TOut.emitRRI(Inst.getOpcode() == M6502::MULOMacro ? M6502::SRA : M6502::DSRA32,
               DstReg, DstReg, 0x1F, IDLoc, STI);

  TOut.emitR(M6502::MFHI, ATReg, IDLoc, STI);

  if (useTraps()) {
    TOut.emitRRI(M6502::TNE, DstReg, ATReg, 6, IDLoc, STI);
  } else {
    MCContext & Context = TOut.getStreamer().getContext();
    MCSymbol * BrTarget = Context.createTempSymbol();
    MCOperand LabelOp =
        MCOperand::createExpr(MCSymbolRefExpr::create(BrTarget, Context));

    TOut.emitRRX(M6502::BEQ, DstReg, ATReg, LabelOp, IDLoc, STI);
    if (AssemblerOptions.back()->isReorder())
      TOut.emitNop(IDLoc, STI);
    TOut.emitII(M6502::BREAK, 6, 0, IDLoc, STI);

    TOut.getStreamer().EmitLabel(BrTarget);
  }
  TOut.emitR(M6502::MFLO, DstReg, IDLoc, STI);

  return false;
}

bool M6502AsmParser::expandMulOU(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned ATReg = M6502::NoRegister;
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned TmpReg = Inst.getOperand(2).getReg();

  ATReg = getATReg(IDLoc);
  if (!ATReg)
    return true;

  TOut.emitRR(Inst.getOpcode() == M6502::MULOUMacro ? M6502::MULTu : M6502::DMULTu,
              SrcReg, TmpReg, IDLoc, STI);

  TOut.emitR(M6502::MFHI, ATReg, IDLoc, STI);
  TOut.emitR(M6502::MFLO, DstReg, IDLoc, STI);
  if (useTraps()) {
    TOut.emitRRI(M6502::TNE, ATReg, M6502::ZERO, 6, IDLoc, STI);
  } else {
    MCContext & Context = TOut.getStreamer().getContext();
    MCSymbol * BrTarget = Context.createTempSymbol();
    MCOperand LabelOp =
        MCOperand::createExpr(MCSymbolRefExpr::create(BrTarget, Context));

    TOut.emitRRX(M6502::BEQ, ATReg, M6502::ZERO, LabelOp, IDLoc, STI);
    if (AssemblerOptions.back()->isReorder())
      TOut.emitNop(IDLoc, STI);
    TOut.emitII(M6502::BREAK, 6, 0, IDLoc, STI);

    TOut.getStreamer().EmitLabel(BrTarget);
  }

  return false;
}

bool M6502AsmParser::expandDMULMacro(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                                    const MCSubtargetInfo *STI) {
  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned DstReg = Inst.getOperand(0).getReg();
  unsigned SrcReg = Inst.getOperand(1).getReg();
  unsigned TmpReg = Inst.getOperand(2).getReg();

  TOut.emitRR(M6502::DMULTu, SrcReg, TmpReg, IDLoc, STI);
  TOut.emitR(M6502::MFLO, DstReg, IDLoc, STI);

  return false;
}

// Expand 'ld $<reg> offset($reg2)' to 'lw $<reg>, offset($reg2);
//                                      lw $<reg+1>>, offset+4($reg2)'
// or expand 'sd $<reg> offset($reg2)' to 'sw $<reg>, offset($reg2);
//                                         sw $<reg+1>>, offset+4($reg2)'
// for O32.
bool M6502AsmParser::expandLoadStoreDMacro(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out,
                                          const MCSubtargetInfo *STI,
                                          bool IsLoad) {
  if (!isABI_O32())
    return true;

  warnIfNoMacro(IDLoc);

  M6502TargetStreamer &TOut = getTargetStreamer();
  unsigned Opcode = IsLoad ? M6502::LW : M6502::SW;
  unsigned FirstReg = Inst.getOperand(0).getReg();
  unsigned SecondReg = nextReg(FirstReg);
  unsigned BaseReg = Inst.getOperand(1).getReg();
  if (!SecondReg)
    return true;

  warnIfRegIndexIsAT(FirstReg, IDLoc);

  assert(Inst.getOperand(2).isImm() &&
         "Offset for load macro is not immediate!");

  MCOperand &FirstOffset = Inst.getOperand(2);
  signed NextOffset = FirstOffset.getImm() + 4;
  MCOperand SecondOffset = MCOperand::createImm(NextOffset);

  if (!isInt<16>(FirstOffset.getImm()) || !isInt<16>(NextOffset))
    return true;

  // For loads, clobber the base register with the second load instead of the
  // first if the BaseReg == FirstReg.
  if (FirstReg != BaseReg || !IsLoad) {
    TOut.emitRRX(Opcode, FirstReg, BaseReg, FirstOffset, IDLoc, STI);
    TOut.emitRRX(Opcode, SecondReg, BaseReg, SecondOffset, IDLoc, STI);
  } else {
    TOut.emitRRX(Opcode, SecondReg, BaseReg, SecondOffset, IDLoc, STI);
    TOut.emitRRX(Opcode, FirstReg, BaseReg, FirstOffset, IDLoc, STI);
  }

  return false;
}

bool M6502AsmParser::expandSeq(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                              const MCSubtargetInfo *STI) {

  warnIfNoMacro(IDLoc);
  M6502TargetStreamer &TOut = getTargetStreamer();

  if (Inst.getOperand(1).getReg() != M6502::ZERO &&
      Inst.getOperand(2).getReg() != M6502::ZERO) {
    TOut.emitRRR(M6502::XOR, Inst.getOperand(0).getReg(),
                 Inst.getOperand(1).getReg(), Inst.getOperand(2).getReg(),
                 IDLoc, STI);
    TOut.emitRRI(M6502::SLTiu, Inst.getOperand(0).getReg(),
                 Inst.getOperand(0).getReg(), 1, IDLoc, STI);
    return false;
  }

  unsigned Reg = 0;
  if (Inst.getOperand(1).getReg() == M6502::ZERO) {
    Reg = Inst.getOperand(2).getReg();
  } else {
    Reg = Inst.getOperand(1).getReg();
  }
  TOut.emitRRI(M6502::SLTiu, Inst.getOperand(0).getReg(), Reg, 1, IDLoc, STI);
  return false;
}

bool M6502AsmParser::expandSeqI(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out,
                               const MCSubtargetInfo *STI) {
  warnIfNoMacro(IDLoc);
  M6502TargetStreamer &TOut = getTargetStreamer();

  unsigned Opc;
  int64_t Imm = Inst.getOperand(2).getImm();
  unsigned Reg = Inst.getOperand(1).getReg();

  if (Imm == 0) {
    TOut.emitRRI(M6502::SLTiu, Inst.getOperand(0).getReg(),
                 Inst.getOperand(1).getReg(), 1, IDLoc, STI);
    return false;
  } else {

    if (Reg == M6502::ZERO) {
      Warning(IDLoc, "comparison is always false");
      TOut.emitRRR(isGP64bit() ? M6502::DADDu : M6502::ADDu,
                   Inst.getOperand(0).getReg(), Reg, Reg, IDLoc, STI);
      return false;
    }

    if (Imm > -0x8000 && Imm < 0) {
      Imm = -Imm;
      Opc = isGP64bit() ? M6502::DADDiu : M6502::ADDiu;
    } else {
      Opc = M6502::XORi;
    }
  }
  if (!isUInt<16>(Imm)) {
    unsigned ATReg = getATReg(IDLoc);
    if (!ATReg)
      return true;

    if (loadImmediate(Imm, ATReg, M6502::NoRegister, true, isGP64bit(), IDLoc,
                      Out, STI))
      return true;

    TOut.emitRRR(M6502::XOR, Inst.getOperand(0).getReg(),
                 Inst.getOperand(1).getReg(), ATReg, IDLoc, STI);
    TOut.emitRRI(M6502::SLTiu, Inst.getOperand(0).getReg(),
                 Inst.getOperand(0).getReg(), 1, IDLoc, STI);
    return false;
  }

  TOut.emitRRI(Opc, Inst.getOperand(0).getReg(), Inst.getOperand(1).getReg(),
               Imm, IDLoc, STI);
  TOut.emitRRI(M6502::SLTiu, Inst.getOperand(0).getReg(),
               Inst.getOperand(0).getReg(), 1, IDLoc, STI);
  return false;
}

unsigned
M6502AsmParser::checkEarlyTargetMatchPredicate(MCInst &Inst,
                                              const OperandVector &Operands) {
  switch (Inst.getOpcode()) {
  default:
    return Match_Success;
  case M6502::DATI:
  case M6502::DAHI:
  case M6502::DATI_MM64R6:
  case M6502::DAHI_MM64R6:
    if (static_cast<M6502Operand &>(*Operands[1])
            .isValidForTie(static_cast<M6502Operand &>(*Operands[2])))
      return Match_Success;
    return Match_RequiresSameSrcAndDst;
  }
}

unsigned M6502AsmParser::checkTargetMatchPredicate(MCInst &Inst) {
  switch (Inst.getOpcode()) {
  // As described by the M6502R6 spec, daui must not use the zero operand for
  // its source operand.
  case M6502::DAUI:
  case M6502::DAUI_MM64R6:
    if (Inst.getOperand(1).getReg() == M6502::ZERO ||
        Inst.getOperand(1).getReg() == M6502::ZERO_64)
      return Match_RequiresNoZeroRegister;
    return Match_Success;
  // As described by the M650232r2 spec, the registers Rd and Rs for
  // jalr.hb must be different.
  // It also applies for registers Rt and Rs of microM6502r6 jalrc.hb instruction
  // and registers Rd and Base for microM6502 lwp instruction
  case M6502::JALR_HB:
  case M6502::JALRC_HB_MMR6:
  case M6502::JALRC_MMR6:
    if (Inst.getOperand(0).getReg() == Inst.getOperand(1).getReg())
      return Match_RequiresDifferentSrcAndDst;
    return Match_Success;
  case M6502::LWP_MM:
  case M6502::LWP_MMR6:
    if (Inst.getOperand(0).getReg() == Inst.getOperand(2).getReg())
      return Match_RequiresDifferentSrcAndDst;
    return Match_Success;
  case M6502::SYNC:
    if (Inst.getOperand(0).getImm() != 0 && !hasM650232())
      return Match_NonZeroOperandForSync;
    return Match_Success;
  // As described the M6502R6 spec, the compact branches that compare registers
  // must:
  // a) Not use the zero register.
  // b) Not use the same register twice.
  // c) rs < rt for bnec, beqc.
  //    NB: For this case, the encoding will swap the operands as their
  //    ordering doesn't matter. GAS performs this transformation  too.
  //    Hence, that constraint does not have to be enforced.
  //
  // The compact branches that branch iff the signed addition of two registers
  // would overflow must have rs >= rt. That can be handled like beqc/bnec with
  // operand swapping. They do not have restriction of using the zero register.
  case M6502::BLEZC:   case M6502::BLEZC_MMR6:
  case M6502::BGEZC:   case M6502::BGEZC_MMR6:
  case M6502::BGTZC:   case M6502::BGTZC_MMR6:
  case M6502::BLTZC:   case M6502::BLTZC_MMR6:
  case M6502::BEQZC:   case M6502::BEQZC_MMR6:
  case M6502::BNEZC:   case M6502::BNEZC_MMR6:
  case M6502::BLEZC64:
  case M6502::BGEZC64:
  case M6502::BGTZC64:
  case M6502::BLTZC64:
  case M6502::BEQZC64:
  case M6502::BNEZC64:
    if (Inst.getOperand(0).getReg() == M6502::ZERO ||
        Inst.getOperand(0).getReg() == M6502::ZERO_64)
      return Match_RequiresNoZeroRegister;
    return Match_Success;
  case M6502::BGEC:    case M6502::BGEC_MMR6:
  case M6502::BLTC:    case M6502::BLTC_MMR6:
  case M6502::BGEUC:   case M6502::BGEUC_MMR6:
  case M6502::BLTUC:   case M6502::BLTUC_MMR6:
  case M6502::BEQC:    case M6502::BEQC_MMR6:
  case M6502::BNEC:    case M6502::BNEC_MMR6:
  case M6502::BGEC64:
  case M6502::BLTC64:
  case M6502::BGEUC64:
  case M6502::BLTUC64:
  case M6502::BEQC64:
  case M6502::BNEC64:
    if (Inst.getOperand(0).getReg() == M6502::ZERO ||
        Inst.getOperand(0).getReg() == M6502::ZERO_64)
      return Match_RequiresNoZeroRegister;
    if (Inst.getOperand(1).getReg() == M6502::ZERO ||
        Inst.getOperand(1).getReg() == M6502::ZERO_64)
      return Match_RequiresNoZeroRegister;
    if (Inst.getOperand(0).getReg() == Inst.getOperand(1).getReg())
      return Match_RequiresDifferentOperands;
    return Match_Success;
  case M6502::DINS:
  case M6502::DINS_MM64R6: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for dins!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((0 > (Pos + Size)) || ((Pos + Size) > 32))
      return Match_RequiresPosSizeRange0_32;
    return Match_Success;
  }
  case M6502::DINSM:
  case M6502::DINSM_MM64R6:
  case M6502::DINSU:
  case M6502::DINSU_MM64R6: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for dinsm/dinsu!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((32 >= (Pos + Size)) || ((Pos + Size) > 64))
      return Match_RequiresPosSizeRange33_64;
    return Match_Success;
  }
  case M6502::DEXT:
  case M6502::DEXT_MM64R6: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for DEXTM!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((1 > (Pos + Size)) || ((Pos + Size) > 63))
      return Match_RequiresPosSizeUImm6;
    return Match_Success;
  }
  case M6502::DEXTM:
  case M6502::DEXTU:
  case M6502::DEXTM_MM64R6:
  case M6502::DEXTU_MM64R6: {
    assert(Inst.getOperand(2).isImm() && Inst.getOperand(3).isImm() &&
           "Operands must be immediates for dextm/dextu!");
    const signed Pos = Inst.getOperand(2).getImm();
    const signed Size = Inst.getOperand(3).getImm();
    if ((32 > (Pos + Size)) || ((Pos + Size) > 64))
      return Match_RequiresPosSizeRange33_64;
    return Match_Success;
  }
  }

  uint64_t TSFlags = getInstDesc(Inst.getOpcode()).TSFlags;
  if ((TSFlags & M6502II::HasFCCRegOperand) &&
      (Inst.getOperand(0).getReg() != M6502::FCC0) && !hasEightFccRegisters())
    return Match_NoFCCRegisterForCurrentISA;

  return Match_Success;

}

static SMLoc RefineErrorLoc(const SMLoc Loc, const OperandVector &Operands,
                            uint64_t ErrorInfo) {
  if (ErrorInfo != ~0ULL && ErrorInfo < Operands.size()) {
    SMLoc ErrorLoc = Operands[ErrorInfo]->getStartLoc();
    if (ErrorLoc == SMLoc())
      return Loc;
    return ErrorLoc;
  }
  return Loc;
}

bool M6502AsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                            OperandVector &Operands,
                                            MCStreamer &Out,
                                            uint64_t &ErrorInfo,
                                            bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);

  switch (MatchResult) {
  case Match_Success:
    if (processInstruction(Inst, IDLoc, Out, STI))
      return true;
    return false;
  case Match_MissingFeature:
    Error(IDLoc, "instruction requires a CPU feature not currently enabled");
    return true;
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0ULL) {
      if (ErrorInfo >= Operands.size())
        return Error(IDLoc, "too few operands for instruction");

      ErrorLoc = Operands[ErrorInfo]->getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }

    return Error(ErrorLoc, "invalid operand for instruction");
  }
  case Match_NonZeroOperandForSync:
    return Error(IDLoc, "s-type must be zero or unspecified for pre-M650232 ISAs");
  case Match_MnemonicFail:
    return Error(IDLoc, "invalid instruction");
  case Match_RequiresDifferentSrcAndDst:
    return Error(IDLoc, "source and destination must be different");
  case Match_RequiresDifferentOperands:
    return Error(IDLoc, "registers must be different");
  case Match_RequiresNoZeroRegister:
    return Error(IDLoc, "invalid operand ($zero) for instruction");
  case Match_RequiresSameSrcAndDst:
    return Error(IDLoc, "source and destination must match");
  case Match_NoFCCRegisterForCurrentISA:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "non-zero fcc register doesn't exist in current ISA level");
  case Match_Immz:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo), "expected '0'");
  case Match_UImm1_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 1-bit unsigned immediate");
  case Match_UImm2_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 2-bit unsigned immediate");
  case Match_UImm2_1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 1 .. 4");
  case Match_UImm3_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 3-bit unsigned immediate");
  case Match_UImm4_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 4-bit unsigned immediate");
  case Match_SImm4_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 4-bit signed immediate");
  case Match_UImm5_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 5-bit unsigned immediate");
  case Match_SImm5_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 5-bit signed immediate");
  case Match_UImm5_1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 1 .. 32");
  case Match_UImm5_32:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 32 .. 63");
  case Match_UImm5_33:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 33 .. 64");
  case Match_UImm5_0_Report_UImm6:
    // This is used on UImm5 operands that have a corresponding UImm5_32
    // operand to avoid confusing the user.
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 6-bit unsigned immediate");
  case Match_UImm5_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 7-bit unsigned immediate and multiple of 4");
  case Match_UImmRange2_64:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range 2 .. 64");
  case Match_UImm6_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 6-bit unsigned immediate");
  case Match_UImm6_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 8-bit unsigned immediate and multiple of 4");
  case Match_SImm6_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 6-bit signed immediate");
  case Match_UImm7_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 7-bit unsigned immediate");
  case Match_UImm7_N1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected immediate in range -1 .. 126");
  case Match_SImm7_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 9-bit signed immediate and multiple of 4");
  case Match_UImm8_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 8-bit unsigned immediate");
  case Match_UImm10_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 10-bit unsigned immediate");
  case Match_SImm10_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 10-bit signed immediate");
  case Match_SImm11_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 11-bit signed immediate");
  case Match_UImm16:
  case Match_UImm16_Relaxed:
  case Match_UImm16_AltRelaxed:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 16-bit unsigned immediate");
  case Match_SImm16:
  case Match_SImm16_Relaxed:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 16-bit signed immediate");
  case Match_SImm19_Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected both 19-bit signed immediate and multiple of 4");
  case Match_UImm20_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 20-bit unsigned immediate");
  case Match_UImm26_0:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 26-bit unsigned immediate");
  case Match_SImm32:
  case Match_SImm32_Relaxed:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 32-bit signed immediate");
  case Match_UImm32_Coerced:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected 32-bit immediate");
  case Match_MemSImm9:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 9-bit signed offset");
  case Match_MemSImm10:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 10-bit signed offset");
  case Match_MemSImm10Lsl1:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 11-bit signed offset and multiple of 2");
  case Match_MemSImm10Lsl2:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 12-bit signed offset and multiple of 4");
  case Match_MemSImm10Lsl3:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 13-bit signed offset and multiple of 8");
  case Match_MemSImm11:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 11-bit signed offset");
  case Match_MemSImm12:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 12-bit signed offset");
  case Match_MemSImm16:
    return Error(RefineErrorLoc(IDLoc, Operands, ErrorInfo),
                 "expected memory with 16-bit signed offset");
  case Match_RequiresPosSizeRange0_32: {
    SMLoc ErrorStart = Operands[3]->getStartLoc();
    SMLoc ErrorEnd = Operands[4]->getEndLoc();
    return Error(ErrorStart, "size plus position are not in the range 0 .. 32",
                 SMRange(ErrorStart, ErrorEnd));
    }
  case Match_RequiresPosSizeUImm6: {
    SMLoc ErrorStart = Operands[3]->getStartLoc();
    SMLoc ErrorEnd = Operands[4]->getEndLoc();
    return Error(ErrorStart, "size plus position are not in the range 1 .. 63",
                 SMRange(ErrorStart, ErrorEnd));
    }
  case Match_RequiresPosSizeRange33_64: {
    SMLoc ErrorStart = Operands[3]->getStartLoc();
    SMLoc ErrorEnd = Operands[4]->getEndLoc();
    return Error(ErrorStart, "size plus position are not in the range 33 .. 64",
                 SMRange(ErrorStart, ErrorEnd));
    }
  }

  llvm_unreachable("Implement any new match types added!");
}

void M6502AsmParser::warnIfRegIndexIsAT(unsigned RegIndex, SMLoc Loc) {
  if (RegIndex != 0 && AssemblerOptions.back()->getATRegIndex() == RegIndex)
    Warning(Loc, "used $at (currently $" + Twine(RegIndex) +
                     ") without \".set noat\"");
}

void M6502AsmParser::warnIfNoMacro(SMLoc Loc) {
  if (!AssemblerOptions.back()->isMacro())
    Warning(Loc, "macro instruction expanded into multiple instructions");
}

void
M6502AsmParser::printWarningWithFixIt(const Twine &Msg, const Twine &FixMsg,
                                     SMRange Range, bool ShowColors) {
  getSourceManager().PrintMessage(Range.Start, SourceMgr::DK_Warning, Msg,
                                  Range, SMFixIt(Range, FixMsg),
                                  ShowColors);
}

int M6502AsmParser::matchCPURegisterName(StringRef Name) {
  int CC;

  CC = StringSwitch<unsigned>(Name)
           .Case("zero", 0)
           .Cases("at", "AT", 1)
           .Case("a0", 4)
           .Case("a1", 5)
           .Case("a2", 6)
           .Case("a3", 7)
           .Case("v0", 2)
           .Case("v1", 3)
           .Case("s0", 16)
           .Case("s1", 17)
           .Case("s2", 18)
           .Case("s3", 19)
           .Case("s4", 20)
           .Case("s5", 21)
           .Case("s6", 22)
           .Case("s7", 23)
           .Case("k0", 26)
           .Case("k1", 27)
           .Case("gp", 28)
           .Case("sp", 29)
           .Case("fp", 30)
           .Case("s8", 30)
           .Case("ra", 31)
           .Case("t0", 8)
           .Case("t1", 9)
           .Case("t2", 10)
           .Case("t3", 11)
           .Case("t4", 12)
           .Case("t5", 13)
           .Case("t6", 14)
           .Case("t7", 15)
           .Case("t8", 24)
           .Case("t9", 25)
           .Default(-1);

  if (!(isABI_N32() || isABI_N64()))
    return CC;

  if (12 <= CC && CC <= 15) {
    // Name is one of t4-t7
    AsmToken RegTok = getLexer().peekTok();
    SMRange RegRange = RegTok.getLocRange();

    StringRef FixedName = StringSwitch<StringRef>(Name)
                              .Case("t4", "t0")
                              .Case("t5", "t1")
                              .Case("t6", "t2")
                              .Case("t7", "t3")
                              .Default("");
    assert(FixedName != "" &&  "Register name is not one of t4-t7.");

    printWarningWithFixIt("register names $t4-$t7 are only available in O32.",
                          "Did you mean $" + FixedName + "?", RegRange);
  }

  // Although SGI documentation just cuts out t0-t3 for n32/n64,
  // GNU pushes the values of t0-t3 to override the o32/o64 values for t4-t7
  // We are supporting both cases, so for t0-t3 we'll just push them to t4-t7.
  if (8 <= CC && CC <= 11)
    CC += 4;

  if (CC == -1)
    CC = StringSwitch<unsigned>(Name)
             .Case("a4", 8)
             .Case("a5", 9)
             .Case("a6", 10)
             .Case("a7", 11)
             .Case("kt0", 26)
             .Case("kt1", 27)
             .Default(-1);

  return CC;
}

int M6502AsmParser::matchHWRegsRegisterName(StringRef Name) {
  int CC;

  CC = StringSwitch<unsigned>(Name)
            .Case("hwr_cpunum", 0)
            .Case("hwr_synci_step", 1)
            .Case("hwr_cc", 2)
            .Case("hwr_ccres", 3)
            .Case("hwr_ulr", 29)
            .Default(-1);

  return CC;
}

int M6502AsmParser::matchFPURegisterName(StringRef Name) {
  if (Name[0] == 'f') {
    StringRef NumString = Name.substr(1);
    unsigned IntVal;
    if (NumString.getAsInteger(10, IntVal))
      return -1;     // This is not an integer.
    if (IntVal > 31) // Maximum index for fpu register.
      return -1;
    return IntVal;
  }
  return -1;
}

int M6502AsmParser::matchFCCRegisterName(StringRef Name) {
  if (Name.startswith("fcc")) {
    StringRef NumString = Name.substr(3);
    unsigned IntVal;
    if (NumString.getAsInteger(10, IntVal))
      return -1;    // This is not an integer.
    if (IntVal > 7) // There are only 8 fcc registers.
      return -1;
    return IntVal;
  }
  return -1;
}

int M6502AsmParser::matchACRegisterName(StringRef Name) {
  if (Name.startswith("ac")) {
    StringRef NumString = Name.substr(2);
    unsigned IntVal;
    if (NumString.getAsInteger(10, IntVal))
      return -1;    // This is not an integer.
    if (IntVal > 3) // There are only 3 acc registers.
      return -1;
    return IntVal;
  }
  return -1;
}

int M6502AsmParser::matchMSA128RegisterName(StringRef Name) {
  unsigned IntVal;

  if (Name.front() != 'w' || Name.drop_front(1).getAsInteger(10, IntVal))
    return -1;

  if (IntVal > 31)
    return -1;

  return IntVal;
}

int M6502AsmParser::matchMSA128CtrlRegisterName(StringRef Name) {
  int CC;

  CC = StringSwitch<unsigned>(Name)
           .Case("msair", 0)
           .Case("msacsr", 1)
           .Case("msaaccess", 2)
           .Case("msasave", 3)
           .Case("msamodify", 4)
           .Case("msarequest", 5)
           .Case("msamap", 6)
           .Case("msaunmap", 7)
           .Default(-1);

  return CC;
}

bool M6502AsmParser::canUseATReg() {
  return AssemblerOptions.back()->getATRegIndex() != 0;
}

unsigned M6502AsmParser::getATReg(SMLoc Loc) {
  unsigned ATIndex = AssemblerOptions.back()->getATRegIndex();
  if (ATIndex == 0) {
    reportParseError(Loc,
                     "pseudo-instruction requires $at, which is not available");
    return 0;
  }
  unsigned AT = getReg(
      (isGP64bit()) ? M6502::GPR64RegClassID : M6502::GPR32RegClassID, ATIndex);
  return AT;
}

unsigned M6502AsmParser::getReg(int RC, int RegNo) {
  return *(getContext().getRegisterInfo()->getRegClass(RC).begin() + RegNo);
}

bool M6502AsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {
  MCAsmParser &Parser = getParser();
  DEBUG(dbgs() << "parseOperand\n");

  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);
  if (ResTy == MatchOperand_Success)
    return false;
  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.
  if (ResTy == MatchOperand_ParseFail)
    return true;

  DEBUG(dbgs() << ".. Generic Parser\n");

  switch (getLexer().getKind()) {
  case AsmToken::Dollar: {
    // Parse the register.
    SMLoc S = Parser.getTok().getLoc();

    // Almost all registers have been parsed by custom parsers. There is only
    // one exception to this. $zero (and it's alias $0) will reach this point
    // for div, divu, and similar instructions because it is not an operand
    // to the instruction definition but an explicit register. Special case
    // this situation for now.
    if (parseAnyRegister(Operands) != MatchOperand_NoMatch)
      return false;

    // Maybe it is a symbol reference.
    StringRef Identifier;
    if (Parser.parseIdentifier(Identifier))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    MCSymbol *Sym = getContext().getOrCreateSymbol("$" + Identifier);
    // Otherwise create a symbol reference.
    const MCExpr *Res =
        MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());

    Operands.push_back(M6502Operand::CreateImm(Res, S, E, *this));
    return false;
  }
  default: {
    DEBUG(dbgs() << ".. generic integer expression\n");

    const MCExpr *Expr;
    SMLoc S = Parser.getTok().getLoc(); // Start location of the operand.
    if (getParser().parseExpression(Expr))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

    Operands.push_back(M6502Operand::CreateImm(Expr, S, E, *this));
    return false;
  }
  } // switch(getLexer().getKind())
  return true;
}

bool M6502AsmParser::isEvaluated(const MCExpr *Expr) {
  switch (Expr->getKind()) {
  case MCExpr::Constant:
    return true;
  case MCExpr::SymbolRef:
    return (cast<MCSymbolRefExpr>(Expr)->getKind() != MCSymbolRefExpr::VK_None);
  case MCExpr::Binary: {
    const MCBinaryExpr *BE = cast<MCBinaryExpr>(Expr);
    if (!isEvaluated(BE->getLHS()))
      return false;
    return isEvaluated(BE->getRHS());
  }
  case MCExpr::Unary:
    return isEvaluated(cast<MCUnaryExpr>(Expr)->getSubExpr());
  case MCExpr::Target:
    return true;
  }
  return false;
}

bool M6502AsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                  SMLoc &EndLoc) {
  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> Operands;
  OperandMatchResultTy ResTy = parseAnyRegister(Operands);
  if (ResTy == MatchOperand_Success) {
    assert(Operands.size() == 1);
    M6502Operand &Operand = static_cast<M6502Operand &>(*Operands.front());
    StartLoc = Operand.getStartLoc();
    EndLoc = Operand.getEndLoc();

    // AFAIK, we only support numeric registers and named GPR's in CFI
    // directives.
    // Don't worry about eating tokens before failing. Using an unrecognised
    // register is a parse error.
    if (Operand.isGPRAsmReg()) {
      // Resolve to GPR32 or GPR64 appropriately.
      RegNo = isGP64bit() ? Operand.getGPR64Reg() : Operand.getGPR32Reg();
    }

    return (RegNo == (unsigned)-1);
  }

  assert(Operands.size() == 0);
  return (RegNo == (unsigned)-1);
}

bool M6502AsmParser::parseMemOffset(const MCExpr *&Res, bool isParenExpr) {
  SMLoc S;

  if (isParenExpr)
    return getParser().parseParenExprOfDepth(0, Res, S);
  return getParser().parseExpression(Res);
}

OperandMatchResultTy
M6502AsmParser::parseMemOperand(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  DEBUG(dbgs() << "parseMemOperand\n");
  const MCExpr *IdVal = nullptr;
  SMLoc S;
  bool isParenExpr = false;
  OperandMatchResultTy Res = MatchOperand_NoMatch;
  // First operand is the offset.
  S = Parser.getTok().getLoc();

  if (getLexer().getKind() == AsmToken::LParen) {
    Parser.Lex();
    isParenExpr = true;
  }

  if (getLexer().getKind() != AsmToken::Dollar) {
    if (parseMemOffset(IdVal, isParenExpr))
      return MatchOperand_ParseFail;

    const AsmToken &Tok = Parser.getTok(); // Get the next token.
    if (Tok.isNot(AsmToken::LParen)) {
      M6502Operand &Mnemonic = static_cast<M6502Operand &>(*Operands[0]);
      if (Mnemonic.getToken() == "la" || Mnemonic.getToken() == "dla") {
        SMLoc E =
            SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
        Operands.push_back(M6502Operand::CreateImm(IdVal, S, E, *this));
        return MatchOperand_Success;
      }
      if (Tok.is(AsmToken::EndOfStatement)) {
        SMLoc E =
            SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

        // Zero register assumed, add a memory operand with ZERO as its base.
        // "Base" will be managed by k_Memory.
        auto Base = M6502Operand::createGPRReg(
            0, "0", getContext().getRegisterInfo(), S, E, *this);
        Operands.push_back(
            M6502Operand::CreateMem(std::move(Base), IdVal, S, E, *this));
        return MatchOperand_Success;
      }
      MCBinaryExpr::Opcode Opcode;
      // GAS and LLVM treat comparison operators different. GAS will generate -1
      // or 0, while LLVM will generate 0 or 1. Since a comparsion operator is
      // highly unlikely to be found in a memory offset expression, we don't
      // handle them.
      switch (Tok.getKind()) {
      case AsmToken::Plus:
        Opcode = MCBinaryExpr::Add;
        Parser.Lex();
        break;
      case AsmToken::Minus:
        Opcode = MCBinaryExpr::Sub;
        Parser.Lex();
        break;
      case AsmToken::Star:
        Opcode = MCBinaryExpr::Mul;
        Parser.Lex();
        break;
      case AsmToken::Pipe:
        Opcode = MCBinaryExpr::Or;
        Parser.Lex();
        break;
      case AsmToken::Amp:
        Opcode = MCBinaryExpr::And;
        Parser.Lex();
        break;
      case AsmToken::LessLess:
        Opcode = MCBinaryExpr::Shl;
        Parser.Lex();
        break;
      case AsmToken::GreaterGreater:
        Opcode = MCBinaryExpr::LShr;
        Parser.Lex();
        break;
      case AsmToken::Caret:
        Opcode = MCBinaryExpr::Xor;
        Parser.Lex();
        break;
      case AsmToken::Slash:
        Opcode = MCBinaryExpr::Div;
        Parser.Lex();
        break;
      case AsmToken::Percent:
        Opcode = MCBinaryExpr::Mod;
        Parser.Lex();
        break;
      default:
        Error(Parser.getTok().getLoc(), "'(' or expression expected");
        return MatchOperand_ParseFail;
      }
      const MCExpr * NextExpr;
      if (getParser().parseExpression(NextExpr))
        return MatchOperand_ParseFail;
      IdVal = MCBinaryExpr::create(Opcode, IdVal, NextExpr, getContext());
    }

    Parser.Lex(); // Eat the '(' token.
  }

  Res = parseAnyRegister(Operands);
  if (Res != MatchOperand_Success)
    return Res;

  if (Parser.getTok().isNot(AsmToken::RParen)) {
    Error(Parser.getTok().getLoc(), "')' expected");
    return MatchOperand_ParseFail;
  }

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

  Parser.Lex(); // Eat the ')' token.

  if (!IdVal)
    IdVal = MCConstantExpr::create(0, getContext());

  // Replace the register operand with the memory operand.
  std::unique_ptr<M6502Operand> op(
      static_cast<M6502Operand *>(Operands.back().release()));
  // Remove the register from the operands.
  // "op" will be managed by k_Memory.
  Operands.pop_back();
  // Add the memory operand.
  if (const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(IdVal)) {
    int64_t Imm;
    if (IdVal->evaluateAsAbsolute(Imm))
      IdVal = MCConstantExpr::create(Imm, getContext());
    else if (BE->getLHS()->getKind() != MCExpr::SymbolRef)
      IdVal = MCBinaryExpr::create(BE->getOpcode(), BE->getRHS(), BE->getLHS(),
                                   getContext());
  }

  Operands.push_back(M6502Operand::CreateMem(std::move(op), IdVal, S, E, *this));
  return MatchOperand_Success;
}

bool M6502AsmParser::searchSymbolAlias(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  MCSymbol *Sym = getContext().lookupSymbol(Parser.getTok().getIdentifier());
  if (Sym) {
    SMLoc S = Parser.getTok().getLoc();
    const MCExpr *Expr;
    if (Sym->isVariable())
      Expr = Sym->getVariableValue();
    else
      return false;
    if (Expr->getKind() == MCExpr::SymbolRef) {
      const MCSymbolRefExpr *Ref = static_cast<const MCSymbolRefExpr *>(Expr);
      StringRef DefSymbol = Ref->getSymbol().getName();
      if (DefSymbol.startswith("$")) {
        OperandMatchResultTy ResTy =
            matchAnyRegisterNameWithoutDollar(Operands, DefSymbol.substr(1), S);
        if (ResTy == MatchOperand_Success) {
          Parser.Lex();
          return true;
        } else if (ResTy == MatchOperand_ParseFail)
          llvm_unreachable("Should never ParseFail");
        return false;
      }
    }
  }
  return false;
}

OperandMatchResultTy
M6502AsmParser::matchAnyRegisterNameWithoutDollar(OperandVector &Operands,
                                                 StringRef Identifier,
                                                 SMLoc S) {
  int Index = matchCPURegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(M6502Operand::createGPRReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchHWRegsRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(M6502Operand::createHWRegsReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchFPURegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(M6502Operand::createFGRReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchFCCRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(M6502Operand::createFCCReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchACRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(M6502Operand::createACCReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchMSA128RegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(M6502Operand::createMSA128Reg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  Index = matchMSA128CtrlRegisterName(Identifier);
  if (Index != -1) {
    Operands.push_back(M6502Operand::createMSACtrlReg(
        Index, Identifier, getContext().getRegisterInfo(), S,
        getLexer().getLoc(), *this));
    return MatchOperand_Success;
  }

  return MatchOperand_NoMatch;
}

OperandMatchResultTy
M6502AsmParser::matchAnyRegisterWithoutDollar(OperandVector &Operands, SMLoc S) {
  MCAsmParser &Parser = getParser();
  auto Token = Parser.getLexer().peekTok(false);

  if (Token.is(AsmToken::Identifier)) {
    DEBUG(dbgs() << ".. identifier\n");
    StringRef Identifier = Token.getIdentifier();
    OperandMatchResultTy ResTy =
        matchAnyRegisterNameWithoutDollar(Operands, Identifier, S);
    return ResTy;
  } else if (Token.is(AsmToken::Integer)) {
    DEBUG(dbgs() << ".. integer\n");
    Operands.push_back(M6502Operand::createNumericReg(
        Token.getIntVal(), Token.getString(), getContext().getRegisterInfo(), S,
        Token.getLoc(), *this));
    return MatchOperand_Success;
  }

  DEBUG(dbgs() << Parser.getTok().getKind() << "\n");

  return MatchOperand_NoMatch;
}

OperandMatchResultTy
M6502AsmParser::parseAnyRegister(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  DEBUG(dbgs() << "parseAnyRegister\n");

  auto Token = Parser.getTok();

  SMLoc S = Token.getLoc();

  if (Token.isNot(AsmToken::Dollar)) {
    DEBUG(dbgs() << ".. !$ -> try sym aliasing\n");
    if (Token.is(AsmToken::Identifier)) {
      if (searchSymbolAlias(Operands))
        return MatchOperand_Success;
    }
    DEBUG(dbgs() << ".. !symalias -> NoMatch\n");
    return MatchOperand_NoMatch;
  }
  DEBUG(dbgs() << ".. $\n");

  OperandMatchResultTy ResTy = matchAnyRegisterWithoutDollar(Operands, S);
  if (ResTy == MatchOperand_Success) {
    Parser.Lex(); // $
    Parser.Lex(); // identifier
  }
  return ResTy;
}

OperandMatchResultTy
M6502AsmParser::parseJumpTarget(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  DEBUG(dbgs() << "parseJumpTarget\n");

  SMLoc S = getLexer().getLoc();

  // Registers are a valid target and have priority over symbols.
  OperandMatchResultTy ResTy = parseAnyRegister(Operands);
  if (ResTy != MatchOperand_NoMatch)
    return ResTy;

  // Integers and expressions are acceptable
  const MCExpr *Expr = nullptr;
  if (Parser.parseExpression(Expr)) {
    // We have no way of knowing if a symbol was consumed so we must ParseFail
    return MatchOperand_ParseFail;
  }
  Operands.push_back(
      M6502Operand::CreateImm(Expr, S, getLexer().getLoc(), *this));
  return MatchOperand_Success;
}

OperandMatchResultTy
M6502AsmParser::parseInvNum(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  const MCExpr *IdVal;
  // If the first token is '$' we may have register operand. We have to reject
  // cases where it is not a register. Complicating the matter is that
  // register names are not reserved across all ABIs.
  // Peek past the dollar to see if it's a register name for this ABI.
  SMLoc S = Parser.getTok().getLoc();
  if (Parser.getTok().is(AsmToken::Dollar)) {
    return matchCPURegisterName(Parser.getLexer().peekTok().getString()) == -1
               ? MatchOperand_ParseFail
               : MatchOperand_NoMatch;
  }
  if (getParser().parseExpression(IdVal))
    return MatchOperand_ParseFail;
  const MCConstantExpr *MCE = dyn_cast<MCConstantExpr>(IdVal);
  if (!MCE)
    return MatchOperand_NoMatch;
  int64_t Val = MCE->getValue();
  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(M6502Operand::CreateImm(
      MCConstantExpr::create(0 - Val, getContext()), S, E, *this));
  return MatchOperand_Success;
}

OperandMatchResultTy
M6502AsmParser::parseRegisterList(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  SmallVector<unsigned, 10> Regs;
  unsigned RegNo;
  unsigned PrevReg = M6502::NoRegister;
  bool RegRange = false;
  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 8> TmpOperands;

  if (Parser.getTok().isNot(AsmToken::Dollar))
    return MatchOperand_ParseFail;

  SMLoc S = Parser.getTok().getLoc();
  while (parseAnyRegister(TmpOperands) == MatchOperand_Success) {
    SMLoc E = getLexer().getLoc();
    M6502Operand &Reg = static_cast<M6502Operand &>(*TmpOperands.back());
    RegNo = isGP64bit() ? Reg.getGPR64Reg() : Reg.getGPR32Reg();
    if (RegRange) {
      // Remove last register operand because registers from register range
      // should be inserted first.
      if ((isGP64bit() && RegNo == M6502::RA_64) ||
          (!isGP64bit() && RegNo == M6502::RA)) {
        Regs.push_back(RegNo);
      } else {
        unsigned TmpReg = PrevReg + 1;
        while (TmpReg <= RegNo) {
          if ((((TmpReg < M6502::S0) || (TmpReg > M6502::S7)) && !isGP64bit()) ||
              (((TmpReg < M6502::S0_64) || (TmpReg > M6502::S7_64)) &&
               isGP64bit())) {
            Error(E, "invalid register operand");
            return MatchOperand_ParseFail;
          }

          PrevReg = TmpReg;
          Regs.push_back(TmpReg++);
        }
      }

      RegRange = false;
    } else {
      if ((PrevReg == M6502::NoRegister) &&
          ((isGP64bit() && (RegNo != M6502::S0_64) && (RegNo != M6502::RA_64)) ||
          (!isGP64bit() && (RegNo != M6502::S0) && (RegNo != M6502::RA)))) {
        Error(E, "$16 or $31 expected");
        return MatchOperand_ParseFail;
      } else if (!(((RegNo == M6502::FP || RegNo == M6502::RA ||
                    (RegNo >= M6502::S0 && RegNo <= M6502::S7)) &&
                    !isGP64bit()) ||
                   ((RegNo == M6502::FP_64 || RegNo == M6502::RA_64 ||
                    (RegNo >= M6502::S0_64 && RegNo <= M6502::S7_64)) &&
                    isGP64bit()))) {
        Error(E, "invalid register operand");
        return MatchOperand_ParseFail;
      } else if ((PrevReg != M6502::NoRegister) && (RegNo != PrevReg + 1) &&
                 ((RegNo != M6502::FP && RegNo != M6502::RA && !isGP64bit()) ||
                  (RegNo != M6502::FP_64 && RegNo != M6502::RA_64 &&
                   isGP64bit()))) {
        Error(E, "consecutive register numbers expected");
        return MatchOperand_ParseFail;
      }

      Regs.push_back(RegNo);
    }

    if (Parser.getTok().is(AsmToken::Minus))
      RegRange = true;

    if (!Parser.getTok().isNot(AsmToken::Minus) &&
        !Parser.getTok().isNot(AsmToken::Comma)) {
      Error(E, "',' or '-' expected");
      return MatchOperand_ParseFail;
    }

    Lex(); // Consume comma or minus
    if (Parser.getTok().isNot(AsmToken::Dollar))
      break;

    PrevReg = RegNo;
  }

  SMLoc E = Parser.getTok().getLoc();
  Operands.push_back(M6502Operand::CreateRegList(Regs, S, E, *this));
  parseMemOperand(Operands);
  return MatchOperand_Success;
}

OperandMatchResultTy
M6502AsmParser::parseRegisterPair(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();

  SMLoc S = Parser.getTok().getLoc();
  if (parseAnyRegister(Operands) != MatchOperand_Success)
    return MatchOperand_ParseFail;

  SMLoc E = Parser.getTok().getLoc();
  M6502Operand Op = static_cast<M6502Operand &>(*Operands.back());

  Operands.pop_back();
  Operands.push_back(M6502Operand::CreateRegPair(Op, S, E, *this));
  return MatchOperand_Success;
}

OperandMatchResultTy
M6502AsmParser::parseMovePRegPair(OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 8> TmpOperands;
  SmallVector<unsigned, 10> Regs;

  if (Parser.getTok().isNot(AsmToken::Dollar))
    return MatchOperand_ParseFail;

  SMLoc S = Parser.getTok().getLoc();

  if (parseAnyRegister(TmpOperands) != MatchOperand_Success)
    return MatchOperand_ParseFail;

  M6502Operand *Reg = &static_cast<M6502Operand &>(*TmpOperands.back());
  unsigned RegNo = isGP64bit() ? Reg->getGPR64Reg() : Reg->getGPR32Reg();
  Regs.push_back(RegNo);

  SMLoc E = Parser.getTok().getLoc();
  if (Parser.getTok().isNot(AsmToken::Comma)) {
    Error(E, "',' expected");
    return MatchOperand_ParseFail;
  }

  // Remove comma.
  Parser.Lex();

  if (parseAnyRegister(TmpOperands) != MatchOperand_Success)
    return MatchOperand_ParseFail;

  Reg = &static_cast<M6502Operand &>(*TmpOperands.back());
  RegNo = isGP64bit() ? Reg->getGPR64Reg() : Reg->getGPR32Reg();
  Regs.push_back(RegNo);

  Operands.push_back(M6502Operand::CreateRegList(Regs, S, E, *this));

  return MatchOperand_Success;
}

/// Sometimes (i.e. load/stores) the operand may be followed immediately by
/// either this.
/// ::= '(', register, ')'
/// handle it before we iterate so we don't get tripped up by the lack of
/// a comma.
bool M6502AsmParser::parseParenSuffix(StringRef Name, OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  if (getLexer().is(AsmToken::LParen)) {
    Operands.push_back(
        M6502Operand::CreateToken("(", getLexer().getLoc(), *this));
    Parser.Lex();
    if (parseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token in argument list");
    }
    if (Parser.getTok().isNot(AsmToken::RParen)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token, expected ')'");
    }
    Operands.push_back(
        M6502Operand::CreateToken(")", getLexer().getLoc(), *this));
    Parser.Lex();
  }
  return false;
}

/// Sometimes (i.e. in MSA) the operand may be followed immediately by
/// either one of these.
/// ::= '[', register, ']'
/// ::= '[', integer, ']'
/// handle it before we iterate so we don't get tripped up by the lack of
/// a comma.
bool M6502AsmParser::parseBracketSuffix(StringRef Name,
                                       OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  if (getLexer().is(AsmToken::LBrac)) {
    Operands.push_back(
        M6502Operand::CreateToken("[", getLexer().getLoc(), *this));
    Parser.Lex();
    if (parseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token in argument list");
    }
    if (Parser.getTok().isNot(AsmToken::RBrac)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token, expected ']'");
    }
    Operands.push_back(
        M6502Operand::CreateToken("]", getLexer().getLoc(), *this));
    Parser.Lex();
  }
  return false;
}

bool M6502AsmParser::ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                                     SMLoc NameLoc, OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  DEBUG(dbgs() << "ParseInstruction\n");

  // We have reached first instruction, module directive are now forbidden.
  getTargetStreamer().forbidModuleDirective();

  // Check if we have valid mnemonic
  if (!mnemonicIsValid(Name, 0)) {
    return Error(NameLoc, "unknown instruction");
  }
  // First operand in MCInst is instruction mnemonic.
  Operands.push_back(M6502Operand::CreateToken(Name, NameLoc, *this));

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (parseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token in argument list");
    }
    if (getLexer().is(AsmToken::LBrac) && parseBracketSuffix(Name, Operands))
      return true;
    // AFAIK, parenthesis suffixes are never on the first operand

    while (getLexer().is(AsmToken::Comma)) {
      Parser.Lex(); // Eat the comma.
      // Parse and remember the operand.
      if (parseOperand(Operands, Name)) {
        SMLoc Loc = getLexer().getLoc();
        return Error(Loc, "unexpected token in argument list");
      }
      // Parse bracket and parenthesis suffixes before we iterate
      if (getLexer().is(AsmToken::LBrac)) {
        if (parseBracketSuffix(Name, Operands))
          return true;
      } else if (getLexer().is(AsmToken::LParen) &&
                 parseParenSuffix(Name, Operands))
        return true;
    }
  }
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    return Error(Loc, "unexpected token in argument list");
  }
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

// FIXME: Given that these have the same name, these should both be
// consistent on affecting the Parser.
bool M6502AsmParser::reportParseError(Twine ErrorMsg) {
  SMLoc Loc = getLexer().getLoc();
  return Error(Loc, ErrorMsg);
}

bool M6502AsmParser::reportParseError(SMLoc Loc, Twine ErrorMsg) {
  return Error(Loc, ErrorMsg);
}

bool M6502AsmParser::parseSetNoAtDirective() {
  MCAsmParser &Parser = getParser();
  // Line should look like: ".set noat".

  // Set the $at register to $0.
  AssemblerOptions.back()->setATRegIndex(0);

  Parser.Lex(); // Eat "noat".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().emitDirectiveSetNoAt();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetAtDirective() {
  // Line can be: ".set at", which sets $at to $1
  //          or  ".set at=$reg", which sets $at to $reg.
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "at".

  if (getLexer().is(AsmToken::EndOfStatement)) {
    // No register was specified, so we set $at to $1.
    AssemblerOptions.back()->setATRegIndex(1);

    getTargetStreamer().emitDirectiveSetAt();
    Parser.Lex(); // Consume the EndOfStatement.
    return false;
  }

  if (getLexer().isNot(AsmToken::Equal)) {
    reportParseError("unexpected token, expected equals sign");
    return false;
  }
  Parser.Lex(); // Eat "=".

  if (getLexer().isNot(AsmToken::Dollar)) {
    if (getLexer().is(AsmToken::EndOfStatement)) {
      reportParseError("no register specified");
      return false;
    } else {
      reportParseError("unexpected token, expected dollar sign '$'");
      return false;
    }
  }
  Parser.Lex(); // Eat "$".

  // Find out what "reg" is.
  unsigned AtRegNo;
  const AsmToken &Reg = Parser.getTok();
  if (Reg.is(AsmToken::Identifier)) {
    AtRegNo = matchCPURegisterName(Reg.getIdentifier());
  } else if (Reg.is(AsmToken::Integer)) {
    AtRegNo = Reg.getIntVal();
  } else {
    reportParseError("unexpected token, expected identifier or integer");
    return false;
  }

  // Check if $reg is a valid register. If it is, set $at to $reg.
  if (!AssemblerOptions.back()->setATRegIndex(AtRegNo)) {
    reportParseError("invalid register");
    return false;
  }
  Parser.Lex(); // Eat "reg".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().emitDirectiveSetAtWithArg(AtRegNo);

  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetReorderDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  AssemblerOptions.back()->setReorder();
  getTargetStreamer().emitDirectiveSetReorder();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetNoReorderDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  AssemblerOptions.back()->setNoReorder();
  getTargetStreamer().emitDirectiveSetNoReorder();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetMacroDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  AssemblerOptions.back()->setMacro();
  getTargetStreamer().emitDirectiveSetMacro();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetNoMacroDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  if (AssemblerOptions.back()->isReorder()) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  AssemblerOptions.back()->setNoMacro();
  getTargetStreamer().emitDirectiveSetNoMacro();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetMsaDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  setFeatureBits(M6502::FeatureMSA, "msa");
  getTargetStreamer().emitDirectiveSetMsa();
  return false;
}

bool M6502AsmParser::parseSetNoMsaDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  clearFeatureBits(M6502::FeatureMSA, "msa");
  getTargetStreamer().emitDirectiveSetNoMsa();
  return false;
}

bool M6502AsmParser::parseSetNoDspDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nodsp".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(M6502::FeatureDSP, "dsp");
  getTargetStreamer().emitDirectiveSetNoDsp();
  return false;
}

bool M6502AsmParser::parseSetM650216Directive() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "m650216".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  setFeatureBits(M6502::FeatureM650216, "m650216");
  getTargetStreamer().emitDirectiveSetM650216();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetNoM650216Directive() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nom650216".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(M6502::FeatureM650216, "m650216");
  getTargetStreamer().emitDirectiveSetNoM650216();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetFpDirective() {
  MCAsmParser &Parser = getParser();
  M6502ABIFlagsSection::FpABIKind FpAbiVal;
  // Line can be: .set fp=32
  //              .set fp=xx
  //              .set fp=64
  Parser.Lex(); // Eat fp token
  AsmToken Tok = Parser.getTok();
  if (Tok.isNot(AsmToken::Equal)) {
    reportParseError("unexpected token, expected equals sign '='");
    return false;
  }
  Parser.Lex(); // Eat '=' token.
  Tok = Parser.getTok();

  if (!parseFpABIValue(FpAbiVal, ".set"))
    return false;

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }
  getTargetStreamer().emitDirectiveSetFp(FpAbiVal);
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetOddSPRegDirective() {
  MCAsmParser &Parser = getParser();

  Parser.Lex(); // Eat "oddspreg".
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(M6502::FeatureNoOddSPReg, "nooddspreg");
  getTargetStreamer().emitDirectiveSetOddSPReg();
  return false;
}

bool M6502AsmParser::parseSetNoOddSPRegDirective() {
  MCAsmParser &Parser = getParser();

  Parser.Lex(); // Eat "nooddspreg".
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  setFeatureBits(M6502::FeatureNoOddSPReg, "nooddspreg");
  getTargetStreamer().emitDirectiveSetNoOddSPReg();
  return false;
}

bool M6502AsmParser::parseSetMtDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "mt".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  setFeatureBits(M6502::FeatureMT, "mt");
  getTargetStreamer().emitDirectiveSetMt();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetNoMtDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex(); // Eat "nomt".

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  clearFeatureBits(M6502::FeatureMT, "mt");

  getTargetStreamer().emitDirectiveSetNoMt();
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseSetPopDirective() {
  MCAsmParser &Parser = getParser();
  SMLoc Loc = getLexer().getLoc();

  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  // Always keep an element on the options "stack" to prevent the user
  // from changing the initial options. This is how we remember them.
  if (AssemblerOptions.size() == 2)
    return reportParseError(Loc, ".set pop with no .set push");

  MCSubtargetInfo &STI = copySTI();
  AssemblerOptions.pop_back();
  setAvailableFeatures(
      ComputeAvailableFeatures(AssemblerOptions.back()->getFeatures()));
  STI.setFeatureBits(AssemblerOptions.back()->getFeatures());

  getTargetStreamer().emitDirectiveSetPop();
  return false;
}

bool M6502AsmParser::parseSetPushDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  // Create a copy of the current assembler options environment and push it.
  AssemblerOptions.push_back(
        llvm::make_unique<M6502AssemblerOptions>(AssemblerOptions.back().get()));

  getTargetStreamer().emitDirectiveSetPush();
  return false;
}

bool M6502AsmParser::parseSetSoftFloatDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  setFeatureBits(M6502::FeatureSoftFloat, "soft-float");
  getTargetStreamer().emitDirectiveSetSoftFloat();
  return false;
}

bool M6502AsmParser::parseSetHardFloatDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  clearFeatureBits(M6502::FeatureSoftFloat, "soft-float");
  getTargetStreamer().emitDirectiveSetHardFloat();
  return false;
}

bool M6502AsmParser::parseSetAssignment() {
  StringRef Name;
  const MCExpr *Value;
  MCAsmParser &Parser = getParser();

  if (Parser.parseIdentifier(Name))
    reportParseError("expected identifier after .set");

  if (getLexer().isNot(AsmToken::Comma))
    return reportParseError("unexpected token, expected comma");
  Lex(); // Eat comma

  if (Parser.parseExpression(Value))
    return reportParseError("expected valid expression after comma");

  MCSymbol *Sym = getContext().getOrCreateSymbol(Name);
  Sym->setVariableValue(Value);

  return false;
}

bool M6502AsmParser::parseSetM65020Directive() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  // Reset assembler options to their initial values.
  MCSubtargetInfo &STI = copySTI();
  setAvailableFeatures(
      ComputeAvailableFeatures(AssemblerOptions.front()->getFeatures()));
  STI.setFeatureBits(AssemblerOptions.front()->getFeatures());
  AssemblerOptions.back()->setFeatures(AssemblerOptions.front()->getFeatures());

  getTargetStreamer().emitDirectiveSetM65020();
  return false;
}

bool M6502AsmParser::parseSetArchDirective() {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::Equal))
    return reportParseError("unexpected token, expected equals sign");

  Parser.Lex();
  StringRef Arch;
  if (Parser.parseIdentifier(Arch))
    return reportParseError("expected arch identifier");

  StringRef ArchFeatureName =
      StringSwitch<StringRef>(Arch)
          .Case("m65021", "m65021")
          .Case("m65022", "m65022")
          .Case("m65023", "m65023")
          .Case("m65024", "m65024")
          .Case("m65025", "m65025")
          .Case("m650232", "m650232")
          .Case("m650232r2", "m650232r2")
          .Case("m650232r3", "m650232r3")
          .Case("m650232r5", "m650232r5")
          .Case("m650232r6", "m650232r6")
          .Case("m650264", "m650264")
          .Case("m650264r2", "m650264r2")
          .Case("m650264r3", "m650264r3")
          .Case("m650264r5", "m650264r5")
          .Case("m650264r6", "m650264r6")
          .Case("octeon", "cnm6502")
          .Case("r4000", "m65023") // This is an implementation of M65023.
          .Default("");

  if (ArchFeatureName.empty())
    return reportParseError("unsupported architecture");

  selectArch(ArchFeatureName);
  getTargetStreamer().emitDirectiveSetArch(Arch);
  return false;
}

bool M6502AsmParser::parseSetFeature(uint64_t Feature) {
  MCAsmParser &Parser = getParser();
  Parser.Lex();
  if (getLexer().isNot(AsmToken::EndOfStatement))
    return reportParseError("unexpected token, expected end of statement");

  switch (Feature) {
  default:
    llvm_unreachable("Unimplemented feature");
  case M6502::FeatureDSP:
    setFeatureBits(M6502::FeatureDSP, "dsp");
    getTargetStreamer().emitDirectiveSetDsp();
    break;
  case M6502::FeatureDSPR2:
    setFeatureBits(M6502::FeatureDSPR2, "dspr2");
    getTargetStreamer().emitDirectiveSetDspr2();
    break;
  case M6502::FeatureMicroM6502:
    setFeatureBits(M6502::FeatureMicroM6502, "microm6502");
    getTargetStreamer().emitDirectiveSetMicroM6502();
    break;
  case M6502::FeatureM65021:
    selectArch("m65021");
    getTargetStreamer().emitDirectiveSetM65021();
    break;
  case M6502::FeatureM65022:
    selectArch("m65022");
    getTargetStreamer().emitDirectiveSetM65022();
    break;
  case M6502::FeatureM65023:
    selectArch("m65023");
    getTargetStreamer().emitDirectiveSetM65023();
    break;
  case M6502::FeatureM65024:
    selectArch("m65024");
    getTargetStreamer().emitDirectiveSetM65024();
    break;
  case M6502::FeatureM65025:
    selectArch("m65025");
    getTargetStreamer().emitDirectiveSetM65025();
    break;
  case M6502::FeatureM650232:
    selectArch("m650232");
    getTargetStreamer().emitDirectiveSetM650232();
    break;
  case M6502::FeatureM650232r2:
    selectArch("m650232r2");
    getTargetStreamer().emitDirectiveSetM650232R2();
    break;
  case M6502::FeatureM650232r3:
    selectArch("m650232r3");
    getTargetStreamer().emitDirectiveSetM650232R3();
    break;
  case M6502::FeatureM650232r5:
    selectArch("m650232r5");
    getTargetStreamer().emitDirectiveSetM650232R5();
    break;
  case M6502::FeatureM650232r6:
    selectArch("m650232r6");
    getTargetStreamer().emitDirectiveSetM650232R6();
    break;
  case M6502::FeatureM650264:
    selectArch("m650264");
    getTargetStreamer().emitDirectiveSetM650264();
    break;
  case M6502::FeatureM650264r2:
    selectArch("m650264r2");
    getTargetStreamer().emitDirectiveSetM650264R2();
    break;
  case M6502::FeatureM650264r3:
    selectArch("m650264r3");
    getTargetStreamer().emitDirectiveSetM650264R3();
    break;
  case M6502::FeatureM650264r5:
    selectArch("m650264r5");
    getTargetStreamer().emitDirectiveSetM650264R5();
    break;
  case M6502::FeatureM650264r6:
    selectArch("m650264r6");
    getTargetStreamer().emitDirectiveSetM650264R6();
    break;
  }
  return false;
}

bool M6502AsmParser::eatComma(StringRef ErrorStr) {
  MCAsmParser &Parser = getParser();
  if (getLexer().isNot(AsmToken::Comma)) {
    SMLoc Loc = getLexer().getLoc();
    return Error(Loc, ErrorStr);
  }

  Parser.Lex(); // Eat the comma.
  return true;
}

// Used to determine if .cpload, .cprestore, and .cpsetup have any effect.
// In this class, it is only used for .cprestore.
// FIXME: Only keep track of IsPicEnabled in one place, instead of in both
// M6502TargetELFStreamer and M6502AsmParser.
bool M6502AsmParser::isPicAndNotNxxAbi() {
  return inPicMode() && !(isABI_N32() || isABI_N64());
}

bool M6502AsmParser::parseDirectiveCpLoad(SMLoc Loc) {
  if (AssemblerOptions.back()->isReorder())
    Warning(Loc, ".cpload should be inside a noreorder section");

  if (inM650216Mode()) {
    reportParseError(".cpload is not supported in M650216 mode");
    return false;
  }

  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> Reg;
  OperandMatchResultTy ResTy = parseAnyRegister(Reg);
  if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
    reportParseError("expected register containing function address");
    return false;
  }

  M6502Operand &RegOpnd = static_cast<M6502Operand &>(*Reg[0]);
  if (!RegOpnd.isGPRAsmReg()) {
    reportParseError(RegOpnd.getStartLoc(), "invalid register");
    return false;
  }

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().emitDirectiveCpLoad(RegOpnd.getGPR32Reg());
  return false;
}

bool M6502AsmParser::parseDirectiveCpRestore(SMLoc Loc) {
  MCAsmParser &Parser = getParser();

  // Note that .cprestore is ignored if used with the N32 and N64 ABIs or if it
  // is used in non-PIC mode.

  if (inM650216Mode()) {
    reportParseError(".cprestore is not supported in M650216 mode");
    return false;
  }

  // Get the stack offset value.
  const MCExpr *StackOffset;
  int64_t StackOffsetVal;
  if (Parser.parseExpression(StackOffset)) {
    reportParseError("expected stack offset value");
    return false;
  }

  if (!StackOffset->evaluateAsAbsolute(StackOffsetVal)) {
    reportParseError("stack offset is not an absolute expression");
    return false;
  }

  if (StackOffsetVal < 0) {
    Warning(Loc, ".cprestore with negative stack offset has no effect");
    IsCpRestoreSet = false;
  } else {
    IsCpRestoreSet = true;
    CpRestoreOffset = StackOffsetVal;
  }

  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  if (!getTargetStreamer().emitDirectiveCpRestore(
          CpRestoreOffset, [&]() { return getATReg(Loc); }, Loc, STI))
    return true;
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseDirectiveCPSetup() {
  MCAsmParser &Parser = getParser();
  unsigned FuncReg;
  unsigned Save;
  bool SaveIsReg = true;

  SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> TmpReg;
  OperandMatchResultTy ResTy = parseAnyRegister(TmpReg);
  if (ResTy == MatchOperand_NoMatch) {
    reportParseError("expected register containing function address");
    return false;
  }

  M6502Operand &FuncRegOpnd = static_cast<M6502Operand &>(*TmpReg[0]);
  if (!FuncRegOpnd.isGPRAsmReg()) {
    reportParseError(FuncRegOpnd.getStartLoc(), "invalid register");
    return false;
  }

  FuncReg = FuncRegOpnd.getGPR32Reg();
  TmpReg.clear();

  if (!eatComma("unexpected token, expected comma"))
    return true;

  ResTy = parseAnyRegister(TmpReg);
  if (ResTy == MatchOperand_NoMatch) {
    const MCExpr *OffsetExpr;
    int64_t OffsetVal;
    SMLoc ExprLoc = getLexer().getLoc();

    if (Parser.parseExpression(OffsetExpr) ||
        !OffsetExpr->evaluateAsAbsolute(OffsetVal)) {
      reportParseError(ExprLoc, "expected save register or stack offset");
      return false;
    }

    Save = OffsetVal;
    SaveIsReg = false;
  } else {
    M6502Operand &SaveOpnd = static_cast<M6502Operand &>(*TmpReg[0]);
    if (!SaveOpnd.isGPRAsmReg()) {
      reportParseError(SaveOpnd.getStartLoc(), "invalid register");
      return false;
    }
    Save = SaveOpnd.getGPR32Reg();
  }

  if (!eatComma("unexpected token, expected comma"))
    return true;

  const MCExpr *Expr;
  if (Parser.parseExpression(Expr)) {
    reportParseError("expected expression");
    return false;
  }

  if (Expr->getKind() != MCExpr::SymbolRef) {
    reportParseError("expected symbol");
    return false;
  }
  const MCSymbolRefExpr *Ref = static_cast<const MCSymbolRefExpr *>(Expr);

  CpSaveLocation = Save;
  CpSaveLocationIsRegister = SaveIsReg;

  getTargetStreamer().emitDirectiveCpsetup(FuncReg, Save, Ref->getSymbol(),
                                           SaveIsReg);
  return false;
}

bool M6502AsmParser::parseDirectiveCPReturn() {
  getTargetStreamer().emitDirectiveCpreturn(CpSaveLocation,
                                            CpSaveLocationIsRegister);
  return false;
}

bool M6502AsmParser::parseDirectiveNaN() {
  MCAsmParser &Parser = getParser();
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    const AsmToken &Tok = Parser.getTok();

    if (Tok.getString() == "2008") {
      Parser.Lex();
      getTargetStreamer().emitDirectiveNaN2008();
      return false;
    } else if (Tok.getString() == "legacy") {
      Parser.Lex();
      getTargetStreamer().emitDirectiveNaNLegacy();
      return false;
    }
  }
  // If we don't recognize the option passed to the .nan
  // directive (e.g. no option or unknown option), emit an error.
  reportParseError("invalid option in .nan directive");
  return false;
}

bool M6502AsmParser::parseDirectiveSet() {
  MCAsmParser &Parser = getParser();
  // Get the next token.
  const AsmToken &Tok = Parser.getTok();

  if (Tok.getString() == "noat") {
    return parseSetNoAtDirective();
  } else if (Tok.getString() == "at") {
    return parseSetAtDirective();
  } else if (Tok.getString() == "arch") {
    return parseSetArchDirective();
  } else if (Tok.getString() == "bopt") {
    Warning(Tok.getLoc(), "'bopt' feature is unsupported");
    getParser().Lex();
    return false;
  } else if (Tok.getString() == "nobopt") {
    // We're already running in nobopt mode, so nothing to do.
    getParser().Lex();
    return false;
  } else if (Tok.getString() == "fp") {
    return parseSetFpDirective();
  } else if (Tok.getString() == "oddspreg") {
    return parseSetOddSPRegDirective();
  } else if (Tok.getString() == "nooddspreg") {
    return parseSetNoOddSPRegDirective();
  } else if (Tok.getString() == "pop") {
    return parseSetPopDirective();
  } else if (Tok.getString() == "push") {
    return parseSetPushDirective();
  } else if (Tok.getString() == "reorder") {
    return parseSetReorderDirective();
  } else if (Tok.getString() == "noreorder") {
    return parseSetNoReorderDirective();
  } else if (Tok.getString() == "macro") {
    return parseSetMacroDirective();
  } else if (Tok.getString() == "nomacro") {
    return parseSetNoMacroDirective();
  } else if (Tok.getString() == "m650216") {
    return parseSetM650216Directive();
  } else if (Tok.getString() == "nom650216") {
    return parseSetNoM650216Directive();
  } else if (Tok.getString() == "nomicrom6502") {
    clearFeatureBits(M6502::FeatureMicroM6502, "microm6502");
    getTargetStreamer().emitDirectiveSetNoMicroM6502();
    Parser.eatToEndOfStatement();
    return false;
  } else if (Tok.getString() == "microm6502") {
    return parseSetFeature(M6502::FeatureMicroM6502);
  } else if (Tok.getString() == "m65020") {
    return parseSetM65020Directive();
  } else if (Tok.getString() == "m65021") {
    return parseSetFeature(M6502::FeatureM65021);
  } else if (Tok.getString() == "m65022") {
    return parseSetFeature(M6502::FeatureM65022);
  } else if (Tok.getString() == "m65023") {
    return parseSetFeature(M6502::FeatureM65023);
  } else if (Tok.getString() == "m65024") {
    return parseSetFeature(M6502::FeatureM65024);
  } else if (Tok.getString() == "m65025") {
    return parseSetFeature(M6502::FeatureM65025);
  } else if (Tok.getString() == "m650232") {
    return parseSetFeature(M6502::FeatureM650232);
  } else if (Tok.getString() == "m650232r2") {
    return parseSetFeature(M6502::FeatureM650232r2);
  } else if (Tok.getString() == "m650232r3") {
    return parseSetFeature(M6502::FeatureM650232r3);
  } else if (Tok.getString() == "m650232r5") {
    return parseSetFeature(M6502::FeatureM650232r5);
  } else if (Tok.getString() == "m650232r6") {
    return parseSetFeature(M6502::FeatureM650232r6);
  } else if (Tok.getString() == "m650264") {
    return parseSetFeature(M6502::FeatureM650264);
  } else if (Tok.getString() == "m650264r2") {
    return parseSetFeature(M6502::FeatureM650264r2);
  } else if (Tok.getString() == "m650264r3") {
    return parseSetFeature(M6502::FeatureM650264r3);
  } else if (Tok.getString() == "m650264r5") {
    return parseSetFeature(M6502::FeatureM650264r5);
  } else if (Tok.getString() == "m650264r6") {
    return parseSetFeature(M6502::FeatureM650264r6);
  } else if (Tok.getString() == "dsp") {
    return parseSetFeature(M6502::FeatureDSP);
  } else if (Tok.getString() == "dspr2") {
    return parseSetFeature(M6502::FeatureDSPR2);
  } else if (Tok.getString() == "nodsp") {
    return parseSetNoDspDirective();
  } else if (Tok.getString() == "msa") {
    return parseSetMsaDirective();
  } else if (Tok.getString() == "nomsa") {
    return parseSetNoMsaDirective();
  } else if (Tok.getString() == "mt") {
    return parseSetMtDirective();
  } else if (Tok.getString() == "nomt") {
    return parseSetNoMtDirective();
  } else if (Tok.getString() == "softfloat") {
    return parseSetSoftFloatDirective();
  } else if (Tok.getString() == "hardfloat") {
    return parseSetHardFloatDirective();
  } else {
    // It is just an identifier, look for an assignment.
    parseSetAssignment();
    return false;
  }

  return true;
}

/// parseDataDirective
///  ::= .word [ expression (, expression)* ]
bool M6502AsmParser::parseDataDirective(unsigned Size, SMLoc L) {
  MCAsmParser &Parser = getParser();
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    while (true) {
      const MCExpr *Value;
      if (getParser().parseExpression(Value))
        return true;

      getParser().getStreamer().EmitValue(Value, Size);

      if (getLexer().is(AsmToken::EndOfStatement))
        break;

      if (getLexer().isNot(AsmToken::Comma))
        return Error(L, "unexpected token, expected comma");
      Parser.Lex();
    }
  }

  Parser.Lex();
  return false;
}

/// parseDirectiveGpWord
///  ::= .gpword local_sym
bool M6502AsmParser::parseDirectiveGpWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitGPRel32Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().EmitGPRel32Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(), 
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveGpDWord
///  ::= .gpdword local_sym
bool M6502AsmParser::parseDirectiveGpDWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitGPRel64Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().EmitGPRel64Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveDtpRelWord
///  ::= .dtprelword tls_sym
bool M6502AsmParser::parseDirectiveDtpRelWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitDTPRel32Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().EmitDTPRel32Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveDtpRelDWord
///  ::= .dtpreldword tls_sym
bool M6502AsmParser::parseDirectiveDtpRelDWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitDTPRel64Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().EmitDTPRel64Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveTpRelWord
///  ::= .tprelword tls_sym
bool M6502AsmParser::parseDirectiveTpRelWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitTPRel32Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().EmitTPRel32Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveTpRelDWord
///  ::= .tpreldword tls_sym
bool M6502AsmParser::parseDirectiveTpRelDWord() {
  MCAsmParser &Parser = getParser();
  const MCExpr *Value;
  // EmitTPRel64Value requires an expression, so we are using base class
  // method to evaluate the expression.
  if (getParser().parseExpression(Value))
    return true;
  getParser().getStreamer().EmitTPRel64Value(Value);

  if (getLexer().isNot(AsmToken::EndOfStatement))
    return Error(getLexer().getLoc(),
                "unexpected token, expected end of statement");
  Parser.Lex(); // Eat EndOfStatement token.
  return false;
}

bool M6502AsmParser::parseDirectiveOption() {
  MCAsmParser &Parser = getParser();
  // Get the option token.
  AsmToken Tok = Parser.getTok();
  // At the moment only identifiers are supported.
  if (Tok.isNot(AsmToken::Identifier)) {
    return Error(Parser.getTok().getLoc(),
                 "unexpected token, expected identifier");
  }

  StringRef Option = Tok.getIdentifier();

  if (Option == "pic0") {
    // M6502AsmParser needs to know if the current PIC mode changes.
    IsPicEnabled = false;

    getTargetStreamer().emitDirectiveOptionPic0();
    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement)) {
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");
    }
    return false;
  }

  if (Option == "pic2") {
    // M6502AsmParser needs to know if the current PIC mode changes.
    IsPicEnabled = true;

    getTargetStreamer().emitDirectiveOptionPic2();
    Parser.Lex();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement)) {
      return Error(Parser.getTok().getLoc(),
                   "unexpected token, expected end of statement");
    }
    return false;
  }

  // Unknown option.
  Warning(Parser.getTok().getLoc(), 
          "unknown option, expected 'pic0' or 'pic2'");
  Parser.eatToEndOfStatement();
  return false;
}

/// parseInsnDirective
///  ::= .insn
bool M6502AsmParser::parseInsnDirective() {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  // The actual label marking happens in
  // M6502ELFStreamer::createPendingLabelRelocs().
  getTargetStreamer().emitDirectiveInsn();

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseRSectionDirective
///  ::= .rdata
bool M6502AsmParser::parseRSectionDirective(StringRef Section) {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  MCSection *ELFSection = getContext().getELFSection(
      Section, ELF::SHT_PROGBITS, ELF::SHF_ALLOC);
  getParser().getStreamer().SwitchSection(ELFSection);

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseSSectionDirective
///  ::= .sbss
///  ::= .sdata
bool M6502AsmParser::parseSSectionDirective(StringRef Section, unsigned Type) {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  MCSection *ELFSection = getContext().getELFSection(
      Section, Type, ELF::SHF_WRITE | ELF::SHF_ALLOC | ELF::SHF_MIPS_GPREL);
  getParser().getStreamer().SwitchSection(ELFSection);

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

/// parseDirectiveModule
///  ::= .module oddspreg
///  ::= .module nooddspreg
///  ::= .module fp=value
///  ::= .module softfloat
///  ::= .module hardfloat
///  ::= .module mt
bool M6502AsmParser::parseDirectiveModule() {
  MCAsmParser &Parser = getParser();
  MCAsmLexer &Lexer = getLexer();
  SMLoc L = Lexer.getLoc();

  if (!getTargetStreamer().isModuleDirectiveAllowed()) {
    // TODO : get a better message.
    reportParseError(".module directive must appear before any code");
    return false;
  }

  StringRef Option;
  if (Parser.parseIdentifier(Option)) {
    reportParseError("expected .module option identifier");
    return false;
  }

  if (Option == "oddspreg") {
    clearModuleFeatureBits(M6502::FeatureNoOddSPReg, "nooddspreg");

    // Synchronize the abiflags information with the FeatureBits information we
    // changed above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated abiflags information.
    // If generating ELF, don't do anything (the .M6502.abiflags section gets
    // emitted at the end).
    getTargetStreamer().emitDirectiveModuleOddSPReg();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "nooddspreg") {
    if (!isABI_O32()) {
      return Error(L, "'.module nooddspreg' requires the O32 ABI");
    }

    setModuleFeatureBits(M6502::FeatureNoOddSPReg, "nooddspreg");

    // Synchronize the abiflags information with the FeatureBits information we
    // changed above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated abiflags information.
    // If generating ELF, don't do anything (the .M6502.abiflags section gets
    // emitted at the end).
    getTargetStreamer().emitDirectiveModuleOddSPReg();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "fp") {
    return parseDirectiveModuleFP();
  } else if (Option == "softfloat") {
    setModuleFeatureBits(M6502::FeatureSoftFloat, "soft-float");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .M6502.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleSoftFloat();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "hardfloat") {
    clearModuleFeatureBits(M6502::FeatureSoftFloat, "soft-float");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .M6502.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleHardFloat();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else if (Option == "mt") {
    setModuleFeatureBits(M6502::FeatureMT, "mt");

    // Synchronize the ABI Flags information with the FeatureBits information we
    // updated above.
    getTargetStreamer().updateABIInfo(*this);

    // If printing assembly, use the recently updated ABI Flags information.
    // If generating ELF, don't do anything (the .M6502.abiflags section gets
    // emitted later).
    getTargetStreamer().emitDirectiveModuleMT();

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    return false; // parseDirectiveModule has finished successfully.
  } else {
    return Error(L, "'" + Twine(Option) + "' is not a valid .module option.");
  }
}

/// parseDirectiveModuleFP
///  ::= =32
///  ::= =xx
///  ::= =64
bool M6502AsmParser::parseDirectiveModuleFP() {
  MCAsmParser &Parser = getParser();
  MCAsmLexer &Lexer = getLexer();

  if (Lexer.isNot(AsmToken::Equal)) {
    reportParseError("unexpected token, expected equals sign '='");
    return false;
  }
  Parser.Lex(); // Eat '=' token.

  M6502ABIFlagsSection::FpABIKind FpABI;
  if (!parseFpABIValue(FpABI, ".module"))
    return false;

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  // Synchronize the abiflags information with the FeatureBits information we
  // changed above.
  getTargetStreamer().updateABIInfo(*this);

  // If printing assembly, use the recently updated abiflags information.
  // If generating ELF, don't do anything (the .M6502.abiflags section gets
  // emitted at the end).
  getTargetStreamer().emitDirectiveModuleFP();

  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool M6502AsmParser::parseFpABIValue(M6502ABIFlagsSection::FpABIKind &FpABI,
                                    StringRef Directive) {
  MCAsmParser &Parser = getParser();
  MCAsmLexer &Lexer = getLexer();
  bool ModuleLevelOptions = Directive == ".module";

  if (Lexer.is(AsmToken::Identifier)) {
    StringRef Value = Parser.getTok().getString();
    Parser.Lex();

    if (Value != "xx") {
      reportParseError("unsupported value, expected 'xx', '32' or '64'");
      return false;
    }

    if (!isABI_O32()) {
      reportParseError("'" + Directive + " fp=xx' requires the O32 ABI");
      return false;
    }

    FpABI = M6502ABIFlagsSection::FpABIKind::XX;
    if (ModuleLevelOptions) {
      setModuleFeatureBits(M6502::FeatureFPXX, "fpxx");
      clearModuleFeatureBits(M6502::FeatureFP64Bit, "fp64");
    } else {
      setFeatureBits(M6502::FeatureFPXX, "fpxx");
      clearFeatureBits(M6502::FeatureFP64Bit, "fp64");
    }
    return true;
  }

  if (Lexer.is(AsmToken::Integer)) {
    unsigned Value = Parser.getTok().getIntVal();
    Parser.Lex();

    if (Value != 32 && Value != 64) {
      reportParseError("unsupported value, expected 'xx', '32' or '64'");
      return false;
    }

    if (Value == 32) {
      if (!isABI_O32()) {
        reportParseError("'" + Directive + " fp=32' requires the O32 ABI");
        return false;
      }

      FpABI = M6502ABIFlagsSection::FpABIKind::S32;
      if (ModuleLevelOptions) {
        clearModuleFeatureBits(M6502::FeatureFPXX, "fpxx");
        clearModuleFeatureBits(M6502::FeatureFP64Bit, "fp64");
      } else {
        clearFeatureBits(M6502::FeatureFPXX, "fpxx");
        clearFeatureBits(M6502::FeatureFP64Bit, "fp64");
      }
    } else {
      FpABI = M6502ABIFlagsSection::FpABIKind::S64;
      if (ModuleLevelOptions) {
        clearModuleFeatureBits(M6502::FeatureFPXX, "fpxx");
        setModuleFeatureBits(M6502::FeatureFP64Bit, "fp64");
      } else {
        clearFeatureBits(M6502::FeatureFPXX, "fpxx");
        setFeatureBits(M6502::FeatureFP64Bit, "fp64");
      }
    }

    return true;
  }

  return false;
}

bool M6502AsmParser::ParseDirective(AsmToken DirectiveID) {
  // This returns false if this function recognizes the directive
  // regardless of whether it is successfully handles or reports an
  // error. Otherwise it returns true to give the generic parser a
  // chance at recognizing it.

  MCAsmParser &Parser = getParser();
  StringRef IDVal = DirectiveID.getString();

  if (IDVal == ".cpload") {
    parseDirectiveCpLoad(DirectiveID.getLoc());
    return false;
  }
  if (IDVal == ".cprestore") {
    parseDirectiveCpRestore(DirectiveID.getLoc());
    return false;
  }
  if (IDVal == ".dword") {
    parseDataDirective(8, DirectiveID.getLoc());
    return false;
  }
  if (IDVal == ".ent") {
    StringRef SymbolName;

    if (Parser.parseIdentifier(SymbolName)) {
      reportParseError("expected identifier after .ent");
      return false;
    }

    // There's an undocumented extension that allows an integer to
    // follow the name of the procedure which AFAICS is ignored by GAS.
    // Example: .ent foo,2
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      if (getLexer().isNot(AsmToken::Comma)) {
        // Even though we accept this undocumented extension for compatibility
        // reasons, the additional integer argument does not actually change
        // the behaviour of the '.ent' directive, so we would like to discourage
        // its use. We do this by not referring to the extended version in
        // error messages which are not directly related to its use.
        reportParseError("unexpected token, expected end of statement");
        return false;
      }
      Parser.Lex(); // Eat the comma.
      const MCExpr *DummyNumber;
      int64_t DummyNumberVal;
      // If the user was explicitly trying to use the extended version,
      // we still give helpful extension-related error messages.
      if (Parser.parseExpression(DummyNumber)) {
        reportParseError("expected number after comma");
        return false;
      }
      if (!DummyNumber->evaluateAsAbsolute(DummyNumberVal)) {
        reportParseError("expected an absolute expression after comma");
        return false;
      }
    }

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    MCSymbol *Sym = getContext().getOrCreateSymbol(SymbolName);

    getTargetStreamer().emitDirectiveEnt(*Sym);
    CurrentFn = Sym;
    IsCpRestoreSet = false;
    return false;
  }

  if (IDVal == ".end") {
    StringRef SymbolName;

    if (Parser.parseIdentifier(SymbolName)) {
      reportParseError("expected identifier after .end");
      return false;
    }

    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    if (CurrentFn == nullptr) {
      reportParseError(".end used without .ent");
      return false;
    }

    if ((SymbolName != CurrentFn->getName())) {
      reportParseError(".end symbol does not match .ent symbol");
      return false;
    }

    getTargetStreamer().emitDirectiveEnd(SymbolName);
    CurrentFn = nullptr;
    IsCpRestoreSet = false;
    return false;
  }

  if (IDVal == ".frame") {
    // .frame $stack_reg, frame_size_in_bytes, $return_reg
    SmallVector<std::unique_ptr<MCParsedAsmOperand>, 1> TmpReg;
    OperandMatchResultTy ResTy = parseAnyRegister(TmpReg);
    if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
      reportParseError("expected stack register");
      return false;
    }

    M6502Operand &StackRegOpnd = static_cast<M6502Operand &>(*TmpReg[0]);
    if (!StackRegOpnd.isGPRAsmReg()) {
      reportParseError(StackRegOpnd.getStartLoc(),
                       "expected general purpose register");
      return false;
    }
    unsigned StackReg = StackRegOpnd.getGPR32Reg();

    if (Parser.getTok().is(AsmToken::Comma))
      Parser.Lex();
    else {
      reportParseError("unexpected token, expected comma");
      return false;
    }

    // Parse the frame size.
    const MCExpr *FrameSize;
    int64_t FrameSizeVal;

    if (Parser.parseExpression(FrameSize)) {
      reportParseError("expected frame size value");
      return false;
    }

    if (!FrameSize->evaluateAsAbsolute(FrameSizeVal)) {
      reportParseError("frame size not an absolute expression");
      return false;
    }

    if (Parser.getTok().is(AsmToken::Comma))
      Parser.Lex();
    else {
      reportParseError("unexpected token, expected comma");
      return false;
    }

    // Parse the return register.
    TmpReg.clear();
    ResTy = parseAnyRegister(TmpReg);
    if (ResTy == MatchOperand_NoMatch || ResTy == MatchOperand_ParseFail) {
      reportParseError("expected return register");
      return false;
    }

    M6502Operand &ReturnRegOpnd = static_cast<M6502Operand &>(*TmpReg[0]);
    if (!ReturnRegOpnd.isGPRAsmReg()) {
      reportParseError(ReturnRegOpnd.getStartLoc(),
                       "expected general purpose register");
      return false;
    }

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    getTargetStreamer().emitFrame(StackReg, FrameSizeVal,
                                  ReturnRegOpnd.getGPR32Reg());
    IsCpRestoreSet = false;
    return false;
  }

  if (IDVal == ".set") {
    parseDirectiveSet();
    return false;
  }

  if (IDVal == ".mask" || IDVal == ".fmask") {
    // .mask bitmask, frame_offset
    // bitmask: One bit for each register used.
    // frame_offset: Offset from Canonical Frame Address ($sp on entry) where
    //               first register is expected to be saved.
    // Examples:
    //   .mask 0x80000000, -4
    //   .fmask 0x80000000, -4
    //

    // Parse the bitmask
    const MCExpr *BitMask;
    int64_t BitMaskVal;

    if (Parser.parseExpression(BitMask)) {
      reportParseError("expected bitmask value");
      return false;
    }

    if (!BitMask->evaluateAsAbsolute(BitMaskVal)) {
      reportParseError("bitmask not an absolute expression");
      return false;
    }

    if (Parser.getTok().is(AsmToken::Comma))
      Parser.Lex();
    else {
      reportParseError("unexpected token, expected comma");
      return false;
    }

    // Parse the frame_offset
    const MCExpr *FrameOffset;
    int64_t FrameOffsetVal;

    if (Parser.parseExpression(FrameOffset)) {
      reportParseError("expected frame offset value");
      return false;
    }

    if (!FrameOffset->evaluateAsAbsolute(FrameOffsetVal)) {
      reportParseError("frame offset not an absolute expression");
      return false;
    }

    // If this is not the end of the statement, report an error.
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token, expected end of statement");
      return false;
    }

    if (IDVal == ".mask")
      getTargetStreamer().emitMask(BitMaskVal, FrameOffsetVal);
    else
      getTargetStreamer().emitFMask(BitMaskVal, FrameOffsetVal);
    return false;
  }

  if (IDVal == ".nan")
    return parseDirectiveNaN();

  if (IDVal == ".gpword") {
    parseDirectiveGpWord();
    return false;
  }

  if (IDVal == ".gpdword") {
    parseDirectiveGpDWord();
    return false;
  }

  if (IDVal == ".dtprelword") {
    parseDirectiveDtpRelWord();
    return false;
  }

  if (IDVal == ".dtpreldword") {
    parseDirectiveDtpRelDWord();
    return false;
  }

  if (IDVal == ".tprelword") {
    parseDirectiveTpRelWord();
    return false;
  }

  if (IDVal == ".tpreldword") {
    parseDirectiveTpRelDWord();
    return false;
  }

  if (IDVal == ".word") {
    parseDataDirective(4, DirectiveID.getLoc());
    return false;
  }

  if (IDVal == ".hword") {
    parseDataDirective(2, DirectiveID.getLoc());
    return false;
  }

  if (IDVal == ".option") {
    parseDirectiveOption();
    return false;
  }

  if (IDVal == ".abicalls") {
    getTargetStreamer().emitDirectiveAbiCalls();
    if (Parser.getTok().isNot(AsmToken::EndOfStatement)) {
      Error(Parser.getTok().getLoc(), 
            "unexpected token, expected end of statement");
    }
    return false;
  }

  if (IDVal == ".cpsetup") {
    parseDirectiveCPSetup();
    return false;
  }
  if (IDVal == ".cpreturn") {
    parseDirectiveCPReturn();
    return false;
  }
  if (IDVal == ".module") {
    parseDirectiveModule();
    return false;
  }
  if (IDVal == ".llvm_internal_m6502_reallow_module_directive") {
    parseInternalDirectiveReallowModule();
    return false;
  }
  if (IDVal == ".insn") {
    parseInsnDirective();
    return false;
  }
  if (IDVal == ".rdata") {
    parseRSectionDirective(".rodata");
    return false;
  }
  if (IDVal == ".sbss") {
    parseSSectionDirective(IDVal, ELF::SHT_NOBITS);
    return false;
  }
  if (IDVal == ".sdata") {
    parseSSectionDirective(IDVal, ELF::SHT_PROGBITS);
    return false;
  }

  return true;
}

bool M6502AsmParser::parseInternalDirectiveReallowModule() {
  // If this is not the end of the statement, report an error.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token, expected end of statement");
    return false;
  }

  getTargetStreamer().reallowModuleDirective();

  getParser().Lex(); // Eat EndOfStatement token.
  return false;
}

extern "C" void LLVMInitializeM6502AsmParser() {
  RegisterMCAsmParser<M6502AsmParser> X(getTheM6502Target());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "M6502GenAsmMatcher.inc"

bool M6502AsmParser::mnemonicIsValid(StringRef Mnemonic, unsigned VariantID) {
  // Find the appropriate table for this asm variant.
  const MatchEntry *Start, *End;
  switch (VariantID) {
  default: llvm_unreachable("invalid variant!");
  case 0: Start = std::begin(MatchTable0); End = std::end(MatchTable0); break;
  }
  // Search the table.
  auto MnemonicRange = std::equal_range(Start, End, Mnemonic, LessOpcode());
  return MnemonicRange.first != MnemonicRange.second;
}
