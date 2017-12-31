//===-- M6502.td - Describe the M6502 Target Machine ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This is the top level entry point for the M6502 target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

// The overall idea of the PredicateControl class is to chop the Predicates list
// into subsets that are usually overridden independently. This allows
// subclasses to partially override the predicates of their superclasses without
// having to re-add all the existing predicates.
class PredicateControl {
  // Predicates for the encoding scheme in use such as HasStdEnc
  list<Predicate> EncodingPredicates = [];
  // Predicates for the GPR size such as IsGP64bit
  list<Predicate> GPRPredicates = [];
  // Predicates for the PTR size such as IsPTR64bit
  list<Predicate> PTRPredicates = [];
  // Predicates for the FGR size and layout such as IsFP64bit
  list<Predicate> FGRPredicates = [];
  // Predicates for the instruction group membership such as ISA's and ASE's
  list<Predicate> InsnPredicates = [];
  // Predicate for marking the instruction as usable in hard-float mode only.
  list<Predicate> HardFloatPredicate = [];
  // Predicates for anything else
  list<Predicate> AdditionalPredicates = [];
  list<Predicate> Predicates = !listconcat(EncodingPredicates,
                                           GPRPredicates,
                                           PTRPredicates,
                                           FGRPredicates,
                                           InsnPredicates,
                                           HardFloatPredicate,
                                           AdditionalPredicates);
}

// Like Requires<> but for the AdditionalPredicates list
class AdditionalRequires<list<Predicate> preds> {
  list<Predicate> AdditionalPredicates = preds;
}

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "M6502RegisterInfo.td"
include "M6502Schedule.td"
include "M6502InstrInfo.td"
include "M6502CallingConv.td"

// Avoid forward declaration issues.
include "M6502ScheduleP5600.td"
include "M6502ScheduleGeneric.td"

def M6502InstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// M6502 Subtarget features                                                    //
//===----------------------------------------------------------------------===//

def FeatureNoABICalls  : SubtargetFeature<"noabicalls", "NoABICalls", "true",
                                "Disable SVR4-style position-independent code">;
def FeaturePTR64Bit    : SubtargetFeature<"ptr64", "IsPTR64bit", "true",
                                "Pointers are 64-bit wide">;
def FeatureGP64Bit     : SubtargetFeature<"gp64", "IsGP64bit", "true",
                                "General Purpose Registers are 64-bit wide">;
def FeatureFP64Bit     : SubtargetFeature<"fp64", "IsFP64bit", "true",
                                "Support 64-bit FP registers">;
def FeatureFPXX        : SubtargetFeature<"fpxx", "IsFPXX", "true",
                                "Support for FPXX">;
def FeatureNaN2008     : SubtargetFeature<"nan2008", "IsNaN2008bit", "true",
                                "IEEE 754-2008 NaN encoding">;
def FeatureSingleFloat : SubtargetFeature<"single-float", "IsSingleFloat",
                                "true", "Only supports single precision float">;
def FeatureSoftFloat   : SubtargetFeature<"soft-float", "IsSoftFloat", "true",
                                "Does not support floating point instructions">;
def FeatureNoOddSPReg  : SubtargetFeature<"nooddspreg", "UseOddSPReg", "false",
                              "Disable odd numbered single-precision "
                              "registers">;
def FeatureVFPU        : SubtargetFeature<"vfpu", "HasVFPU",
                                "true", "Enable vector FPU instructions">;
def FeatureM65021       : SubtargetFeature<"mips1", "M6502ArchVersion", "M65021",
                                "M6502 I ISA Support [highly experimental]">;
def FeatureM65022       : SubtargetFeature<"mips2", "M6502ArchVersion", "M65022",
                                "M6502 II ISA Support [highly experimental]",
                                [FeatureM65021]>;
def FeatureM65023_32    : SubtargetFeature<"mips3_32", "HasM65023_32", "true",
                                "Subset of M6502-III that is also in M650232 "
                                "[highly experimental]">;
def FeatureM65023_32r2  : SubtargetFeature<"mips3_32r2", "HasM65023_32r2", "true",
                                "Subset of M6502-III that is also in M650232r2 "
                                "[highly experimental]">;
def FeatureM65023       : SubtargetFeature<"mips3", "M6502ArchVersion", "M65023",
                                "M6502 III ISA Support [highly experimental]",
                                [FeatureM65022, FeatureM65023_32,
                                 FeatureM65023_32r2, FeatureGP64Bit,
                                 FeatureFP64Bit]>;
def FeatureM65024_32    : SubtargetFeature<"mips4_32", "HasM65024_32", "true",
                                "Subset of M6502-IV that is also in M650232 "
                                "[highly experimental]">;
def FeatureM65024_32r2  : SubtargetFeature<"mips4_32r2", "HasM65024_32r2", "true",
                                "Subset of M6502-IV that is also in M650232r2 "
                                "[highly experimental]">;
def FeatureM65024       : SubtargetFeature<"mips4", "M6502ArchVersion",
                                "M65024", "M6502 IV ISA Support",
                                [FeatureM65023, FeatureM65024_32,
                                 FeatureM65024_32r2]>;
def FeatureM65025_32r2  : SubtargetFeature<"mips5_32r2", "HasM65025_32r2", "true",
                                "Subset of M6502-V that is also in M650232r2 "
                                "[highly experimental]">;
def FeatureM65025       : SubtargetFeature<"mips5", "M6502ArchVersion", "M65025",
                                "M6502 V ISA Support [highly experimental]",
                                [FeatureM65024, FeatureM65025_32r2]>;
def FeatureM650232      : SubtargetFeature<"mips32", "M6502ArchVersion", "M650232",
                                "M650232 ISA Support",
                                [FeatureM65022, FeatureM65023_32,
                                 FeatureM65024_32]>;
def FeatureM650232r2    : SubtargetFeature<"mips32r2", "M6502ArchVersion",
                                "M650232r2", "M650232r2 ISA Support",
                                [FeatureM65023_32r2, FeatureM65024_32r2,
                                 FeatureM65025_32r2, FeatureM650232]>;
def FeatureM650232r3    : SubtargetFeature<"mips32r3", "M6502ArchVersion",
                                "M650232r3", "M650232r3 ISA Support",
                                [FeatureM650232r2]>;
def FeatureM650232r5    : SubtargetFeature<"mips32r5", "M6502ArchVersion",
                                "M650232r5", "M650232r5 ISA Support",
                                [FeatureM650232r3]>;
def FeatureM650232r6    : SubtargetFeature<"mips32r6", "M6502ArchVersion",
                                "M650232r6",
                                "M650232r6 ISA Support [experimental]",
                                [FeatureM650232r5, FeatureFP64Bit,
                                 FeatureNaN2008]>;
def FeatureM650264      : SubtargetFeature<"mips64", "M6502ArchVersion",
                                "M650264", "M650264 ISA Support",
                                [FeatureM65025, FeatureM650232]>;
def FeatureM650264r2    : SubtargetFeature<"mips64r2", "M6502ArchVersion",
                                "M650264r2", "M650264r2 ISA Support",
                                [FeatureM650264, FeatureM650232r2]>;
def FeatureM650264r3    : SubtargetFeature<"mips64r3", "M6502ArchVersion",
                                "M650264r3", "M650264r3 ISA Support",
                                [FeatureM650264r2, FeatureM650232r3]>;
def FeatureM650264r5    : SubtargetFeature<"mips64r5", "M6502ArchVersion",
                                "M650264r5", "M650264r5 ISA Support",
                                [FeatureM650264r3, FeatureM650232r5]>;
def FeatureM650264r6    : SubtargetFeature<"mips64r6", "M6502ArchVersion",
                                "M650264r6",
                                "M650264r6 ISA Support [experimental]",
                                [FeatureM650232r6, FeatureM650264r5,
                                 FeatureNaN2008]>;
def FeatureSym32       : SubtargetFeature<"sym32", "HasSym32", "true",
                                          "Symbols are 32 bit on M650264">;

def FeatureM650216  : SubtargetFeature<"mips16", "InM650216Mode", "true",
                                      "M650216 mode">;

def FeatureDSP : SubtargetFeature<"dsp", "HasDSP", "true", "M6502 DSP ASE">;
def FeatureDSPR2 : SubtargetFeature<"dspr2", "HasDSPR2", "true",
                                    "M6502 DSP-R2 ASE", [FeatureDSP]>;
def FeatureDSPR3
    : SubtargetFeature<"dspr3", "HasDSPR3", "true", "M6502 DSP-R3 ASE",
                       [ FeatureDSP, FeatureDSPR2 ]>;

def FeatureMSA : SubtargetFeature<"msa", "HasMSA", "true", "M6502 MSA ASE">;

def FeatureEVA : SubtargetFeature<"eva", "HasEVA", "true", "M6502 EVA ASE">;

def FeatureMicroM6502  : SubtargetFeature<"micromips", "InMicroM6502Mode", "true",
                                         "microM6502 mode">;

def FeatureCnM6502     : SubtargetFeature<"cnmips", "HasCnM6502",
                                "true", "Octeon cnM6502 Support",
                                [FeatureM650264r2]>;

def FeatureUseTCCInDIV : SubtargetFeature<
                               "use-tcc-in-div",
                               "UseTCCInDIV", "false",
                               "Force the assembler to use trapping">;

def FeatureMadd4 : SubtargetFeature<"nomadd4", "DisableMadd4", "true",
                                    "Disable 4-operand madd.fmt and related instructions">;

def FeatureMT : SubtargetFeature<"mt", "HasMT", "true", "M6502 MT ASE">;

def FeatureLongCalls : SubtargetFeature<"long-calls", "UseLongCalls", "true",
                                        "Disable use of the jal instruction">;

//===----------------------------------------------------------------------===//
// M6502 processors supported.
//===----------------------------------------------------------------------===//

def ImplP5600 : SubtargetFeature<"p5600", "ProcImpl",
                                 "M6502Subtarget::CPU::P5600",
                                 "The P5600 Processor", [FeatureM650232r5]>;

class Proc<string Name, list<SubtargetFeature> Features>
 : ProcessorModel<Name, M6502GenericModel, Features>;

def : Proc<"mips1", [FeatureM65021]>;
def : Proc<"mips2", [FeatureM65022]>;
def : Proc<"mips32", [FeatureM650232]>;
def : Proc<"mips32r2", [FeatureM650232r2]>;
def : Proc<"mips32r3", [FeatureM650232r3]>;
def : Proc<"mips32r5", [FeatureM650232r5]>;
def : Proc<"mips32r6", [FeatureM650232r6]>;

def : Proc<"mips3", [FeatureM65023]>;
def : Proc<"mips4", [FeatureM65024]>;
def : Proc<"mips5", [FeatureM65025]>;
def : Proc<"mips64", [FeatureM650264]>;
def : Proc<"mips64r2", [FeatureM650264r2]>;
def : Proc<"mips64r3", [FeatureM650264r3]>;
def : Proc<"mips64r5", [FeatureM650264r5]>;
def : Proc<"mips64r6", [FeatureM650264r6]>;
def : Proc<"octeon", [FeatureM650264r2, FeatureCnM6502]>;
def : ProcessorModel<"p5600", M6502P5600Model, [ImplP5600]>;

def M6502AsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
}

def M6502AsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def M6502 : Target {
  let InstructionSet = M6502InstrInfo;
  let AssemblyParsers = [M6502AsmParser];
  let AssemblyParserVariants = [M6502AsmParserVariant];
}
