//===----------------------------------------------------------------------===//
//
// This file describes the Simple instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "SimpleInstrFormats.td"

// Not very complicated instruction set
def add_ri : SimpleInstr<(outs GPR32:$dst), (ins GPR32:$b, i32imm:$c),
                         "Hello Mr. Processor, please add two values $b and $c and put result into $dst",
                         [(set GPR32:$dst, (add GPR32:$b, imm:$c))]>;

//def SimpleRet : SDNode<"SimpleISD::RET", SDTBrind,
//                       [SDNPHasChain, SDNPOptInGlue, SDNPMayLoad, SDNPVariadic]>;

//
// Call frame magic
//

// These are target-independent nodes, but have target-specific formats.
def SDT_SimpleCallSeqStart : SDCallSeqStart<[ SDTCisVT<0, i32> ]>;
def SDT_SimpleCallSeqEnd   : SDCallSeqEnd<[ SDTCisVT<0, i32>,
                                         SDTCisVT<1, i32> ]>;

def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_SimpleCallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;
def callseq_end   : SDNode<"ISD::CALLSEQ_END",   SDT_SimpleCallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def ADJCALLSTACKDOWN : SimplePseudoInstr<(outs), (ins i32imm:$amt),
                               "# ADJCALLSTACKDOWN $amt",
                               [(callseq_start timm:$amt)]>;
def ADJCALLSTACKUP : SimplePseudoInstr<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "# ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;
