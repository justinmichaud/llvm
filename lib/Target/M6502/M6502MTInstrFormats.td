//===-- M6502MTInstrFormats.td - M6502 Instruction Formats ---*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Describe the M6502 MT instructions format
//
//  opcode - operation code.
//  rt     - destination register
//
//===----------------------------------------------------------------------===//

class M6502MTInst : M6502Inst<(outs), (ins), "", [], NoItinerary, FrmOther>,
                   PredicateControl {
  let DecoderNamespace = "M6502";
  let EncodingPredicates = [HasStdEnc];
}

class OPCODE1<bits<1> Val> {
  bits<1> Value = Val;
}

def OPCODE_SC_D : OPCODE1<0b0>;
def OPCODE_SC_E : OPCODE1<0b1>;

class FIELD5<bits<5> Val> {
  bits<5> Value = Val;
}

def FIELD5_1_DMT_EMT  : FIELD5<0b00001>;
def FIELD5_2_DMT_EMT  : FIELD5<0b01111>;
def FIELD5_1_2_DVPE_EVPE : FIELD5<0b00000>;

class COP0_MFMC0_MT<FIELD5 Op1, FIELD5 Op2, OPCODE1 sc> : M6502MTInst {
  bits<32> Inst;

  bits<5> rt;
  let Inst{31-26} = 0b010000; // COP0
  let Inst{25-21} = 0b01011;  // MFMC0
  let Inst{20-16} = rt;
  let Inst{15-11} = Op1.Value;
  let Inst{10-6}  = Op2.Value;
  let Inst{5}     = sc.Value;
  let Inst{4-3}   = 0b00;
  let Inst{2-0}   = 0b001;
}

class SPECIAL3_MT_FORK : M6502MTInst {
  bits<32> Inst;

  bits<5> rs;
  bits<5> rt;
  bits<5> rd;
  let Inst{31-26} = 0b011111; // SPECIAL3
  let Inst{25-21} = rs;
  let Inst{20-16} = rt;
  let Inst{15-11} = rd;
  let Inst{10-6}  = 0b00000;
  let Inst{5-0}   = 0b001000; // FORK
}

class SPECIAL3_MT_YIELD : M6502MTInst {
  bits<32> Inst;

  bits<5> rs;
  bits<5> rd;
  let Inst{31-26} = 0b011111; // SPECIAL3
  let Inst{25-21} = rs;
  let Inst{20-16} = 0b00000;
  let Inst{15-11} = rd;
  let Inst{10-6}  = 0b00000;
  let Inst{5-0}   = 0b001001; // FORK
}
