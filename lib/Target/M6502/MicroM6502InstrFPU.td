//==- MicroM6502InstrFPU.td - microM6502 FPU Instruction Info -*- tablegen -*-==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the microM6502 FPU instruction set.
//
//===----------------------------------------------------------------------===//

let isCodeGenOnly = 1 in {
def FADD_S_MM : MMRel, ADDS_FT<"add.s", FGR32Opnd, II_ADD_S, 1, fadd>,
                ADDS_FM_MM<0, 0x30>, ISA_MICROM6502;
def FDIV_S_MM : MMRel, ADDS_FT<"div.s", FGR32Opnd, II_DIV_S, 0, fdiv>,
                ADDS_FM_MM<0, 0xf0>, ISA_MICROM6502;
def FMUL_S_MM : MMRel, ADDS_FT<"mul.s", FGR32Opnd, II_MUL_S, 1, fmul>,
                ADDS_FM_MM<0, 0xb0>, ISA_MICROM6502;
def FSUB_S_MM : MMRel, ADDS_FT<"sub.s", FGR32Opnd, II_SUB_S, 0, fsub>,
                ADDS_FM_MM<0, 0x70>, ISA_MICROM6502;

def FADD_MM  : MMRel, ADDS_FT<"add.d", AFGR64Opnd, II_ADD_D, 1, fadd>,
               ADDS_FM_MM<1, 0x30>, ISA_MICROM6502;
def FDIV_MM  : MMRel, ADDS_FT<"div.d", AFGR64Opnd, II_DIV_D, 0, fdiv>,
               ADDS_FM_MM<1, 0xf0>, ISA_MICROM6502;
def FMUL_MM  : MMRel, ADDS_FT<"mul.d", AFGR64Opnd, II_MUL_D, 1, fmul>,
               ADDS_FM_MM<1, 0xb0>, ISA_MICROM6502;
def FSUB_MM  : MMRel, ADDS_FT<"sub.d", AFGR64Opnd, II_SUB_D, 0, fsub>,
               ADDS_FM_MM<1, 0x70>, ISA_MICROM6502;

def LWXC1_MM : MMRel, LWXC1_FT<"lwxc1", FGR32Opnd, II_LWXC1, load>,
               LWXC1_FM_MM<0x48>, ISA_MICROM650232_NOT_M650232R6;
def SWXC1_MM : MMRel, SWXC1_FT<"swxc1", FGR32Opnd, II_SWXC1, store>,
               SWXC1_FM_MM<0x88>, ISA_MICROM650232_NOT_M650232R6;

// FIXME: These instruction definitions are incorrect. They should be 64-bit
//        FPU only.
def LUXC1_MM : MMRel, LWXC1_FT<"luxc1", AFGR64Opnd, II_LUXC1>,
               LWXC1_FM_MM<0x148>, ISA_MICROM650232_NOT_M650232R6;
def SUXC1_MM : MMRel, SWXC1_FT<"suxc1", AFGR64Opnd, II_SUXC1>,
               SWXC1_FM_MM<0x188>, ISA_MICROM650232_NOT_M650232R6;

def FCMP_S32_MM : MMRel, CEQS_FT<"s", FGR32, II_C_CC_S, M6502FPCmp>,
                  CEQS_FM_MM<0>, ISA_MICROM650232_NOT_M650232R6 {
  // FIXME: This is a required to work around the fact that these instructions
  //        only use $fcc0. Ideally, M6502FPCmp nodes could be removed and the
  //        fcc register set is used directly.
  bits<3> fcc = 0;
}

def FCMP_D32_MM : MMRel, CEQS_FT<"d", AFGR64, II_C_CC_D, M6502FPCmp>,
                  CEQS_FM_MM<1>, ISA_MICROM650232_NOT_M650232R6 {
  // FIXME: This is a required to work around the fact that these instructions
  //        only use $fcc0. Ideally, M6502FPCmp nodes could be removed and the
  //        fcc register set is used directly.
  bits<3> fcc = 0;
}

}

let DecoderNamespace = "MicroM6502" in {
  def BC1F_MM : MMRel, BC1F_FT<"bc1f", brtarget_mm, II_BC1F, M6502_BRANCH_F>,
                BC1F_FM_MM<0x1c>, ISA_MICROM650232_NOT_M650232R6;
  def BC1T_MM : MMRel, BC1F_FT<"bc1t", brtarget_mm, II_BC1T, M6502_BRANCH_T>,
                BC1F_FM_MM<0x1d>, ISA_MICROM650232_NOT_M650232R6;
}

let isCodeGenOnly = 1 in {
def CVT_W_S_MM   : MMRel, ABSS_FT<"cvt.w.s", FGR32Opnd, FGR32Opnd, II_CVT>,
                   ROUND_W_FM_MM<0, 0x24>, ISA_MICROM6502;
def ROUND_W_S_MM : MMRel, StdMMR6Rel, ABSS_FT<"round.w.s", FGR32Opnd, FGR32Opnd,
                                              II_ROUND>, ROUND_W_FM_MM<0, 0xec>,
                   ISA_MICROM6502;

def CEIL_W_MM  : MMRel, ABSS_FT<"ceil.w.d", FGR32Opnd, AFGR64Opnd, II_CEIL>,
                 ROUND_W_FM_MM<1, 0x6c>, ISA_MICROM6502, FGR_32;
def CVT_W_MM   : MMRel, ABSS_FT<"cvt.w.d", FGR32Opnd, AFGR64Opnd, II_CVT>,
                 ROUND_W_FM_MM<1, 0x24>, ISA_MICROM6502, FGR_32;
def FLOOR_W_MM : MMRel, ABSS_FT<"floor.w.d", FGR32Opnd, AFGR64Opnd, II_FLOOR>,
                 ROUND_W_FM_MM<1, 0x2c>, ISA_MICROM6502, FGR_32;
def ROUND_W_MM : MMRel, StdMMR6Rel, ABSS_FT<"round.w.d", FGR32Opnd, AFGR64Opnd,
                                            II_ROUND>, ROUND_W_FM_MM<1, 0xec>,
                 ISA_MICROM6502, FGR_32;
def TRUNC_W_MM : MMRel, ABSS_FT<"trunc.w.d", FGR32Opnd, AFGR64Opnd, II_TRUNC>,
                 ROUND_W_FM_MM<1, 0xac>, ISA_MICROM6502, FGR_32;

def FSQRT_MM : MMRel, ABSS_FT<"sqrt.d", AFGR64Opnd, AFGR64Opnd, II_SQRT_D,
                              fsqrt>, ROUND_W_FM_MM<1, 0x28>,
               ISA_MICROM6502, FGR_32;

def CVT_L_S_MM   : MMRel, ABSS_FT<"cvt.l.s", FGR64Opnd, FGR32Opnd, II_CVT>,
                   ROUND_W_FM_MM<0, 0x4>, ISA_MICROM6502, FGR_64;
def CVT_L_D64_MM : MMRel, ABSS_FT<"cvt.l.d", FGR64Opnd, FGR64Opnd, II_CVT>,
                   ROUND_W_FM_MM<1, 0x4>, ISA_MICROM6502, FGR_64;

}

let DecoderNamespace = "MicroM6502" in {
  def FABS_S_MM : MMRel, ABSS_FT<"abs.s", FGR32Opnd, FGR32Opnd, II_ABS, fabs>,
                  ABS_FM_MM<0, 0xd>, ISA_MICROM6502;
  def FABS_MM : MMRel, ABSS_FT<"abs.d", AFGR64Opnd, AFGR64Opnd, II_ABS, fabs>,
                ABS_FM_MM<1, 0xd>, ISA_MICROM6502, FGR_32;
}

let isCodeGenOnly = 1 in {
def FMOV_S_MM : MMRel, ABSS_FT<"mov.s", FGR32Opnd, FGR32Opnd, II_MOV_S>,
                ABS_FM_MM<0, 0x1>, ISA_MICROM6502;
def FNEG_S_MM : MMRel, ABSS_FT<"neg.s", FGR32Opnd, FGR32Opnd, II_NEG, fneg>,
                ABS_FM_MM<0, 0x2d>, ISA_MICROM6502;
def CVT_D_S_MM : MMRel, ABSS_FT<"cvt.d.s", AFGR64Opnd, FGR32Opnd, II_CVT>,
                 ABS_FM_MM<0, 0x4d>, ISA_MICROM6502, FGR_32;
def CVT_D32_W_MM : MMRel, ABSS_FT<"cvt.d.w", AFGR64Opnd, FGR32Opnd, II_CVT>,
                   ABS_FM_MM<1, 0x4d>, ISA_MICROM6502, FGR_32;
def CVT_S_D32_MM : MMRel, ABSS_FT<"cvt.s.d", FGR32Opnd, AFGR64Opnd, II_CVT>,
                   ABS_FM_MM<0, 0x6d>, ISA_MICROM6502, FGR_32;
def CVT_S_W_MM : MMRel, ABSS_FT<"cvt.s.w", FGR32Opnd, FGR32Opnd, II_CVT>,
                 ABS_FM_MM<1, 0x6d>, ISA_MICROM6502;

def FNEG_MM : MMRel, ABSS_FT<"neg.d", AFGR64Opnd, AFGR64Opnd, II_NEG, fneg>,
              ABS_FM_MM<1, 0x2d>, ISA_MICROM6502, FGR_32;

def FMOV_D32_MM : MMRel, ABSS_FT<"mov.d", AFGR64Opnd, AFGR64Opnd, II_MOV_D>,
                  ABS_FM_MM<1, 0x1>, ISA_MICROM6502, FGR_32;

def MOVZ_I_S_MM : MMRel, CMov_I_F_FT<"movz.s", GPR32Opnd, FGR32Opnd,
                                     II_MOVZ_S>, CMov_I_F_FM_MM<0x78, 0>,
                  ISA_MICROM650232_NOT_M650232R6;
def MOVN_I_S_MM : MMRel, CMov_I_F_FT<"movn.s", GPR32Opnd, FGR32Opnd,
                                     II_MOVN_S>, CMov_I_F_FM_MM<0x38, 0>,
                  ISA_MICROM650232_NOT_M650232R6;
def MOVZ_I_D32_MM : MMRel, CMov_I_F_FT<"movz.d", GPR32Opnd, AFGR64Opnd,
                                       II_MOVZ_D>, CMov_I_F_FM_MM<0x78, 1>,
                    ISA_MICROM650232_NOT_M650232R6, FGR_32;
def MOVN_I_D32_MM : MMRel, CMov_I_F_FT<"movn.d", GPR32Opnd, AFGR64Opnd,
                                       II_MOVN_D>, CMov_I_F_FM_MM<0x38, 1>,
                    ISA_MICROM650232_NOT_M650232R6, FGR_32;

def MOVT_S_MM : MMRel, CMov_F_F_FT<"movt.s", FGR32Opnd, II_MOVT_S,
                                   M6502CMovFP_T>, CMov_F_F_FM_MM<0x60, 0>,
                ISA_MICROM650232_NOT_M650232R6;
def MOVF_S_MM : MMRel, CMov_F_F_FT<"movf.s", FGR32Opnd, II_MOVF_S,
                                   M6502CMovFP_F>, CMov_F_F_FM_MM<0x20, 0>,
                ISA_MICROM650232_NOT_M650232R6;
def MOVT_D32_MM : MMRel, CMov_F_F_FT<"movt.d", AFGR64Opnd, II_MOVT_D,
                                     M6502CMovFP_T>, CMov_F_F_FM_MM<0x60, 1>,
                ISA_MICROM650232_NOT_M650232R6, FGR_32;
def MOVF_D32_MM : MMRel, CMov_F_F_FT<"movf.d", AFGR64Opnd, II_MOVF_D,
                                     M6502CMovFP_F>, CMov_F_F_FM_MM<0x20, 1>,
                  ISA_MICROM650232_NOT_M650232R6, FGR_32;
def MFC1_MM : MMRel, MFC1_FT<"mfc1", GPR32Opnd, FGR32Opnd,
                             II_MFC1, bitconvert>, MFC1_FM_MM<0x80>,
              ISA_MICROM6502;
def MTC1_MM : MMRel, MTC1_FT<"mtc1", FGR32Opnd, GPR32Opnd,
                             II_MTC1, bitconvert>, MFC1_FM_MM<0xa0>,
              ISA_MICROM6502;

def MADD_S_MM : MMRel, MADDS_FT<"madd.s", FGR32Opnd, II_MADD_S, fadd>,
                MADDS_FM_MM<0x1>, ISA_MICROM650232_NOT_M650232R6;
def MSUB_S_MM : MMRel, MADDS_FT<"msub.s", FGR32Opnd, II_MSUB_S, fsub>,
                MADDS_FM_MM<0x21>, ISA_MICROM650232_NOT_M650232R6;
def NMADD_S_MM : MMRel, NMADDS_FT<"nmadd.s", FGR32Opnd, II_NMADD_S, fadd>,
                 MADDS_FM_MM<0x2>, ISA_MICROM650232_NOT_M650232R6;
def NMSUB_S_MM : MMRel, NMADDS_FT<"nmsub.s", FGR32Opnd, II_NMSUB_S, fsub>,
                 MADDS_FM_MM<0x22>, ISA_MICROM650232_NOT_M650232R6;

def MADD_D32_MM  : MMRel, MADDS_FT<"madd.d", AFGR64Opnd, II_MADD_D, fadd>,
                   MADDS_FM_MM<0x9>, ISA_MICROM650232_NOT_M650232R6, FGR_32;
def MSUB_D32_MM  : MMRel, MADDS_FT<"msub.d", AFGR64Opnd, II_MSUB_D, fsub>,
                   MADDS_FM_MM<0x29>, ISA_MICROM650232_NOT_M650232R6, FGR_32;
def NMADD_D32_MM : MMRel, NMADDS_FT<"nmadd.d", AFGR64Opnd, II_NMADD_D, fadd>,
                   MADDS_FM_MM<0xa>, ISA_MICROM650232_NOT_M650232R6, FGR_32;
def NMSUB_D32_MM : MMRel, NMADDS_FT<"nmsub.d", AFGR64Opnd, II_NMSUB_D, fsub>,
                   MADDS_FM_MM<0x2a>, ISA_MICROM650232_NOT_M650232R6, FGR_32;
}

def FLOOR_W_S_MM : MMRel, ABSS_FT<"floor.w.s", FGR32Opnd, FGR32Opnd,
                                  II_FLOOR>, ROUND_W_FM_MM<0, 0x2c>,
                   ISA_MICROM6502;
def TRUNC_W_S_MM : MMRel, StdMMR6Rel, ABSS_FT<"trunc.w.s", FGR32Opnd,
                                              FGR32Opnd, II_TRUNC>,
                   ROUND_W_FM_MM<0, 0xac>, ISA_MICROM6502;
def CEIL_W_S_MM  : MMRel, ABSS_FT<"ceil.w.s", FGR32Opnd, FGR32Opnd, II_CEIL>,
                   ROUND_W_FM_MM<0, 0x6c>, ISA_MICROM6502;
def FSQRT_S_MM : MMRel, ABSS_FT<"sqrt.s", FGR32Opnd, FGR32Opnd, II_SQRT_S,
                                fsqrt>, ROUND_W_FM_MM<0, 0x28>, ISA_MICROM6502;
def MTHC1_MM : MMRel, MTC1_64_FT<"mthc1", AFGR64Opnd, GPR32Opnd, II_MTHC1>,
               MFC1_FM_MM<0xe0>, ISA_MICROM6502, FGR_32;
def MFHC1_MM : MMRel, MFC1_FT<"mfhc1", GPR32Opnd, AFGR64Opnd, II_MFHC1>,
               MFC1_FM_MM<0xc0>, ISA_MICROM6502, FGR_32;

let DecoderNamespace = "MicroM6502" in {
  def CFC1_MM : MMRel, MFC1_FT<"cfc1", GPR32Opnd, CCROpnd, II_CFC1>,
                MFC1_FM_MM<0x40>, ISA_MICROM6502;
  def CTC1_MM : MMRel, MTC1_FT<"ctc1", CCROpnd, GPR32Opnd, II_CTC1>,
                MFC1_FM_MM<0x60>, ISA_MICROM6502;
  def RECIP_S_MM : MMRel, ABSS_FT<"recip.s", FGR32Opnd, FGR32Opnd,
                                    II_RECIP_S>,
                     ROUND_W_FM_MM<0b0, 0b01001000>, ISA_MICROM6502;
  def RECIP_D32_MM : MMRel, ABSS_FT<"recip.d", AFGR64Opnd, AFGR64Opnd,
                                    II_RECIP_D>,
                     ROUND_W_FM_MM<0b1, 0b01001000>, ISA_MICROM6502, FGR_32 {
    let BaseOpcode = "RECIP_D32";
  }
  let DecoderNamespace = "MicroM6502FP64" in
    def RECIP_D64_MM : MMRel, ABSS_FT<"recip.d", FGR64Opnd, FGR64Opnd,
                                      II_RECIP_D>,
                     ROUND_W_FM_MM<0b1, 0b01001000>, ISA_MICROM6502, FGR_64;
  def RSQRT_S_MM : MMRel, ABSS_FT<"rsqrt.s", FGR32Opnd, FGR32Opnd,
                                  II_RECIP_S>,
                   ROUND_W_FM_MM<0b0, 0b00001000>;
  def RSQRT_D32_MM : MMRel, ABSS_FT<"rsqrt.d", AFGR64Opnd, AFGR64Opnd,
                                  II_RECIP_D>,
                   ROUND_W_FM_MM<0b1, 0b00001000>, ISA_MICROM6502, FGR_32 {
    let BaseOpcode = "RSQRT_D32";
  }
  let DecoderNamespace = "MicroM6502FP64" in
    def RSQRT_D64_MM : MMRel, ABSS_FT<"rsqrt.d", FGR64Opnd, FGR64Opnd,
                                      II_RECIP_D>,
                       ROUND_W_FM_MM<0b1, 0b00001000>, ISA_MICROM6502, FGR_64;
}

let DecoderNamespace = "MicroM6502",  DecoderMethod = "DecodeFMemMMR2" in {
  def LDC1_MM : MMRel, LW_FT<"ldc1", AFGR64Opnd, mem_mm_16, II_LDC1, load>,
                LW_FM_MM<0x2f>, ISA_MICROM6502, FGR_32 {
    let BaseOpcode = "LDC132";
  }
  def SDC1_MM : MMRel, SW_FT<"sdc1", AFGR64Opnd, mem_mm_16, II_SDC1, store>,
                LW_FM_MM<0x2e>, ISA_MICROM6502, FGR_32;
  def LWC1_MM : MMRel, LW_FT<"lwc1", FGR32Opnd, mem_mm_16, II_LWC1, load>,
                LW_FM_MM<0x27>, ISA_MICROM6502;
  def SWC1_MM : MMRel, SW_FT<"swc1", FGR32Opnd, mem_mm_16, II_SWC1, store>,
                LW_FM_MM<0x26>, ISA_MICROM6502;
}

multiclass C_COND_MM<string TypeStr, RegisterOperand RC, bits<2> fmt,
                     InstrItinClass itin> {
  def C_F_#NAME#_MM : MMRel, C_COND_FT<"f", TypeStr, RC, itin>,
                      C_COND_FM_MM<fmt, 0> {
    let BaseOpcode = "c.f."#NAME;
    let isCommutable = 1;
  }
  def C_UN_#NAME#_MM : MMRel, C_COND_FT<"un", TypeStr, RC, itin>,
                       C_COND_FM_MM<fmt, 1> {
    let BaseOpcode = "c.un."#NAME;
    let isCommutable = 1;
  }
  def C_EQ_#NAME#_MM : MMRel, C_COND_FT<"eq", TypeStr, RC, itin>,
                       C_COND_FM_MM<fmt, 2> {
    let BaseOpcode = "c.eq."#NAME;
    let isCommutable = 1;
  }
  def C_UEQ_#NAME#_MM : MMRel, C_COND_FT<"ueq", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 3> {
    let BaseOpcode = "c.ueq."#NAME;
    let isCommutable = 1;
  }
  def C_OLT_#NAME#_MM : MMRel, C_COND_FT<"olt", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 4> {
    let BaseOpcode = "c.olt."#NAME;
  }
  def C_ULT_#NAME#_MM : MMRel, C_COND_FT<"ult", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 5> {
    let BaseOpcode = "c.ult."#NAME;
  }
  def C_OLE_#NAME#_MM : MMRel, C_COND_FT<"ole", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 6> {
    let BaseOpcode = "c.ole."#NAME;
  }
  def C_ULE_#NAME#_MM : MMRel, C_COND_FT<"ule", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 7> {
    let BaseOpcode = "c.ule."#NAME;
  }
  def C_SF_#NAME#_MM : MMRel, C_COND_FT<"sf", TypeStr, RC, itin>,
                       C_COND_FM_MM<fmt, 8> {
    let BaseOpcode = "c.sf."#NAME;
    let isCommutable = 1;
  }
  def C_NGLE_#NAME#_MM : MMRel, C_COND_FT<"ngle", TypeStr, RC, itin>,
                         C_COND_FM_MM<fmt, 9> {
    let BaseOpcode = "c.ngle."#NAME;
  }
  def C_SEQ_#NAME#_MM : MMRel, C_COND_FT<"seq", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 10> {
    let BaseOpcode = "c.seq."#NAME;
    let isCommutable = 1;
  }
  def C_NGL_#NAME#_MM : MMRel, C_COND_FT<"ngl", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 11> {
    let BaseOpcode = "c.ngl."#NAME;
  }
  def C_LT_#NAME#_MM : MMRel, C_COND_FT<"lt", TypeStr, RC, itin>,
                       C_COND_FM_MM<fmt, 12> {
    let BaseOpcode = "c.lt."#NAME;
  }
  def C_NGE_#NAME#_MM : MMRel, C_COND_FT<"nge", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 13> {
    let BaseOpcode = "c.nge."#NAME;
  }
  def C_LE_#NAME#_MM : MMRel, C_COND_FT<"le", TypeStr, RC, itin>,
                       C_COND_FM_MM<fmt, 14> {
    let BaseOpcode = "c.le."#NAME;
  }
  def C_NGT_#NAME#_MM : MMRel, C_COND_FT<"ngt", TypeStr, RC, itin>,
                        C_COND_FM_MM<fmt, 15> {
    let BaseOpcode = "c.ngt."#NAME;
  }
}

defm S   : C_COND_MM<"s", FGR32Opnd, 0b00, II_C_CC_S>,
           ISA_MICROM650232_NOT_M650232R6;
defm D32 : C_COND_MM<"d", AFGR64Opnd, 0b01, II_C_CC_D>,
           ISA_MICROM650232_NOT_M650232R6, FGR_32;
let DecoderNamespace = "M650264" in
  defm D64 : C_COND_MM<"d", FGR64Opnd, 0b01, II_C_CC_D>,
             ISA_MICROM650232_NOT_M650232R6, FGR_64;

defm S_MM   : C_COND_ALIASES<"s", FGR32Opnd>, HARDFLOAT,
              ISA_MICROM650232_NOT_M650232R6;
defm D32_MM : C_COND_ALIASES<"d", AFGR64Opnd>, HARDFLOAT,
              ISA_MICROM650232_NOT_M650232R6, FGR_32;
defm D64_MM : C_COND_ALIASES<"d", FGR64Opnd>, HARDFLOAT,
              ISA_MICROM650232_NOT_M650232R6, FGR_64;

defm : BC1_ALIASES<BC1T_MM, "bc1t", BC1F_MM, "bc1f">,
       ISA_MICROM650232_NOT_M650232R6, HARDFLOAT;


// To generate NMADD and NMSUB instructions when fneg node is present
let AdditionalPredicates = [NoNaNsFPMath, HasMadd4,
                            InMicroM6502, NotM650232r6] in {
  defm : NMADD_NMSUB<NMADD_S_MM, NMSUB_S_MM, FGR32Opnd>,
         ISA_MICROM650232_NOT_M650232R6;
  defm : NMADD_NMSUB<NMADD_D32_MM, NMSUB_D32_MM, AFGR64Opnd>,
         ISA_MICROM650232_NOT_M650232R6, FGR_32;
}

//===----------------------------------------------------------------------===//
// Floating Point Patterns
//===----------------------------------------------------------------------===//

// Patterns for loads/stores with a reg+imm operand.
let AddedComplexity = 40 in {
  def : LoadRegImmPat<LDC1_MM, f64, load>, ISA_MICROM6502, FGR_32;
  def : StoreRegImmPat<SDC1_MM, f64>, ISA_MICROM6502, FGR_32;
  def : LoadRegImmPat<LWC1_MM, f32, load>, ISA_MICROM6502;
  def : StoreRegImmPat<SWC1_MM, f32>, ISA_MICROM6502;
}
