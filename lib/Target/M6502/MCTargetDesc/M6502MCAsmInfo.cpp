//===-- M6502MCAsmInfo.cpp - M6502 Asm Properties ---------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the M6502MCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "M6502MCAsmInfo.h"
#include "llvm/ADT/Triple.h"

using namespace llvm;

void M6502MCAsmInfo::anchor() { }

M6502MCAsmInfo::M6502MCAsmInfo(const Triple &TheTriple) {
  IsLittleEndian = TheTriple.isLittleEndian();

  PrivateGlobalPrefix = "$";
  PrivateLabelPrefix = "$";

  AlignmentIsInBytes          = false;
  Data16bitsDirective         = "\t.2byte\t";
  Data32bitsDirective         = "\t.4byte\t";
  Data64bitsDirective         = "\t.8byte\t";
  CommentString               = "#";
  ZeroDirective               = "\t.space\t";
  GPRel32Directive            = "\t.gpword\t";
  GPRel64Directive            = "\t.gpdword\t";
  DTPRel32Directive           = "\t.dtprelword\t";
  DTPRel64Directive           = "\t.dtpreldword\t";
  TPRel32Directive            = "\t.tprelword\t";
  TPRel64Directive            = "\t.tpreldword\t";
  UseAssignmentForEHBegin = true;
  SupportsDebugInformation = true;
  ExceptionsType = ExceptionHandling::DwarfCFI;
  DwarfRegNumForCFI = true;
  HasMipsExpressions = true;
}
