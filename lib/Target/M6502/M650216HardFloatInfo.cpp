//===---- M650216HardFloatInfo.cpp for M650216 Hard Float              -----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the M650216 implementation of M650216HardFloatInfo
// namespace.
//
//===----------------------------------------------------------------------===//

#include "M650216HardFloatInfo.h"
#include <string.h>

namespace llvm {

namespace M650216HardFloatInfo {

const FuncNameSignature PredefinedFuncs[] = {
  { "__floatdidf", { NoSig, DRet } },
  { "__floatdisf", { NoSig, FRet } },
  { "__floatundidf", { NoSig, DRet } },
  { "__fixsfdi", { FSig, NoFPRet } },
  { "__fixunsdfsi", { DSig, NoFPRet } },
  { "__fixunsdfdi", { DSig, NoFPRet } },
  { "__fixdfdi", { DSig, NoFPRet } },
  { "__fixunssfsi", { FSig, NoFPRet } },
  { "__fixunssfdi", { FSig, NoFPRet } },
  { "__floatundisf", { NoSig, FRet } },
  { nullptr, { NoSig, NoFPRet } }
};

// just do a search for now. there are very few of these special cases.
//
extern FuncSignature const *findFuncSignature(const char *name) {
  const char *name_;
  int i = 0;
  while (PredefinedFuncs[i].Name) {
    name_ = PredefinedFuncs[i].Name;
    if (strcmp(name, name_) == 0)
      return &PredefinedFuncs[i].Signature;
    i++;
  }
  return nullptr;
}
}
}
