//===-- M6502ELFObjectWriter.cpp - M6502 ELF Writer -------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/M6502FixupKinds.h"
#include "MCTargetDesc/M6502MCTargetDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <list>
#include <utility>

#define DEBUG_TYPE "m6502-elf-object-writer"

using namespace llvm;

namespace {

/// Holds additional information needed by the relocation ordering algorithm.
struct M6502RelocationEntry {
  const ELFRelocationEntry R; ///< The relocation.
  bool Matched = false;       ///< Is this relocation part of a match.

  M6502RelocationEntry(const ELFRelocationEntry &R) : R(R) {}

  void print(raw_ostream &Out) const {
    R.print(Out);
    Out << ", Matched=" << Matched;
  }
};

#ifndef NDEBUG
raw_ostream &operator<<(raw_ostream &OS, const M6502RelocationEntry &RHS) {
  RHS.print(OS);
  return OS;
}
#endif

class M6502ELFObjectWriter : public MCELFObjectTargetWriter {
public:
  M6502ELFObjectWriter(uint8_t OSABI, bool HasRelocationAddend, bool Is64,
                      bool IsLittleEndian);

  ~M6502ELFObjectWriter() override = default;

  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override;
  void sortRelocs(const MCAssembler &Asm,
                  std::vector<ELFRelocationEntry> &Relocs) override;
};

/// The possible results of the Predicate function used by find_best.
enum FindBestPredicateResult {
  FindBest_NoMatch = 0,  ///< The current element is not a match.
  FindBest_Match,        ///< The current element is a match but better ones are
                         ///  possible.
  FindBest_PerfectMatch, ///< The current element is an unbeatable match.
};

} // end anonymous namespace

/// Copy elements in the range [First, Last) to d1 when the predicate is true or
/// d2 when the predicate is false. This is essentially both std::copy_if and
/// std::remove_copy_if combined into a single pass.
template <class InputIt, class OutputIt1, class OutputIt2, class UnaryPredicate>
static std::pair<OutputIt1, OutputIt2> copy_if_else(InputIt First, InputIt Last,
                                                    OutputIt1 d1, OutputIt2 d2,
                                                    UnaryPredicate Predicate) {
  for (InputIt I = First; I != Last; ++I) {
    if (Predicate(*I)) {
      *d1 = *I;
      d1++;
    } else {
      *d2 = *I;
      d2++;
    }
  }

  return std::make_pair(d1, d2);
}

/// Find the best match in the range [First, Last).
///
/// An element matches when Predicate(X) returns FindBest_Match or
/// FindBest_PerfectMatch. A value of FindBest_PerfectMatch also terminates
/// the search. BetterThan(A, B) is a comparator that returns true when A is a
/// better match than B. The return value is the position of the best match.
///
/// This is similar to std::find_if but finds the best of multiple possible
/// matches.
template <class InputIt, class UnaryPredicate, class Comparator>
static InputIt find_best(InputIt First, InputIt Last, UnaryPredicate Predicate,
                         Comparator BetterThan) {
  InputIt Best = Last;

  for (InputIt I = First; I != Last; ++I) {
    unsigned Matched = Predicate(*I);
    if (Matched != FindBest_NoMatch) {
      DEBUG(dbgs() << std::distance(First, I) << " is a match (";
            I->print(dbgs()); dbgs() << ")\n");
      if (Best == Last || BetterThan(*I, *Best)) {
        DEBUG(dbgs() << ".. and it beats the last one\n");
        Best = I;
      }
    }
    if (Matched == FindBest_PerfectMatch) {
      DEBUG(dbgs() << ".. and it is unbeatable\n");
      break;
    }
  }

  return Best;
}

/// Determine the low relocation that matches the given relocation.
/// If the relocation does not need a low relocation then the return value
/// is ELF::R_M6502_NONE.
static unsigned getMatchingLoType(const ELFRelocationEntry &Reloc) {
  return ELF::R_M6502_NONE;
}

/// Determine whether a relocation (X) matches the one given in R.
///
/// A relocation matches if:
/// - It's type matches that of a corresponding low part. This is provided in
///   MatchingType for efficiency.
/// - It's based on the same symbol.
/// - It's offset of greater or equal to that of the one given in R.
///   It should be noted that this rule assumes the programmer does not use
///   offsets that exceed the alignment of the symbol. The carry-bit will be
///   incorrect if this is not true.
///
/// A matching relocation is unbeatable if:
/// - It is not already involved in a match.
/// - It's offset is exactly that of the one given in R.
static FindBestPredicateResult isMatchingReloc(const M6502RelocationEntry &X,
                                               const ELFRelocationEntry &R,
                                               unsigned MatchingType) {
  if (X.R.Type == MatchingType && X.R.OriginalSymbol == R.OriginalSymbol) {
    if (!X.Matched &&
        X.R.OriginalAddend == R.OriginalAddend)
      return FindBest_PerfectMatch;
    else if (X.R.OriginalAddend >= R.OriginalAddend)
      return FindBest_Match;
  }
  return FindBest_NoMatch;
}

/// Determine whether Candidate or PreviousBest is the better match.
/// The return value is true if Candidate is the better match.
///
/// A matching relocation is a better match if:
/// - It has a smaller addend.
/// - It is not already involved in a match.
static bool compareMatchingRelocs(const M6502RelocationEntry &Candidate,
                                  const M6502RelocationEntry &PreviousBest) {
  if (Candidate.R.OriginalAddend != PreviousBest.R.OriginalAddend)
    return Candidate.R.OriginalAddend < PreviousBest.R.OriginalAddend;
  return PreviousBest.Matched && !Candidate.Matched;
}

#ifndef NDEBUG
/// Print all the relocations.
template <class Container>
static void dumpRelocs(const char *Prefix, const Container &Relocs) {
  for (const auto &R : Relocs)
    dbgs() << Prefix << R << "\n";
}
#endif

M6502ELFObjectWriter::M6502ELFObjectWriter(uint8_t OSABI,
                                         bool HasRelocationAddend, bool Is64,
                                         bool IsLittleEndian)
    : MCELFObjectTargetWriter(Is64, OSABI, ELF::EM_M6502, HasRelocationAddend) {}

unsigned M6502ELFObjectWriter::getRelocType(MCContext &Ctx,
                                           const MCValue &Target,
                                           const MCFixup &Fixup,
                                           bool IsPCRel) const {
  return ELF::R_M6502_NONE;
}

/// Sort relocation table entries by offset except where another order is
/// required by the M6502 ABI.
///
/// M6502 has a few relocations that have an AHL component in the expression used
/// to evaluate them. This AHL component is an addend with the same number of
/// bits as a symbol value but not all of our ABI's are able to supply a
/// sufficiently sized addend in a single relocation.
///
/// The O32 ABI for example, uses REL relocations which store the addend in the
/// section data. All the relocations with AHL components affect 16-bit fields
/// so the addend for a single relocation is limited to 16-bit. This ABI
/// resolves the limitation by linking relocations (e.g. R_M6502_HI16 and
/// R_M6502_LO16) and distributing the addend between the linked relocations. The
/// ABI mandates that such relocations must be next to each other in a
/// particular order (e.g. R_M6502_HI16 must be immediately followed by a
/// matching R_M6502_LO16) but the rule is less strict in practice.
///
/// The de facto standard is lenient in the following ways:
/// - 'Immediately following' does not refer to the next relocation entry but
///   the next matching relocation.
/// - There may be multiple high parts relocations for one low part relocation.
/// - There may be multiple low part relocations for one high part relocation.
/// - The AHL addend in each part does not have to be exactly equal as long as
///   the difference does not affect the carry bit from bit 15 into 16. This is
///   to allow, for example, the use of %lo(foo) and %lo(foo+4) when loading
///   both halves of a long long.
///
/// See getMatchingLoType() for a description of which high part relocations
/// match which low part relocations. One particular thing to note is that
/// R_M6502_GOT16 and similar only have AHL addends if they refer to local
/// symbols.
///
/// It should also be noted that this function is not affected by whether
/// the symbol was kept or rewritten into a section-relative equivalent. We
/// always match using the expressions from the source.
void M6502ELFObjectWriter::sortRelocs(const MCAssembler &Asm,
                                     std::vector<ELFRelocationEntry> &Relocs) {
  // We do not need to sort the relocation table for RELA relocations which
  // N32/N64 uses as the relocation addend contains the value we require,
  // rather than it being split across a pair of relocations.
  if (hasRelocationAddend())
    return;

  if (Relocs.size() < 2)
    return;

  // Sort relocations by the address they are applied to.
  std::sort(Relocs.begin(), Relocs.end(),
            [](const ELFRelocationEntry &A, const ELFRelocationEntry &B) {
              return A.Offset < B.Offset;
            });

  std::list<M6502RelocationEntry> Sorted;
  std::list<ELFRelocationEntry> Remainder;

  DEBUG(dumpRelocs("R: ", Relocs));

  // Separate the movable relocations (AHL relocations using the high bits) from
  // the immobile relocations (everything else). This does not preserve high/low
  // matches that already existed in the input.
  copy_if_else(Relocs.begin(), Relocs.end(), std::back_inserter(Remainder),
               std::back_inserter(Sorted), [](const ELFRelocationEntry &Reloc) {
                 return getMatchingLoType(Reloc) != ELF::R_M6502_NONE;
               });

  for (auto &R : Remainder) {
    DEBUG(dbgs() << "Matching: " << R << "\n");

    unsigned MatchingType = getMatchingLoType(R);
    assert(MatchingType != ELF::R_M6502_NONE &&
           "Wrong list for reloc that doesn't need a match");

    // Find the best matching relocation for the current high part.
    // See isMatchingReloc for a description of a matching relocation and
    // compareMatchingRelocs for a description of what 'best' means.
    auto InsertionPoint =
        find_best(Sorted.begin(), Sorted.end(),
                  [&R, &MatchingType](const M6502RelocationEntry &X) {
                    return isMatchingReloc(X, R, MatchingType);
                  },
                  compareMatchingRelocs);

    // If we matched then insert the high part in front of the match and mark
    // both relocations as being involved in a match. We only mark the high
    // part for cosmetic reasons in the debug output.
    //
    // If we failed to find a match then the high part is orphaned. This is not
    // permitted since the relocation cannot be evaluated without knowing the
    // carry-in. We can sometimes handle this using a matching low part that is
    // already used in a match but we already cover that case in
    // isMatchingReloc and compareMatchingRelocs. For the remaining cases we
    // should insert the high part at the end of the list. This will cause the
    // linker to fail but the alternative is to cause the linker to bind the
    // high part to a semi-matching low part and silently calculate the wrong
    // value. Unfortunately we have no means to warn the user that we did this
    // so leave it up to the linker to complain about it.
    if (InsertionPoint != Sorted.end())
      InsertionPoint->Matched = true;
    Sorted.insert(InsertionPoint, R)->Matched = true;
  }

  DEBUG(dumpRelocs("S: ", Sorted));

  assert(Relocs.size() == Sorted.size() && "Some relocs were not consumed");

  // Overwrite the original vector with the sorted elements. The caller expects
  // them in reverse order.
  unsigned CopyTo = 0;
  for (const auto &R : reverse(Sorted))
    Relocs[CopyTo++] = R.R;
}

bool M6502ELFObjectWriter::needsRelocateWithSymbol(const MCSymbol &Sym,
                                                  unsigned Type) const {
  // If it's a compound relocation for N64 then we need the relocation if any
  // sub-relocation needs it.
  if (!isUInt<8>(Type))
    return needsRelocateWithSymbol(Sym, Type & 0xff) ||
           needsRelocateWithSymbol(Sym, (Type >> 8) & 0xff) ||
           needsRelocateWithSymbol(Sym, (Type >> 16) & 0xff);

  switch (Type) {
  default:
    errs() << Type << "\n";
    llvm_unreachable("Unexpected relocation");
    return true;

  // This relocation doesn't affect the section data.
  case ELF::R_M6502_NONE:
    return false;
  }
}

std::unique_ptr<MCObjectWriter>
llvm::createM6502ELFObjectWriter(raw_pwrite_stream &OS, const Triple &TT,
                                bool IsN32) {
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TT.getOS());
  bool IsN64 = TT.isArch64Bit() && !IsN32;
  bool HasRelocationAddend = TT.isArch64Bit();
  auto MOTW = llvm::make_unique<M6502ELFObjectWriter>(
      OSABI, HasRelocationAddend, IsN64, TT.isLittleEndian());
  return createELFObjectWriter(std::move(MOTW), OS, TT.isLittleEndian());
}
