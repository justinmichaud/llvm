//===-- llvm-rc.cpp - Compile .rc scripts into .res -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Compile .rc scripts into .res files. This is intended to be a
// platform-independent port of Microsoft's rc.exe tool.
//
//===----------------------------------------------------------------------===//

#include "ResourceFileWriter.h"
#include "ResourceScriptParser.h"
#include "ResourceScriptStmt.h"
#include "ResourceScriptToken.h"

#include "llvm/Option/Arg.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/PrettyStackTrace.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/Signals.h"
#include "llvm/Support/raw_ostream.h"

#include <system_error>

using namespace llvm;
using namespace llvm::rc;

namespace {

// Input options tables.

enum ID {
  OPT_INVALID = 0, // This is not a correct option ID.
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM,  \
               HELPTEXT, METAVAR, VALUES)                                      \
  OPT_##ID,
#include "Opts.inc"
#undef OPTION
};

#define PREFIX(NAME, VALUE) const char *const NAME[] = VALUE;
#include "Opts.inc"
#undef PREFIX

static const opt::OptTable::Info InfoTable[] = {
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM,  \
               HELPTEXT, METAVAR, VALUES)                                      \
  {                                                                            \
      PREFIX,      NAME,      HELPTEXT,                                        \
      METAVAR,     OPT_##ID,  opt::Option::KIND##Class,                        \
      PARAM,       FLAGS,     OPT_##GROUP,                                     \
      OPT_##ALIAS, ALIASARGS, VALUES},
#include "Opts.inc"
#undef OPTION
};

class RcOptTable : public opt::OptTable {
public:
  RcOptTable() : OptTable(InfoTable, /* IgnoreCase = */ true) {}
};

static ExitOnError ExitOnErr;

LLVM_ATTRIBUTE_NORETURN static void fatalError(const Twine &Message) {
  errs() << Message << "\n";
  exit(1);
}

} // anonymous namespace

int main(int argc_, const char *argv_[]) {
  sys::PrintStackTraceOnErrorSignal(argv_[0]);
  PrettyStackTraceProgram X(argc_, argv_);

  ExitOnErr.setBanner("llvm-rc: ");

  SmallVector<const char *, 256> argv;
  SpecificBumpPtrAllocator<char> ArgAllocator;
  ExitOnErr(errorCodeToError(sys::Process::GetArgumentVector(
      argv, makeArrayRef(argv_, argc_), ArgAllocator)));

  llvm_shutdown_obj Y;

  RcOptTable T;
  unsigned MAI, MAC;
  ArrayRef<const char *> ArgsArr = makeArrayRef(argv_ + 1, argc_);
  opt::InputArgList InputArgs = T.ParseArgs(ArgsArr, MAI, MAC);

  // The tool prints nothing when invoked with no command-line arguments.
  if (InputArgs.hasArg(OPT_HELP)) {
    T.PrintHelp(outs(), "rc", "Resource Converter", false);
    return 0;
  }

  const bool BeVerbose = InputArgs.hasArg(OPT_VERBOSE);

  std::vector<std::string> InArgsInfo = InputArgs.getAllArgValues(OPT_INPUT);
  if (InArgsInfo.size() != 1) {
    fatalError("Exactly one input file should be provided.");
  }

  // Read and tokenize the input file.
  ErrorOr<std::unique_ptr<MemoryBuffer>> File =
      MemoryBuffer::getFile(InArgsInfo[0]);
  if (!File) {
    fatalError("Error opening file '" + Twine(InArgsInfo[0]) +
               "': " + File.getError().message());
  }

  std::unique_ptr<MemoryBuffer> FileContents = std::move(*File);
  StringRef Contents = FileContents->getBuffer();

  std::vector<RCToken> Tokens = ExitOnErr(tokenizeRC(Contents));

  if (BeVerbose) {
    const Twine TokenNames[] = {
#define TOKEN(Name) #Name,
#define SHORT_TOKEN(Name, Ch) #Name,
#include "ResourceScriptTokenList.h"
#undef TOKEN
#undef SHORT_TOKEN
    };

    for (const RCToken &Token : Tokens) {
      outs() << TokenNames[static_cast<int>(Token.kind())] << ": "
             << Token.value();
      if (Token.kind() == RCToken::Kind::Int)
        outs() << "; int value = " << Token.intValue();

      outs() << "\n";
    }
  }

  SearchParams Params;
  SmallString<128> InputFile(InArgsInfo[0]);
  llvm::sys::fs::make_absolute(InputFile);
  Params.InputFilePath = InputFile;
  Params.Include = InputArgs.getAllArgValues(OPT_INCLUDE);
  Params.NoInclude = InputArgs.getAllArgValues(OPT_NOINCLUDE);

  std::unique_ptr<ResourceFileWriter> Visitor;
  bool IsDryRun = InputArgs.hasArg(OPT_DRY_RUN);

  if (!IsDryRun) {
    auto OutArgsInfo = InputArgs.getAllArgValues(OPT_FILEOUT);
    if (OutArgsInfo.size() != 1)
      fatalError(
          "Exactly one output file should be provided (using /FO flag).");

    std::error_code EC;
    auto FOut =
        llvm::make_unique<raw_fd_ostream>(OutArgsInfo[0], EC, sys::fs::F_RW);
    if (EC)
      fatalError("Error opening output file '" + OutArgsInfo[0] +
                 "': " + EC.message());
    Visitor = llvm::make_unique<ResourceFileWriter>(Params, std::move(FOut));
    Visitor->AppendNull = InputArgs.hasArg(OPT_ADD_NULL);

    ExitOnErr(NullResource().visit(Visitor.get()));

    // Set the default language; choose en-US arbitrarily.
    ExitOnErr(LanguageResource(0x09, 0x01).visit(Visitor.get()));
  }

  rc::RCParser Parser{std::move(Tokens)};
  while (!Parser.isEof()) {
    auto Resource = ExitOnErr(Parser.parseSingleResource());
    if (BeVerbose)
      Resource->log(outs());
    if (!IsDryRun)
      ExitOnErr(Resource->visit(Visitor.get()));
  }

  // STRINGTABLE resources come at the very end.
  if (!IsDryRun)
    ExitOnErr(Visitor->dumpAllStringTables());

  return 0;
}
