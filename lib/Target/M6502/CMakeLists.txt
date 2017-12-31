set(LLVM_TARGET_DEFINITIONS M6502.td)

tablegen(LLVM M6502GenRegisterInfo.inc -gen-register-info)
tablegen(LLVM M6502GenInstrInfo.inc -gen-instr-info)
tablegen(LLVM M6502GenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM M6502GenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM M6502GenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM M6502GenDAGISel.inc -gen-dag-isel)
tablegen(LLVM M6502GenFastISel.inc -gen-fast-isel)
tablegen(LLVM M6502GenCallingConv.inc -gen-callingconv)
tablegen(LLVM M6502GenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM M6502GenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM M6502GenMCPseudoLowering.inc -gen-pseudo-lowering)
add_public_tablegen_target(M6502CommonTableGen)

add_llvm_target(M6502CodeGen
  M650216FrameLowering.cpp
  M650216HardFloat.cpp
  M650216HardFloatInfo.cpp
  M650216InstrInfo.cpp
  M650216ISelDAGToDAG.cpp
  M650216ISelLowering.cpp
  M650216RegisterInfo.cpp
  M6502AnalyzeImmediate.cpp
  M6502AsmPrinter.cpp
  M6502CCState.cpp
  M6502ConstantIslandPass.cpp
  M6502DelaySlotFiller.cpp
  M6502FastISel.cpp
  M6502HazardSchedule.cpp
  M6502InstrInfo.cpp
  M6502ISelDAGToDAG.cpp
  M6502ISelLowering.cpp
  M6502FrameLowering.cpp
  M6502LongBranch.cpp
  M6502MCInstLower.cpp
  M6502MachineFunction.cpp
  M6502ModuleISelDAGToDAG.cpp
  M6502OptimizePICCall.cpp
  M6502Os16.cpp
  M6502RegisterInfo.cpp
  M6502SEFrameLowering.cpp
  M6502SEInstrInfo.cpp
  M6502SEISelDAGToDAG.cpp
  M6502SEISelLowering.cpp
  M6502SERegisterInfo.cpp
  M6502Subtarget.cpp
  M6502TargetMachine.cpp
  M6502TargetObjectFile.cpp
  MicroM6502SizeReduction.cpp
  )

add_subdirectory(InstPrinter)
add_subdirectory(Disassembler)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
add_subdirectory(AsmParser)

