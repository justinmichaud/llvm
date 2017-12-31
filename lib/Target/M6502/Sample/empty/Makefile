build:
	~/llvm/build/bin/clang -target mips-unknown-unknown -c main.c -emit-llvm -o main.bc
	~/llvm/build/bin/llc -march=m6502 -relocation-model=static -filetype=asm main.bc -o main.s

clean:
	rm -f *.bc *.s
