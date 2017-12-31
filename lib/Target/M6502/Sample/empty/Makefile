build:
	~/llvm/build/bin/clang -target mips-unknown-linux-gnu -c main.c -emit-llvm -o main.bc
	~/llvm/build/bin/llc -march=m6502 -relocation-model=pic -filetype=asm main.bc -o main.s

clean:
	rm -f *.bc *.s
