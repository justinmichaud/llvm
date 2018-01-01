build:
	~/llvm/build/bin/clang -target m6502-unknown-unknown -c main.c -emit-llvm -o main.bc
	~/llvm/build/bin/llc -mtriple=m6502-unknown-unknown -relocation-model=static -filetype=asm main.bc -o main.s

clean:
	rm -f *.bc *.s
