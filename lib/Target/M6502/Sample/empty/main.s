	.text
	.abicalls
	.section	.mdebug.abi32,"",@progbits
	.nan	legacy
	.file	"main.c"
	.text
	.globl	main                    # -- Begin function main
	.p2align	2
	.type	main,@function
	.set	nomicrom6502
	.set	nom650216
	.ent	main
main:                                   # @main
	.frame	$fp,16,$ra
	.mask 	0x40000000,-4
	.fmask	0x00000000,0
	.set	noreorder
	.set	nomacro
	.set	noat
# BB#0:                                 # %entry
	addiu	$sp, $sp, -16
	sw	$fp, 12($sp)            # 4-byte Folded Spill
	move	 $fp, $sp
	addiu	$1, $zero, 1
	addiu	$2, $zero, 5
	addiu	$3, $zero, 0
	sw	$zero, 8($fp)
	sw	$2, 4($fp)
	lw	$2, 4($fp)
	addu	$1, $2, $1
	sw	$1, 4($fp)
	lw	$2, 4($fp)
	move	 $sp, $fp
	lw	$fp, 12($sp)            # 4-byte Folded Reload
	jr	$ra
	addiu	$sp, $sp, 16
	.set	at
	.set	macro
	.set	reorder
	.end	main
$func_end0:
	.size	main, ($func_end0)-main
                                        # -- End function

	.ident	"clang version 6.0.0 (https://github.com/llvm-mirror/clang.git 5a52b9e6b0480ced82bf0bccbddd0f1d7804752d) (git@github.com:justinmichaud/llvm.git 77b0a61234e502f353d3f7e14cf20f63eaa1b855)"
	.section	".note.GNU-stack","",@progbits
	.text
