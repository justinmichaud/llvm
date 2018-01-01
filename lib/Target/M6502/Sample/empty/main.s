	.text
	.abicalls
	.option	pic0
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
	addiu	$1, $zero, 0
	sb	$zero, 9($fp)
	sb	$zero, 8($fp)
	sb	$zero, 4($fp)
	addiu	$1, $zero, 5
	sb	$1, 5($fp)
	lbu	$1, 4($fp)
	sll	$1, $1, 8
	lbu	$2, 5($fp)
	or	$1, $1, $2
	addiu	$1, $1, 1
	srl	$2, $1, 8
	sb	$2, 4($fp)
	sb	$1, 5($fp)
	lbu	$1, 4($fp)
	sll	$1, $1, 8
	lbu	$2, 5($fp)
	or	$2, $1, $2
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

	.ident	"clang version 6.0.0 (https://github.com/llvm-mirror/clang.git 5a52b9e6b0480ced82bf0bccbddd0f1d7804752d) (https://github.com/igor-laevsky/llvm-simple-backend.git 3b4fb32f1245a0cc3357b695299d1ac123c22794)"
	.section	".note.GNU-stack","",@progbits
	.text
