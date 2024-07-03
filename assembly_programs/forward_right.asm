.global _boot
.text

_boot:                    /* x0  = 0    0x000 */
    /* Test ADDI */
    addi x0, x0, 0
    li x10, 0
    li x9, 1
	
    beq x0, x0, check_sense
    
    check_sense:
    	lw x8, 0(x29)
        beq x8, x10, forward
        beq x8, x9, turn_right
        beq x0, x0, check_sense
        
    forward:
    	li x7, 230000
        li x13, 3
        sw x7, 0(x30)
        sw x13, 0(x31)
        sw x7, 0(x28)
        beq x0, x0, check_sense
        
   	motor_off:
    	li x7, 230000
        li x13, 0
        sw x7, 0(x30)
        sw x13, 0(x31)
        sw x7, 0(x28)
        beq x0, x0, check_sense
        
   	turn_right:
    	li x7, 230000
        li x6, 0
        li x13, 3
        sw x7, 0(x30)
        sw x13, 0(x31)
        sw x6, 0(x28)
        beq x0, x0, check_sense
    	
    
.data
variable:
	.word 0xdeadbeef