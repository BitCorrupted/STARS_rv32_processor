.global _boot
.text

_boot:                    /* x0  = 0    0x000 */
    /* Test ADDI */
    addi x0, x0, 0
    addi x8, x0, 1
    li x11, 6000
    li x12, 200000

    
  loop1:
  
  	addi x10, x10, 1
    beq x10, x11, rst1
    beq x0, x0, loop1
  
  rst1:
  
  	addi x10, x0, 0
    addi x8, x0, 0
    beq x0, x0, loop3
  
  loop3:
  	addi x10, x10, 1
  	beq x10, x12, rst2
    beq x0, x0, loop3
 
 rst2:
 	addi x10, x0, 0
    addi x8, x0, 1
    beq x0, x0, loop1
    
  
    
.data
variable:
	.word 0xdeadbeef