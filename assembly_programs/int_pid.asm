.global _boot
.text

_boot:                    /* x0  = 0    0x000 */
    /* Test ADDI */
    addi x0, x0, 0
    li x9, 1 /*value 1*/
    li x10, 4 /*setpoint*/
    li x13, 0 /*p_term*/
    li x14, 0 /*i_term*/
    li x15, 0 /*d_temp*/
    li x16, 0 /*d_term*/
    li x17, 15 /*k_p*/
    li x18, 0 /*k_i*/
    li x19, 0 /*k_d*/
    li x20, 0 /*pid val*/
    li x21, 240000 /*max pwm*/
    li x23,240000 /*min pwm*/
    li x22, 0 /*output_reg*/
    
    beq x0, x0, main_loop
    
    main_loop:
    	lw x11, 0(x29) /*sensor data*/
        /*add x8, x0, x11*/ /*display input*/
        li x12, 0 /*msb*/
        beq x0, x0, get_msb /*generate msb*/
        
	get_msb:
    	beq x0, x11, pid /*if zero go to pid*/
        srl x11, x11, x9 /*shifts right 1*/
        addi x12, x12, 1 /*increments bit check*/
        beq x0, x0, get_msb /*else keep shifting*/
        
	pid:
    	add x8, x0, x12 /*see shifted num should be 0*/
        
        sub x13, x12, x10 /*error or p_term*/
        sll x20, x13, x17 /*k_p * P*/
        
        bge x20, x21, max_logic /*set max pwm*/
        blt x20, x23, min_logic /*set min pwm*/
        
        sw x20, 0(x30) /*set left pwm*/
        sw x20, 0(x28) /*set right pwm*/
        
        /*5 forward, 10 backward, 9 right, 6 left*/
        bge x20, x0, turn_right /*turn right if negative*/
        li x22, 6 /*turn left value*/
        sw x22, 0(x31) /*turn left enable*/
    	beq x0, x0, main_loop /*restart loop*/
    
    max_logic:
    	add x20, x0, x21 /*set pwm to max*/
        sw x20, 0(x30) /*set left pwm*/
        sw x20, 0(x28) /*set right pwm*/
        
        /*5 forward, 10 backward, 9 right, 6 left*/
        bge x20, x0, turn_right /*turn right if negative*/
        li x22, 6 /*turn left value*/
        sw x22, 0(x31) /*turn left enable*/
    	beq x0, x0, main_loop /*restart loop*/
        
    min_logic:
    	add x20, x0, x23 /*set pwm to min*/
        sw x20, 0(x30) /*set left pwm*/
        sw x20, 0(x28) /*set right pwm*/
        
        /*5 forward, 10 backward, 9 right, 6 left*/
        bge x20, x0, turn_right /*turn right if negative*/
        li x22, 6 /*turn left value*/
        sw x22, 0(x31) /*turn left enable*/
    	beq x0, x0, main_loop /*restart loop*/
        
        
	turn_right:
    	li x22, 9/*turn right value*/
        sw x22, 0(x31) /*turn right enable*/
    	beq x0, x0, main_loop /*restart loop*/
    	

.data
variable:
	.word 0xdeadbeef