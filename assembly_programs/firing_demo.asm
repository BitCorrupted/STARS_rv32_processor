.global _boot
.text

_boot:                    /* x0  = 0    0x000 */
    /* Test ADDI */
    addi x0, x0, 0
    
    beq x0, x0, pid_setup /*setup the pid*/
    
    pid_setup:
      li x9, 1 /*value 1*/
      li x10, 4 /*setpoint*/
      li x13, 0 /*p_term*/
      li x14, 0 /*i_term*/
      li x15, 0 /*d_temp*/
      li x16, 0 /*d_term*/
      li x17, 16 /*k_p*/
      li x18, 32 /*k_i*/
      li x19, 4 /*k_d*/
      li x20, 0 /*pid val*/
      li x21, 100000 /*max pwm*/
      li x22, 0 /*output_reg*/

      beq x0, x0, pid_loop
    
    pid_loop:
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
        add x14, x14, x13 /*i term*/
        sub x16, x15, x13 /*d_term*/
        li x15, 0
        add x15, x15, x13 /*d_temp becomes error*/
        
        add x24, x0, x13 /*error thingy*/
        
        sll x13, x13, x17 /*k_p * P*/
        sll x14, x14, x18 /*k_i * I*/
        sll x16, x16, x19 /*k_d * D*/
        add x20, x13, x14 
        add x20, x20, x16 /*add to get PID val*/
        
        beq x24, x0, fire_coil_gun /*start firing sequence if error is 0*/
        
        beq x12, x0, forward /*go forward*/
        sw x20, 0(x30) /*set left pwm*/
        sw x20, 0(x28) /*set right pwm*/
        
        /*5 forward, 10 backward, 9 right, 6 left*/
        bge x20, x0, turn_right /*turn right if negative*/
        li x22, 6 /*turn left value*/
        sw x22, 0(x31) /*turn left enable*/
    	beq x0, x0, pid_loop /*restart loop*/
        
        
	turn_right:
    	li x22, 9/*turn right value*/
        sw x22, 0(x31) /*turn right enable*/
    	beq x0, x0, pid_loop /*restart loop*/
        
	forward:
    	sw x21, 0(x30) /*set left pwm*/
        sw x21, 0(x28) /*set right pwm*/
    /*5 forward, 10 backward, 9 right, 6 left*/
        bge x20, x0, turn_right /*turn right if negative*/
        li x22, 5 /*turn left value*/
        sw x22, 0(x31) /*turn left enable*/
    	beq x0, x0, pid_loop /*restart loop*/
        
	fire_coil_gun:
    	sw x0, 0(x30) /*set left pwm*/
        sw x0, 0(x28) /*set right pwm*/
    	li x25, 256 /*coil gun signal*/
        sw x25, 0(x31) /*output signal*/
        li x25, 0 /*toggle switch*/
        sw x25, 0(x31) /*toggle mem*/
        beq x0,x0, do_nothing
       
	do_nothing:
    	nop
        beq x0, x0, do_nothing
    	
    	

.data
variable:
	.word 0xdeadbeef