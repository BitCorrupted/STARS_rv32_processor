.global _boot
.text

_boot:                    /* x0  = 0    0x000 */
    /* Test ADDI */
    addi x0, x0, 0
    li x9, 1 /*value 1*/
    li x10, 8 /*setpoint*/
    li x13, 0 /*p_term*/
    li x14, 0 /*i_term*/
    li x15, 0 /*d_temp*/
    li x16, 0 /*d_term*/
    li x17, 2 /*k_p*/
    li x18, 0 /*k_i*/
    li x19, 1 /*k_d*/
    li x20, 0 /*pid val*/
    li x21, 240000 /*max pwm*/
    li x22, 0 /*output_reg*/
    
    beq x0, x0, main_loop
    
    main_loop:
    	lw x11, 0(x29) /*sensor data*/
        li x12, 0 /*msb*/
        beq x0, x0, get_msb
        
	get_msb:
        srl x11, x11, x9 /*shifts right 1*/
        addi x12, x12, 1 /*increments bit check*/
        beq x0, x11, pid /*if zero go to pid*/
        beq x0, x0, get_msb /*else keep shifting*/
        
	pid:
    	sub x13, x11, x10 /*error or p_term*/
        add x14, x14, x13 /*i_term*/
        sub x16, x15, x13 /*d_term*/
        li x15, 0
        add x15, x15, x13 /*d_temp becomes error*/
        
        /*calc PID value*/
        sll x13, x13, x17 /*k_p * P*/
        sll x14, x14, x18 /*k_i * I*/
        sll x16, x16, x19 /*k_d * D*/
        add x20, x13, x14 
        add x20, x20, x16 /*add to get PID val*/
        /*calc PID value*/
        
        /*calc pwm*/
        bge x20, x21, max_speed /*regulate if max*/
        add x13, x0, x20 /*dut right motor*/
        sw x20, 0(x30) /*pwm right motor*/
        sw x20, 0(x28) /*pwm left motor*/
        bge x13, x0, right_turn /*turn right*/
        li x22, 5 /* left turn enable*/
        sw x22, 0(x31) /*set enable output*/
        beq x0, x0, main_loop /*restart loop*/
        
	max_speed:
    	add x20, x0, x21 /*make pid max_speed*/
        add x13, x0, x20 /*dut right motor*/
        sw x20, 0(x30) /*pwm right motor*/
        sw x20, 0(x28) /*pwm left motor*/
        bge x13, x0, right_turn /*turn right*/
        li x22, 5 /* left turn enable*/
        sw x22, 0(x31) /*set enable output*/
        beq x0, x0, main_loop /*restart loop*/
        
        
 	right_turn:
    	li x22, 3 /* left turn enable*/
        sw x22, 0(x31) /*set enable output*/
        beq x0, x0, main_loop /*restart loop*/
        beq x0, x0, main_loop /*restart loop*/
    	
.data
variable:
	.word 0xdeadbeef
                    