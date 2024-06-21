module writeback(
    input [31:0] memory_value,
    input [31:0] ALU_value,
    input [31:0] pc_4_value,
    input mem_to_reg,
    input load_byte,
    input read_pc_4,
    output register_value
);

//logic [31:0] register_value;

always_comb begin
    if(read_pc_4)
        register_value = pc_4_value;
    else if(~mem_to_reg)
        register_value = ALU_value;
    else if(load_byte)
        register_value = {24'b0,memory_value[7:0]};
    else
        register_value = memory_value;
end

//assign register_write = register_value;

endmodule