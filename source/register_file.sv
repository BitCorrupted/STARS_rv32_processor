module register_file(
    input logic clk, rst,
    input logic [4:0] regA_address, regB_address, rd_address,
    input logic register_write_en,
    input logic [31:0] register_write_data,
    output logic [31:0] regA_data, regB_data

);

logic [31:0] registers_state [31:0];
logic [31:0] next_registers_state [31:0];

always_comb begin
    next_registers_state = registers_state;
    regA_data = next_registers_state[regA_address];
    regB_data = next_registers_state[regB_address];

    if (register_write_en) begin
        next_registers_state[rd_address] = register_write_data;
    end
end


always_ff @(posedge clk, negedge rst) begin
    if (!rst) begin
        for (integer i = 0; i < 32; i++) begin
            registers_state[i] <= 32'b0;
        end

        //registers_state <= '{'0};
    end

    else begin
        registers_state <= next_registers_state;
    end


end

endmodule