module memory_module (
    input logic clk, rst,
    input logic [15:0] data_address, instruction_address,
    input logic dm_read_en, dm_write_en,
    input logic [31:0] data_to_write,
    output logic [31:0] instruction_read, data_read
);

logic [15:0][8191:0] mem;
initial $readmemh("cpu.mem", mem, 0, 4095);

always_ff @(posedge clk) begin

    if (dm_write_en && data_address) begin
        mem[data_address] <= data_to_write;
    end

end

always_ff @(posedge clk, negedge rst) begin
    if (!rst) begin
        data_read <= '0;
        instruction_read =< mem[16'b0];
    end

    else if (dm_read_en && data_address) begin
        data_read <= mem[data_address];

    end
    
    else if (!dm_read_en && data_address) begin
        instruction_read <= mem[instruction_address];

    end

    else begin
        instruction_read <= 32'b00000000000000000000000000010011;
        data_read <= '0;

    end
end



endmodule