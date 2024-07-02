module IO_mod(
    input logic clk, rst,
    input logic write_mem, read_mem,
    input logic [31:0] data_from_mem,
    input logic [31:0] data_address, data_to_write,
    output logic [31:0] data_read,
    output logic [31:0] IO_out,
    input logic [31:0] IO_in

);
    logic [31:0] enable_reg, output_reg, input_reg;
    logic [31:0] next_enable_reg, next_output_reg, next_input_reg;

    always_comb begin
        next_enable_reg = enable_reg;
        next_output_reg = output_reg;
        next_input_reg = input_reg;

        if (write_mem) begin
            if (data_address == 32'hFFFFFFFF) begin
                next_enable_reg = data_to_write;
                data_read = data_from_mem;
            end
            else if(data_address == 32'hFFFFFFFD) begin
                next_output_reg = data_to_write;
                data_read = data_from_mem;
            end
        end

        else if(read_mem) begin
            if(data_address == 32'hFFFFFFFC) begin
                next_input_reg = IO_in;
                data_read = input_reg;
            end
        end
        
        else begin
            data_read = data_from_mem;
        end
    end

    assign IO_out = output_reg;

    always_ff @(posedge clk, posedge rst) begin
        if (rst) begin
            enable_reg <= '0;
            output_reg <= '0;
            input_reg <= '0;

        end
        else begin

            enable_reg <= next_enable_reg;
            output_reg <= next_output_reg;
            input_reg <= next_input_reg;
        end


    end

    



endmodule