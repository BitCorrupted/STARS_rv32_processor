`timescale 1ms/10ps
module tb;


initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    #3 $finish;
end

endmodule
