`timescale 1ns / 1ps

module tb_processor;

// Clock and reset signals
logic clk;
logic reset;

// Target coordinates for the robotic system
logic [15:0] target_x;
logic [15:0] target_y;
logic [15:0] target_z;

// Control signals
logic start;

// Instantiate the processor design
processor uut (
    .clk(clk),
    .reset(reset),
    .target_x(target_x),
    .target_y(target_y),
    .target_z(target_z),
    .start(start)
);

// Clock generation
always begin
    #5 clk = ~clk;
end

// Testbench initial block
initial begin
    // Initialize the clock, reset, and control signals
    clk = 0;
    reset = 1;
    start = 0;
    target_x = 16'd0;
    target_y = 16'd0;
    target_z = 16'd0;

    // Create a VCD file for waveform generation
    $dumpfile("dump.vcd");  // Specify the VCD file name
    $dumpvars(0, tb_processor);  // Dump all variables in the testbench

    // Apply reset
    #10 reset = 0;

    // Test case 1: Idle state (no movement)
    $display("Test case 1: Idle state (no movement)");
    #50;
    
    // Test case 2: Move to a fixed target (positive coordinates)
    $display("Test case 2: Move to a fixed target (positive coordinates)");
    target_x = 16'd100;
    target_y = 16'd50;
    target_z = 16'd25;
    start = 1;
    #50; // Wait for movement
    start = 0;

    // Test case 3: Move to a fixed target (negative coordinates)
    $display("Test case 3: Move to a fixed target (negative coordinates)");
    target_x = 16'd100 + (16'd100 - 16'd200);  // Equivalent to -100
    target_y = 16'd50 + (16'd50 - 16'd100);    // Equivalent to -50
    target_z = 16'd25 + (16'd25 - 16'd50);     // Equivalent to -25
    start = 1;
    #50;
    start = 0;

    // Test case 4: Oscillate between positive and negative X, Y, and Z
    $display("Test case 4: Oscillate between positive and negative X, Y, and Z");
    for (int i = 0; i < 10; i++) begin
        target_x = (i % 2 == 0) ? 16'd100 : 16'd100 + (16'd100 - 16'd200);  // Alternating between positive and negative
        target_y = (i % 2 == 0) ? 16'd50 : 16'd50 + (16'd50 - 16'd100);    // Alternating between positive and negative
        target_z = (i % 2 == 0) ? 16'd25 : 16'd25 + (16'd25 - 16'd50);     // Alternating between positive and negative
        start = 1;
        #50;
        start = 0;
    end

    // Test case 5: Continuous movement in one direction
    $display("Test case 5: Continuous movement in one direction");
    target_x = 16'd200;
    target_y = 16'd100;
    target_z = 16'd50;
    start = 1;
    #200; // Move for a longer period
    start = 0;

    // Test case 6: Rapid movement back and forth
    $display("Test case 6: Rapid movement back and forth");
    for (int i = 0; i < 5; i++) begin
        target_x = (i % 2 == 0) ? 16'd100 : 16'd100 + (16'd100 - 16'd200);  // Alternating between positive and negative
        target_y = (i % 2 == 0) ? 16'd50 : 16'd50 + (16'd50 - 16'd100);    // Alternating between positive and negative
        target_z = (i % 2 == 0) ? 16'd25 : 16'd25 + (16'd25 - 16'd50);     // Alternating between positive and negative
        start = 1;
        #50;
        start = 0;
    end

    // Test case 7: Move to extreme coordinates
    $display("Test case 7: Move to extreme coordinates");
    target_x = 16'd1000;
    target_y = 16'd500;
    target_z = 16'd250;
    start = 1;
    #50;
    start = 0;

    // Test case 8: Simulate a reset during movement
    $display("Test case 8: Simulate a reset during movement");
    target_x = 16'd500;
    target_y = 16'd250;
    target_z = 16'd125;
    start = 1;
    #30;
    reset = 1; // Apply reset
    #10;
    reset = 0;
    #50; // Wait for reset to finish

    // Test case 9: Large oscillations in X, Y, Z
    $display("Test case 9: Large oscillations in X, Y, Z");
    for (int i = 0; i < 10; i++) begin
        target_x = (i % 2 == 0) ? 16'd500 : 16'd500 + (16'd500 - 16'd1000);  // Alternating between large positive and negative
        target_y = (i % 2 == 0) ? 16'd250 : 16'd250 + (16'd250 - 16'd500);  // Alternating between large positive and negative
        target_z = (i % 2 == 0) ? 16'd125 : 16'd125 + (16'd125 - 16'd250);  // Alternating between large positive and negative
        start = 1;
        #50;
        start = 0;
    end

    // Test case 10: Random targets with various movements
    $display("Test case 10: Random targets with various movements");
    target_x = 16'd75 + (16'd75 - 16'd150);   // Equivalent to -75
    target_y = 16'd125;
    target_z = 16'd50;
    start = 1;
    #50;
    start = 0;
    
    // End simulation
    $finish;
end

endmodule
