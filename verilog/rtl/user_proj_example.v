`default_nettype none

module user_proj_example #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

     // IOs - connections to chip 
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb

);

    wire b; wire clk;  wire L1; wire L2;
    // sensor inputs
    wire S1;  wire S2; wire S3; wire S4;
    // motor outputs (to relay module)
    wire MIN11; wire MIN21; wire MIN12; wire MIN22;
    
    // to connect s & m 
    wire Dbit_from_s_to_m;
    wire Abit_from_s_to_m;

    // inputs
    assign clk = io_in[10];
    assign io_oeb[10] = 1'b1;

    assign b = io_in[11];
    assign io_oeb[11] = 1'b1;

    assign S1 = io_in[14];
    assign io_oeb[14] = 1'b1;

    assign S2 = io_in[15];
    assign io_oeb[15] = 1'b1;

    assign S3 = io_in[16];
    assign io_oeb[16] = 1'b1;

    assign S1 = io_in[17];
    assign io_oeb[17] = 1'b1;

    // outputs
    assign io_out[12] = L1;
    assign io_oeb[12] = 1'b0;

    assign io_out[13] = L2;
    assign io_oeb[13] = 1'b0;

    assign io_out[18] = MIN11;
    assign io_oeb[18] = 1'b0;

    assign io_out[19] = MIN21;
    assign io_oeb[19] = 1'b0;

    assign io_out[20] = MIN12;
    assign io_oeb[20] = 1'b0;

    assign io_out[21] = MIN22;
    assign io_oeb[21] = 1'b0;


    // instantiate the modules 
    state_logic s(
        .b(b),
        .clk(clk), 
        .L1(L1),
        .L2(L2),
        .D(Dbit_from_s_to_m),
        .A(Abit_from_s_to_m) );

    motor_logic m(
        .D(Dbit_from_s_to_m), 
        .A(Abit_from_s_to_m),
        .S1(S1), .S2(S2), .S3(S3), .S4(S4),
        .MIN11(MIN11), .MIN21(MIN21), .MIN12(MIN12), .MIN22(MIN22)
    );

endmodule

module state_logic(
    input b,
    input clk,
    output L1, 
    output L2,
    output reg D,
    output reg A);

    reg l1, l2 = 0;

   // state encodings
   // OFF - 00, Defense - 01, Attack - 10
   localparam   BL_OFF = 2'b00,
                BL_D = 2'b01,
                BL_A = 2'b10;

   // array (2-bit) to store the state
   reg [1:0] BL_State;

   initial begin
       // set any initial values
     BL_State = BL_OFF;
     end

   always @(posedge clk) begin
          // state transitions
           case (BL_State)

               BL_OFF: begin
                   if (!b) begin
                       BL_State = BL_OFF;
                   end
                   else if (b) begin
                       BL_State = BL_D;
                   end
               end
               BL_D: begin
                   if (!b) begin
                       BL_State = BL_D;
                   end
                   else if (b) begin
                       BL_State = BL_A;
                   end
               end
               BL_A: begin
                   if (!b) begin
                       BL_State = BL_A;
                   end
                   else if (b) begin
                       BL_State = BL_OFF;
                   end
               end
               default: begin
                   BL_State = BL_OFF;
               end
           endcase

           // state actions
           case (BL_State) 
               BL_OFF: begin
                   l1 = 0;
                   l2 = 0;
                   A = 0;
                   D = 0;
               end
               BL_D: begin
                   l1 = 1;
                   l2 = 0;
                   A = 0;
                   D = 1;
               end
               BL_A: begin
                   l1 = 0;
                   l2 = 1;
                   A = 1;
                   D = 0;
               end
               default: begin
                   A = 0;
                   D = 0;
               end
           endcase
   end

    /* wires must be assigned values after always block */
   assign L1 = l1;  
   assign L2 = l2;

endmodule

module motor_logic
    (D, A,
    S1, S2, S3, S4,
    MIN11, MIN21, MIN12, MIN22
    );

    input D, A, S1, S2, S3, S4;
    output reg MIN11, MIN21, MIN12, MIN22;
    
    always @(D, A, S1, S2, S3, S4) begin

        MIN11 = ~D & A & ~S2 & ~S3 & S4 ||
         ~D & A & S3 & ~S4 || ~D & A & S1 & S4 ||
          D & ~A & ~S1 & S2 & ~S4 || D & ~A & S1 & 
          ~S2 & ~S3 || D & ~A & S1 & ~S3 & S4 ||
           D & ~A & S1 & S2 & S3;

        MIN21 = ~D & A & ~S1 & S3 & S4 ||
         ~D & A & ~S1 & S2 & ~S3 || ~D & A & S1 & 
         ~S3 & ~S4 || D & ~A & ~S1 & S4 || D & ~A & 
         ~S2 & S3 || D & ~A & S1 & S2 & ~S3 & ~S4;

        MIN12 = ~D & A & ~S1 & ~S3 & S4 || ~D &
         A & ~S1 & S2 & S3 || ~D & A & S1 & ~S2 & ~S3
          || D & ~A & ~S1 & S3 & ~S4 || D & ~A & ~S1 & S2
           || D & ~A & S2 & S3 & ~S4 || D & ~A & S1 & ~S2 & ~S3 & S4;

        MIN22 = ~D & A & ~S2 & S3 || ~D & A & S2 &
         ~S3 & ~S4 || ~D & A & S1 & S2 || D & ~A & ~S1
          & ~S2 & S4 || D & ~A & S1 & ~S2 & ~S4 || D &
           ~A & S1 & S3 & S4 || D & ~A & S1 & S2 & ~S3;
           
    end
endmodule

`default_nettype wire