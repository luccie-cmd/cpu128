timeunit 1ns;
timeprecision 100ps;

typedef enum bit[1:0] {
    ALU_ADD
} aluOps_t;
typedef enum bit[4:0] {
    Zero,
    Sign,
    Carry,
    Overflow,
    ExceptionEnable
} flags_t;
typedef enum bit[5:0] {
    R0  = 0,
    R29 = 29,
    RSP = 30,
    RIP = 31,
    RFLAGS = 32,
    RCYCLE = 33,
    RFETCHRIP = 34,
    RFAULTRIP = 35,
    RDECODERIP = 36,
    RTEMP = 37,
    READDR0 = 38,
    READDR1 = 39,
    READDR2 = 40,
    READDR3 = 41,
    READDR4 = 42,
    READDR5 = 43,
    READDR6 = 44,
    READDR7 = 45
} regs_t;
typedef enum bit[2:0] {
    FETCH_STATE_NONE=0,
    FETCH_STATE_START=1,
    FETCH_STATE_WAIT=2,
    FETCH_STATE_MISS=3,
    FETCH_STATE_DONE=4
} fetch_state_t;

reg [134217727:0] RAM;
reg [35071:0] microCodeBuffer;
reg [15:0] microCodeBufferHead;
reg [1023:0] instructionBuffer;
reg [9:0] instructionBufferHead;
`define MAX_INSTRUCTIONBUFFER_HEAD 128
`define HIGHWATER_INSTRUCTIONBUFFER_HEAD 115
`define LOWWATER_INSTRUCTIONBUFFER_HEAD 51
reg canFetch;
reg LSUDone;
reg [63:0] LSUData;

reg [63:0] RegisterFileDatas [45:0];

module RegisterFile (
    input wire [5:0] selectIn,
    input wire [5:0] selectA,
    input wire [5:0] selectB,
    input wire [63:0] inputData,
    input wire writeEnable,
    input wire clk,
    output wire [63:0] outputDataA,
    output wire [63:0] outputDataB
);
    always_ff @(posedge clk) begin
        if (writeEnable == 1) begin
            RegisterFileDatas[selectIn] = inputData;
        end
    end
    assign outputDataA = RegisterFileDatas[selectA];
    assign outputDataB = RegisterFileDatas[selectB];
    // always #5 begin
    //     $display("A = %h = RegisterFileDatas[%d] B = %h = RegisterFileDatas[%d] IN = %h = RegisterFileDatas[%d]", outputDataA, selectA, outputDataB, selectB, inputData, selectIn);
    // end
endmodule

module LSU (
    input wire [63:0] eaddr,
    input wire [1:0] size
);
    always @(*) begin
        
    end
endmodule

module Fetcher (
    input wire clk,
    input fetch_state_t state,
    output fetch_state_t nextState
);
    reg [63:0] fetchIP = 0;
    always_ff @(posedge clk) begin
        case (state)
            FETCH_STATE_NONE: begin
                nextState <= state;
            end
            FETCH_STATE_START: begin
                nextState <= FETCH_STATE_WAIT;
                if (instructionBufferHead + 8 < `MAX_INSTRUCTIONBUFFER_HEAD) begin
                    if (fetchIP % 8 == 0) begin
                        $display("Aligned fetch");
                    end else begin
                        $display("Unaligned fetch");
                    end
                end
            end
            FETCH_STATE_WAIT: begin
                if (LSUDone == 1) begin
                    nextState <= FETCH_STATE_DONE;
                end else begin
                    nextState <= state;
                end
            end
            default: begin
                $display("TODO: Handle case %d\n", state);
                $finish;
            end 
        endcase
    end
endmodule

module main;
    logic clk = 1'b0;
    logic RFWriteEnable;
    wire [63:0] RFOutA;
    wire [63:0] RFOutB;
    reg [63:0] RFIn;
    reg [5:0] RFSelectIn;
    reg [5:0] RFSelectA;
    reg [5:0] RFSelectB;
    fetch_state_t fetcherState = FETCH_STATE_NONE;
    fetch_state_t fetcherStateNext;

    always #5 clk = ~clk;
    RegisterFile RF(
        .clk(clk),
        .selectIn(RFSelectIn),
        .selectA(RFSelectA),
        .selectB(RFSelectB),
        .inputData(RFIn),
        .writeEnable(RFWriteEnable),
        .outputDataA(RFOutA),
        .outputDataB(RFOutB)
    );
    Fetcher fetcher(
        .clk(clk),
        .state(fetcherState),
        .nextState(fetcherStateNext)
    );
    // Setup CPU in good state
    initial begin
        canFetch = 1;
        instructionBufferHead = 0;

        RFWriteEnable = 1'b1;
        RFIn = 64'hffffffffffffffff;
        RFSelectIn = RFAULTRIP;
        @(posedge clk);
        #1;
        RFIn = 64'h0;
        RFSelectIn = RIP;
        @(posedge clk);
        #1;
        RFIn = 64'h0;
        RFSelectIn = RFETCHRIP;
        @(posedge clk);
        #1;
        RFIn = 64'h0;
        RFSelectIn = RDECODERIP;
        @(posedge clk);
        #1;
        RFWriteEnable = 1'b0;
        fetcherState <= FETCH_STATE_START;
        // Let's get some cycles
        for (int i = 0; i < 100; ++i) begin
            @(posedge clk);
            fetcherState <= fetcherStateNext;
        end
        $finish;
    end
endmodule