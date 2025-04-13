module datapath(
    input  logic        clk, reset,
    input  logic [1:0]  result_src, 
    input  logic        pc_src, alu_src,
    input  logic        reg_write,
    input  logic [1:0]  imm_src,
    input  logic [2:0]  alu_control,
    output logic        zero,
    output logic [31:0] pc,
    input  logic [31:0] instr,
    output logic [31:0] alu_result, write_data,
    input  logic [31:0] read_data
);

  logic [31:0] pc_next, pc_plus_4, pc_target;
  logic [31:0] imm_ext;
  logic [31:0] src_a, src_b;
  logic [31:0] result;

  // next pc logic
  flopr #(32) pc_reg(clk, reset, pc_next, pc); 
  adder       pc_add_4(pc, 32'd4, pc_plus_4);
  adder       pc_add_branch(pc, imm_ext, pc_target);
  mux2 #(32)  pc_mux(pc_plus_4, pc_target, pc_src, pc_next);
 
  // register file logic
  regfile     reg_file(clk, reg_write, instr[19:15], instr[24:20], 
                       instr[11:7], result, src_a, write_data);
  extend      extender(instr[31:7], imm_src, imm_ext);

  // alu logic
  mux2 #(32)  src_b_mux(write_data, imm_ext, alu_src, src_b);
  alu         alu_unit(src_a, src_b, alu_control, alu_result, zero);
  mux3 #(32)  result_mux(alu_result, read_data, pc_plus_4, result_src, result);

endmodule
