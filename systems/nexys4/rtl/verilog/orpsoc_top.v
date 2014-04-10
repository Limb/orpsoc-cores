//`include "orpsoc-defines.v"
module orpsoc_top #(
parameter	rom0_aw = 6,
parameter	uart0_aw = 3
)(
input	sys_clk_pad_i,
input	rst_n_pad_i,

// UART
input	uart0_srx_pad_i,
output	uart0_stx_pad_o,

// GPIO
inout	[7:0]	gpio0_io

);

parameter	IDCODE_VALUE=32'h13631093;
localparam	MEM_SIZE_BITS = 12;

////////////////////////////////////////////////////////////////////////
//
// Clock and reset generation module
//
////////////////////////////////////////////////////////////////////////

wire	wb_clk, wb_rst;

assign wb_clk = sys_clk_pad_i;
assign wb_rst = rst_n_pad_i;

////////////////////////////////////////////////////////////////////////
//
// Modules interconnections
//
////////////////////////////////////////////////////////////////////////
`include "wb_intercon.vh"

////////////////////////////////////////////////////////////////////////
//
// ARTIX7 JTAG TAP
//
////////////////////////////////////////////////////////////////////////

wire dbg_capture_o;
wire dbg_drck_o;
wire dbg_reset_o;
wire dbg_runtest_o;
wire dbg_sel_o;
wire dbg_shift_o;
wire dbg_tck_o;
wire dbg_tdi_o;
wire dbg_tms_o;
wire dbg_update_o;
wire dbg_tdo_i;

wire dbg_pause_o;
assign dbg_pause_o = 1'b0;


BSCANE2 #(
   .JTAG_CHAIN(1)  // Value for USER command. Possible values: 1-4.
)
BSCANE2_inst (
   .CAPTURE(dbg_capture_o), // 1-bit output: CAPTURE output from TAP controller.
   .DRCK(dbg_drck_o),       // 1-bit output: Gated TCK output. When SEL is asserted, DRCK toggles when CAPTURE or
                      // SHIFT are asserted.

   .RESET(dbg_reset_o),     // 1-bit output: Reset output for TAP controller.
   .RUNTEST(dbg_runtest_o), // 1-bit output: Output asserted when TAP controller is in Run Test/Idle state.
   .SEL(dbg_sel_o),         // 1-bit output: USER instruction active output.
   .SHIFT(dbg_shift_o),     // 1-bit output: SHIFT output from TAP controller.
   .TCK(dbg_tck_o),         // 1-bit output: Test Clock output. Fabric connection to TAP Clock pin.
   .TDI(dbg_tdi_o),         // 1-bit output: Test Data Input (TDI) output from TAP controller.
   .TMS(dbg_tms_o),         // 1-bit output: Test Mode Select output. Fabric connection to TAP.
   .UPDATE(dbg_update_o),   // 1-bit output: UPDATE output from TAP controller
   .TDO(dbg_tdo_i)          // 1-bit input: Test Data Output (TDO) input for USER function.
);
////////////////////////////////////////////////////////////////////////
//
// OR1K CPU
//
////////////////////////////////////////////////////////////////////////

wire	[31:0]	or1k_irq;
wire	or1k_rst;

wire	[31:0]	or1k_dbg_adr_i;
wire	[31:0]	or1k_dbg_dat_i;
wire		or1k_dbg_stb_i;
wire		or1k_dbg_we_i;
wire	[31:0]	or1k_dbg_dat_o;
wire		or1k_dbg_ack_o;
wire		or1k_dbg_stall_i;
wire		or1k_dbg_bp_o;
wire		or1k_dbg_rst;

assign or1k_rst = wb_rst | or1k_dbg_rst;

mor1kx #(
	.FEATURE_DEBUGUNIT("ENABLED"),
	.FEATURE_CMOV("ENABLED"),
	.FEATURE_INSTRUCTIONCACHE("ENABLED"),
	.OPTION_ICACHE_BLOCK_WIDTH(5),
	.OPTION_ICACHE_SET_WIDTH(8),
	.OPTION_ICACHE_WAYS(2),
	.OPTION_ICACHE_LIMIT_WIDTH(32),
	.FEATURE_IMMU("ENABLED"),
	.FEATURE_DATACACHE("ENABLED"),
	.OPTION_DCACHE_BLOCK_WIDTH(5),
	.OPTION_DCACHE_SET_WIDTH(8),
	.OPTION_DCACHE_WAYS(2),
	.OPTION_DCACHE_LIMIT_WIDTH(31),
	.FEATURE_DMMU("ENABLED"),
	.OPTION_PIC_TRIGGER("LATCHED_LEVEL"),

	.IBUS_WB_TYPE("B3_REGISTERED_FEEDBACK"),
	.DBUS_WB_TYPE("B3_REGISTERED_FEEDBACK"),
	.OPTION_CPU0("CAPPUCCINO"),
	.OPTION_RESET_PC(32'hf0000100)
) mor1kx0 (
	.iwbm_adr_o(wb_m2s_or1k_i_adr),
	.iwbm_stb_o(wb_m2s_or1k_i_stb),
	.iwbm_cyc_o(wb_m2s_or1k_i_cyc),
	.iwbm_sel_o(wb_m2s_or1k_i_sel),
	.iwbm_we_o (wb_m2s_or1k_i_we),
	.iwbm_cti_o(wb_m2s_or1k_i_cti),
	.iwbm_bte_o(wb_m2s_or1k_i_bte),
	.iwbm_dat_o(wb_m2s_or1k_i_dat),

	.dwbm_adr_o(wb_m2s_or1k_d_adr),
	.dwbm_stb_o(wb_m2s_or1k_d_stb),
	.dwbm_cyc_o(wb_m2s_or1k_d_cyc),
	.dwbm_sel_o(wb_m2s_or1k_d_sel),
	.dwbm_we_o (wb_m2s_or1k_d_we ),
	.dwbm_cti_o(wb_m2s_or1k_d_cti),
	.dwbm_bte_o(wb_m2s_or1k_d_bte),
	.dwbm_dat_o(wb_m2s_or1k_d_dat),

	.clk(wb_clk),
	.rst(or1k_rst),

	.iwbm_err_i(wb_s2m_or1k_i_err),
	.iwbm_ack_i(wb_s2m_or1k_i_ack),
	.iwbm_dat_i(wb_s2m_or1k_i_dat),
	.iwbm_rty_i(wb_s2m_or1k_i_rty),

	.dwbm_err_i(wb_s2m_or1k_d_err),
	.dwbm_ack_i(wb_s2m_or1k_d_ack),
	.dwbm_dat_i(wb_s2m_or1k_d_dat),
	.dwbm_rty_i(wb_s2m_or1k_d_rty),

	.irq_i(or1k_irq),

	.du_addr_i(or1k_dbg_adr_i[15:0]),
	.du_stb_i(or1k_dbg_stb_i),
	.du_dat_i(or1k_dbg_dat_i),
	.du_we_i(or1k_dbg_we_i),
	.du_dat_o(or1k_dbg_dat_o),
	.du_ack_o(or1k_dbg_ack_o),
	.du_stall_i(or1k_dbg_stall_i),
	.du_stall_o(or1k_dbg_bp_o)
);

////////////////////////////////////////////////////////////////////////
//
// Debug Interface
//
////////////////////////////////////////////////////////////////////////

adbg_top dbg_if0 (
	// OR1K interface
	.cpu0_clk_i	(wb_clk),
	.cpu0_rst_o	(or1k_dbg_rst),
	.cpu0_addr_o	(or1k_dbg_adr_i),
	.cpu0_data_o	(or1k_dbg_dat_i),
	.cpu0_stb_o	(or1k_dbg_stb_i),
	.cpu0_we_o	(or1k_dbg_we_i),
	.cpu0_data_i	(or1k_dbg_dat_o),
	.cpu0_ack_i	(or1k_dbg_ack_o),
	.cpu0_stall_o	(or1k_dbg_stall_i),
	.cpu0_bp_i	(or1k_dbg_bp_o),

	// TAP interface
	.tck_i		(dbg_tck_o),
	.tdi_i		(dbg_tdi_o),
	.tdo_o		(dbg_tdo_i),
	.rst_i		(wb_rst),
	.capture_dr_i	(dbg_capture_o),
	.shift_dr_i	(dbg_shift_o),
	.pause_dr_i	(dbg_pause_o),
	.update_dr_i	(dbg_update_o),
	.debug_select_i	(dbg_sel_o),

	// Wishbone debug master
	.wb_rst_i	(wb_rst),
	.wb_clk_i	(wb_clk),
	.wb_dat_i	(wb_s2m_dbg_dat),
	.wb_ack_i	(wb_s2m_dbg_ack),
	.wb_err_i	(wb_s2m_dbg_err),

	.wb_adr_o	(wb_m2s_dbg_adr),
	.wb_dat_o	(wb_m2s_dbg_dat),
	.wb_cyc_o	(wb_m2s_dbg_cyc),
	.wb_stb_o	(wb_m2s_dbg_stb),
	.wb_sel_o	(wb_m2s_dbg_sel),
	.wb_we_o	(wb_m2s_dbg_we),
	.wb_cti_o	(wb_m2s_dbg_cti),
	.wb_bte_o	(wb_m2s_dbg_bte)
);


////////////////////////////////////////////////////////////////////////
//
// wb_ram0
//
////////////////////////////////////////////////////////////////////////
 wb_ram
     #(.depth (2**MEM_SIZE_BITS))
   wb_ram0
     (
      //Wishbone Master interface
      .wb_clk_i (wb_clk),
      .wb_rst_i (wb_rst),
      .wb_adr_i	(wb_m2s_mem_adr[MEM_SIZE_BITS-1:0]),
      .wb_dat_i	(wb_m2s_mem_dat),
      .wb_sel_i	(wb_m2s_mem_sel),
      .wb_we_i	(wb_m2s_mem_we),
      .wb_cyc_i	(wb_m2s_mem_cyc),
      .wb_stb_i	(wb_m2s_mem_stb),
      .wb_cti_i	(wb_m2s_mem_cti),
      .wb_bte_i	(wb_m2s_mem_bte),
      .wb_dat_o	(wb_s2m_mem_dat),
      .wb_ack_o	(wb_s2m_mem_ack),
      .wb_err_o (wb_s2m_mem_err),
      .wb_rty_o (wb_s2m_mem_rty)
);

////////////////////////////////////////////////////////////////////////
//
// UART0
//
////////////////////////////////////////////////////////////////////////

wire	uart0_irq;

uart_top uart16550_0 (
// Wishbone slave interface
.wb_clk_i	(wb_clk),
.wb_rst_i	(wb_rst),
.wb_adr_i	(wb_m2s_uart0_adr[uart0_aw-1:0]),
.wb_dat_i	(wb_m2s_uart0_dat),
.wb_we_i	(wb_m2s_uart0_we),
.wb_stb_i	(wb_m2s_uart0_stb),
.wb_cyc_i	(wb_m2s_uart0_cyc),
.wb_sel_i	(4'b0), // Not used in 8-bit mode
.wb_dat_o	(wb_s2m_uart0_dat),
.wb_ack_o	(wb_s2m_uart0_ack),

// Outputs
.int_o	(uart0_irq),
.stx_pad_o	(uart0_stx_pad_o),
.rts_pad_o	(),
.dtr_pad_o	(),

// Inputs
.srx_pad_i	(uart0_srx_pad_i),
.cts_pad_i	(1'b0),
.dsr_pad_i	(1'b0),
.ri_pad_i	(1'b0),
.dcd_pad_i	(1'b0)
);

////////////////////////////////////////////////////////////////////////
//
// GPIO 0
//
////////////////////////////////////////////////////////////////////////

wire [7:0]	gpio0_in;
wire [7:0]	gpio0_out;
wire [7:0]	gpio0_dir;

// Tristate logic for IO
// 0 = input, 1 = output
genvar i;
generate
for (i = 0; i < 8; i = i+1) begin: gpio0_tris
assign gpio0_io[i] = gpio0_dir[i] ? gpio0_out[i] : 1'bz;
assign gpio0_in[i] = gpio0_dir[i] ? gpio0_out[i] : gpio0_io[i];
end
endgenerate

gpio gpio0 (
// GPIO bus
.gpio_i	(gpio0_in),
.gpio_o	(gpio0_out),
.gpio_dir_o	(gpio0_dir),
// Wishbone slave interface
.wb_adr_i	(wb_m2s_gpio0_adr[0]),
.wb_dat_i	(wb_m2s_gpio0_dat),
.wb_we_i	(wb_m2s_gpio0_we),
.wb_cyc_i	(wb_m2s_gpio0_cyc),
.wb_stb_i	(wb_m2s_gpio0_stb),
.wb_cti_i	(wb_m2s_gpio0_cti),
.wb_bte_i	(wb_m2s_gpio0_bte),
.wb_dat_o	(wb_s2m_gpio0_dat),
.wb_ack_o	(wb_s2m_gpio0_ack),
.wb_err_o	(wb_s2m_gpio0_err),
.wb_rty_o	(wb_s2m_gpio0_rty),

.wb_clk	(wb_clk),
.wb_rst	(wb_rst)
);

////////////////////////////////////////////////////////////////////////
//
// Interrupt assignment
//
////////////////////////////////////////////////////////////////////////

assign or1k_irq[0] = 0; // Non-maskable inside OR1K
assign or1k_irq[1] = 0; // Non-maskable inside OR1K
assign or1k_irq[2] = uart0_irq;
assign or1k_irq[3] = 0;
assign or1k_irq[4] = 0;
assign or1k_irq[5] = 0;
assign or1k_irq[6] = 0;
assign or1k_irq[7] = 0;
assign or1k_irq[8] = 0;
assign or1k_irq[9] = 0;
assign or1k_irq[10] = 0;
assign or1k_irq[11] = 0;
assign or1k_irq[12] = 0;
assign or1k_irq[13] = 0;
assign or1k_irq[14] = 0;
assign or1k_irq[15] = 0;
assign or1k_irq[16] = 0;
assign or1k_irq[17] = 0;
assign or1k_irq[18] = 0;
assign or1k_irq[19] = 0;
assign or1k_irq[20] = 0;
assign or1k_irq[21] = 0;
assign or1k_irq[22] = 0;
assign or1k_irq[23] = 0;
assign or1k_irq[24] = 0;
assign or1k_irq[25] = 0;
assign or1k_irq[26] = 0;
assign or1k_irq[27] = 0;
assign or1k_irq[28] = 0;
assign or1k_irq[29] = 0;
assign or1k_irq[30] = 0;
assign or1k_irq[31] = 0;

endmodule // orpsoc_top
