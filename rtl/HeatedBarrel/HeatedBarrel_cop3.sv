// SPDX-License-Identifier: GPL-3.0-or-later
/*  HeatedBarrel_MiSTer — SEI300 (COP3) coprocessor, v2 rewrite

    MAME-faithful implementation of the Seibu SEI300 chip as used by the
    `legionna` driver (HeatedBarrel 1992 — Banpresto/TAD).

    Reference (in this repo):
      reference/mame_seibu/seibucop.cpp
      reference/mame_seibu/seibucop_cmd.ipp
      reference/mame_seibu/seibucop_dma.ipp
      reference/mame_seibu/legionna.cpp

    Mapped on the 68K bus at $100400-$1006FF (legionna_cop_map).
    The `addr` input here is the WORD offset relative to $100400 (so
    addr=10'h000 → byte 0x100400, addr=10'h17E → byte 0x1006FC, etc).

    Drop-in replacement for the v1 module; identical port list so
    HeatedBarrel_main_top.sv needs no changes.

    Author: rmonic79 — 2026-05-20
*/

module HeatedBarrel_cop3 (
	input  wire        clk,
	input  wire        reset,

	// CPU bus (region decoded externally)
	input  wire        cs,
	input  wire        wr,
	input  wire        rd,
	input  wire [10:1] addr,        // word offset relative to 0x100400
	input  wire  [1:0] dsn,
	input  wire [15:0] wdata,
	output reg  [15:0] rdata,

	// DMA master — Main RAM (port B for source-read, port A for write)
	output reg  [15:0] dma_ram_addr,
	output reg  [15:0] dma_ram_wdata,
	input  wire [15:0] dma_ram_rdata,
	output reg         dma_ram_we,
	// byte-enable della write: default 2'b11 (word). Per i cop_write_byte (es.
	// l'angolo 138e a 0x37=byte basso) usare 2'b01 per NON azzerare il byte
	// adiacente (0x36 = divisore/ampiezza letto da 42c2 e sin/cos).
	output reg   [1:0] dma_ram_be,

	// DMA SOURCE read: byte address on 68K bus, decoded by main_top mux
	output reg  [23:0] dma_src_byte /*verilator public_flat_rd*/,
	input  wire [15:0] dma_src_rdata,

	// ROM read (descrittori hitbox b100/b900 sono in MAIN ROM): handshake
	// req/ready via sub-rom port SDRAM (latency variabile). Vedi M_B1_* states.
	output reg  [23:0] cop_rom_addr /*verilator public_flat_rd*/,
	output reg         cop_rom_req /*verilator public_flat_rd*/,
	input  wire [15:0] cop_rom_rdata,
	input  wire        cop_rom_ready,

	// DMA master — Spriteram (write only)
	output reg  [10:0] dma_spr_addr,
	output reg  [15:0] dma_spr_wdata,
	output reg         dma_spr_we,

	// DMA master — VRAM tilemap (cmd 0x14)
	output reg  [12:0] dma_vram_addr,
	output reg  [15:0] dma_vram_wdata,
	output reg         dma_vram_we,

	// VRAM write-intent for the CURRENT cycle (combinational). The registered
	// dma_vram_we/_addr above are 1 cycle LATE: asserted during the NEXT D_READ,
	// hijacking the shared M10K port-A address (read src vs write dst collide on
	// real HW). main_top drives the bg/fg/mg/txt port-A mux from these instead so
	// the write commits in D_WRITE(i) using src_rdata (mem[src_i], valid now),
	// freeing the next D_READ for the src fetch. Fixes tilemap shift dst[i]=src[i-1].
	output wire [12:0] dma_vram_addr_now,
	output wire        dma_vram_we_now,
	output wire [15:0] dma_vram_wdata_now,
	// cmd 0x118 FILL -> SCRATCH VRAM ($100800-$102FFF). main_top instrada alle BRAM scratch
	// bg/fg/mg/txt; il DMA 0x14 poi copia lo scratch pulito nel render-buffer.
	output wire        dma_fill_we,
	output wire [22:0] dma_fill_addr,    // indirizzo VRAM word assoluto (byte/2)
	output wire [15:0] dma_fill_wdata,

	// DMA master — Palette renderer (cmd 0x15 only)
	output reg  [10:0] dma_pal_addr,
	output reg  [15:0] dma_pal_wdata,
	output reg         dma_pal_we,

	// DMA master — Palette STAGING (cmd 0x80-0x87 pre-fade scratch copy)
	// Shares addr+wdata bus with dma_pal_*; only the we strobe differs.
	output reg         dma_pal_stage_we,

	// Busy: high during cmd or DMA execution (FSM-based ONLY).
	// Drives the port-A MUXes in main_top (RAM/VRAM/sprite/palette hijack):
	// must reflect the cycles where cop_dma_* actually drive valid values.
	output wire        dma_busy,

	// CPU stall: dma_busy OR the combinational trigger pulse. Asserted ONE cycle
	// EARLIER than dma_busy (already in the trigger write cycle) to close the
	// 1-cycle DTACK hole between back-to-back COP commands (a180/a980/b100/b900
	// confine check). Used ONLY for the CPU bus_busy stall in main_top — NEVER
	// for the port MUXes (which would hijack ports while cop_dma_* are stale).
	output wire        cpu_stall
);

	// ────────────────────────────────────────────────────────────────────────
	// Register file
	// ────────────────────────────────────────────────────────────────────────

	// cop_regs[0..6] — 7 pointers, 32-bit each (high/low banked at 0x4A0/0x4C0)
	reg [15:0] cop_reg_hi [0:6] /*verilator public_flat_rd*/;
	reg [15:0] cop_reg_lo [0:6] /*verilator public_flat_rd*/;

	// Per-mode DMA params (mode index = cop_dma_mode[7:0], range 0..255)
	reg [15:0] cop_dma_src   [0:255];
	reg [15:0] cop_dma_size  [0:255];
	reg [15:0] cop_dma_dst   [0:255];

	// Misc registers
	reg [8:0]  cop_dma_mode;        // selects bank for src/size/dst
	reg [15:0] cop_dma_adr_rel;
	reg [15:0] cop_dma_v1;
	reg [15:0] cop_dma_v2;
	reg [15:0] cop_scale;           // only bits 1..0 effective
	reg [15:0] cop_angle_target;
	reg [15:0] cop_angle_step;
	reg [15:0] cop_pal_brightness_val;
	reg [15:0] cop_pal_brightness_mode;
	reg [15:0] cop_unk_param_a;
	reg [15:0] cop_unk_param_b;
	reg [15:0] cop_hit_baseadr /*verilator public_flat_rd*/;
	reg [15:0] cop_prng_maxvalue;
	reg [15:0] cop_itoa_low;
	reg [15:0] cop_itoa_high;
	reg [15:0] cop_itoa_mode;
	// MAME cop_itoa_digits[10]: BCD digits of cop_itoa (low<<0 | high<<16).
	// 9 digits (0..8) + terminator (9)=0. Read via cop_itoa_digits_r at 0x100590.
	reg [7:0]  cop_itoa_digits [0:9];
	// BCD update FSM: triggered by write to cop_itoa_low/high. Multi-cycle.
	reg        bcd_pending;
	reg [3:0]  bcd_step;       // 0..8 = 9 digits
	reg [31:0] bcd_val;        // current N for subtract loop
	reg [31:0] bcd_quotient;   // counts sub10 → becomes next N

	// ────────────────────────────────────────────────────────────────────────
	// MAME cop_program command table (seibucop.cpp lines 318-457)
	// At boot, game code uploads 32 macro slots × 8 micro-op words via
	//   pgm_addr  (0x100434) → slot*8 + sub-index
	//   pgm_data  (0x100432) → micro-op word
	//   pgm_value (0x100438) → cop_func_value[slot]
	//   pgm_mask  (0x10043A) → cop_func_mask[slot]
	//   pgm_trigger (0x10043C) → cop_func_trigger[slot]
	// Then game writes a trigger value to 0x100500 (cmd_w). MAME calls
	// find_trigger_match(triggerval, 0xff00) to find the slot whose
	// cop_func_trigger matches. The slot's micro-ops decide what runs.
	//
	// Our RTL keeps the hardcoded math dispatch (sin/cos/atan2/dist/etc.)
	// but uses the table to translate the raw triggerval written by the
	// game into the "canonical" triggerval understood by the dispatch.
	// This unblocks game code that writes triggerval with different LSBs
	// (e.g. 0x8123 instead of 0x8100) — the table maps both to slot N
	// whose canonical trigger is 0x8100.
	reg [15:0] cop_program       [0:255];  // 32 slots × 8 micro-op words
	reg [15:0] cop_func_trigger  [0:31];
	reg [15:0] cop_func_value    [0:31];
	reg [15:0] cop_func_mask     [0:31];
	reg [15:0] cop_latch_trigger;
	reg [15:0] cop_latch_value;
	reg [15:0] cop_latch_mask;
	reg [7:0]  cop_latch_addr;

	// Sprite DMA latches (stubs — HeatedBarrel ROM doesn't trigger sprite DMA via COP)
	reg [15:0] cop_spr_dma_param_lo, cop_spr_dma_param_hi;
	reg [15:0] cop_spr_dma_size;
	reg [15:0] cop_spr_dma_src_hi,   cop_spr_dma_src_lo;
	reg [15:0] cop_spr_dma_abs_x,    cop_spr_dma_abs_y;

	// Read-back status
	reg [15:0] cop_status /*verilator public_flat_rd*/;
	reg [15:0] cop_angle;
	reg [15:0] cop_dist;

	// Collision state
	// pos[slot][axis], allow_swap[slot], flags_swap[slot]
	reg [15:0] coll_pos    [0:1][0:2] /*verilator public_flat_rd*/;      // pos word for slot 0,1 — axes Y,X,Z
	reg        coll_allow_swap [0:1] /*verilator public_flat_rd*/;
	reg [15:0] coll_flags_swap [0:1] /*verilator public_flat_rd*/;
	reg [15:0] coll_min    [0:1][0:2] /*verilator public_flat_rd*/;
	reg [15:0] coll_max    [0:1][0:2] /*verilator public_flat_rd*/;
	reg [15:0] cop_hit_status /*verilator public_flat_rd*/;
	reg [15:0] cop_hit_val_stat /*verilator public_flat_rd*/;
	reg [15:0] cop_hit_val [0:2] /*verilator public_flat_rd*/;

	// PRNG: free-running counter
	reg [31:0] cycle_cnt;
	always @(posedge clk) begin
		if (reset) cycle_cnt <= 0;
		else       cycle_cnt <= cycle_cnt + 1;
	end

	// ────────────────────────────────────────────────────────────────────────
	// CPU bus decode
	// ────────────────────────────────────────────────────────────────────────
	wire [9:0] widx = addr;          // word offset
	// Edge-detect cs+wr combinato (rising = nuovo write bus cycle). 68k bus
	// pattern: AS_n giù → DSn giù (1/2 ciclo dopo). Edge solo su cs perde
	// il primo ciclo se wr non ancora attivo. Senza edge, dma_pending si
	// re-arma in loop quando CPU stalla → DMA infinito.
	wire       cs_wr_now = cs & wr;
	reg        cs_wr_prev;
	always @(posedge clk) begin
		if (reset) cs_wr_prev <= 1'b0;
		else       cs_wr_prev <= cs_wr_now;
	end
	wire       cpu_wr_pulse = cs_wr_now & ~cs_wr_prev;

	// Triggers (write-only addresses)
	// MAME legionna.cpp:172 maps cop_cmd_w to $100500-$100505 (3 words).
	// Game ROM writes sequential macros to all 3 offsets (verified by static
	// decode: $100500 14 writes, $100502 4 writes, $100504 2 writes).
	wire trig_macro    = cpu_wr_pulse && (widx == 10'h080 ||
	                                       widx == 10'h081 ||
	                                       widx == 10'h082);   // 0x100500/02/04
	wire trig_dma      = cpu_wr_pulse && (widx == 10'h17E);    // 0x1006FC
	wire trig_sort_dma = cpu_wr_pulse && (widx == 10'h17F);    // 0x1006FE (unused)
	wire trig_spr_inc  = cpu_wr_pulse && (widx == 10'h008);    // 0x100410

	// ────────────────────────────────────────────────────────────────────────
	// CPU writes
	// ────────────────────────────────────────────────────────────────────────
	integer i_r;
	always @(posedge clk) begin
		if (reset) begin
			// Mass reset
			for (i_r = 0; i_r < 7;   i_r = i_r + 1) begin
				cop_reg_hi[i_r] <= 16'd0;
				cop_reg_lo[i_r] <= 16'd0;
			end
			for (i_r = 0; i_r < 256; i_r = i_r + 1) begin
				cop_dma_src [i_r] <= 16'd0;
				cop_dma_size[i_r] <= 16'd0;
				cop_dma_dst [i_r] <= 16'd0;
				cop_program [i_r] <= 16'd0;
			end
			for (i_r = 0; i_r < 32; i_r = i_r + 1) begin
				cop_func_trigger[i_r] <= 16'd0;
				cop_func_value  [i_r] <= 16'd0;
				cop_func_mask   [i_r] <= 16'd0;
			end
			cop_latch_addr    <= 8'd0;
			cop_latch_trigger <= 16'd0;
			cop_latch_value   <= 16'd0;
			cop_latch_mask    <= 16'd0;
			cop_dma_mode             <= 9'd0;
			cop_dma_adr_rel          <= 0;
			cop_dma_v1               <= 0;
			cop_dma_v2               <= 0;
			cop_scale                <= 0;
			cop_angle_target         <= 0;
			cop_angle_step           <= 0;
			cop_pal_brightness_val   <= 0;
			cop_pal_brightness_mode  <= 0;
			cop_unk_param_a          <= 0;
			cop_unk_param_b          <= 0;
			cop_hit_baseadr          <= 0;
			cop_prng_maxvalue        <= 16'h00FF;
			cop_itoa_low             <= 0;
			cop_itoa_high            <= 0;
			cop_itoa_mode            <= 0;
			cop_itoa_digits[0]<=8'h30; cop_itoa_digits[1]<=8'h20;
			cop_itoa_digits[2]<=8'h20; cop_itoa_digits[3]<=8'h20;
			cop_itoa_digits[4]<=8'h20; cop_itoa_digits[5]<=8'h20;
			cop_itoa_digits[6]<=8'h20; cop_itoa_digits[7]<=8'h20;
			cop_itoa_digits[8]<=8'h20; cop_itoa_digits[9]<=8'h00;
			bcd_pending <= 0; bcd_step <= 0; bcd_val <= 0;
			bcd_quotient <= 0;
			cop_spr_dma_param_lo     <= 0;
			cop_spr_dma_param_hi     <= 0;
			cop_spr_dma_size         <= 0;
			cop_spr_dma_src_hi       <= 0;
			cop_spr_dma_src_lo       <= 0;
			cop_spr_dma_abs_x        <= 0;
			cop_spr_dma_abs_y        <= 0;
		end else if (cpu_wr_pulse) begin
			case (widx)
				// Sprite DMA latches (treated as plain RW latches)
				10'h000: cop_spr_dma_param_lo <= wdata;  // 0x100400
				10'h001: cop_spr_dma_param_hi <= wdata;  // 0x100402
				10'h006: cop_spr_dma_size     <= wdata;  // 0x10040C
				10'h008: begin
					// 0x100410 sprite_dma_inc (MAME seibucop.cpp:1279):
					//   if x_clip in [-160, 320): cop_regs[4] += 8
					//   cop_sprite_dma_src += 6; cop_sprite_dma_size--;
					//   cop_status bit 1 = (size > 0) ? 0 : 1
					// (Sprite DMA x_clip non implementato: assumiamo in-range come MAME
					// default per HeatedBarrel — il game non usa sprite DMA via COP comunque)
					cop_reg_lo[4]      <= cop_reg_lo[4] + 16'd8;
					{cop_spr_dma_src_hi, cop_spr_dma_src_lo} <=
						{cop_spr_dma_src_hi, cop_spr_dma_src_lo} + 32'd6;
					if (cop_spr_dma_size != 16'd0) begin
						cop_spr_dma_size <= cop_spr_dma_size - 16'd1;
						if (cop_spr_dma_size == 16'd1) cop_status[1] <= 1'b1;
						else                            cop_status[1] <= 1'b0;
					end else begin
						cop_status[1] <= 1'b1;
					end
				end
				10'h009: cop_spr_dma_src_hi   <= wdata;  // 0x100412
				10'h00A: cop_spr_dma_src_lo   <= wdata;  // 0x100414
				10'h00E: cop_angle_target     <= wdata;  // 0x10041C
				10'h00F: cop_angle_step       <= wdata;  // 0x10041E
				10'h010: begin
					cop_itoa_low <= wdata;                            // 0x100420
					bcd_pending  <= 1'b1;
					bcd_step     <= 4'd0;
					bcd_val      <= {cop_itoa_high, wdata};
					bcd_quotient <= 32'd0;
				end
				10'h011: begin
					cop_itoa_high <= wdata;                           // 0x100422
					bcd_pending   <= 1'b1;
					bcd_step      <= 4'd0;
					bcd_val       <= {wdata, cop_itoa_low};
					bcd_quotient  <= 32'd0;
				end
				10'h012: cop_itoa_mode        <= wdata;  // 0x100424
				10'h014: cop_dma_v1           <= wdata;  // 0x100428
				10'h015: cop_dma_v2           <= wdata;  // 0x10042A
				10'h016: cop_prng_maxvalue    <= wdata;  // 0x10042C
				10'h01B: cop_hit_baseadr      <= wdata;  // 0x100436

				// Macro program upload — MAME seibucop.cpp lines 318-457
				// pgm_data writes one micro-op word to cop_program[latch_addr]
				//   AND latches the slot's trigger/value/mask from current latch regs.
				10'h019: begin                            // 0x100432 pgm_data
					cop_program[cop_latch_addr] <= wdata;
					cop_func_trigger[cop_latch_addr[7:3]] <= cop_latch_trigger;
					cop_func_value  [cop_latch_addr[7:3]] <= cop_latch_value;
					cop_func_mask   [cop_latch_addr[7:3]] <= cop_latch_mask;
				end
				10'h01A: cop_latch_addr    <= wdata[7:0]; // 0x100434 pgm_addr
				10'h01C: cop_latch_value   <= wdata;      // 0x100438 pgm_value
				10'h01D: cop_latch_mask    <= wdata;      // 0x10043A pgm_mask
				10'h01E: cop_latch_trigger <= wdata;      // 0x10043C pgm_trigger

				// Misc
				10'h020: cop_unk_param_a            <= wdata;  // 0x100440
				10'h021: cop_unk_param_b            <= wdata;  // 0x100442
				10'h022: cop_scale                  <= wdata & 16'h0003;  // 0x100444
				10'h02D: cop_pal_brightness_val     <= wdata;  // 0x10045A
				10'h02E: cop_pal_brightness_mode    <= wdata;  // 0x10045C
				10'h03B: cop_dma_adr_rel            <= wdata;  // 0x100476
				10'h03C: cop_dma_src [cop_dma_mode[7:0]] <= wdata;  // 0x100478
				10'h03D: cop_dma_size[cop_dma_mode[7:0]] <= wdata;  // 0x10047A
				10'h03E: cop_dma_dst [cop_dma_mode[7:0]] <= wdata;  // 0x10047C
				10'h03F: cop_dma_mode               <= wdata[8:0];  // 0x10047E
				10'h046: cop_spr_dma_abs_y          <= wdata;  // 0x10048C
				10'h047: cop_spr_dma_abs_x          <= wdata;  // 0x10048E

				// cop_regs hi (0x1004A0..0x1004AD = widx 0x50..0x56)
				10'h050: cop_reg_hi[0] <= wdata;
				10'h051: cop_reg_hi[1] <= wdata;
				10'h052: cop_reg_hi[2] <= wdata;
				10'h053: cop_reg_hi[3] <= wdata;
				10'h054: cop_reg_hi[4] <= wdata;
				10'h055: cop_reg_hi[5] <= wdata;
				10'h056: cop_reg_hi[6] <= wdata;

				// cop_regs lo (0x1004C0..0x1004CD = widx 0x60..0x66)
				10'h060: cop_reg_lo[0] <= wdata;
				10'h061: cop_reg_lo[1] <= wdata;
				10'h062: cop_reg_lo[2] <= wdata;
				10'h063: cop_reg_lo[3] <= wdata;
				10'h064: cop_reg_lo[4] <= wdata;
				10'h065: cop_reg_lo[5] <= wdata;
				10'h066: cop_reg_lo[6] <= wdata;

				default: ;
			endcase
		end

		// BCD update step (separato dal cpu_wr_pulse block).
		// Race protection: skip se cpu_wr_pulse appena scrive itoa low/high (i.e.
		// widx 0x010/0x011), perché il write CPU ha già azzerato bcd_step e
		// bcd_val viene sovrascritto. Il blocco sotto NON deve toccare bcd_val
		// nello stesso ciclo del write CPU.
		if (bcd_pending && !(cpu_wr_pulse && (widx == 10'h010 || widx == 10'h011))) begin
			if (bcd_val >= 32'd10) begin
				bcd_val       <= bcd_val - 32'd10;
				bcd_quotient  <= bcd_quotient + 32'd1;
			end else begin
				// val<10: digit_corrente = val, prossimo val = quoziente
				if (bcd_val == 32'd0 && bcd_quotient == 32'd0 && bcd_step != 4'd0) begin
					cop_itoa_digits[bcd_step] <= (cop_itoa_mode == 16'd3) ? 8'h30 : 8'h20;
				end else begin
					cop_itoa_digits[bcd_step] <= {4'h3, bcd_val[3:0]};
				end
				bcd_val      <= bcd_quotient;
				bcd_quotient <= 32'd0;
				if (bcd_step == 4'd8) bcd_pending <= 1'b0;
				else                  bcd_step    <= bcd_step + 4'd1;
			end
		end
	end

	// ────────────────────────────────────────────────────────────────────────
	// CPU reads — combinational on `addr`, registered output `rdata`
	// ────────────────────────────────────────────────────────────────────────
	always @(posedge clk) begin
		if (reset) begin
			rdata <= 16'hFFFF;
		end else if (cs && rd) begin
			case (widx)
				10'h016: rdata <= cop_prng_maxvalue;
				10'h03F: rdata <= {7'd0, cop_dma_mode};
				// cop_regs read-back
				10'h050: rdata <= cop_reg_hi[0];
				10'h051: rdata <= cop_reg_hi[1];
				10'h052: rdata <= cop_reg_hi[2];
				10'h053: rdata <= cop_reg_hi[3];
				10'h054: rdata <= cop_reg_hi[4];
				10'h055: rdata <= cop_reg_hi[5];
				10'h056: rdata <= cop_reg_hi[6];
				10'h060: rdata <= cop_reg_lo[0];
				10'h061: rdata <= cop_reg_lo[1];
				10'h062: rdata <= cop_reg_lo[2];
				10'h063: rdata <= cop_reg_lo[3];
				10'h064: rdata <= cop_reg_lo[4];
				10'h065: rdata <= cop_reg_lo[5];
				10'h066: rdata <= cop_reg_lo[6];
				// itoa_digits (MAME 0x100590-99, 5 word). digits[offset*2] | (digits[offset*2+1]<<8)
				10'h0C8: rdata <= {cop_itoa_digits[1], cop_itoa_digits[0]};  // 0x100590
				10'h0C9: rdata <= {cop_itoa_digits[3], cop_itoa_digits[2]};  // 0x100592
				10'h0CA: rdata <= {cop_itoa_digits[5], cop_itoa_digits[4]};  // 0x100594
				10'h0CB: rdata <= {cop_itoa_digits[7], cop_itoa_digits[6]};  // 0x100596
				10'h0CC: rdata <= {cop_itoa_digits[9], cop_itoa_digits[8]};  // 0x100598
				// Collision status
				10'h0C0: rdata <= cop_hit_status;    // 0x100580
				10'h0C1: rdata <= cop_hit_val[0];    // 0x100582 dY
				10'h0C2: rdata <= cop_hit_val[1];    // 0x100584 dX
				10'h0C3: rdata <= cop_hit_val[2];    // 0x100586 dZ
				10'h0C4: rdata <= cop_hit_val_stat;  // 0x100588
				// Status / dist / angle (MAME seibucop.cpp:916 cop_status_r → cop_status)
				10'h0D8: rdata <= cop_status;        // 0x1005B0
				10'h0D9: rdata <= cop_dist;          // 0x1005B2
				10'h0DA: rdata <= cop_angle;         // 0x1005B4
				// PRNG: lower N bits of cycle counter mod (max+1).
				// HW spec: total_cycles % (maxvalue + 1). We approximate by
				// returning the LSBs masked, which matches the period for
				// power-of-2 maxvalues commonly used by legionna.
				10'h0D0,10'h0D1,10'h0D2,10'h0D3: begin
					rdata <= cycle_cnt[15:0] & cop_prng_maxvalue;
				end
				default: rdata <= 16'hFFFF;  // MAME open bus
			endcase
		end
	end

	// ────────────────────────────────────────────────────────────────────────
	// FSM — serializes macro commands and DMA so port B / port A are
	// never used concurrently.
	// ────────────────────────────────────────────────────────────────────────
	localparam [6:0]
		S_IDLE       = 6'd0,
		// DMA paths
		D_PREP       = 6'd1,
		D_READ       = 6'd2,
		D_WRITE      = 6'd3,
		D_FILL_W     = 6'd4,
		// Macro: 0x0205 movement
		M_0205_RD_PPOS_HI = 6'd10,
		M_0205_RD_PPOS_LO = 6'd11,
		M_0205_RD_VEL_HI  = 6'd12,
		M_0205_RD_VEL_LO  = 6'd13,
		M_0205_WR_NPOS_HI = 6'd14,
		M_0205_WR_NPOS_LO = 6'd15,
		M_0205_RD_SCRN    = 6'd16,
		M_0205_WR_SCRN    = 6'd17,
		// Macro: 0x0905 jump (add velocity to dy)
		M_0905_RD_HI      = 6'd18,
		M_0905_RD_LO      = 6'd19,
		M_0905_RD_GRAV_HI = 6'd20,
		M_0905_RD_GRAV_LO = 6'd21,
		M_0905_WR_HI      = 6'd22,
		M_0905_WR_LO      = 6'd23,
		// Macro: 0x138e atan
		M_138E_RD0_Y_HI = 6'd24,
		M_138E_RD0_Y_LO = 6'd25,
		M_138E_RD1_Y_HI = 6'd26,
		M_138E_RD1_Y_LO = 6'd27,
		M_138E_RD0_X_HI = 6'd28,
		M_138E_RD0_X_LO = 6'd29,
		M_138E_RD1_X_HI = 6'd30,
		M_138E_RD1_X_LO = 6'd31,
		M_138E_CALC     = 6'd32,
		// Macro: 0x3bb0 dist
		M_3BB0_LOAD     = 6'd33,
		M_3BB0_CALC     = 6'd34,
		// Macro: 0x42c2 divide
		M_42C2_RD_DIV   = 6'd35,
		M_42C2_CALC     = 6'd36,
		// Macro: 0x8100/0x8900 sin/cos
		M_SC_RD_ANG     = 6'd37,
		M_SC_RD_AMP     = 6'd38,
		M_SC_CALC       = 6'd39,
		M_SC_WR_HI      = 6'd40,
		M_SC_WR_LO      = 6'd41,
		// Macro: 0xa180/0xa980 collision read pos
		M_A1_RD_FLAGS   = 6'd42,
		M_A1_RD_POS_Y   = 6'd43,
		M_A1_RD_POS_X   = 6'd44,
		M_A1_RD_POS_Z   = 6'd45,
		M_A1_RD_POS_ZW  = 6'd54,    // wait extra ciclo per latchare pos[2]
		// Macro: 0xb100/0xb900 collision update hitbox (stati REQ/WAIT ROM piu sotto)
		M_B1_CALC       = 6'd50,
		// DMA cmd 0x80 mode 5 palette fade: 2 read (src + target) + write blend
		D_FADE_RD_TGT   = 6'd51,    // emit target addr, latch paldata
		D_FADE_WAIT_TGT = 6'd52,    // wait target rdata
		D_FADE_CALC     = 7'd64,    // pipeline: calcola 6 fade_table (registra)
		D_FADE_WRITE    = 6'd53,    // pipeline: somme + write palette
		// (6'd54 = M_A1_RD_POS_ZW, già definito sopra)
		// atan 138e/338e: riusa gli stati M_138E_RD0_Y_HI..M_138E_CALC (24-32)
		//   per le 4 letture dword + il loop CORDIC dentro M_138E_CALC.
		// dist 3bb0: riusa M_3BB0_LOAD..M_3BB0_CALC (33-34) + stati extra qui sotto.
		// divide 42c2/4aa0: riusa M_42C2_RD_DIV..M_42C2_CALC (35-36) + extra.
		M_3BB0_RD_X1LO  = 6'd55,    // letture dword dx/dy per dist
		M_3BB0_RD_X0HI  = 6'd56,
		M_3BB0_RD_X0LO  = 6'd57,
		M_3BB0_RD_Y1HI  = 6'd58,
		M_3BB0_RD_Y1LO  = 6'd59,
		M_3BB0_RD_Y0HI  = 6'd60,
		M_3BB0_RD_Y0LO  = 6'd61,
		M_3BB0_SQRT     = 6'd62,    // restoring sqrt loop (16 iter via contatore)
		M_42C2_DIV      = 6'd63,    // divide loop (16 iter via contatore)
		M_3BB0_WR       = 6'd5,     // dist: write word cop_dist + set status
		M_42C2_WR       = 6'd6,     // divide: write result word (42c2/4aa0)
		M_3BB0_CALC2    = 6'd7,     // dist: stage 1 (differenze)
		M_3BB0_CALCM    = 6'd9,     // dist: stage 2 (moltiplicazioni)
		M_3BB0_CALC3    = 6'd8,     // dist: stage 3 (somma + init sqrt)
		M_138E_CORDIC   = 7'd65,    // atan: loop CORDIC vectoring 1 iter/ciclo
		M_138E_WR       = 7'd66,    // atan: write cop_angle + byte r0+0x34
		M_0205_WAIT_SCRN = 7'd67,   // 0205: wait latency BRAM su read 0x1C (scroll)
		M_B1_CALC2      = 7'd69,    // hitbox: stage 2 (comparatori overlap, path corto)
		// Hitbox b100/b900: TUTTE le letture (puntatore + 3 assi descrittore) sono
		// in MAIN ROM -> handshake req/ready via cop_rom_* (latency variabile).
		// Sequenza: REQ (1 ciclo req) -> WAIT (stalla finche ready, latcha) -> next.
		M_B1_REQ_PTR    = 7'd70,    // emit addr puntatore, req
		M_B1_WAIT_PTR   = 7'd71,    // wait ready, latcha puntatore -> hb_adr2
		M_B1_REQ_H0     = 7'd72,
		M_B1_WAIT_H0    = 7'd73,
		M_B1_REQ_H1     = 7'd74,
		M_B1_WAIT_H1    = 7'd75,
		M_B1_REQ_H2     = 7'd76,
		M_B1_WAIT_H2    = 7'd77,
		M_0905_RD0      = 7'd78;    // 0905: primo read vel_hi con macro_offset (coerente)

	reg [6:0]  fsm /*verilator public_flat_rd*/;
	reg [5:0]  return_state;     // tail-call target after a read latch
	reg        cmd_pending;
	reg [15:0] cmd_value;
	reg        dma_pending;
	// Extend dma_busy 1 ciclo dopo S_IDLE return per coprire l'ultimo write
	// pulse (dma_pal_we / dma_vram_we / dma_ram_we sono registered: scrittura
	// fisica avviene al posedge DOPO che fsm è tornato S_IDLE).
	reg        dma_busy_tail;
	reg        dma_busy_tail2;   // 2o ciclo di coda: margine per la read CPU dopo commit LSW
	// Trigger latches — MAME seibucop.cpp cop_cmd_w line 1069.
	// MAME dispatches on raw data after clearing cop_status bit 15.
	// PIPELINE: la ricerca canonical su 32 slot era combinational profonda
	// (43 livelli logici, -33ns slack a 96MHz). Spezzata in 3 stage registered:
	//   S1: trig_macro registra wdata in cmd_search_data + bit match[k]
	//   S2: priority encode su 32 match bit → cmd_match_slot[4:0]
	//   S3: lookup cop_func_trigger[slot] → cmd_value
	// (Dichiarati PRIMA dell'uso in dma_busy: ModelSim richiede decl-before-use.)
	reg [15:0] cmd_search_data;
	reg [1:0]  cmd_offset;        // 0..2 (word offset of cmd write: 0x100500/02/04)
	reg [1:0]  macro_offset;      // cmd_offset latchato all'avvio macro (immune a trigger successivi)
	reg [31:0] cmd_match_bits;
	reg [1:0]  cmd_search_state;  // 0=idle, 1=do_match, 2=do_encode, 3=lookup
	reg [4:0]  cmd_match_slot;
	reg        cmd_match_found;
	integer    pe_k;

	// dma_busy_tail a 2 CICLI: l'ultima write COP in Main RAM (es. 0905 LSW $4e,
	// parte bassa di $4c=velZ) si committa nel ciclo S_IDLE DOPO che fsm e' tornato
	// IDLE. Con tail a 1 ciclo dma_busy cadeva NELLO STESSO ciclo del commit LSW ->
	// margine ZERO -> dipendente dal fasamento cpu_cen: nel caso peggiore la CPU
	// rilegge $4c ($7640 move.l $4c,d0 subito dopo il 0905) STANTIO di 1 frame ->
	// la traiettoria Z del salto del bruto diverge -> il rimbalzo converge in 5 colpi
	// invece di 3 -> $48 (rampa decel) attraversa lo zero -> il bruto torna indietro.
	// 2 cicli danno il margine: la read CPU vede SEMPRE la write committata. +1 ciclo
	// di stall per macro (trascurabile). Solo timing della porta, NON logica.
	always @(posedge clk) begin
		if (reset) begin dma_busy_tail <= 1'b0; dma_busy_tail2 <= 1'b0; end
		else begin
			dma_busy_tail  <= (fsm != S_IDLE);
			dma_busy_tail2 <= dma_busy_tail;
		end
	end
	// dma_busy: FSM-based ONLY (drives port MUXes in main_top). True solo nei
	// cicli in cui cop_dma_* pilotano valori VALIDI. NON include i trigger
	// combinatori (che salirebbero quando cop_dma_* sono ancora stale -> i mux
	// dirotterebbero BG/FG/MG/TXT/RAM/sprite verso garbage -> moonwalk/strabordo).
	// bcd_pending RIMOSSO da dma_busy: la conversione BCD (itoa) e' multi-ciclo (1 sub/-10
	// per clock, fino a ~V/9 cicli) e teneva cpu_stall alto -> CPU congelata via DTACK ad ogni
	// write itoa (score->cifre). Effetto: animazioni CPU-paced (es. scomparsa logo TAD title)
	// rallentate 14x (14s invece di istantaneo). MAME converte itoa ISTANTANEO (seibucop.cpp:858).
	// Il BCD non usa porte A/B DMA -> toglierlo da dma_busy non tocca i mux BG/FG/MG/TXT/RAM/spr.
	// I digit sono riletti il frame DOPO la write ($6A74->$6A7E, $85B4->$85BE) -> multi-ciclo innocuo.
	assign     dma_busy = (fsm != S_IDLE) || cmd_pending || dma_pending || dma_busy_tail ||
	                       dma_busy_tail2 || (cmd_search_state != 2'd0);

	// cpu_stall: dma_busy + i trigger combinatori (cpu_wr_pulse). I trigger alzano
	// lo stall GIA' nel ciclo del write del trigger, chiudendo il buco di 1 ciclo
	// tra comandi COP consecutivi (es. b100->b900 nel check-confine a 4 trigger) in
	// cui tutti i termini FSM sono 0 (fsm=S_IDLE, cmd_pending consumato, tail
	// scaduto, cmd_search_state non ancora salito). Senza, DTACKn cadeva 1 ciclo ->
	// la CPU leggeva cop_hit_val_stat RESIDUO prima del commit di b900 -> nemici non
	// bloccati ai confini. Influenza SOLO bus_busy, NON i mux di porta.
	//
	assign     cpu_stall = dma_busy || trig_macro || trig_dma;

	// Per-FSM working registers usati dagli assign dma_vram_*_now sotto: dichiarati
	// QUI (prima dell'uso) per compatibilità con tool Verilog strict (ModelSim).
	// Quartus/Verilator accettano l'uso-prima-della-decl, ModelSim no.
	reg [15:0] dma_dst;             // local dest word index (BG/Pal i)
	reg [8:0]  dma_mode_lat;

	// Current-cycle VRAM write intent. cmd 0x14 (D_WRITE) usa dma_dst (indice 0-based).
	// cmd 0x118 FILL (D_FILL_W) verso VRAM: heatbrl usa 0x118 per pulire le tilemap
	// (disasm $1B60/$1AF8/$1B2C: fill $100800/$101000/$101800). L'indirizzo VRAM assoluto
	// $100800+ mappa all'indice render 0-based: (dma_src - $100800) >> 1.
	// cmd 0x118-0x11F: dma_mode_lat[8]=1 e [7:4]=1 (0x11x). Il fill copre BG/FG/MG/TEXT
	// ($100800-$102FFF) E la SPRITE RAM ($103000-$103FFF). heatbrl azzera gli sprite col
	// fill 0x118 su $103000 (disasm $1B2C: src $40C0 = $103000): senza instradarlo alla
	// sprite RAM, il logo TAD non si cancella (persiste, "scompare 1 pezzo alla volta").
	// IL FILL PULISCE LO SCRATCH (le VRAM da cui il DMA 0x14 legge); per gli sprite pulisce
	// la sprite RAM CPU-side (u_spr_ram_cpu) da cui lo shadow copia.
	wire        fill_is_vram = (dma_mode_lat[8:4] == 5'b1_0001)
	                           && (dma_src >= 24'h100800) && (dma_src < 24'h104000);
	assign dma_vram_we_now   = (fsm == D_WRITE)  && (dma_mode_lat[3:0] == 4'h4);
	assign dma_vram_addr_now = dma_dst[12:0];
	assign dma_vram_wdata_now = dma_src_rdata;
	// Fill -> scratch: esposti al main_top (scrive nelle BRAM scratch bg/fg/mg/txt).
	assign dma_fill_we    = (fsm == D_FILL_W) && fill_is_vram;
	assign dma_fill_addr  = dma_src[23:1];        // byte_addr/2 = indirizzo VRAM word assoluto
	assign dma_fill_wdata = cop_dma_v1;

	always @(posedge clk) begin : trig_blk
		if (reset) begin
			cmd_pending      <= 1'b0;
			cmd_value        <= 0;
			cmd_offset       <= 2'd0;
			dma_pending      <= 1'b0;
			cmd_search_data  <= 0;
			cmd_match_bits   <= 32'd0;
			cmd_search_state <= 2'd0;
			cmd_match_slot   <= 5'd0;
			cmd_match_found  <= 1'b0;
		end else begin
			case (cmd_search_state)
				2'd0: begin // idle
					if (trig_macro) begin
						cmd_search_data  <= wdata;
						cmd_offset       <= widx[1:0];   // 0=0x100500, 1=0x100502, 2=0x100504
						cmd_search_state <= 2'd1;
					end
				end
				2'd1: begin // S1: compute match[k] for all 32 slots (parallel, 1 ciclo)
					for (pe_k = 0; pe_k < 32; pe_k = pe_k + 1) begin
						cmd_match_bits[pe_k] <= (cop_func_trigger[pe_k] != 16'd0) &&
						   ((cmd_search_data & 16'hf800) == (cop_func_trigger[pe_k] & 16'hf800));
					end
					cmd_search_state <= 2'd2;
				end
				2'd2: begin // S2: priority encode (first hit wins) — tree-based log2(32)=5 levels
					// Casewise priority encoder con cascade comb 5 livelli.
					casez (cmd_match_bits)
						32'b???????????????????????????????1: begin cmd_match_slot <= 5'd0;  cmd_match_found <= 1'b1; end
						32'b??????????????????????????????10: begin cmd_match_slot <= 5'd1;  cmd_match_found <= 1'b1; end
						32'b?????????????????????????????100: begin cmd_match_slot <= 5'd2;  cmd_match_found <= 1'b1; end
						32'b????????????????????????????1000: begin cmd_match_slot <= 5'd3;  cmd_match_found <= 1'b1; end
						32'b???????????????????????????10000: begin cmd_match_slot <= 5'd4;  cmd_match_found <= 1'b1; end
						32'b??????????????????????????100000: begin cmd_match_slot <= 5'd5;  cmd_match_found <= 1'b1; end
						32'b?????????????????????????1000000: begin cmd_match_slot <= 5'd6;  cmd_match_found <= 1'b1; end
						32'b????????????????????????10000000: begin cmd_match_slot <= 5'd7;  cmd_match_found <= 1'b1; end
						32'b???????????????????????100000000: begin cmd_match_slot <= 5'd8;  cmd_match_found <= 1'b1; end
						32'b??????????????????????1000000000: begin cmd_match_slot <= 5'd9;  cmd_match_found <= 1'b1; end
						32'b?????????????????????10000000000: begin cmd_match_slot <= 5'd10; cmd_match_found <= 1'b1; end
						32'b????????????????????100000000000: begin cmd_match_slot <= 5'd11; cmd_match_found <= 1'b1; end
						32'b???????????????????1000000000000: begin cmd_match_slot <= 5'd12; cmd_match_found <= 1'b1; end
						32'b??????????????????10000000000000: begin cmd_match_slot <= 5'd13; cmd_match_found <= 1'b1; end
						32'b?????????????????100000000000000: begin cmd_match_slot <= 5'd14; cmd_match_found <= 1'b1; end
						32'b????????????????1000000000000000: begin cmd_match_slot <= 5'd15; cmd_match_found <= 1'b1; end
						32'b???????????????10000000000000000: begin cmd_match_slot <= 5'd16; cmd_match_found <= 1'b1; end
						32'b??????????????100000000000000000: begin cmd_match_slot <= 5'd17; cmd_match_found <= 1'b1; end
						32'b?????????????1000000000000000000: begin cmd_match_slot <= 5'd18; cmd_match_found <= 1'b1; end
						32'b????????????10000000000000000000: begin cmd_match_slot <= 5'd19; cmd_match_found <= 1'b1; end
						32'b???????????100000000000000000000: begin cmd_match_slot <= 5'd20; cmd_match_found <= 1'b1; end
						32'b??????????1000000000000000000000: begin cmd_match_slot <= 5'd21; cmd_match_found <= 1'b1; end
						32'b?????????10000000000000000000000: begin cmd_match_slot <= 5'd22; cmd_match_found <= 1'b1; end
						32'b????????100000000000000000000000: begin cmd_match_slot <= 5'd23; cmd_match_found <= 1'b1; end
						32'b???????1000000000000000000000000: begin cmd_match_slot <= 5'd24; cmd_match_found <= 1'b1; end
						32'b??????10000000000000000000000000: begin cmd_match_slot <= 5'd25; cmd_match_found <= 1'b1; end
						32'b?????100000000000000000000000000: begin cmd_match_slot <= 5'd26; cmd_match_found <= 1'b1; end
						32'b????1000000000000000000000000000: begin cmd_match_slot <= 5'd27; cmd_match_found <= 1'b1; end
						32'b???10000000000000000000000000000: begin cmd_match_slot <= 5'd28; cmd_match_found <= 1'b1; end
						32'b??100000000000000000000000000000: begin cmd_match_slot <= 5'd29; cmd_match_found <= 1'b1; end
						32'b?1000000000000000000000000000000: begin cmd_match_slot <= 5'd30; cmd_match_found <= 1'b1; end
						32'b10000000000000000000000000000000: begin cmd_match_slot <= 5'd31; cmd_match_found <= 1'b1; end
						default:                              begin cmd_match_slot <= 5'd0;  cmd_match_found <= 1'b0; end
					endcase
					cmd_search_state <= 2'd3;
				end
				2'd3: begin // S3: signal cmd_pending. MAME cop_cmd_w (seibucop.cpp:1075):
					// switch(data) raw, NON canonical. find_trigger_match è solo per log.
					// Quindi cmd_value = wdata originale del game.
					cmd_value        <= cmd_search_data;
					cmd_pending      <= 1'b1;
					cmd_search_state <= 2'd0;
				end
			endcase

			if (trig_dma) begin
				dma_pending <= 1'b1;
			end
			// FSM consumes pending in S_IDLE → clears them itself
			if (fsm == S_IDLE && (cmd_pending || dma_pending)) begin
				cmd_pending <= 1'b0;
				dma_pending <= 1'b0;
			end
		end
	end

	// Sprite DMA inc status — merged into CPU writes block to avoid multi-driver.
	// See trig_spr_inc branch inside the main always block above (case 10'h008).

	// ────────────────────────────────────────────────────────────────────────
	// cop_regs_byte_addr: helper per ottenere il byte-address 24-bit completo
	// di cop_regs[N] = {hi, lo}. cop_reg_hi è già scritto dalla CPU al boot
	// (vedi routine $3290 in maincpu: scrive hi a $1004A0 + lo a $1004C0).
	function [23:0] cop_regs_byte_addr(input integer n, input [15:0] offs);
		begin
			cop_regs_byte_addr = {cop_reg_hi[n][7:0], cop_reg_lo[n]} + {8'd0, offs};
		end
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// find_trigger_match (MAME seibucop.cpp:464)
	// Cerca slot N tale che (trigger & 0xf800) == (cop_func_trigger[N] & 0xf800)
	// && cop_func_trigger[N] != 0. Restituisce canonical trigger value del slot
	// (NB: MAME LEGACY_cop_cmd_w usa il match slot per dispatchare; in pratica
	// equivale a restituire cop_func_trigger[slot]).
	// Se nessun match: ritorna l'input originale (= no-op nel dispatcher).
	function [15:0] cop_trigger_canonical(input [15:0] trig);
		reg [15:0] r;
		integer k;
		begin
			r = trig;
			for (k = 0; k < 32; k = k + 1) begin
				if (cop_func_trigger[k] != 16'd0 &&
				    ((trig & 16'hf800) == (cop_func_trigger[k] & 16'hf800)) &&
				    r == trig) begin
					r = cop_func_trigger[k];
				end
			end
			cop_trigger_canonical = r;
		end
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// fade_table — MAME seibucop.cpp:733 (reverse engineered from Seibu Cup Soccer bootleg)
	//   v = pal_channel(5-bit) | (brightness_val_xor_X)  [10-bit input]
	//   low  = v & 0x1F
	//   high = v & 0x3E0
	//   return (low * (high | (high>>5)) + 0x210) >> 10  [8-bit out, range 0..31]
	function [7:0] fade_table_fn(input [9:0] v);
		reg [4:0] low;
		reg [9:0] high;
		reg [9:0] hi_dup;
		reg [19:0] prod;
		reg [19:0] sum;
		begin
			low    = v[4:0];
			high   = {v[9:5], 5'b00000};                     // bit 9..5, low cleared
			hi_dup = high | {5'b00000, v[9:5]};              // high | (high>>5)
			prod   = low * hi_dup;
			sum    = prod + 20'h00210;
			fade_table_fn = sum[17:10];                       // >> 10 → bit [17:10]
		end
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// Per-FSM working registers
	// ────────────────────────────────────────────────────────────────────────
	reg [15:0] dma_cnt;
	reg [23:0] dma_src;             // byte address (full 68K bus)
	// dma_dst / dma_mode_lat dichiarati più in alto (prima degli assign *_now).

	// Macro scratch
	reg [15:0] tmp_hi, tmp_lo;
	// Fade blend pipeline: risultati fade_table registrati (spezza 6 mul in 2 stadi)
	reg  [7:0] fade_fb_t, fade_fb_c, fade_fg_t, fade_fg_c, fade_fr_t, fade_fr_c;
	reg [15:0] fade_paldata;
	reg        fade_nofade;
	reg [31:0] tmp32_a, tmp32_b;
	reg [15:0] m_target_addr;        // used by M_*_WR_*

	// ── Hitbox b100/b900 scratch (MAME cop_collision_update_hitbox) ──────────────
	// dx[i] = int8 (signed) byte basso, size[i] = uint8 (unsigned) byte alto del
	// word descrittore @ hitadr2+2*i. 3 assi (Y,X,Z) per HeatedBarrel.
	reg signed [15:0] hb_dx   [0:2] /*verilator public_flat_rd*/;
	reg        [15:0] hb_size [0:2] /*verilator public_flat_rd*/;
	reg        [23:0] hb_adr2 /*verilator public_flat_rd*/;        // base descrittore hitbox (in ROM)
	reg        [23:0] hb_ptr_addr;    // addr del puntatore (cop_regs[2/3], in ROM)
	reg               hb_3axis;       // 1 = 3 assi (cmd_value[8]=1, legionna); 0 = 2 assi (heatbrl).
	                                  // MAME: res init = (3axis ? 7 : 3) -> con 2 assi il bit2 NON
	                                  // deve restare 1 (altrimenti res mai 0 -> collisione mai rilevata).

	// ── CORDIC / math scratch (atan 138e, dist 3bb0, divide 42c2) ──────────────
	// dx/dy 32-bit signed (differenze posizione r1-r0), risultati cop_angle/cop_dist.
	reg signed [31:0] math_dx, math_dy;
	reg signed [31:0] cordic_x, cordic_y;     // CORDIC vectoring accumulators
	reg signed [23:0] cordic_z;               // angolo accumulato ×65536 (16-bit frazione)
	reg        [4:0]  cordic_i;               // iterazione 0..23
	reg signed [31:0] sqrt_acc, sqrt_rem;     // Newton/restoring sqrt
	// dist 3bb0 restoring sqrt scratch (34-bit resto + radicando shiftato)
	reg        [33:0] sqrt_rem34;
	reg        [31:0] sqrt_radsh;
	reg        [4:0]  sqrt_i;
	reg        [31:0] div_num;                // dividendo per 42c2/4aa0
	reg        [15:0] div_den;
	reg        [5:0]  div_i;                  // 6-bit per contare 32 iter (review D1)
	reg        [31:0] div_q, div_r;

	// atan LUT: arctan(2^-i) in unità angolo Seibu SCALATE ×256 (1/256 di unità).
	//   valore = round( arctan(2^-i) * 128/π * 256 ).  i=0..15.
	// Il loop CORDIC accumula in cordic_z (×256); il finale fa (z+128)>>8 per
	// tornare a unità intere con rounding. Valori esatti (calcolati, no approx):
	// atan LUT a 16-BIT DI FRAZIONE (angolo_byte ×65536), 24 entry. La precisione
	// ×256/16-iter sovrastimava ~+0.006 al bordo intero -> cop_angle cadeva sul lato
	// sbagliato del bordo della tabella-decisione $48DA del nemico-martello (0x07/08,
	// 0x77/78, 0x87/88, 0xf7/f8) -> non azzerava velX -> il bruto SLITTAVA durante
	// l'attacco. Con 16-bit frac + 24 iter + z>>16: 0 divergenze di decisione vs MAME
	// int(atan(dx/dy)*128/pi) su ~640k casi (dimostrato numericamente).
	function signed [23:0] cordic_atan_lut(input [4:0] i);
		case (i)
			5'd0:  cordic_atan_lut = 24'sd2097152; // 45.000° ×65536
			5'd1:  cordic_atan_lut = 24'sd1238021; // 26.565°
			5'd2:  cordic_atan_lut = 24'sd654136;  // 14.036°
			5'd3:  cordic_atan_lut = 24'sd332050;  // 7.125°
			5'd4:  cordic_atan_lut = 24'sd166669;  // 3.576°
			5'd5:  cordic_atan_lut = 24'sd83416;   // 1.790°
			5'd6:  cordic_atan_lut = 24'sd41718;   // 0.895°
			5'd7:  cordic_atan_lut = 24'sd20860;   // 0.448°
			5'd8:  cordic_atan_lut = 24'sd10430;   // 0.224°
			5'd9:  cordic_atan_lut = 24'sd5215;    // 0.112°
			5'd10: cordic_atan_lut = 24'sd2608;    // 0.056°
			5'd11: cordic_atan_lut = 24'sd1304;
			5'd12: cordic_atan_lut = 24'sd652;
			5'd13: cordic_atan_lut = 24'sd326;
			5'd14: cordic_atan_lut = 24'sd163;
			5'd15: cordic_atan_lut = 24'sd81;
			5'd16: cordic_atan_lut = 24'sd41;
			5'd17: cordic_atan_lut = 24'sd20;
			5'd18: cordic_atan_lut = 24'sd10;
			5'd19: cordic_atan_lut = 24'sd5;
			5'd20: cordic_atan_lut = 24'sd3;
			5'd21: cordic_atan_lut = 24'sd1;
			default: cordic_atan_lut = 24'sd0;   // i>=22
		endcase
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// Sin/Cos table — 256 entries of (sin(i * π/128) * 32768), signed 16-bit
	// Precomputed; covers the full 8-bit angle space.
	// ────────────────────────────────────────────────────────────────────────
	function [15:0] sin_table_lookup(input [7:0] a);
		case (a)
			8'h00:sin_table_lookup=16'sd0;     8'h01:sin_table_lookup=16'sd804;
			8'h02:sin_table_lookup=16'sd1608;  8'h03:sin_table_lookup=16'sd2410;
			8'h04:sin_table_lookup=16'sd3212;  8'h05:sin_table_lookup=16'sd4011;
			8'h06:sin_table_lookup=16'sd4808;  8'h07:sin_table_lookup=16'sd5602;
			8'h08:sin_table_lookup=16'sd6393;  8'h09:sin_table_lookup=16'sd7179;
			8'h0a:sin_table_lookup=16'sd7962;  8'h0b:sin_table_lookup=16'sd8739;
			8'h0c:sin_table_lookup=16'sd9512;  8'h0d:sin_table_lookup=16'sd10278;
			8'h0e:sin_table_lookup=16'sd11039; 8'h0f:sin_table_lookup=16'sd11793;
			8'h10:sin_table_lookup=16'sd12539; 8'h11:sin_table_lookup=16'sd13279;
			8'h12:sin_table_lookup=16'sd14010; 8'h13:sin_table_lookup=16'sd14732;
			8'h14:sin_table_lookup=16'sd15446; 8'h15:sin_table_lookup=16'sd16151;
			8'h16:sin_table_lookup=16'sd16846; 8'h17:sin_table_lookup=16'sd17530;
			8'h18:sin_table_lookup=16'sd18204; 8'h19:sin_table_lookup=16'sd18868;
			8'h1a:sin_table_lookup=16'sd19519; 8'h1b:sin_table_lookup=16'sd20159;
			8'h1c:sin_table_lookup=16'sd20787; 8'h1d:sin_table_lookup=16'sd21403;
			8'h1e:sin_table_lookup=16'sd22005; 8'h1f:sin_table_lookup=16'sd22594;
			8'h20:sin_table_lookup=16'sd23170; 8'h21:sin_table_lookup=16'sd23731;
			8'h22:sin_table_lookup=16'sd24279; 8'h23:sin_table_lookup=16'sd24811;
			8'h24:sin_table_lookup=16'sd25329; 8'h25:sin_table_lookup=16'sd25832;
			8'h26:sin_table_lookup=16'sd26319; 8'h27:sin_table_lookup=16'sd26790;
			8'h28:sin_table_lookup=16'sd27245; 8'h29:sin_table_lookup=16'sd27683;
			8'h2a:sin_table_lookup=16'sd28105; 8'h2b:sin_table_lookup=16'sd28510;
			8'h2c:sin_table_lookup=16'sd28898; 8'h2d:sin_table_lookup=16'sd29268;
			8'h2e:sin_table_lookup=16'sd29621; 8'h2f:sin_table_lookup=16'sd29956;
			8'h30:sin_table_lookup=16'sd30273; 8'h31:sin_table_lookup=16'sd30571;
			8'h32:sin_table_lookup=16'sd30852; 8'h33:sin_table_lookup=16'sd31113;
			8'h34:sin_table_lookup=16'sd31356; 8'h35:sin_table_lookup=16'sd31580;
			8'h36:sin_table_lookup=16'sd31785; 8'h37:sin_table_lookup=16'sd31971;
			8'h38:sin_table_lookup=16'sd32137; 8'h39:sin_table_lookup=16'sd32285;
			8'h3a:sin_table_lookup=16'sd32412; 8'h3b:sin_table_lookup=16'sd32521;
			8'h3c:sin_table_lookup=16'sd32609; 8'h3d:sin_table_lookup=16'sd32678;
			8'h3e:sin_table_lookup=16'sd32728; 8'h3f:sin_table_lookup=16'sd32757;
			8'h40:sin_table_lookup=16'sd32767; 8'h41:sin_table_lookup=16'sd32757;
			8'h42:sin_table_lookup=16'sd32728; 8'h43:sin_table_lookup=16'sd32678;
			8'h44:sin_table_lookup=16'sd32609; 8'h45:sin_table_lookup=16'sd32521;
			8'h46:sin_table_lookup=16'sd32412; 8'h47:sin_table_lookup=16'sd32285;
			8'h48:sin_table_lookup=16'sd32137; 8'h49:sin_table_lookup=16'sd31971;
			8'h4a:sin_table_lookup=16'sd31785; 8'h4b:sin_table_lookup=16'sd31580;
			8'h4c:sin_table_lookup=16'sd31356; 8'h4d:sin_table_lookup=16'sd31113;
			8'h4e:sin_table_lookup=16'sd30852; 8'h4f:sin_table_lookup=16'sd30571;
			8'h50:sin_table_lookup=16'sd30273; 8'h51:sin_table_lookup=16'sd29956;
			8'h52:sin_table_lookup=16'sd29621; 8'h53:sin_table_lookup=16'sd29268;
			8'h54:sin_table_lookup=16'sd28898; 8'h55:sin_table_lookup=16'sd28510;
			8'h56:sin_table_lookup=16'sd28105; 8'h57:sin_table_lookup=16'sd27683;
			8'h58:sin_table_lookup=16'sd27245; 8'h59:sin_table_lookup=16'sd26790;
			8'h5a:sin_table_lookup=16'sd26319; 8'h5b:sin_table_lookup=16'sd25832;
			8'h5c:sin_table_lookup=16'sd25329; 8'h5d:sin_table_lookup=16'sd24811;
			8'h5e:sin_table_lookup=16'sd24279; 8'h5f:sin_table_lookup=16'sd23731;
			8'h60:sin_table_lookup=16'sd23170; 8'h61:sin_table_lookup=16'sd22594;
			8'h62:sin_table_lookup=16'sd22005; 8'h63:sin_table_lookup=16'sd21403;
			8'h64:sin_table_lookup=16'sd20787; 8'h65:sin_table_lookup=16'sd20159;
			8'h66:sin_table_lookup=16'sd19519; 8'h67:sin_table_lookup=16'sd18868;
			8'h68:sin_table_lookup=16'sd18204; 8'h69:sin_table_lookup=16'sd17530;
			8'h6a:sin_table_lookup=16'sd16846; 8'h6b:sin_table_lookup=16'sd16151;
			8'h6c:sin_table_lookup=16'sd15446; 8'h6d:sin_table_lookup=16'sd14732;
			8'h6e:sin_table_lookup=16'sd14010; 8'h6f:sin_table_lookup=16'sd13279;
			8'h70:sin_table_lookup=16'sd12539; 8'h71:sin_table_lookup=16'sd11793;
			8'h72:sin_table_lookup=16'sd11039; 8'h73:sin_table_lookup=16'sd10278;
			8'h74:sin_table_lookup=16'sd9512;  8'h75:sin_table_lookup=16'sd8739;
			8'h76:sin_table_lookup=16'sd7962;  8'h77:sin_table_lookup=16'sd7179;
			8'h78:sin_table_lookup=16'sd6393;  8'h79:sin_table_lookup=16'sd5602;
			8'h7a:sin_table_lookup=16'sd4808;  8'h7b:sin_table_lookup=16'sd4011;
			8'h7c:sin_table_lookup=16'sd3212;  8'h7d:sin_table_lookup=16'sd2410;
			8'h7e:sin_table_lookup=16'sd1608;  8'h7f:sin_table_lookup=16'sd804;
			default: begin
				// Negative half: sin(x+π) = -sin(x). Lookup mirror.
				sin_table_lookup = -sin_table_lookup_lo(a - 8'h80);
			end
		endcase
	endfunction

	// Helper for negative-half lookup
	function [15:0] sin_table_lookup_lo(input [7:0] a);
		// Identical to first half (a in 0..7F) — duplicated to avoid recursion.
		case (a)
			8'h00:sin_table_lookup_lo=16'sd0;     8'h01:sin_table_lookup_lo=16'sd804;
			8'h02:sin_table_lookup_lo=16'sd1608;  8'h03:sin_table_lookup_lo=16'sd2410;
			8'h04:sin_table_lookup_lo=16'sd3212;  8'h05:sin_table_lookup_lo=16'sd4011;
			8'h06:sin_table_lookup_lo=16'sd4808;  8'h07:sin_table_lookup_lo=16'sd5602;
			8'h08:sin_table_lookup_lo=16'sd6393;  8'h09:sin_table_lookup_lo=16'sd7179;
			8'h0a:sin_table_lookup_lo=16'sd7962;  8'h0b:sin_table_lookup_lo=16'sd8739;
			8'h0c:sin_table_lookup_lo=16'sd9512;  8'h0d:sin_table_lookup_lo=16'sd10278;
			8'h0e:sin_table_lookup_lo=16'sd11039; 8'h0f:sin_table_lookup_lo=16'sd11793;
			8'h10:sin_table_lookup_lo=16'sd12539; 8'h11:sin_table_lookup_lo=16'sd13279;
			8'h12:sin_table_lookup_lo=16'sd14010; 8'h13:sin_table_lookup_lo=16'sd14732;
			8'h14:sin_table_lookup_lo=16'sd15446; 8'h15:sin_table_lookup_lo=16'sd16151;
			8'h16:sin_table_lookup_lo=16'sd16846; 8'h17:sin_table_lookup_lo=16'sd17530;
			8'h18:sin_table_lookup_lo=16'sd18204; 8'h19:sin_table_lookup_lo=16'sd18868;
			8'h1a:sin_table_lookup_lo=16'sd19519; 8'h1b:sin_table_lookup_lo=16'sd20159;
			8'h1c:sin_table_lookup_lo=16'sd20787; 8'h1d:sin_table_lookup_lo=16'sd21403;
			8'h1e:sin_table_lookup_lo=16'sd22005; 8'h1f:sin_table_lookup_lo=16'sd22594;
			8'h20:sin_table_lookup_lo=16'sd23170; 8'h21:sin_table_lookup_lo=16'sd23731;
			8'h22:sin_table_lookup_lo=16'sd24279; 8'h23:sin_table_lookup_lo=16'sd24811;
			8'h24:sin_table_lookup_lo=16'sd25329; 8'h25:sin_table_lookup_lo=16'sd25832;
			8'h26:sin_table_lookup_lo=16'sd26319; 8'h27:sin_table_lookup_lo=16'sd26790;
			8'h28:sin_table_lookup_lo=16'sd27245; 8'h29:sin_table_lookup_lo=16'sd27683;
			8'h2a:sin_table_lookup_lo=16'sd28105; 8'h2b:sin_table_lookup_lo=16'sd28510;
			8'h2c:sin_table_lookup_lo=16'sd28898; 8'h2d:sin_table_lookup_lo=16'sd29268;
			8'h2e:sin_table_lookup_lo=16'sd29621; 8'h2f:sin_table_lookup_lo=16'sd29956;
			8'h30:sin_table_lookup_lo=16'sd30273; 8'h31:sin_table_lookup_lo=16'sd30571;
			8'h32:sin_table_lookup_lo=16'sd30852; 8'h33:sin_table_lookup_lo=16'sd31113;
			8'h34:sin_table_lookup_lo=16'sd31356; 8'h35:sin_table_lookup_lo=16'sd31580;
			8'h36:sin_table_lookup_lo=16'sd31785; 8'h37:sin_table_lookup_lo=16'sd31971;
			8'h38:sin_table_lookup_lo=16'sd32137; 8'h39:sin_table_lookup_lo=16'sd32285;
			8'h3a:sin_table_lookup_lo=16'sd32412; 8'h3b:sin_table_lookup_lo=16'sd32521;
			8'h3c:sin_table_lookup_lo=16'sd32609; 8'h3d:sin_table_lookup_lo=16'sd32678;
			8'h3e:sin_table_lookup_lo=16'sd32728; 8'h3f:sin_table_lookup_lo=16'sd32757;
			8'h40:sin_table_lookup_lo=16'sd32767; 8'h41:sin_table_lookup_lo=16'sd32757;
			8'h42:sin_table_lookup_lo=16'sd32728; 8'h43:sin_table_lookup_lo=16'sd32678;
			8'h44:sin_table_lookup_lo=16'sd32609; 8'h45:sin_table_lookup_lo=16'sd32521;
			8'h46:sin_table_lookup_lo=16'sd32412; 8'h47:sin_table_lookup_lo=16'sd32285;
			8'h48:sin_table_lookup_lo=16'sd32137; 8'h49:sin_table_lookup_lo=16'sd31971;
			8'h4a:sin_table_lookup_lo=16'sd31785; 8'h4b:sin_table_lookup_lo=16'sd31580;
			8'h4c:sin_table_lookup_lo=16'sd31356; 8'h4d:sin_table_lookup_lo=16'sd31113;
			8'h4e:sin_table_lookup_lo=16'sd30852; 8'h4f:sin_table_lookup_lo=16'sd30571;
			8'h50:sin_table_lookup_lo=16'sd30273; 8'h51:sin_table_lookup_lo=16'sd29956;
			8'h52:sin_table_lookup_lo=16'sd29621; 8'h53:sin_table_lookup_lo=16'sd29268;
			8'h54:sin_table_lookup_lo=16'sd28898; 8'h55:sin_table_lookup_lo=16'sd28510;
			8'h56:sin_table_lookup_lo=16'sd28105; 8'h57:sin_table_lookup_lo=16'sd27683;
			8'h58:sin_table_lookup_lo=16'sd27245; 8'h59:sin_table_lookup_lo=16'sd26790;
			8'h5a:sin_table_lookup_lo=16'sd26319; 8'h5b:sin_table_lookup_lo=16'sd25832;
			8'h5c:sin_table_lookup_lo=16'sd25329; 8'h5d:sin_table_lookup_lo=16'sd24811;
			8'h5e:sin_table_lookup_lo=16'sd24279; 8'h5f:sin_table_lookup_lo=16'sd23731;
			8'h60:sin_table_lookup_lo=16'sd23170; 8'h61:sin_table_lookup_lo=16'sd22594;
			8'h62:sin_table_lookup_lo=16'sd22005; 8'h63:sin_table_lookup_lo=16'sd21403;
			8'h64:sin_table_lookup_lo=16'sd20787; 8'h65:sin_table_lookup_lo=16'sd20159;
			8'h66:sin_table_lookup_lo=16'sd19519; 8'h67:sin_table_lookup_lo=16'sd18868;
			8'h68:sin_table_lookup_lo=16'sd18204; 8'h69:sin_table_lookup_lo=16'sd17530;
			8'h6a:sin_table_lookup_lo=16'sd16846; 8'h6b:sin_table_lookup_lo=16'sd16151;
			8'h6c:sin_table_lookup_lo=16'sd15446; 8'h6d:sin_table_lookup_lo=16'sd14732;
			8'h6e:sin_table_lookup_lo=16'sd14010; 8'h6f:sin_table_lookup_lo=16'sd13279;
			8'h70:sin_table_lookup_lo=16'sd12539; 8'h71:sin_table_lookup_lo=16'sd11793;
			8'h72:sin_table_lookup_lo=16'sd11039; 8'h73:sin_table_lookup_lo=16'sd10278;
			8'h74:sin_table_lookup_lo=16'sd9512;  8'h75:sin_table_lookup_lo=16'sd8739;
			8'h76:sin_table_lookup_lo=16'sd7962;  8'h77:sin_table_lookup_lo=16'sd7179;
			8'h78:sin_table_lookup_lo=16'sd6393;  8'h79:sin_table_lookup_lo=16'sd5602;
			8'h7a:sin_table_lookup_lo=16'sd4808;  8'h7b:sin_table_lookup_lo=16'sd4011;
			8'h7c:sin_table_lookup_lo=16'sd3212;  8'h7d:sin_table_lookup_lo=16'sd2410;
			8'h7e:sin_table_lookup_lo=16'sd1608;  8'h7f:sin_table_lookup_lo=16'sd804;
			default: sin_table_lookup_lo = 16'sd0;
		endcase
	endfunction

	// ────────────────────────────────────────────────────────────────────────
	// FSM body
	// ────────────────────────────────────────────────────────────────────────
	always @(posedge clk) begin
		if (reset) begin
			fsm           <= S_IDLE;
			return_state  <= S_IDLE;
			dma_cnt       <= 0;
			dma_src       <= 24'd0;
			dma_dst       <= 0;
			dma_src_byte  <= 24'd0;
			dma_mode_lat  <= 0;
			dma_ram_addr  <= 0;
			dma_ram_wdata <= 0;
			dma_ram_we    <= 0;
			dma_ram_be    <= 2'b11;
			dma_spr_addr  <= 0;
			dma_spr_wdata <= 0;
			dma_spr_we    <= 0;
			cop_rom_addr  <= 0;
			cop_rom_req   <= 0;
			dma_vram_addr <= 0;
			dma_vram_wdata<= 0;
			dma_vram_we   <= 0;
			dma_pal_addr  <= 0;
			dma_pal_wdata <= 0;
			dma_pal_we    <= 0;
			dma_pal_stage_we <= 0;
			// cop_status[1] è gestito da cpu_wr_pulse block (sprite_dma_inc). Qui
			// resetto solo gli altri bit per evitare multi-driver di [1].
			cop_status[15:2] <= 0;
			cop_status[0]    <= 0;
			cop_angle     <= 0;
			cop_dist      <= 0;
			cop_hit_status   <= 0;
			cop_hit_val_stat <= 0;
			cop_hit_val[0]   <= 0; cop_hit_val[1] <= 0; cop_hit_val[2] <= 0;
			tmp_hi <= 0; tmp_lo <= 0; tmp32_a <= 0; tmp32_b <= 0;
			m_target_addr <= 0;
		end else begin
			// Default each cycle: deassert write strobes
			dma_ram_we    <= 1'b0;
			dma_ram_be    <= 2'b11;   // default word-write (i byte-write lo override)
			dma_spr_we    <= 1'b0;
			dma_vram_we   <= 1'b0;
			dma_pal_we    <= 1'b0;
			dma_pal_stage_we <= 1'b0;

			case (fsm)
			// ════════════════════════════════════════════════════════════════
			S_IDLE: begin
				// Priority: DMA > CMD (so VBlank DMA doesn't get starved)
				if (dma_pending) begin
					// MAME cop_dma_trigger_w: NON modifica cop_status (vedi seibucop.cpp:741).
					// Lo facevamo per errore — l'unica cosa che modifica status durante
					// DMA è cop_sprite_dma_inc_w che gestisce bit 1 (in/out of size).
					dma_mode_lat <= cop_dma_mode;
					case (cop_dma_mode)
						9'h014: begin
							// MAME dma_tilemap_buffer: copy staging RAM → renderer BRAM
							// src = cop_dma_src[mode] << 6 (byte addr 0x101000+ = staging)
							// dst = i (loop counter 0..0x13FF = renderer index)
							// count = 0x2800/2 = 5120 words
							dma_src <= {2'd0, cop_dma_src[cop_dma_mode[7:0]], 6'd0};
							dma_dst <= 16'd0;
							dma_cnt <= 16'd5120;
							fsm     <= D_PREP;
						end
						9'h015: begin
							// MAME dma_palette_buffer: copy staging palette → renderer
							// src = cop_dma_src[mode] << 6 (0x104000 = palette staging)
							// dst = i (loop counter 0..0x7FF = renderer index)
							// count = 0x1000/2 = 2048 words
							dma_src <= {2'd0, cop_dma_src[cop_dma_mode[7:0]], 6'd0};
							dma_dst <= 16'd0;
							dma_cnt <= 16'd2048;
							fsm     <= D_PREP;
						end
						9'h009, 9'h00E: begin
							// generic RAM-to-RAM
							dma_src <= {2'd0, cop_dma_src[cop_dma_mode[7:0]], 6'd0};
							dma_dst <= cop_dma_dst[cop_dma_mode[7:0]] << 5; // word offset
							// size = ((size<<5) - (dst<<6) + 0x20) / 2
							dma_cnt <= ((cop_dma_size[cop_dma_mode[7:0]] << 4)
							           - (cop_dma_dst [cop_dma_mode[7:0]] << 5)
							           + 16'd16);  // simplified, MAME size>>1
							fsm     <= D_PREP;
						end
						9'h080, 9'h081, 9'h082, 9'h083,
						9'h084, 9'h085, 9'h086, 9'h087: begin
							// MAME seibucop_dma.ipp dma_palette_brightness() mode 5/4.
							// Init HeatedBarrel ($35B2): src=$4180 ($106000), dst=$4100 ($104000),
							// size=$827F, adr_rel=4.
							// MAME formulas:
							//   src   = cop_dma_src[mode] << 6
							//   dst   = cop_dma_dst[mode] << 6
							//   size  = ((cop_dma_size << 5) - (cop_dma_dst << 6) + $20)/2
							// Init: src=$106000, dst=$104000, size=($827F<<5 - $4100<<6 + $20)/2
							//       = ($104FE0 - $104000 + $20)/2 = $1000/2 = $800 words.
							// Mode 5: blend tra paldata(src) e targetdata(src+adr_rel*$400)
							// usando pal_brightness_val (0..0x1F) come ratio fade.
							dma_src <= {2'd0, cop_dma_src[cop_dma_mode[7:0]], 6'd0};
							// dst offset (entry in palette renderer): (dst<<6 - $104000)/2 = (dst<<5) - $2000
							dma_dst <= (cop_dma_dst[cop_dma_mode[7:0]] << 5) - 16'h2000;
							// MAME size = (((size<<5) - (dst<<6) + 0x20)/2)
							dma_cnt <= ((cop_dma_size[cop_dma_mode[7:0]] << 4)
							           - (cop_dma_dst [cop_dma_mode[7:0]] << 5)
							           + 16'd16);
							// dma_mode_lat = $80 → branch dedicato D_FADE_*
							dma_mode_lat <= cop_dma_mode;
							fsm <= D_PREP;
						end
						9'h118, 9'h119, 9'h11A, 9'h11B,
						9'h11C, 9'h11D, 9'h11E, 9'h11F: begin
							// Skip if dst != 0 (MAME guard, dma.ipp:135)
							if (cop_dma_dst[cop_dma_mode[7:0]] != 16'd0) begin
								fsm <= S_IDLE;
							end else begin
								dma_src <= {2'd0, cop_dma_src[cop_dma_mode[7:0]], 6'd0};
								// length bytes = (size+1) << 5; we step in words → cnt = (size+1)<<4
								dma_cnt <= ((cop_dma_size[cop_dma_mode[7:0]] + 16'd1) << 4);
								dma_mode_lat <= cop_dma_mode;   // per fill_is_vram (D_FILL_W)
								fsm     <= D_FILL_W;
							end
						end
						default: fsm <= S_IDLE;
					endcase
				end else if (cmd_pending) begin
					// MAME cop_cmd_w (seibucop.cpp:1073): cop_status &= 0x7fff (clear bit 15).
					// NO set bit 2:0 = 111 (era assunzione errata).
					// MAME: cop_status &= 0x7fff (clear bit 15). cop_status[1] gestito altrove.
				cop_status[15]   <= 1'b0;
				cop_status[14:2] <= cop_status[14:2];
				cop_status[0]    <= cop_status[0];
					// Latch dell'offset del comando: i 3 trigger 0905 ($100/$102/$104)
					// condividono cmd_offset. Senza questo latch, il trigger successivo
					// sovrascrive cmd_offset mentre la macro multi-ciclo lo usa ->
					// 0905 offset 1 legge accel da $64 (neg) invece di $60 (+) ->
					// bruto-martello accelera all'indietro. macro_offset lo congela.
					macro_offset <= cmd_offset;
					case (cmd_value)
						// ════════════════════════════════════════════════════════════
						// 0x0205 — linear movement (offset = 0)
						//   ppos = *(cop_regs[0]+4)
						//   npos = ppos + *(cop_regs[0]+0x10)
						//   *(cop_regs[0]+4) = npos
						//   delta = (npos>>16) - (ppos>>16)
						//   *(cop_regs[0]+0x1E) += delta
						16'h0205: begin
							// MAME execute_0205: read dword at cop_regs[0]+4+offset*4 (ppos)
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0004 + {12'd0, cmd_offset, 2'd0});
							fsm <= M_0205_RD_PPOS_HI;
						end

						// 0x0905 — jump: *(cop_regs[0]+0x10) += *(cop_regs[0]+0x28)
						// Il PRIMO read (vel_hi) NON viene emesso qui col cmd_offset: il
						// dispatch usa cmd_offset, gli stati interni macro_offset. Se i 3
						// trigger 0905 ($100/$102/$104) sono back-to-back e un trigger
						// successivo cambia cmd_offset tra il dispatch e il primo stato
						// interno, vel_hi (da cmd_offset) e vel_lo/grav (da macro_offset)
						// finiscono su OFFSET DIVERSI -> vel/grav MISTI tra assi -> per
						// l'offset 1 ($48) il 0905 fa $48 += valore sbagliato (es. $64
						// gravita' invece di $60 decel) -> $48 ACCELERA invece di decelerare
						// -> la martellata-scivolata non si ferma e va fuori schermo.
						// FIX: il primo read si emette in M_0905_RD0 usando macro_offset
						// (ormai registrato e stabile), coerente con tutti gli altri read.
						16'h0905, 16'h0904: begin
							fsm <= M_0905_RD0;
						end

						// 0x138e / 0x338e — atan(dx/dy) MAME execute_338e.
						// Legge gli STESSI 8 word del 3BB0 (dword r1+4,r0+4,r1+8,r0+8).
						// Parte da r1+4 hi (offset 0x0004), come 3BB0/MAME.
						16'h138e, 16'h338e: begin
							dma_src_byte <= cop_regs_byte_addr(1, 16'h0004);  // r1+4 hi
							fsm <= M_138E_RD0_Y_HI;
						end

						// 0x3bb0 — dist (Pythagoras)
						16'h3bb0: begin
							dma_src_byte <= cop_regs_byte_addr(1, 16'h0004);  // r1+4 hi (MAME read_dword(r1+4) primo)
							fsm <= M_3BB0_LOAD;
						end

						// 0x42c2 / 0x4aa0 — divide. Endian asimmetrico (review D1):
						// 42c2 div=cop_read_word(r0+0x36)→host 0x34; 4aa0 div=read RAW 0x38.
						16'h42c2: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0034);  // 0x36^2
							fsm <= M_42C2_RD_DIV;
						end
						16'h4aa0: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0038);  // RAW
							fsm <= M_42C2_RD_DIV;
						end

						// 0x8100 — sin. MAME: raw_angle = cop_read_word(r0+0x34) =
						// host_read_word(0x34^2 = 0x36); amp = cop_read_word(r0+0x36) =
						// host_read_word(0x36^2 = 0x34). Host endianness word^2.
						// → angle a host+0x36 (letto in tmp_lo), amp a host+0x34 (letto in CALC).
						16'h8100: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0036);
							fsm <= M_SC_RD_ANG;
							tmp_hi <= 16'h0001; // tag = sin (write +0x10 dword)
						end

						// 0x8900 — cos
						16'h8900: begin
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0036);
							fsm <= M_SC_RD_ANG;
							tmp_hi <= 16'h0002; // tag = cos (write +0x14 dword)
						end

						// collision read pos slot 0 — heatbrl a100 (bit7=0), legionna a180 (bit7=1)
						16'ha100, 16'ha180: begin
							coll_allow_swap[0] <= 1'b1; // known-good hardcoded (cmd_value[7] regredisce)
							dma_src_byte <= cop_regs_byte_addr(0, 16'h0000);
							tmp_hi <= 16'h0000; // tag = slot 0
							fsm <= M_A1_RD_FLAGS;
						end

						// collision read pos slot 1 — heatbrl a900, legionna a980
						16'ha900, 16'ha980: begin
							coll_allow_swap[1] <= 1'b1; // known-good hardcoded (cmd_value[7] regredisce)
							dma_src_byte <= cop_regs_byte_addr(1, 16'h0000);
							tmp_hi <= 16'h0001; // tag = slot 1
							fsm <= M_A1_RD_FLAGS;
						end

						// hitbox slot 0 — heatbrl b080 (bit8=0, 2 assi), legionna b100 (bit8=1, 3 assi)
						16'hb080, 16'hb100: begin
							hb_ptr_addr <= cop_regs_byte_addr(2, 16'h0000);
							tmp_hi <= 16'h0000;
							fsm <= M_B1_REQ_PTR;
						end

						// hitbox slot 1 — heatbrl b880, legionna b900
						16'hb880, 16'hb900: begin
							hb_ptr_addr <= cop_regs_byte_addr(3, 16'h0000);
							tmp_hi <= 16'h0001;
							fsm <= M_B1_REQ_PTR;
						end

						default: begin
							// unknown macro — return idle
							fsm <= S_IDLE;
						end
					endcase
				end
			end

			// ════════════════════════════════════════════════════════════════
			// DMA path: src is byte addr, advances by 2 per word.
			// dst is local word index for VRAM/Pal target.
			// Mode $80-$87 (fade): 2 read (src=paldata, src+adr_rel*$400=target)
			//                       + 1 write con blend mode 5.
			// ════════════════════════════════════════════════════════════════
			D_PREP: begin
				dma_src_byte <= dma_src;
				fsm <= D_READ;
			end
			D_READ: begin
				// BRAM staging/main-RAM port B: output REGISTRATO, latency 1. L'addr
				// (word i) e' emesso a fine ciclo precedente (D_PREP / loop), quindi
				// in D_READ dma_src_rdata = mem[i] NON e' ancora valido: contiene
				// mem[i-1]. Per il DMA non-fade scriviamo dma_*_wdata DIRETTAMENTE da
				// dma_src_rdata in D_WRITE (1 ciclo dopo, mem[i] valido). dma_*_wdata
				// sono gia' registri (1 livello logico) -> nessun path lungo da
				// spezzare qui, quindi NIENTE latch anticipato (causava renderer[i]=
				// staging[i-1] -> palette sprite shiftata -> silhouette).
				if (dma_mode_lat[7:4] == 4'h8) begin
					// FADE: src rdata valido al prossimo ciclo → vai a D_FADE_RD_TGT
					fsm <= D_FADE_RD_TGT;
				end else begin
					fsm <= D_WRITE;
				end
			end
			D_WRITE: begin
				case (dma_mode_lat[3:0])
					4'h4: begin // 0x14 tilemap
						dma_vram_we    <= 1'b1;
						dma_vram_addr  <= dma_dst[12:0];
						dma_vram_wdata <= dma_src_rdata;   // mem[i] valido (latency 1, D_WRITE = emit+1)
					end
					4'h5: begin // 0x15 palette renderer
						dma_pal_we     <= 1'b1;
						dma_pal_addr   <= dma_dst[10:0];
						dma_pal_wdata  <= dma_src_rdata;
					end
					4'h9, 4'hE: begin // RAM-to-RAM copy
						dma_ram_we     <= 1'b1;
						dma_ram_addr   <= dma_dst;
						dma_ram_wdata  <= dma_src_rdata;
					end
					default: ;
				endcase
				dma_src <= dma_src + 24'd2;
				dma_dst <= dma_dst + 16'd1;
				if (dma_cnt == 16'd1) begin
					fsm <= S_IDLE;
				end else begin
					dma_cnt      <= dma_cnt - 16'd1;
					dma_src_byte <= dma_src + 24'd2;
					fsm          <= D_READ;
				end
			end

			// ── MODE 5 FADE: 1° rdata = paldata. Emit target addr. ─────────
			D_FADE_RD_TGT: begin
				tmp_lo <= dma_src_rdata;                                // paldata latched
				dma_src_byte <= dma_src + ({8'd0, cop_dma_adr_rel} * 24'h400);
				fsm <= D_FADE_WAIT_TGT;
			end
			D_FADE_WAIT_TGT: begin
				// 1 ciclo wait per BRAM read latency
				fsm <= D_FADE_CALC;
			end
			D_FADE_CALC: begin
				// Pipeline stage 1: calcola i 6 fade_table_fn (6 mul) e REGISTRA.
				// Spezza il path lungo (6 mul + somme in 1 ciclo era -3ns).
				begin : fade_calc_blk
					reg [15:0] paldata, tgtdata;
					reg  [4:0] bv, bv_xor_inv;
					paldata    = tmp_lo;
					tgtdata    = dma_src_rdata;
					bv         = cop_pal_brightness_val[4:0];
					bv_xor_inv = bv ^ 5'h1F;
					fade_paldata <= paldata;
					fade_nofade  <= paldata[15];
					fade_fb_t <= fade_table_fn({tgtdata[14:10], bv});
					fade_fb_c <= fade_table_fn({paldata[14:10], bv_xor_inv});
					fade_fg_t <= fade_table_fn({tgtdata[9:5],  bv});
					fade_fg_c <= fade_table_fn({paldata[9:5],  bv_xor_inv});
					fade_fr_t <= fade_table_fn({tgtdata[4:0],  bv});
					fade_fr_c <= fade_table_fn({paldata[4:0],  bv_xor_inv});
				end
				fsm <= D_FADE_WRITE;
			end
			D_FADE_WRITE: begin
				// Pipeline stage 2: somme (input registrati = path corto) + write.
				begin : fade_wr_blk
					reg [15:0] pal_val;
					reg  [4:0] out_b, out_g, out_r;
					if (fade_nofade) begin
						pal_val = fade_paldata;
					end else begin
						// Tronca OGNI canale a 5 bit PRIMA del concat: senza i temp
						// reg[4:0], (8bit+8bit)&8'h1F resta 8-bit self-determined ->
						// concat 25-bit -> troncato a 16 -> G shiftato, B perso ->
						// palette corrotta -> sprite "scompaiono". (regressione pipeline)
						out_b = (fade_fb_t + fade_fb_c) & 8'h1F;
						out_g = (fade_fg_t + fade_fg_c) & 8'h1F;
						out_r = (fade_fr_t + fade_fr_c) & 8'h1F;
						pal_val = {1'b0, out_b, out_g, out_r};
					end
					dma_pal_stage_we <= 1'b1;
					dma_pal_addr     <= dma_dst[10:0];
					dma_pal_wdata    <= pal_val;
				end
				dma_src <= dma_src + 24'd2;
				dma_dst <= dma_dst + 16'd1;
				if (dma_cnt == 16'd1) begin
					fsm <= S_IDLE;
				end else begin
					dma_cnt      <= dma_cnt - 16'd1;
					dma_src_byte <= dma_src + 24'd2;
					fsm          <= D_READ;
				end
			end
			D_FILL_W: begin
				// Fill VRAM ($100800-$102FFF) -> SCRATCH via dma_fill_* (combinatorio, sopra).
				// Fill Main RAM (altri indirizzi) -> qui. Mai entrambi: evita doppia scrittura.
				dma_ram_we    <= ~fill_is_vram;
				dma_ram_addr  <= dma_src[16:1];
				dma_ram_wdata <= cop_dma_v1;
				dma_src <= dma_src + 24'd2;
				if (dma_cnt == 16'd1) begin
					fsm <= S_IDLE;
				end else begin
					dma_cnt <= dma_cnt - 16'd1;
				end
			end

			// ════════════════════════════════════════════════════════════════
			// Macro 0x0205 — linear movement (MAME execute_0205)
			//   ppos = read_dword(cop_regs[0]+0x04)
			//   vel  = read_dword(cop_regs[0]+0x10)
			//   npos = ppos + vel
			//   write_dword(cop_regs[0]+0x04, npos)
			//   delta = (npos>>16) - (ppos>>16)
			//   write_word(cop_regs[0]+0x1E, read_word(cop_regs[0]+0x1E) + delta)
			// dma_ram_addr è WORD addr → byte_addr[16:1]
			// ════════════════════════════════════════════════════════════════
			M_0205_RD_PPOS_HI: begin
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0006 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_RD_PPOS_LO;
			end
			M_0205_RD_PPOS_LO: begin
				tmp_hi <= dma_src_rdata;                          // ppos hi latched
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0010 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_RD_VEL_HI;
			end
			M_0205_RD_VEL_HI: begin
				tmp_lo <= dma_src_rdata;                          // ppos lo latched
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0012 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_RD_VEL_LO;
			end
			M_0205_RD_VEL_LO: begin
				tmp32_a <= {tmp_hi, tmp_lo};                      // ppos
				tmp32_b <= {dma_src_rdata, 16'h0};                // partial vel (hi)
				fsm <= M_0205_WR_NPOS_HI;
			end
			M_0205_WR_NPOS_HI: begin
				tmp32_b[15:0] <= dma_src_rdata;                   // vel lo
				tmp32_a       <= tmp32_a + {tmp32_b[31:16], dma_src_rdata};
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0004 + {12'd0, macro_offset, 2'd0}) >> 1;
				dma_ram_wdata <= tmp32_a[31:16] + tmp32_b[31:16] + ((tmp32_a[15:0] + dma_src_rdata) < tmp32_a[15:0] ? 16'd1 : 16'd0);
				fsm <= M_0205_WR_NPOS_LO;
			end
			M_0205_WR_NPOS_LO: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0006 + {12'd0, macro_offset, 2'd0}) >> 1;
				dma_ram_wdata <= tmp32_a[15:0];
				fsm <= M_0205_RD_SCRN;
			end
			M_0205_RD_SCRN: begin
				// MAME cop_read/write_word(cop_regs[0]+0x1e) = host word @ (0x1e^2)=0x1c
				// (host endianness word^2; conferma doc 06 micro-op 'addmem16 0x1C(r0)').
				dma_src_byte <= cop_regs_byte_addr(0, 16'h001C + {12'd0, macro_offset, 2'd0});
				fsm <= M_0205_WAIT_SCRN;
			end
			M_0205_WAIT_SCRN: begin
				// Wait 1 stato: read 0x1C valida 2 stati dopo emit (latency BRAM=2).
				// Senza, WR_SCRN leggeva addr vecchio (vel_lo) -> scroll corrotto.
				fsm <= M_0205_WR_SCRN;
			end
			M_0205_WR_SCRN: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h001C + {12'd0, macro_offset, 2'd0}) >> 1;
				dma_ram_wdata <= dma_src_rdata + (tmp32_a[31:16] - tmp_hi);
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// Macro 0x0905/0x0904 — jump (MAME execute_0904)
			//   write_dword(cop_regs[0]+0x10, read_dword(cop_regs[0]+0x10)
			//                                  ± read_dword(cop_regs[0]+0x28))
			//   0x0905 (bit 0=1) = +, 0x0904 (bit 0=0) = -
			// ════════════════════════════════════════════════════════════════
			M_0905_RD0: begin
				// Primo read vel_hi @ 0x10+macro_offset*4 (macro_offset ora STABILE,
				// registrato al dispatch). Tutti i read del 0905 usano lo stesso offset.
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0010 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_HI;
			end
			M_0905_RD_HI: begin
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0012 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_LO;
			end
			M_0905_RD_LO: begin
				tmp_hi <= dma_src_rdata;
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0028 + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_GRAV_HI;
			end
			M_0905_RD_GRAV_HI: begin
				tmp_lo <= dma_src_rdata;
				dma_src_byte <= cop_regs_byte_addr(0, 16'h002A + {12'd0, macro_offset, 2'd0});
				fsm <= M_0905_RD_GRAV_LO;
			end
			M_0905_RD_GRAV_LO: begin
				// MAME execute_0904: bit 0 di data = 1 → add, =0 → sub
				// tmp32_a = vel ± grav. vel = {tmp_hi, tmp_lo}. grav_hi = dma_src_rdata.
				if (cmd_value[0])
					tmp32_a <= {tmp_hi, tmp_lo} + {dma_src_rdata, 16'h0};
				else
					tmp32_a <= {tmp_hi, tmp_lo} - {dma_src_rdata, 16'h0};
				fsm <= M_0905_WR_HI;
			end
			M_0905_WR_HI: begin
				// dma_src_rdata = grav_lo. Combina vel+grav (con carry/borrow propagato).
				if (cmd_value[0]) begin
					tmp32_a[15:0] <= tmp32_a[15:0] + dma_src_rdata;
					// carry propagato a hi
					dma_ram_wdata <= tmp32_a[31:16] + ((tmp32_a[15:0] + dma_src_rdata) < tmp32_a[15:0] ? 16'd1 : 16'd0);
				end else begin
					tmp32_a[15:0] <= tmp32_a[15:0] - dma_src_rdata;
					// borrow propagato a hi
					dma_ram_wdata <= tmp32_a[31:16] - ((tmp32_a[15:0] < dma_src_rdata) ? 16'd1 : 16'd0);
				end
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0010 + {12'd0, macro_offset, 2'd0}) >> 1;
				fsm <= M_0905_WR_LO;
			end
			M_0905_WR_LO: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0012 + {12'd0, macro_offset, 2'd0}) >> 1;
				dma_ram_wdata <= tmp32_a[15:0];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0x138E/0x338E atan(dx/dy) — MAME execute_338e (cmd.ipp:159)
			//   dx = read_dword(r1+4) - read_dword(r0+4)
			//   dy = read_dword(r1+8) - read_dword(r0+8)
			//   if(!dy){status|=0x8000; angle=0} else angle=atan(dx/dy)*128/pi; dy<0→+0x80
			//   if(data&0x80) write_byte(r0+0x34, angle)
			// dword 68k BE: hi@off+0, lo@off+2. Latenza BRAM 2 cicli. Dispatch emette r1+4(hi).
			// ════════════════════════════════════════════════════════════════
				// Read chain IDENTICA al 3BB0 (latenza BRAM 2 cicli). Mappatura:
				// tmp32_a=r1x, tmp32_b=r0x, math_dx=r1y, math_dy=r0y. Dispatch r1+4 hi.
				M_138E_RD0_Y_HI: begin
					dma_src_byte <= cop_regs_byte_addr(1, 16'h0006);
					fsm <= M_138E_RD0_Y_LO;
				end
				M_138E_RD0_Y_LO: begin
					tmp32_a[31:16] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h0004);
					fsm <= M_138E_RD1_Y_HI;
				end
				M_138E_RD1_Y_HI: begin
					tmp32_a[15:0] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h0006);
					fsm <= M_138E_RD1_Y_LO;
				end
				M_138E_RD1_Y_LO: begin
					tmp32_b[31:16] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(1, 16'h0008);
					fsm <= M_138E_RD0_X_HI;
				end
				M_138E_RD0_X_HI: begin
					tmp32_b[15:0] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(1, 16'h000A);
					fsm <= M_138E_RD0_X_LO;
				end
				M_138E_RD0_X_LO: begin
					math_dx[31:16] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h0008);
					fsm <= M_138E_RD1_X_HI;
				end
				M_138E_RD1_X_HI: begin
					math_dx[15:0] <= dma_src_rdata;
					dma_src_byte <= cop_regs_byte_addr(0, 16'h000A);
					fsm <= M_138E_RD1_X_LO;
				end
				M_138E_RD1_X_LO: begin
					math_dy[31:16] <= dma_src_rdata;
					fsm <= M_138E_CALC;
				end
				M_138E_CALC: begin
					// MAME: angle = atan(dx/dy)*128/pi (+0x80 se dy<0). Il vectoring
					// accumula atan(y/x) con x>0. Per ottenere atan(dx/dy) col segno
					// giusto di dy: x=|dy|, y = (dy<0)? -dx : dx. Cosi' z = atan(dx/dy)
					// e il +0x80 (in WR) copre solo il quadrante, come MAME.
					// NIENTE prescale: i dword diff sono FIXED-POINT coord<<16 (come il
					// 3BB0 che usa dx32[31:16] e MAME dx>>16), NON interi piccoli. Un
					// <<8 strariperebbe signed[31:0] per |delta|>=128px -> overflow ->
					// angolo sbagliato. atan e' scale-invariant: i dword pieni bastano
					// (max |axdy|~2^25, *gain 1.65 ~2^26, dentro signed[31:0]).
					begin : atan_init_blk
						reg signed [31:0] dx32, dy32, axdy, aydx;
						dx32 = $signed(tmp32_a) - $signed(tmp32_b);
						dy32 = $signed(math_dx) - $signed({math_dy[31:16], dma_src_rdata});
						axdy = (dy32 < 0) ? -dy32 : dy32;        // |dy|
						aydx = (dy32 < 0) ? -dx32 : dx32;        // dx col segno di dy
						cordic_x <= axdy;                        // dword pieno (coord<<16)
						cordic_y <= aydx;
						cordic_z <= 24'sd0;
						math_dy  <= dy32;
					end
					cordic_i <= 5'd0;
					fsm <= M_138E_CORDIC;
				end
				M_138E_CORDIC: begin
					begin : atan_cordic_blk
						reg signed [31:0] xs, ys;
						xs = cordic_x >>> cordic_i;
						ys = cordic_y >>> cordic_i;
						if (cordic_y >= 0) begin
							cordic_x <= cordic_x + ys;
							cordic_y <= cordic_y - xs;
							cordic_z <= cordic_z + cordic_atan_lut(cordic_i);
						end else begin
							cordic_x <= cordic_x - ys;
							cordic_y <= cordic_y + xs;
							cordic_z <= cordic_z - cordic_atan_lut(cordic_i);
						end
					end
					if (cordic_i == 5'd23) fsm <= M_138E_WR;
					else                   cordic_i <= cordic_i + 5'd1;
				end
				M_138E_WR: begin
					begin : atan_wr_blk
						reg signed [15:0] ang_unit;
						reg        [7:0]  ang_byte;
						// MAME (int)(...) tronca VERSO ZERO. cordic_z e' ×65536 (16-bit
						// frazione, 24 iter) -> ang_unit = z>>16 toward-zero.
						// BUG TROVATO (validato vs MAME su 29241 casi): il CORDIC SOTTO-CONVERGE
						// di 1 LSB sulle DIAGONALI (es. dx==dy -> z=2097151 invece di 2097152=
						// 32.0×65536). z>>16 toward-zero da' 31 invece di 32 -> cop_angle 0x9f
						// invece di 0xa0 -> settore $48DA sbagliato (0x9f>>3=0x13 vs 0xa0>>3=0x14)
						// -> d1 bit0 errato -> clr.l $48 NON scatta quando il bruto e' in DIAGONALE
						// col player -> il martello scivola nella posa. Compenso l'epsilon di
						// sotto-convergenza con +1 toward-zero PRIMA del troncamento: bias=1
						// azzera tutte le 133 divergenze (bias=0) -> 0/29241 vs MAME.
						begin : ang_round
							reg signed [23:0] z_comp;
							z_comp  = (cordic_z >= 0) ? (cordic_z + 24'sd1) : (cordic_z - 24'sd1);
							ang_unit = (z_comp >= 0) ? (z_comp >>> 16)
							                         : -((-z_comp) >>> 16);
						end
						// MAME execute_338e (seibucop_cmd.ipp:175-188): la write dell'angolo
						// in RAM (obj+0x34 host, gated da data&0x80) e' FUORI dall'if/else di
						// dy -> avviene SEMPRE, anche a dy==0 (dove cop_angle=0 -> scrive 0^0x80
						// = 0x80). Il mio scriveva SOLO nel ramo dy!=0 -> a dy==0 (bruto
						// allineato vert col player) l'angolo in RAM NON si aggiornava -> il
						// sincos rileggeva l'angolo VECCHIO -> velocita' sbagliata -> la mossa
						// scivolata tornava indietro. Ora calcolo ang_byte in entrambi i rami
						// e scrivo SEMPRE, come MAME.
						// is_yflip (HeatedBarrel TAD): XOR 0x80 SOLO sulla write RAM (obj+0x34,
						// che il SINCOS 8100 rilegge), NON sul registro cop_angle readback.
						// STATO BUONO (EDIT#238, martello NON scivola, confermato HW
						// 2026-06-08): con dy==0 il 138e mette cop_angle=0 e status[15]=1
						// ma NON scrive l'angolo in RAM (la write resta DENTRO il ramo
						// dy!=0). Scrivere l'angolo anche a dy==0 ("1:1 MAME") REGREDISCE:
						// il martello scivola/moonwalk. Ramo dy==0 INTOCCABILE.
						if (math_dy == 32'sd0) begin
							ang_byte       = 8'h00;          // MAME: dy==0 -> cop_angle=0
							cop_angle      <= 16'h0000;
							cop_status[15] <= 1'b1;
						end else begin
							ang_byte       = ang_unit[7:0] + ((math_dy < 0) ? 8'h80 : 8'h00);
							cop_angle      <= {8'h00, ang_byte};          // readback: SENZA xor
							cop_status[15] <= 1'b0;
							if (cmd_value[7]) begin           // data&0x80: write SOLO nel ramo dy!=0
								dma_ram_we    <= 1'b1;
								dma_ram_be    <= 2'b01;
								dma_ram_addr  <= cop_regs_byte_addr(0, 16'h0036) >> 1;
								dma_ram_wdata <= {8'h00, ang_byte ^ 8'h80};  // RAM: CON xor (sincos)
							end
						end
					end
					cop_status[2]    <= 1'b1;
					cop_status[14:3] <= 12'd0;
					cop_status[0]    <= 1'b1;
					fsm <= S_IDLE;
				end


			// ════════════════════════════════════════════════════════════════
			// 0x3BB0 dist — MAME execute_3b30 (cmd.ipp:197)
			//   dx = (read_dword(r1+4) - read_dword(r0+4)) >> 16  (aritmetico)
			//   dy = (read_dword(r1+8) - read_dword(r0+8)) >> 16
			//   cop_dist = sqrt(dx*dx + dy*dy)
			//   if(data&0x80) write_word(r0 + (data&0x200?0x3a:0x38), dist)
			//   3bb0: bit 0x200 set → cop 0x3a → host word (0x3a^2)=0x38.
			// Stesse 8 letture word di atan. Dispatch emette r1+4(hi).
			// ════════════════════════════════════════════════════════════════
			// 0x3BB0 dist — MAME execute_3b30 (cmd.ipp:197): 1:1
			//   dx = (read_dword(r1+4) - read_dword(r0+4)) >> 16  (signed)
			//   dy = (read_dword(r1+8) - read_dword(r0+8)) >> 16
			//   cop_dist = floor(sqrt(dx*dx + dy*dy))
			//   if(data&0x80) write_word(r0+(data&0x200?0x3a:0x38), dist)
			// Accumulatori: tmp32_a=r1x, tmp32_b=r0x, math_dx=r1y, math_dy=r0y.
			// Dispatch ha emesso r1+4(hi). Latenza BRAM: rdata valido 2 stati dopo emit.
			M_3BB0_LOAD: begin
				dma_src_byte <= cop_regs_byte_addr(1, 16'h0006);  // r1+4 lo
				fsm <= M_3BB0_RD_X1LO;
			end
			M_3BB0_RD_X1LO: begin
				tmp32_a[31:16] <= dma_src_rdata;                  // r1x hi
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0004);  // r0+4 hi
				fsm <= M_3BB0_RD_X0HI;
			end
			M_3BB0_RD_X0HI: begin
				tmp32_a[15:0] <= dma_src_rdata;                   // r1x lo
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0006);  // r0+4 lo
				fsm <= M_3BB0_RD_X0LO;
			end
			M_3BB0_RD_X0LO: begin
				tmp32_b[31:16] <= dma_src_rdata;                  // r0x hi
				dma_src_byte <= cop_regs_byte_addr(1, 16'h0008);  // r1+8 hi
				fsm <= M_3BB0_RD_Y1HI;
			end
			M_3BB0_RD_Y1HI: begin
				tmp32_b[15:0] <= dma_src_rdata;                   // r0x lo
				dma_src_byte <= cop_regs_byte_addr(1, 16'h000A);  // r1+8 lo
				fsm <= M_3BB0_RD_Y1LO;
			end
			M_3BB0_RD_Y1LO: begin
				math_dx[31:16] <= dma_src_rdata;                  // r1y hi
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0008);  // r0+8 hi
				fsm <= M_3BB0_RD_Y0HI;
			end
			M_3BB0_RD_Y0HI: begin
				math_dx[15:0] <= dma_src_rdata;                   // r1y lo
				dma_src_byte <= cop_regs_byte_addr(0, 16'h000A);  // r0+8 lo
				fsm <= M_3BB0_RD_Y0LO;
			end
			M_3BB0_RD_Y0LO: begin
				math_dy[31:16] <= dma_src_rdata;                  // r0y hi
				fsm <= M_3BB0_CALC;
			end
			M_3BB0_CALC: begin
				// Pipeline stage 0 (timing): registra SOLO la differenza r0y completata
				// con dma_src_rdata (dato dalla Main RAM = path critico -5.3ns).
				// Spezza il path RAM→sottrazione→mul. Il resto in CALC2/CALC3.
				math_dy <= {math_dy[31:16], dma_src_rdata};  // completa r0y lo
				fsm <= M_3BB0_CALC2;
			end
			M_3BB0_CALC2: begin
				// Stage 1: SOLO differenze 32-bit (registra dxh/dyh in tmp32_a/b[15:0]).
				// Spezza sottrazione dalla moltiplicazione (era catena sub->mul 12ns).
				begin : dist_diff_blk
					reg signed [31:0] dx32, dy32;
					dx32 = $signed(tmp32_a) - $signed(tmp32_b);  // r1x - r0x
					dy32 = $signed(math_dx) - $signed(math_dy);  // r1y - r0y
					tmp32_a <= {16'd0, dx32[31:16]};             // dxh registrato
					tmp32_b <= {16'd0, dy32[31:16]};             // dyh registrato
				end
				fsm <= M_3BB0_CALCM;
			end
			M_3BB0_CALCM: begin
				// Stage 2: moltiplicazioni (input registrati = path corto).
				math_dx <= $signed(tmp32_a[15:0]) * $signed(tmp32_a[15:0]);  // dx*dx
				math_dy <= $signed(tmp32_b[15:0]) * $signed(tmp32_b[15:0]);  // dy*dy
				fsm <= M_3BB0_CALC3;
			end
			M_3BB0_CALC3: begin
				// Stage 3: somma i prodotti registrati + init sqrt.
				sqrt_radsh <= math_dx + math_dy;
				sqrt_rem34 <= 34'd0;
				sqrt_acc   <= 32'd0;
				sqrt_i     <= 5'd16;
				fsm <= M_3BB0_SQRT;
			end
			M_3BB0_SQRT: begin
				// restoring integer sqrt, 1 bit/ciclo, 16 iter.
				begin : dist_sqrt_blk
					reg [33:0] rem_next;
					reg [17:0] cand;
					rem_next = {sqrt_rem34[31:0], sqrt_radsh[31:30]};
					cand     = {sqrt_acc[15:0], 2'b01};          // (root<<2)|1
					if (rem_next >= {16'd0, cand}) begin
						sqrt_rem34 <= rem_next - {16'd0, cand};
						sqrt_acc   <= {sqrt_acc[14:0], 1'b1};    // (root<<1)|1
					end else begin
						sqrt_rem34 <= rem_next;
						sqrt_acc   <= {sqrt_acc[14:0], 1'b0};    // root<<1
					end
					sqrt_radsh <= sqrt_radsh << 2;
				end
				if (sqrt_i == 5'd1) fsm <= M_3BB0_WR;
				else                sqrt_i <= sqrt_i - 5'd1;
			end
			M_3BB0_WR: begin
				cop_dist <= sqrt_acc[15:0];                      // floor(sqrt(rad))
				if (cmd_value[7]) begin                          // data&0x80
					dma_ram_we    <= 1'b1;
					// Offset di scrittura distanza. (Il fix MAME-aligned 0x3a:0x38
					// faceva fuggire i nemici -> MAME invertito vs HW. Ripristinato.)
					dma_ram_addr  <= cmd_value[9]                // data&0x200 ? 0x38 : 0x3a
					                 ? (cop_regs_byte_addr(0, 16'h0038) >> 1)
					                 : (cop_regs_byte_addr(0, 16'h003a) >> 1);
					dma_ram_wdata <= sqrt_acc[15:0];
				end
				// MAME: macro completata → cop_status = 7 (gate-poll del gioco).
				// bit[1] gestito dal cpu_wr_pulse block → NON toccare.
				cop_status[2]    <= 1'b1;
				cop_status[14:3] <= 12'd0;
				cop_status[0]    <= 1'b1;
				cop_status[15]   <= 1'b0;
				fsm <= S_IDLE;
			end

			// 0x42C2/0x4AA0 divide — MAME execute_42c2/4aa0 (cmd.ipp:219/247): 1:1
			//   DIVIDEND = cop_dist << (5 - cop_scale)   (cop_scale 0..3 → shift 5..2)
			//   42c2: div=cop_read_word(r0+0x36)→host 0x34; if(!div){status|=0x8000;
			//         write 0 a host 0x3a} else write DIVIDEND/div a host 0x3a.
			//   4aa0: div=read RAW 0x38; if(!div)div=1; write DIVIDEND/div a host 0x36.
			//   Divide = restoring unsigned 32-iter (1 bit/ciclo, no operatore /).
			//   Dispatch ha emesso src corretto. Latenza BRAM: rdata=div in M_42C2_CALC.
			M_42C2_RD_DIV: begin
				fsm <= M_42C2_CALC;   // wait latenza BRAM
			end
			M_42C2_CALC: begin
				// dma_src_rdata = div. cop_status=7 (gate-poll); bit1 non toccare.
				cop_status[2]    <= 1'b1;
				cop_status[14:3] <= 12'd0;
				cop_status[0]    <= 1'b1;
				cop_status[15]   <= 1'b0;
				if (dma_src_rdata == 16'd0) begin
					if (cmd_value == 16'h42c2) begin
						// 42c2 div0: status|=0x8000, write 0 a host 0x3a, return
						cop_status[15] <= 1'b1;
						dma_ram_we    <= 1'b1;
						dma_ram_addr  <= cop_regs_byte_addr(0, 16'h003a) >> 1;
						dma_ram_wdata <= 16'd0;
						fsm <= S_IDLE;
					end else begin
						// 4aa0 div0: div=1 → result=DIVIDEND. Procedi.
						div_den <= 16'd1;
						case (cop_scale[1:0])
							2'd0: div_num <= {11'd0, cop_dist} << 5;
							2'd1: div_num <= {11'd0, cop_dist} << 4;
							2'd2: div_num <= {11'd0, cop_dist} << 3;
							2'd3: div_num <= {11'd0, cop_dist} << 2;
						endcase
						div_q <= 32'd0; div_r <= 32'd0; div_i <= 6'd32;
						fsm   <= M_42C2_DIV;
					end
				end else begin
					div_den <= dma_src_rdata;
					case (cop_scale[1:0])
						2'd0: div_num <= {11'd0, cop_dist} << 5;
						2'd1: div_num <= {11'd0, cop_dist} << 4;
						2'd2: div_num <= {11'd0, cop_dist} << 3;
						2'd3: div_num <= {11'd0, cop_dist} << 2;
					endcase
					div_q <= 32'd0; div_r <= 32'd0; div_i <= 6'd32;
					fsm   <= M_42C2_DIV;
				end
			end
			M_42C2_DIV: begin
				// Restoring division unsigned, 1 bit/ciclo, MSB-first, 32 iter.
				begin : div_blk
					reg [32:0] rem_sh;
					rem_sh = {div_r[31:0], div_num[31]};   // rem<<1 | num MSB
					if (rem_sh >= {17'd0, div_den}) begin
						div_r <= rem_sh - {17'd0, div_den};
						div_q <= (div_q << 1) | 32'd1;
					end else begin
						div_r <= rem_sh[31:0];
						div_q <= div_q << 1;
					end
					div_num <= div_num << 1;
				end
				if (div_i == 6'd1) fsm <= M_42C2_WR;
				else               div_i <= div_i - 6'd1;
			end
			M_42C2_WR: begin
				// div_q = DIVIDEND/div. Write 16-bit risultato.
				//   42c2 → host 0x3a (0x38^2) ; 4aa0 → host 0x36 (RAW)
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= (cmd_value == 16'h42c2)
				                 ? (cop_regs_byte_addr(0, 16'h003a) >> 1)
				                 : (cop_regs_byte_addr(0, 16'h0036) >> 1);
				dma_ram_wdata <= div_q[15:0];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0x8100/0x8900 sin/cos (MAME execute_8100/8900)
			//   raw_angle = cop_read_word(cop_regs[0]+0x34) & 0xff
			//   amp       = (65536>>5) * (cop_read_word(cop_regs[0]+0x36) & 0xff)
			//                = 2048 * amp_byte
			//   if (raw_angle == 0xC0 sin / 0x80 cos): amp *= 2
			//   res = int(amp * sin/cos(raw_angle * pi/128)) << cop_scale
			//   write_dword(cop_regs[0] + 0x10 sin / 0x14 cos, res)
			//
			// sin_table_lookup ritorna signed16 = sin(i*pi/128) * 32768
			// amp_full * sin_norm = (2048 * amp_byte) * (sin_lut/32768)
			//                     = (amp_byte * sin_lut) / 16
			// → in fixed: signed_product[31:0] = amp_byte * sin_lut → shift>>4
			// Per cos: usa angle+0x40
			// ════════════════════════════════════════════════════════════════
			M_SC_RD_ANG: begin
				// pre-emit amp byte addr (host+0x34). Dispatch ha emesso host+0x36
				// (angle); per latenza BRAM 2-cicli, tmp_lo catturerà read(0x36)=angle
				// e CALC vedrà dma_src_rdata=read(0x34)=amp.
				dma_src_byte <= cop_regs_byte_addr(0, 16'h0034);
				fsm <= M_SC_RD_AMP;
			end
			M_SC_RD_AMP: begin
				tmp_lo <= dma_src_rdata;  // angle word (host+0x36, byte basso = angle)
				fsm <= M_SC_CALC;
			end
			M_SC_CALC: begin
				// angle_byte = tmp_lo[7:0]; amp_byte = dma_src_rdata[7:0]
				// tmp_hi: 1=sin (write +0x10), 2=cos (write +0x14, angle+=0x40)
				begin : sc_blk
					reg  [7:0] raw_angle;
					reg  [7:0] angle_byte;
					reg  [8:0] amp_ext;             // 9-bit per doubling caso speciale
					reg signed [15:0] sin_v;
					reg signed [16:0] sin_full;
					reg signed [31:0] prod;
					reg signed [31:0] res;
					raw_angle  = tmp_lo[7:0];
					angle_byte = (tmp_hi == 16'd2) ? (raw_angle + 8'h40) : raw_angle;
					amp_ext    = {1'b0, dma_src_rdata[7:0]};
					// MAME special-case: sin && raw==0xC0 OR cos && raw==0x80 → amp *= 2
					if (tmp_hi == 16'd1 && raw_angle == 8'hC0) amp_ext = {dma_src_rdata[7:0], 1'b0};
					if (tmp_hi == 16'd2 && raw_angle == 8'h80) amp_ext = {dma_src_rdata[7:0], 1'b0};
					sin_v = sin_table_lookup(angle_byte);
					// PICCO al pieno SOLO nel calcolo velocita': la sin_table satura a 32767
					// (valore HW corretto per i nemici, NON toccare la tabella - project_sintable_fix).
					// Ma nel PRODOTTO l'HW reale usa 32768 pieno: al picco cos(0)/cos(180)
					// (sin_v=±32767) la vel $48=amp*32767>>4=131068 NON e' mult.16; la decel della
					// martellata-scivolata ($60=-$48/16, stop $48==0 esatto, state[11] $6E8C) scavalca
					// lo zero -> $48 runaway -> scivola fuori schermo. Con 32768 -> 131072 (mult.16)
					// -> ferma. Tocca SOLO il picco (1 angolo, 0x40), tabella INVARIATA per i nemici.
					if      (sin_v ==  16'sd32767) sin_full =  17'sd32768;
					else if (sin_v == -16'sd32767) sin_full = -17'sd32768;
					else                           sin_full = {sin_v[15], sin_v};
					prod  = $signed({1'b0, amp_ext}) * sin_full;
					// res = prod >> 4 (= /16 per matchare formula MAME 2048/32768 = 1/16)
					res = prod >>> 4;
					case (cop_scale[1:0])
						2'd0: tmp32_a <= res;
						2'd1: tmp32_a <= res <<< 1;
						2'd2: tmp32_a <= res <<< 2;
						2'd3: tmp32_a <= res <<< 3;
					endcase
				end
				dma_ram_we <= 1'b0;
				fsm <= M_SC_WR_HI;
			end
			M_SC_WR_HI: begin
				// write hi-word a +0x10 (sin) o +0x14 (cos)
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= (tmp_hi == 16'd1)
				                 ? (cop_regs_byte_addr(0, 16'h0010) >> 1)
				                 : (cop_regs_byte_addr(0, 16'h0014) >> 1);
				dma_ram_wdata <= tmp32_a[31:16];
				fsm <= M_SC_WR_LO;
			end
			M_SC_WR_LO: begin
				dma_ram_we    <= 1'b1;
				dma_ram_addr  <= (tmp_hi == 16'd1)
				                 ? (cop_regs_byte_addr(0, 16'h0012) >> 1)
				                 : (cop_regs_byte_addr(0, 16'h0016) >> 1);
				dma_ram_wdata <= tmp32_a[15:0];
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0xa180/0xa980 — read pos (MAME cop_collision_read_pos)
			//   flags_swap = cop_read_word(spradr+2)  → host_read_word(spradr+0)
			//   pos[0] = cop_read_word(spradr+6)      → host_read_word(spradr+4)
			//   pos[1] = cop_read_word(spradr+10)     → host_read_word(spradr+8)
			//   pos[2] = cop_read_word(spradr+14)     → host_read_word(spradr+12)
			// spradr = cop_regs[0/1] (passato +2 da macro setup → leggo +2 da setup)
			// In macro setup ho già impostato dma_src_byte = cop_regs_byte_addr(N, +2).
			// Però MAME cop_read_word(+2) = host_read_word(+2^2) = host_read_word(+0).
			// Quindi flags effettivi sono a host_byte_addr = spradr+0.
			// ════════════════════════════════════════════════════════════════
			M_A1_RD_FLAGS: begin
				// Pre-emit addr pos[0] @ host+4. Wait state per BRAM latency.
				dma_src_byte <= (tmp_hi == 16'd0)
				                ? cop_regs_byte_addr(0, 16'h0004)
				                : cop_regs_byte_addr(1, 16'h0004);
				fsm <= M_A1_RD_POS_Y;
			end
			M_A1_RD_POS_Y: begin
				// rdata = word @ host+0 = flags (MAME cop_read_word(spradr+2) ^ 2).
				coll_flags_swap[tmp_hi[0]] <= dma_src_rdata;
				// Pre-emit pos[1] @ host+8.
				dma_src_byte <= (tmp_hi == 16'd0)
				                ? cop_regs_byte_addr(0, 16'h0008)
				                : cop_regs_byte_addr(1, 16'h0008);
				fsm <= M_A1_RD_POS_X;
			end
			M_A1_RD_POS_X: begin
				// rdata = word @ host+4 = pos[0] (Y axis per MAME)
				coll_pos[tmp_hi[0]][0] <= dma_src_rdata;
				// Pre-emit pos[2] @ host+12.
				dma_src_byte <= (tmp_hi == 16'd0)
				                ? cop_regs_byte_addr(0, 16'h000C)
				                : cop_regs_byte_addr(1, 16'h000C);
				fsm <= M_A1_RD_POS_Z;
			end
			M_A1_RD_POS_Z: begin
				// rdata = word @ host+8 = pos[1] (X axis)
				coll_pos[tmp_hi[0]][1] <= dma_src_rdata;
				fsm <= M_A1_RD_POS_ZW;
			end
			M_A1_RD_POS_ZW: begin
				// rdata = word @ host+12 = pos[2] (Z axis)
				coll_pos[tmp_hi[0]][2] <= dma_src_rdata;
				fsm <= S_IDLE;
			end

			// ════════════════════════════════════════════════════════════════
			// 0xb100/0xb900 — update hitbox & compute intersection (1:1 MAME
			// cop_collision_update_hitbox, seibucop.cpp:1003-1065). slot j=tmp_hi[0].
			// TUTTE le letture sono in MAIN ROM (puntatore cop_regs[2/3]=$04xxxx +
			// descrittore 3 word @ hb_adr2+2*i). Lettura via cop_rom_* handshake:
			// REQ alza cop_rom_req 1 ciclo, WAIT stalla finche cop_rom_ready, latcha.
			// dx[i]=int8 [7:0] (signed), size[i]=uint8 [15:8] (unsigned).
			// hb_adr2 = read_word(regs[slot]) | (cop_hit_baseadr[7:0]<<16).
			// ════════════════════════════════════════════════════════════════
			M_B1_REQ_PTR: begin
				cop_rom_addr <= hb_ptr_addr;     // addr del puntatore (in ROM)
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_PTR;
			end
			M_B1_WAIT_PTR: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					// cop_rom_rdata = read_word(cop_regs[slot]) = base descrittore.
					hb_adr2 <= {cop_hit_baseadr[7:0], cop_rom_rdata};
					fsm <= M_B1_REQ_H0;
				end
			end
			M_B1_REQ_H0: begin
				cop_rom_addr <= hb_adr2;          // asse0 @ +0
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_H0;
			end
			M_B1_WAIT_H0: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					hb_dx[0]   <= {{8{cop_rom_rdata[7]}}, cop_rom_rdata[7:0]};  // int8
					hb_size[0] <= {8'd0, cop_rom_rdata[15:8]};                 // uint8
					fsm <= M_B1_REQ_H1;
				end
			end
			M_B1_REQ_H1: begin
				cop_rom_addr <= hb_adr2 + 24'd2;  // asse1 @ +2
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_H1;
			end
			M_B1_WAIT_H1: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					hb_dx[1]   <= {{8{cop_rom_rdata[7]}}, cop_rom_rdata[7:0]};
					hb_size[1] <= {8'd0, cop_rom_rdata[15:8]};
					// num_axis: heatbrl b080/b880 (bit8=0) = 2 assi; legionna b100/b900
					// (bit8=1) = 3 assi. MAME seibucop.cpp:1010 (data & 0x0100).
					hb_3axis <= cmd_value[8];     // ricorda num_axis per il calcolo di res in CALC2
					if (cmd_value[8]) begin
						fsm <= M_B1_REQ_H2;   // 3 assi: leggi asse Z
					end else begin
						hb_dx[2]   <= 16'd0;  // 2 assi: azzera asse Z (no garbage)
						hb_size[2] <= 16'd0;
						fsm <= M_B1_CALC;
					end
				end
			end
			M_B1_REQ_H2: begin
				cop_rom_addr <= hb_adr2 + 24'd4;  // asse2 @ +4
				cop_rom_req  <= 1'b1;
				fsm <= M_B1_WAIT_H2;
			end
			M_B1_WAIT_H2: begin
				cop_rom_req <= 1'b0;
				if (cop_rom_ready) begin
					hb_dx[2]   <= {{8{cop_rom_rdata[7]}}, cop_rom_rdata[7:0]};
					hb_size[2] <= {8'd0, cop_rom_rdata[15:8]};
					fsm <= M_B1_CALC;
				end
			end
			// STAGE 1: solo box min/max (1 add/sub per canale) -> REGISTRA in
			// coll_min/max[j]. Path corto (1 livello aritmetico). I comparatori
			// overlap stanno nello stage 2 con input registrati.
			M_B1_CALC: begin : b1_calc_blk
				reg signed [15:0] nmin, nmax;
				reg signed [15:0] sz;
				integer i;
				reg j;
				j = tmp_hi[0];
				for (i = 0; i < 3; i = i + 1) begin
					sz = $signed({1'b0, hb_size[i][14:0]});
					if (coll_allow_swap[j] && coll_flags_swap[j][i]) begin
						nmax = $signed(coll_pos[j][i]) - hb_dx[i];
						nmin = nmax - sz;
					end else begin
						nmin = $signed(coll_pos[j][i]) + hb_dx[i];
						nmax = nmin + sz;
					end
					coll_min[j][i] <= nmin;
					coll_max[j][i] <= nmax;
				end
				// dY/dX/dZ: 1 sub per canale, indipendente (path corto separato)
				cop_hit_val[0] <= coll_pos[0][0] - coll_pos[1][0];
				cop_hit_val[1] <= coll_pos[0][1] - coll_pos[1][1];
				cop_hit_val[2] <= coll_pos[0][2] - coll_pos[1][2];
				fsm <= M_B1_CALC2;
			end
			// STAGE 2: comparatori overlap su coll_min/max REGISTRATI (entrambi gli
			// slot ora validi). res init 7, clear bit i su overlap signed. Path
			// corto (solo comparatori, nessuna catena aritmetica a monte).
			M_B1_CALC2: begin : b1_calc2_blk
				reg [2:0] res;
				integer i;
				// MAME seibucop.cpp:1038: res init = (num_axis==3 ? 7 : 3). heatbrl=2 assi -> 3'b011.
				// Con 3'b111 + 2 assi il bit2 resta 1 -> res mai 0 -> collisione MAI rilevata
				// (nemici non muoiono). num_axis assi valutati (asse 2 saltato se 2 assi).
				res = hb_3axis ? 3'b111 : 3'b011;
				for (i = 0; i < 3; i = i + 1) begin
					if ((hb_3axis || i < 2) &&
					    ((($signed(coll_max[0][i]) > $signed(coll_min[1][i]) &&
					      $signed(coll_min[0][i]) < $signed(coll_max[1][i])) ||
					     ($signed(coll_max[1][i]) > $signed(coll_min[0][i]) &&
					      $signed(coll_min[1][i]) < $signed(coll_max[0][i])))))
						res[i] = 1'b0;   // overlap su asse i -> collide
				end
				cop_hit_status   <= {13'd0, res};
				cop_hit_val_stat <= {13'd0, res};
				// Segnala "macro completata" come fanno 138e/3bb0/8100 (il gioco
				// polla cop_status[2] prima di leggere cop_hit_status). Senza
				// questo il gioco leggeva il risultato a timing casuale (la lettura
				// ROM hitbox ha latency variabile) -> confine nemici INTERMITTENTE.
				cop_status[2]    <= 1'b1;
				cop_status[14:3] <= 12'd0;
				cop_status[0]    <= 1'b1;
				fsm <= S_IDLE;
			end

			default: fsm <= S_IDLE;
			endcase
		end
	end

endmodule
