// SPDX-License-Identifier: GPL-3.0-or-later
/*  This file is part of HeatedBarrel_MiSTer.

    HeatedBarrel_MiSTer is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    HeatedBarrel_MiSTer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with HeatedBarrel_MiSTer.  If not, see <http://www.gnu.org/licenses/>.

    Author: Umberto Parisi (rmonic79)
    Version: 1.0
    Date: 2026

*/

/*  Audio subsystem Seibu Blood Bros (Z80 + YM3812 + OKI6295)

    Spec da MAME (reference/mame_seibu/seibusound.cpp + bloodbro.cpp):
      - Z80 @ 3.579545 MHz (XTAL 7.159090 / 2)
      - YM3812 (jtopl2) @ 3.579545 MHz
      - OKI M6295 (jt6295) @ 1.000 MHz (XTAL 12/12), PIN7=HIGH

    Z80 memory map: Seibu standard (seibu_sound_map).
    Main↔sub comm @ 0xA0000-0xA000D: standard MAME seibu_sound_device.

    IRQ Z80 (IM0): RST10 ← YM IRQ, RST18 ← main wakeup.
*/

module HeatedBarrel_audio_z80 (
	input  wire        clk,
	input  wire        reset,
	input  wire        pause,
	input  wire  [1:0] clk_sel,

	input  wire        ioctl_download,
	input  wire        ioctl_wr,
	input  wire [26:0] ioctl_addr,
	input  wire [15:0] ioctl_dout,

	input  wire        snd_cs,
	input  wire  [4:1] snd_addr,
	input  wire        snd_wr,
	input  wire        snd_rd,
	input  wire [15:0] snd_wdata,
	output wire [15:0] snd_rdata,
	input  wire        snd_nmi_n,
	input  wire        snd_reset_in,

	input  wire  [7:0] coin_input,

	// OKI ADPCM ROM bridge (128KB SDRAM, blood bros = bb_08 128KB)
	output wire [17:0] oki_rom_addr,
	input  wire  [7:0] oki_rom_data,
	input  wire        oki_rom_ok,

	input  wire  [5:0] fm_vol_q44,
	input  wire  [5:0] oki_vol_q44,

	output reg signed [15:0] audio_l,
	output reg signed [15:0] audio_r
);

	// ─── Clock enable: clk_sys (96 MHz) → Z80/YM 3.58 MHz, OKI 1.0 MHz ──────
	// Z80/YM: 96/3.579545 = 26.82 → div 27 (3.555 MHz)
	// OKI: 96/1.0 = 96 → div 96, target 1.000 MHz (PIN7_HIGH bloodbro)
	reg [4:0] cen_z80_cnt;
	reg       cen_z80;
	always @(posedge clk) begin
		if (reset) begin
			cen_z80_cnt <= 5'd0;
			cen_z80     <= 1'b0;
		end else if (cen_z80_cnt == 5'd26) begin
			cen_z80_cnt <= 5'd0;
			cen_z80     <= 1'b1;
		end else begin
			cen_z80_cnt <= cen_z80_cnt + 5'd1;
			cen_z80     <= 1'b0;
		end
	end

	reg [6:0] cen_oki_cnt;
	reg       cen_oki;
	always @(posedge clk) begin
		if (reset) begin
			cen_oki_cnt <= 7'd0;
			cen_oki     <= 1'b0;
		end else if (cen_oki_cnt == 7'd95) begin
			cen_oki_cnt <= 7'd0;
			cen_oki     <= 1'b1;
		end else begin
			cen_oki_cnt <= cen_oki_cnt + 7'd1;
			cen_oki     <= 1'b0;
		end
	end

	wire cen_z80_g = cen_z80 & ~pause;
	wire cen_oki_g = cen_oki & ~pause;

	// ─── Z80 signals ─────────────────────────────────────────────────────────
	wire [15:0] z80_addr;
	wire  [7:0] z80_dout;
	reg   [7:0] z80_din;
	wire        z80_mreq_n, z80_iorq_n, z80_rd_n, z80_wr_n, z80_m1_n;
	wire        z80_int_n;
	wire        z80_busak_n, z80_halt_n;

	wire rom_lo_cs   = ~z80_mreq_n && (z80_addr[15:13] == 3'b000);
	wire rom_hi_cs   = ~z80_mreq_n && (z80_addr[15] == 1'b1);
	wire ram_cs      = ~z80_mreq_n && (z80_addr[15:11] == 5'b00100);
	wire reg_cs      = ~z80_mreq_n && (z80_addr[15:5] == 11'h200);
	wire oki_cs      = ~z80_mreq_n && (z80_addr[15:12] == 4'h6);

	// ─── ROM Z80 — Blood Bros bb_07 32KB, layout BRAM 64KB con alias bank 1 ─
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80_rom_lo [0:32767];
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80_rom_hi [0:32767];
	reg [7:0] z80_rom_lo_q = 8'h00, z80_rom_hi_q = 8'h00;

	wire z80_rom_dl_wr =
		ioctl_download && ioctl_wr && (ioctl_addr >= 27'h4E0000) && (ioctl_addr < 27'h4F0000);
	wire [14:0] z80_rom_dl_word = ioctl_addr[15:1];

	reg rom_bank;

	wire [15:0] z80_rom_byte_addr =
		rom_lo_cs              ? z80_addr :
		(rom_hi_cs & ~rom_bank) ? z80_addr :
		(rom_hi_cs &  rom_bank) ? {1'b0, z80_addr[14:0]} :
		                          z80_addr;

	reg z80_addr_lsb_d;
	always @(posedge clk) begin
		if (z80_rom_dl_wr) begin
			z80_rom_lo[z80_rom_dl_word] <= ioctl_dout[7:0];
			z80_rom_hi[z80_rom_dl_word] <= ioctl_dout[15:8];
		end
		z80_rom_lo_q   <= z80_rom_lo[z80_rom_byte_addr[15:1]];
		z80_rom_hi_q   <= z80_rom_hi[z80_rom_byte_addr[15:1]];
		z80_addr_lsb_d <= z80_rom_byte_addr[0];
	end

	wire [7:0] z80_rom_q = z80_addr_lsb_d ? z80_rom_hi_q : z80_rom_lo_q;

	// ─── RAM Z80 2KB ─────────────────────────────────────────────────────────
	(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80_ram [0:2047];
	reg [7:0] z80_ram_q = 8'h00;

	// synthesis translate_off
	integer z80_ram_init_i;
	initial begin
		for (z80_ram_init_i = 0; z80_ram_init_i < 2048; z80_ram_init_i = z80_ram_init_i + 1)
			z80_ram[z80_ram_init_i] = 8'h00;
	end
	// synthesis translate_on

	always @(posedge clk) begin
		if (ram_cs && !z80_wr_n) z80_ram[z80_addr[10:0]] <= z80_dout;
		z80_ram_q <= z80_ram[z80_addr[10:0]];
	end

	// ─── Sub-region decoder ──────────────────────────────────────────────────
	wire is_pending_w   = reg_cs && (z80_addr[4:0] == 5'h00) && !z80_wr_n;
	wire is_irq_clear   = reg_cs && (z80_addr[4:0] == 5'h01) && !z80_wr_n;
	wire is_rst10_ack   = reg_cs && (z80_addr[4:0] == 5'h02) && !z80_wr_n;
	wire is_rst18_ack   = reg_cs && (z80_addr[4:0] == 5'h03) && !z80_wr_n;
	wire is_bank_w      = reg_cs && (z80_addr[4:0] == 5'h07) && !z80_wr_n;
	wire is_ym_access   = reg_cs && (z80_addr[4:1] == 4'h4);
	wire is_ym_w        = is_ym_access && !z80_wr_n;
	wire is_ym_r        = is_ym_access && !z80_rd_n;
	wire is_latch_lo_r  = reg_cs && (z80_addr[4:0] == 5'h10) && !z80_rd_n;
	wire is_latch_hi_r  = reg_cs && (z80_addr[4:0] == 5'h11) && !z80_rd_n;
	wire is_pending_r   = reg_cs && (z80_addr[4:0] == 5'h12) && !z80_rd_n;
	wire is_coin_r      = reg_cs && (z80_addr[4:0] == 5'h13) && !z80_rd_n;
	wire is_data_lo_w   = reg_cs && (z80_addr[4:0] == 5'h18) && !z80_wr_n;
	wire is_data_hi_w   = reg_cs && (z80_addr[4:0] == 5'h19) && !z80_wr_n;
	wire is_coin_w      = reg_cs && (z80_addr[4:0] == 5'h1B) && !z80_wr_n;

	always @(posedge clk) begin
		if (reset)
			rom_bank <= 1'b0;
		else if (cen_z80 && is_bank_w)
			rom_bank <= z80_dout[0];
	end

	// ─── Soundlatch main↔sub state ───────────────────────────────────────────
	reg [7:0] main2sub [0:1];
	reg [7:0] sub2main [0:1];
	reg       main2sub_pending;
	reg       sub2main_pending;

	// ─── IRQ controller (IM0 RST10/RST18) ────────────────────────────────────
	reg rst10_irq, rst10_service;
	reg rst18_irq, rst18_service;
	wire ym_irq_n;
	wire ym_irq = ~ym_irq_n;
	reg  ym_irq_d;

	wire iack_active = ~z80_m1_n && ~z80_iorq_n;
	reg  iack_active_d;
	reg  [7:0] iack_vector_latched;
	wire [7:0] iack_vector_now =
	    (rst18_irq && !rst18_service) ? 8'hDF :
	    (rst10_irq && !rst10_service) ? 8'hD7 :
	                                    8'h00;
	always @(posedge clk) begin
		if (reset) begin
			iack_active_d       <= 1'b0;
			iack_vector_latched <= 8'h00;
		end else begin
			iack_active_d <= iack_active;
			if (iack_active && !iack_active_d) begin
				iack_vector_latched <= iack_vector_now;
			end
		end
	end
	wire [7:0] iack_vector = iack_active_d ? iack_vector_latched : iack_vector_now;

	wire irq_active = (rst10_irq && !rst10_service) || (rst18_irq && !rst18_service);
	assign z80_int_n = ~irq_active;

	always @(posedge clk) begin
		if (reset) begin
			rst10_irq     <= 1'b0;
			rst10_service <= 1'b0;
			rst18_irq     <= 1'b0;
			rst18_service <= 1'b0;
			ym_irq_d      <= 1'b0;
		end else begin
			ym_irq_d <= ym_irq;
			if (ym_irq && !ym_irq_d)        rst10_irq <= 1'b1;
			else if (!ym_irq && ym_irq_d)   rst10_irq <= 1'b0;

			if (snd_cs && snd_wr && snd_addr == 4'd4)
				rst18_irq <= 1'b1;

			if (iack_active_d && !iack_active) begin
				if (iack_vector_latched == 8'hDF) begin
					rst18_service <= 1'b1;
					rst18_irq     <= 1'b0;
				end else if (iack_vector_latched == 8'hD7) begin
					rst10_service <= 1'b1;
				end
			end

			if (cen_z80) begin
				if (is_irq_clear)  rst18_service <= 1'b0;
				if (is_rst10_ack)  rst10_service <= 1'b0;
				if (is_rst18_ack)  rst18_service <= 1'b0;
			end
		end
	end

	// ─── Soundlatch main_w/r logic ──────────────────────────────────────────
`ifdef MISTER_SIM
	// SIM MODE: stub Z80 — simula coin1 + start1 a tempi prestabiliti per
	// avanzare oltre la INTRO/title screen del game.
	//
	// SEIBU coin protocol: main scrive a 0x100702 (snd_addr=1, "cmd_b"),
	// Z80 risponde tramite sub2main[0] (letto da main 68K a 0x100704).
	// Valore 0xA0 = idle (no coin), 0xA1 = coin1 inserted.
	//
	// Plan:
	//   - Per ~10M tick (≈ 100ms sim) restituisco 0xA0 (idle, attract mode)
	//   - Per i successivi 5M tick restituisco 0xA1 (coin1 inserted)
	//   - Poi di nuovo 0xA0 (release)
	//   - Poi 0xA1 di nuovo (start1 — same encoding tipicamente)
	//   - Poi 0xA0 fisso (gioco in corso)
	reg [27:0] sim_phase_cnt = 0;
	reg [7:0]  sim_coin_resp = 8'hA0;
	always @(posedge clk) begin
		if (reset) begin
			sim_phase_cnt <= 0;
			sim_coin_resp <= 8'hA0;
		end else begin
			sim_phase_cnt <= sim_phase_cnt + 1;
			// 50M tick (~520ms @96MHz) → press coin1
			if      (sim_phase_cnt == 28'd50_000_000) sim_coin_resp <= 8'hA1;
			else if (sim_phase_cnt == 28'd55_000_000) sim_coin_resp <= 8'hA0;
			// 70M tick → press start1
			else if (sim_phase_cnt == 28'd70_000_000) sim_coin_resp <= 8'hA1;
			else if (sim_phase_cnt == 28'd75_000_000) sim_coin_resp <= 8'hA0;
		end
	end

	always @(posedge clk) begin
		if (reset) begin
			main2sub[0]      <= 8'd0;
			main2sub[1]      <= 8'd0;
			sub2main[0]      <= 8'd0;
			sub2main[1]      <= 8'd0;
			main2sub_pending <= 1'b0;
			sub2main_pending <= 1'b0;
		end else begin
			if (snd_cs && snd_wr) begin
				case (snd_addr)
					4'd0: begin
						main2sub[0]      <= snd_wdata[7:0];
						main2sub_pending <= 1'b1;
					end
					4'd1: begin
						// SEIBU cmd: simula Z80 response. Il 68k legge STATUS da
						// $10070C=sub2main[1] e STROBE/coin-bitmask da $100708=sub2main[0]
						// ($CCA: cmpi.b #$A0,$108050(=sub2main[1]); tst.b $108052(=sub2main[0])).
						// PRIMA era invertito (status in [0]) -> coin mai accreditata in sim.
						main2sub[1]      <= snd_wdata[7:0];
						sub2main[1]      <= sim_coin_resp;   // STATUS $A0/$A1 -> $10070C
						sub2main[0]      <= (sim_coin_resp == 8'hA1) ? 8'h01 : 8'h00; // coin strobe bit0 -> $100708
						sub2main_pending <= 1'b1;
						main2sub_pending <= 1'b0;
					end
					4'd2, 4'd6: begin
						sub2main_pending <= 1'b0;
						main2sub_pending <= 1'b1;
					end
					default: ;
				endcase
			end
		end
	end
`else
	always @(posedge clk) begin
		if (reset) begin
			main2sub[0]      <= 8'd0;
			main2sub[1]      <= 8'd0;
			sub2main[0]      <= 8'd0;
			sub2main[1]      <= 8'd0;
			main2sub_pending <= 1'b0;
			sub2main_pending <= 1'b0;
		end else begin
			if (snd_cs && snd_wr) begin
				case (snd_addr)
					4'd0: main2sub[0] <= snd_wdata[7:0];
					4'd1: main2sub[1] <= snd_wdata[7:0];
					4'd2, 4'd6: begin
						sub2main_pending <= 1'b0;
						main2sub_pending <= 1'b1;
					end
					default: ;
				endcase
			end
			if (cen_z80) begin
				if (is_data_lo_w) sub2main[0] <= z80_dout;
				if (is_data_hi_w) sub2main[1] <= z80_dout;
				if (is_pending_w) begin
					main2sub_pending <= 1'b0;
					sub2main_pending <= 1'b1;
				end
			end
		end
	end
`endif

	// MAME seibu_sound_device::main_r (reference/mame_seibu/seibusound.cpp:279):
	//   case 2,3: return sub2main[offset-2]   (soundlatch data: coin/input dal Z80)
	//   case 5:   return main2sub_pending ? 1 : 0
	//   default:  return 0xff
	// snd_addr ora E' l'offset seibu (0..7) = bus_addr[4:2]. Mappa 68k:
	//   $100708->2 sub2main[0], $10070C->3 sub2main[1], $100714->5 main2sub_pending.
	// Il 68k a $B1E/$B24 legge $100708/$10070C = sub2main (risposta coin Z80 $A0/$A1);
	// a $B10 legge $100714 = main2sub_pending (BTST#0: 0 a riposo -> boot prosegue).
	wire [7:0] main_r_data =
		(snd_addr == 4'd2)  ? sub2main[0] :
		(snd_addr == 4'd3)  ? sub2main[1] :
		(snd_addr == 4'd5)  ? {7'd0, main2sub_pending} :
		                       8'hFF;
	assign snd_rdata = {8'h00, main_r_data};

	// ─── YM3812 (jtopl2) mono ────────────────────────────────────────────────
	wire [7:0] ym_dout;
	wire signed [15:0] ym_snd;
	wire        ym_sample;
	jtopl2 u_jtopl2 (
		.rst    (reset),
		.clk    (clk),
		.cen    (cen_z80_g),
		.din    (z80_dout),
		.addr   (z80_addr[0]),
		.cs_n   (~is_ym_access),
		.wr_n   (z80_wr_n),
		.dout   (ym_dout),
		.irq_n  (ym_irq_n),
		.snd    (ym_snd),
		.sample (ym_sample)
	);

	// ─── OKI M6295 (jt6295) — PIN7_HIGH (bloodbro.cpp:872) ──────────────────
	wire [7:0] oki_dout;
	wire signed [13:0] oki_sound;
	wire        oki_sample;

	jt6295 #(.INTERPOL(0)) u_jt6295 (
		.rst       (reset),
		.clk       (clk),
		.cen       (cen_oki_g),
		.ss        (1'b1),                // PIN7 = HIGH (Blood Bros)
		.wrn       (~(oki_cs & ~z80_wr_n)),
		.din       (z80_dout),
		.dout      (oki_dout),
		.rom_addr  (oki_rom_addr),
		.rom_data  (oki_rom_data),
		.rom_ok    (oki_rom_ok),
		.sound     (oki_sound),
		.sample    (oki_sample)
	);

	// ─── Z80 din mux (pending_r = sub2main_pending, verified DCon) ──────────
	always @(*) begin
		if (iack_active)         z80_din = iack_vector;
		else if (rom_lo_cs)      z80_din = z80_rom_q;
		else if (rom_hi_cs)      z80_din = z80_rom_q;
		else if (ram_cs)         z80_din = z80_ram_q;
		else if (is_ym_r)        z80_din = ym_dout;
		else if (is_latch_lo_r)  z80_din = main2sub[0];
		else if (is_latch_hi_r)  z80_din = main2sub[1];
		else if (is_pending_r)   z80_din = {7'd0, sub2main_pending};
		else if (is_coin_r)      z80_din = coin_input;
		else if (oki_cs)         z80_din = oki_dout;
		else                     z80_din = 8'hFF;
	end

	// ─── T80s Z80 core ───────────────────────────────────────────────────────
	wire t80_halt_n_g  = ~pause;
	wire t80_busrq_n   = 1'b1;
	wire t80_wait_n    = 1'b1;
	wire t80_nmi_n     = 1'b1;
	wire t80_reset_n   = ~reset & ~snd_reset_in;

	T80s u_z80 (
		.RESET_n (t80_reset_n),
		.CLK     (clk),
		.CEN     (cen_z80 & t80_halt_n_g),
		.WAIT_n  (t80_wait_n),
		.INT_n   (z80_int_n),
		.NMI_n   (t80_nmi_n),
		.BUSRQ_n (t80_busrq_n),
		.M1_n    (z80_m1_n),
		.MREQ_n  (z80_mreq_n),
		.IORQ_n  (z80_iorq_n),
		.RD_n    (z80_rd_n),
		.WR_n    (z80_wr_n),
		.RFSH_n  (),
		.HALT_n  (z80_halt_n),
		.BUSAK_n (z80_busak_n),
		.OUT0    (1'b0),
		.A       (z80_addr),
		.DI      (z80_din),
		.DO      (z80_dout),
		.REG     ()
	);

	// ─── Mixer audio: YM3812 mono + OKI mono → AUDIO_L/R ────────────────────
	wire signed [22:0] ym_v   = $signed(ym_snd)              * $signed({1'b0, fm_vol_q44});
	wire signed [22:0] oki_v  = $signed({oki_sound, 2'b00})  * $signed({1'b0, oki_vol_q44});
	wire signed [18:0] ym_s   = ym_v[22:4];
	wire signed [18:0] oki_s  = oki_v[22:4];
	wire signed [19:0] mix    = {ym_s[18], ym_s} + {oki_s[18], oki_s};

	always @(posedge clk) begin
		if (reset) begin
			audio_l <= 16'sd0;
			audio_r <= 16'sd0;
		end else begin
			audio_l <= (mix > 20'sd32767)   ? 16'sh7FFF :
			           (mix < -20'sd32767) ? 16'sh8000 :
			                                  mix[15:0];
			audio_r <= audio_l;
		end
	end

endmodule
