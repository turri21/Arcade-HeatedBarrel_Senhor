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

/*  Sprite renderer Seibu SEI252 / RISE (HeatedBarrel family).

    Derivato dal sprite renderer SEI0211 di GundamSD/DCon. Formato spriteram
    quasi identico (entrambi MAME sei021x_sei0220_spr.cpp) — differenze:
      - Spriteram @ 0x105000-0x105FFF (4KB, 1024 word, 256 entry × 4 word)
      - w3 bit 15 = tile bank extra bit (denjinmk usa, HeatedBarrel base = 0)

    Format word (MAME legionna_v.cpp:288-312):
      w0 bit 15      = enable (0=draw, 1=skip — TODO verify HeatedBarrel)
      w0 bit 14      = flip_x
      w0 bit 13      = flip_y (??? non sicuro in HeatedBarrel)
      w0 bit 12..10  = sizex (3-bit, +1) → 1..8 tile larghezza
      w0 bit 9..7    = sizey (3-bit, +1) → 1..8 tile altezza
      w0 bit 6       = tile bank bit (denjinmk) / extra priority pin (grainbow)
      w0 bit 5..0    = color (6-bit)
      w1 bit 15..14  = priority code (2-bit → m_sprite_pri_mask[pri])
      w1 bit 13..0   = tile_code (14-bit)
      w2 bit 11..0   = X (signed 12-bit con sign extension a 9-bit)
      w3 bit 15      = tile bank extra (Denjin Makai)
      w3 bit 11..0   = Y (signed 12-bit)

    Tile size 16×16 4bpp = 128 byte/tile = 32 word 16-bit.

    Priority callback legionna_v.cpp:314-317:
      pri_mask[0] = 0x0000 (sopra tutto)
      pri_mask[1] = 0xFFF0
      pri_mask[2] = 0xFFFC
      pri_mask[3] = 0xFFFE

    Pen 15 = trasparente.

    Architettura:
      - Sprite scan FSM durante linea N: scorre 256 entry, per quelle che
        intersecano linea N+1 (target_y), fetch SDRAM dei tile coperti e
        scrive in line buffer non-attivo.
      - Read side: a hpos legge line buffer attivo, restituisce pen+pri_code.
      - Ping-pong al new_line.

    ─── Ottimizzazioni performance (no logic change) ────────────────────
    Line buffer = 8 bank interleaved da 32×14 (256 pixel totali, lane =
    dx[2:0], riga = dx[7:3]). Permette:
      - SC_DECODE in 1 ciclo invece di 8: tutti gli 8 pixel della mezza-
        riga scritti in parallelo (un write per bank).
      - SC_CLEAR in 32 cicli invece di 256: 8 entry azzerate/ciclo.
    Early-skip entry: legge solo w0/w3, valuta enable&&in_y&&layer_en,
    e fetcha w1/w2 SOLO se lo sprite è visibile. Risparmio ~3 cicli sulla
    maggioranza degli slot (disabled/off-screen). Stesso pattern Blood Bros.

    Worst case: 256 entry × max 8 sizex × 1 fetch/tile = 2048 fetch/linea.
    Realistico: 30 sprite × 4 sizex avg = 120 fetch. Banda OK.
*/

module HeatedBarrel_sprite_renderer (
	input  wire        clk,
	input  wire        reset,
	input  wire        ce_pix,

	input  wire  [9:0] hpos,        // 0..319 logico
	input  wire  [8:0] vpos,        // 0..223 logico
	input  wire        de,
	input  wire        layer_en,
	input  wire        new_line,

	// OSD offset di rendering (debug pixel-hunting)
	input  wire signed [9:0] xoff,
	input  wire signed [9:0] yoff,

	// Sprite RAM read port (dual-port lato B)
	output reg  [10:0] spr_addr,    // 2048 word total (512 entry × 4 word)
	input  wire [15:0] spr_data,

	// SDRAM tile fetch via arbiter (client r3, kind=3, no cache)
	output reg         rom_req,
	output reg  [23:0] rom_addr,
	input  wire [31:0] rom_data,
	input  wire        rom_valid,

	// Output pixel (combinatorio come i tile layer: stessa latenza = no shift)
	output wire        opaque,
	output wire [10:0] pen_index,   // sprite color base = 0 (palette[0..1023])
	output wire  [1:0] pri_code     // 2-bit priority dal w1
);

	// ─── Line buffer ping-pong (8 bank interleaved per parallel write) ──
	// Layout 14-bit: [13:8]=color, [7:6]=pri_code, [5:4]=00, [3:0]=pen
	// Valid pixel = pen != 0xF (sentinel "no sprite") — usiamo 0xF come "vuoto"
	// perché pen 15 reale = trasparente (skipped al draw).
	// Bank b contiene i pixel con dx[2:0]==b → indicizzato da dx[7:3] (0..31).
	// Permette 8 write paralleli in 1 ciclo (decode 8 pixel/ciclo) e clear
	// in 32 cicli invece di 256. Vista esterna: identica al singolo array.
	localparam [13:0] LB_EMPTY = 14'h003F;  // color=0, pri=0, pen=15 (trasparente)
	// Line buffer = 16 BRAM (8 lane x 2 buffer ping-pong) via spram_dp =
	// altsyncram M10K GARANTITO (no ALM). Ogni bank: 32 entry x 14 bit.
	// Write port muxato clear/decode; read port a rd_row. read latency = 1.
	reg active_buf;

	// ─── Sprite scan FSM ─────────────────────────────────────────────────────
	// Stati: IDLE → CLEAR (azzera buffer non-attivo a pen=15) → RW0/RW1/RW3
	// (early-read w0,w3) → CHECK → RW2/CHECK2/CHECK2_W2 (read w1,w2 se visibile)
	// → ROM_REQ/W → DECODE → NEXT_TX/E → DONE.
	localparam SC_IDLE      = 4'd0;
	localparam SC_CLEAR     = 4'd1;
	localparam SC_RW0       = 4'd2;
	localparam SC_RW1       = 4'd3;
	localparam SC_RW2       = 4'd4;
	localparam SC_RW3       = 4'd5;
	localparam SC_CHECK     = 4'd6;
	localparam SC_CHECK2    = 4'd7;
	localparam SC_ROM_REQ   = 4'd8;
	localparam SC_ROM_W     = 4'd9;
	localparam SC_DECODE    = 4'd10;
	localparam SC_NEXT_TX   = 4'd11;
	localparam SC_NEXT_E    = 4'd12;
	localparam SC_DONE      = 4'd13;
	localparam SC_CHECK2_W2 = 4'd14;

	reg [3:0] sc_state;
	reg [8:0] entry_idx;       // 0..511 (MAME size/2/4 = 4096/2/4 = 512)
	reg [4:0] clear_idx;       // 0..31 per clear buffer (8 lane in parallelo)
	reg [15:0] sp_w0, sp_w1, sp_w2, sp_w3;
	reg        pf_side;        // 0=metà sx tile (col 0..7), 1=metà dx (col 8..15)

	// Decoded fields — MAME sei0210_device::draw (non-alt format, riga 143):
	//   if (BIT(~spriteram[i], 15)) continue;  → draw quando w0[15]==1.
	wire        sp_enable = sp_w0[15];
	wire        sp_flipx  = sp_w0[14];
	wire        sp_flipy  = sp_w0[13];
	wire  [2:0] sp_sizex  = sp_w0[12:10];      // +1 → 1..8 tile
	wire  [2:0] sp_sizey  = sp_w0[9:7];        // +1
	wire  [5:0] sp_color  = sp_w0[5:0];
	wire  [1:0] sp_pri    = sp_w1[15:14];
	wire [13:0] sp_code   = sp_w1[13:0];
	// SEI0211 get_coordinate (MAME sei021x_sei0220_spr.h):
	//   coord &= 0x1ff;
	//   return (coord >= 0x180) ? coord - 0x200 : coord;
	// Range effettivo: -128..+383 (positivi 0..0x17F, negativi 0x180..0x1FF).
	wire [8:0] sp_xraw = sp_w2[8:0];
	wire [8:0] sp_yraw = sp_w3[8:0];
	wire signed [10:0] sp_x_raw = (sp_xraw >= 9'h180) ? ({2'b00, sp_xraw} - 11'h200) : {2'b00, sp_xraw};
	wire signed [10:0] sp_y_raw = (sp_yraw >= 9'h180) ? ({2'b00, sp_yraw} - 11'h200) : {2'b00, sp_yraw};
	// MAME SEI0211 default m_xoffset/m_yoffset = 0 (sei021x_sei0220_spr.cpp:40-41).
	// HeatedBarrel NON chiama set_offset (legionna.cpp:1214) → offset 0,0.
	// +1 pixel a destra: offset verificato con pixel hunting su MAME (rmonic79).
	// L'offset (xoff+1) è REGISTRATO fuori dal path critico sp_w2→linebuffer: una
	// seconda addizione su sp_x (sp_x_raw + xoff + 1) allungava la catena combinatoria
	// più lunga del design (slack -0.5ns su sp_w2) → metastabilità sull'indirizzo del
	// line buffer → sprite scritti a X instabile = MOONWALK (martello che scivola).
	// xoff cambia solo da OSD (statico durante il rendering): 1 ciclo di latenza ok.
	reg signed [10:0] xoff_eff_r;
	always @(posedge clk) xoff_eff_r <= {xoff[9], xoff} + 11'sd1;
	wire signed [10:0] sp_x = sp_x_raw + xoff_eff_r;
	wire signed [10:0] sp_y = sp_y_raw + {yoff[9], yoff};

	wire [3:0] sp_w  = {1'b0, sp_sizex} + 4'd1;    // 1..8
	wire [3:0] sp_h  = {1'b0, sp_sizey} + 4'd1;    // 1..8

	// Target Y per la linea che stiamo prefetchando (= linea corrente + 1, wrap)
	wire [8:0] target_y = (vpos == 9'd255) ? 9'd0 : (vpos + 9'd1);  // 256x256 heatbrl

	// Sprite intersect check: target_y in [sp_y, sp_y + sp_h*16)
	wire signed [10:0] dy_top = {2'b00, target_y} - sp_y;
	wire        in_y     = (dy_top >= 0) && (dy_top < {3'd0, sp_h, 4'd0});  // sp_h*16
	wire  [3:0] tile_y_in = dy_top[7:4];   // tile row 0..7 dentro sprite
	wire  [3:0] row_in    = dy_top[3:0];   // row dentro tile 0..15
	// flip Y
	wire  [3:0] eff_tile_y = sp_flipy ? (sp_h - 4'd1 - tile_y_in) : tile_y_in;
	wire  [3:0] eff_row    = sp_flipy ? (4'd15 - row_in)          : row_in;

	// Iteratore tile_x
	reg  [3:0] tile_x_pf;
	reg [31:0] pf_rom_data;
	wire  [3:0] eff_tile_x = sp_flipx ? (sp_w - 4'd1 - tile_x_pf) : tile_x_pf;

	// Tile code finale Y-MAJOR (MAME draw_internal: ax=outer, ay=inner, code++):
	//   sub_index = ax*sizey + ay  →  cur_tile = code + eff_tile_x*sp_h + eff_tile_y
	wire [13:0] cur_tile = sp_code + ({10'd0, eff_tile_x} * {10'd0, sp_h}) + {10'd0, eff_tile_y};

	// new_line gating
	wire vpos_visible = (vpos < 9'd256);  // 256x256 heatbrl
	wire gated_new_line = new_line & vpos_visible;

	// ─── Parallel decode (8 pixel della mezza-riga in un colpo) ──────────────
	// dcon_tilelayout: rom_data 32-bit = 8 pixel per row.
	// pf_side=0 → pixel col 0..7, pf_side=1 → col 8..15 (offset +64 byte).
	// k<4 → byte alti del word, k>=4 → byte bassi.
	//   byte_lo = pf_rom_data[31:24]/[15:8], byte_hi = [23:16]/[7:0]
	// Plane order DCBA (verificato su HW, come BG/MG/FG):
	//   pen[0] dal bit "D" (byte_hi[3-sub]), pen[3] dal bit "A" (byte_lo[7-sub]).
	function [3:0] pen_at;
		input integer k;
		reg [7:0] byte_lo, byte_hi;
		reg [2:0] sub;
		begin
			if (k < 4) begin byte_lo = pf_rom_data[31:24]; byte_hi = pf_rom_data[23:16]; end
			else       begin byte_lo = pf_rom_data[15:8];  byte_hi = pf_rom_data[7:0];  end
			sub = 3'd3 - k[1:0];
			pen_at[0] = byte_hi[3 - sub];  // D
			pen_at[1] = byte_hi[7 - sub];  // C
			pen_at[2] = byte_lo[3 - sub];  // B
			pen_at[3] = byte_lo[7 - sub];  // A
		end
	endfunction

	wire [3:0] pen0 = pen_at(0);
	wire [3:0] pen1 = pen_at(1);
	wire [3:0] pen2 = pen_at(2);
	wire [3:0] pen3 = pen_at(3);
	wire [3:0] pen4 = pen_at(4);
	wire [3:0] pen5 = pen_at(5);
	wire [3:0] pen6 = pen_at(6);
	wire [3:0] pen7 = pen_at(7);

	// Posizione X del primo pixel della mezza-riga sullo schermo.
	//   dx = sp_x + tile_x_pf*16 + col_in_tile, col_in_tile = pf_side*8 + k
	//   flipX: col_in_tile = 15 - col_in_tile
	wire signed [10:0] base_x = sp_x + ({6'd0, tile_x_pf, 4'd0});

	function signed [10:0] dx_at;
		input integer k;
		reg [4:0] eff_col;
		begin
			eff_col = sp_flipx ? (5'd15 - {pf_side, k[2:0]})
			                   : {pf_side, k[2:0]};
			dx_at = base_x + {6'd0, eff_col};
		end
	endfunction

	wire signed [10:0] dx0 = dx_at(0);
	wire signed [10:0] dx1 = dx_at(1);
	wire signed [10:0] dx2 = dx_at(2);
	wire signed [10:0] dx3 = dx_at(3);
	wire signed [10:0] dx4 = dx_at(4);
	wire signed [10:0] dx5 = dx_at(5);
	wire signed [10:0] dx6 = dx_at(6);
	wire signed [10:0] dx7 = dx_at(7);

	// Mask "scrivibile": pen != 15 e dx in [0,256) (256 pixel visibili)
	wire wr0 = (pen0 != 4'd15) && (dx0 >= 0) && (dx0 < 256);
	wire wr1 = (pen1 != 4'd15) && (dx1 >= 0) && (dx1 < 256);
	wire wr2 = (pen2 != 4'd15) && (dx2 >= 0) && (dx2 < 256);
	wire wr3 = (pen3 != 4'd15) && (dx3 >= 0) && (dx3 < 256);
	wire wr4 = (pen4 != 4'd15) && (dx4 >= 0) && (dx4 < 256);
	wire wr5 = (pen5 != 4'd15) && (dx5 >= 0) && (dx5 < 256);
	wire wr6 = (pen6 != 4'd15) && (dx6 >= 0) && (dx6 < 256);
	wire wr7 = (pen7 != 4'd15) && (dx7 >= 0) && (dx7 < 256);

	// Bank di destinazione per ogni step: dx[2:0]. Lane address: dx[7:3].
	wire [2:0] ln0 = dx0[2:0];   wire [4:0] rw0_a = dx0[7:3];
	wire [2:0] ln1 = dx1[2:0];   wire [4:0] rw1_a = dx1[7:3];
	wire [2:0] ln2 = dx2[2:0];   wire [4:0] rw2_a = dx2[7:3];
	wire [2:0] ln3 = dx3[2:0];   wire [4:0] rw3_a = dx3[7:3];
	wire [2:0] ln4 = dx4[2:0];   wire [4:0] rw4_a = dx4[7:3];
	wire [2:0] ln5 = dx5[2:0];   wire [4:0] rw5_a = dx5[7:3];
	wire [2:0] ln6 = dx6[2:0];   wire [4:0] rw6_a = dx6[7:3];
	wire [2:0] ln7 = dx7[2:0];   wire [4:0] rw7_a = dx7[7:3];

	// Dato da scrivere per ogni step (formato linebuf 14-bit)
	wire [13:0] wd0 = {sp_color, sp_pri, 2'd0, pen0};
	wire [13:0] wd1 = {sp_color, sp_pri, 2'd0, pen1};
	wire [13:0] wd2 = {sp_color, sp_pri, 2'd0, pen2};
	wire [13:0] wd3 = {sp_color, sp_pri, 2'd0, pen3};
	wire [13:0] wd4 = {sp_color, sp_pri, 2'd0, pen4};
	wire [13:0] wd5 = {sp_color, sp_pri, 2'd0, pen5};
	wire [13:0] wd6 = {sp_color, sp_pri, 2'd0, pen6};
	wire [13:0] wd7 = {sp_color, sp_pri, 2'd0, pen7};

	reg        bank_we [0:7];
	reg [4:0]  bank_row [0:7];
	reg [13:0] bank_wd  [0:7];

	always @(*) begin : bank_select
		integer b;
		for (b = 0; b < 8; b = b + 1) begin
			bank_we[b]  = 1'b0;
			bank_row[b] = 5'd0;
			bank_wd[b]  = 14'd0;
		end
		// Priority encoder: step basso vince in caso di collisione di lane (raro).
		if (wr0) begin bank_we[ln0] = 1'b1; bank_row[ln0] = rw0_a; bank_wd[ln0] = wd0; end
		if (wr1 && !(wr0 && ln0==ln1)) begin bank_we[ln1] = 1'b1; bank_row[ln1] = rw1_a; bank_wd[ln1] = wd1; end
		if (wr2 && !((wr0 && ln0==ln2) || (wr1 && ln1==ln2))) begin bank_we[ln2] = 1'b1; bank_row[ln2] = rw2_a; bank_wd[ln2] = wd2; end
		if (wr3 && !((wr0 && ln0==ln3) || (wr1 && ln1==ln3) || (wr2 && ln2==ln3))) begin bank_we[ln3] = 1'b1; bank_row[ln3] = rw3_a; bank_wd[ln3] = wd3; end
		if (wr4 && !((wr0 && ln0==ln4) || (wr1 && ln1==ln4) || (wr2 && ln2==ln4) || (wr3 && ln3==ln4))) begin bank_we[ln4] = 1'b1; bank_row[ln4] = rw4_a; bank_wd[ln4] = wd4; end
		if (wr5 && !((wr0 && ln0==ln5) || (wr1 && ln1==ln5) || (wr2 && ln2==ln5) || (wr3 && ln3==ln5) || (wr4 && ln4==ln5))) begin bank_we[ln5] = 1'b1; bank_row[ln5] = rw5_a; bank_wd[ln5] = wd5; end
		if (wr6 && !((wr0 && ln0==ln6) || (wr1 && ln1==ln6) || (wr2 && ln2==ln6) || (wr3 && ln3==ln6) || (wr4 && ln4==ln6) || (wr5 && ln5==ln6))) begin bank_we[ln6] = 1'b1; bank_row[ln6] = rw6_a; bank_wd[ln6] = wd6; end
		if (wr7 && !((wr0 && ln0==ln7) || (wr1 && ln1==ln7) || (wr2 && ln2==ln7) || (wr3 && ln3==ln7) || (wr4 && ln4==ln7) || (wr5 && ln5==ln7) || (wr6 && ln6==ln7))) begin bank_we[ln7] = 1'b1; bank_row[ln7] = rw7_a; bank_wd[ln7] = wd7; end
	end

	always @(posedge clk) begin
		if (reset) begin
			sc_state    <= SC_IDLE;
			entry_idx   <= 9'd511;
			tile_x_pf   <= 4'd0;
			rom_req     <= 1'b0;
			spr_addr    <= 11'd0;
			active_buf  <= 1'b0;
			clear_idx   <= 5'd0;
		end else begin
			case (sc_state)
				SC_IDLE: begin
					if (gated_new_line) begin
						active_buf <= ~active_buf;
						// Scan INVERSO 511→0 (verificato su HW: ordine sprite-su-sprite
						// opposto al loop MAME 0→511). Con last-write-wins, scandendo
						// 511→0 l'entry 0 scrive per ultima e vince → entry 0 sopra.
						entry_idx  <= 9'd511;
						tile_x_pf  <= 4'd0;
						clear_idx  <= 5'd0;
						sc_state   <= SC_CLEAR;
					end
				end

				// CLEAR: 8 lane in parallelo → 32 cicli per 256 pixel (LB_EMPTY,
				// pen=15 trasparente). Buffer scritto = quello NON attivo.
				SC_CLEAR: begin
					// write delle BRAM pilotato fuori dalla FSM (vedi bank_*_we/addr/data
					// + istanze spram_dp). Qui solo l'avanzamento del counter.
					if (clear_idx == 5'd31) begin
						clear_idx <= 5'd0;
						spr_addr  <= {entry_idx, 2'd0};   // primo scan = entry 511
						sc_state  <= SC_RW0;
					end else begin
						clear_idx <= clear_idx + 5'd1;
					end
				end

				// EARLY-SKIP: leggo w0 e w3 PRIMA (servono per enable/in_y). w1, w2
				// letti SOLO se lo sprite è visibile → risparmio ~3 cicli sulla
				// maggioranza degli slot (disabled o off-screen).
				SC_RW0: begin
					spr_addr <= {entry_idx, 2'd3};   // pre-emit addr w3
					sc_state <= SC_RW1;
				end
				SC_RW1: begin
					sp_w0    <= spr_data;            // w0 valido
					sc_state <= SC_RW3;
				end
				SC_RW3: begin
					sp_w3    <= spr_data;            // w3 valido
					sc_state <= SC_CHECK;
				end

				SC_CHECK: begin
					if (sp_enable && in_y && layer_en) begin
						spr_addr <= {entry_idx, 2'd1};   // ora leggo w1
						sc_state <= SC_RW2;
					end else begin
						sc_state <= SC_NEXT_E;
					end
				end

				SC_RW2: begin
					spr_addr <= {entry_idx, 2'd2};   // pre-emit addr w2
					sc_state <= SC_CHECK2;
				end

				SC_CHECK2: begin
					sp_w1    <= spr_data;            // w1 valido
					sc_state <= SC_CHECK2_W2;
				end

				SC_CHECK2_W2: begin
					sp_w2     <= spr_data;           // w2 valido
					tile_x_pf <= 4'd0;
					pf_side   <= 1'b0;
					sc_state  <= SC_ROM_REQ;
				end

				SC_ROM_REQ: begin
					// Sprite tile addr in SDRAM region (byte_offset relativo):
					// tile_code * 128 (32 word) + (pf_side ? 64 byte : 0) + eff_row * 4
					rom_addr <= ({4'd0, cur_tile, 7'd0})        // tile*128
					           + (pf_side ? 24'd64 : 24'd0)     // metà dx tile
					           + ({18'd0, eff_row, 2'd0});       // row*4
					rom_req  <= 1'b1;
					sc_state <= SC_ROM_W;
				end

				SC_ROM_W: begin
					if (rom_valid) begin
						pf_rom_data <= rom_data;
						rom_req     <= 1'b0;
						sc_state    <= SC_DECODE;
					end
				end

				// DECODE 1-CICLO: 8 pixel scritti in parallelo sui 8 bank.
				SC_DECODE: begin
					sc_state <= SC_NEXT_TX;
				end

				SC_NEXT_TX: begin
					if (pf_side == 1'b0) begin
						// Appena finito metà sx → fai metà dx dello stesso tile
						pf_side  <= 1'b1;
						sc_state <= SC_ROM_REQ;
					end else begin
						// Finito anche metà dx → passa al prossimo tile_x o entry
						pf_side <= 1'b0;
						if (tile_x_pf == sp_w - 4'd1) begin
							sc_state <= SC_NEXT_E;
						end else begin
							tile_x_pf <= tile_x_pf + 4'd1;
							sc_state  <= SC_ROM_REQ;
						end
					end
				end

				SC_NEXT_E: begin
					// Scan DECRESCENTE: 511 → 510 → ... → 0, poi DONE.
					if (entry_idx == 9'd0) begin
						sc_state <= SC_DONE;
					end else begin
						entry_idx <= entry_idx - 9'd1;
						spr_addr  <= {entry_idx - 9'd1, 2'd0};
						sc_state  <= SC_RW0;
					end
				end

				SC_DONE: begin
					if (gated_new_line) begin
						active_buf <= ~active_buf;
						entry_idx  <= 9'd511;
						tile_x_pf  <= 4'd0;
						clear_idx  <= 5'd0;
						sc_state   <= SC_CLEAR;
					end
				end

				default: sc_state <= SC_IDLE;
			endcase
		end
	end

	// ─── Write port di ogni bank: mux CLEAR vs DECODE ───────────────────────
	// CLEAR (SC_CLEAR): scrive LB_EMPTY a clear_idx su TUTTI gli 8 bank.
	// DECODE (SC_DECODE): scrive bank_wd[b] a bank_row[b] dove bank_we[b].
	// Buffer scritto = quello NON attivo (~active_buf).
	wire        clearing = (sc_state == SC_CLEAR);
	wire        decoding = (sc_state == SC_DECODE);

	wire        b_we   [0:7];
	wire [4:0]  b_wadr [0:7];
	wire [13:0] b_wdat [0:7];
	genvar gi;
	generate
		for (gi = 0; gi < 8; gi = gi + 1) begin : gen_wmux
			assign b_we[gi]   = clearing ? 1'b1 : (decoding & bank_we[gi]);
			assign b_wadr[gi] = clearing ? clear_idx : bank_row[gi];
			assign b_wdat[gi] = clearing ? LB_EMPTY  : bank_wd[gi];
		end
	endgenerate

	// ─── Read port: bank = hpos[2:0], row = hpos[7:3] ───────────────────────
	wire [2:0] rd_lane = hpos[2:0];
	wire [4:0] rd_row  = hpos[7:3];

	// 16 BRAM (8 lane x 2 buffer ping-pong). spram_dp = altsyncram M10K garantito.
	// Write sul buffer NON attivo; read da entrambi, poi mux su active_buf.
	wire [13:0] q0 [0:7];   // output buffer 0 (per lane)
	wire [13:0] q1 [0:7];   // output buffer 1 (per lane)
	generate
		for (gi = 0; gi < 8; gi = gi + 1) begin : gen_bank
			// buffer 0
			spram_dp #(.DW(14), .AW(5)) u_lb0 (
				.clk(clk),
				.we   (b_we[gi] & (active_buf == 1'b1)),   // scrivo buf0 quando attivo e' buf1
				.waddr(b_wadr[gi]), .wdata(b_wdat[gi]),
				.raddr(rd_row),     .rdata(q0[gi])
			);
			// buffer 1
			spram_dp #(.DW(14), .AW(5)) u_lb1 (
				.clk(clk),
				.we   (b_we[gi] & (active_buf == 1'b0)),   // scrivo buf1 quando attivo e' buf0
				.waddr(b_wadr[gi]), .wdata(b_wdat[gi]),
				.raddr(rd_row),     .rdata(q1[gi])
			);
		end
	endgenerate

	// read dato = buffer attivo, lane = rd_lane (mux combinatorio su q0/q1).
	wire [13:0] read_data_c = active_buf ? q1[rd_lane] : q0[rd_lane];

	// FIX glitch sprite-solo-su-CRT: la spram_dp ha read latency 2 (address_reg +
	// outdata_reg). hpos e' costante 16 clk tra due ce_pix, ma nei PRIMI 2 clk
	// dopo un tick ce_pix rd_lane e' gia' cambiato (combinatorio) mentre q riflette
	// ancora la row precedente -> read_data_c = q[row_vecchia][lane_nuova] per 2 clk
	// = pixel transitorio errato. Output combinatorio -> il transitorio entra
	// nell'RGB grezzo; il DAC analogico (campiona sub-ce_pix) lo mostra, HDMI/hsize no.
	// Fix: campiono read_data/hpos/de @ce_pix -> al tick il dato e' quello del pixel
	// precedente, STABILE da 16 clk -> nessun transitorio. Latenza netta +1 = tile HB.
	reg [13:0] read_data;
	reg        de_r;
	reg  [9:0] hpos_r;
	always @(posedge clk) if (ce_pix) begin
		read_data <= read_data_c;
		de_r      <= de;
		hpos_r    <= hpos;
	end

	wire  [3:0] read_pen   = read_data[3:0];
	wire  [1:0] read_pri   = read_data[7:6];
	wire  [5:0] read_color = read_data[13:8];

	// clip a hpos<256 + pen!=15. Gate hpos_r/de_r allineati a read_data (tutti @ce_pix).
	wire spr_active = de_r & layer_en & (hpos_r < 10'd256) & (read_pen != 4'd15);
	assign opaque    = spr_active;
	// MAME gfx_legionna_spr: color base = 64*16 = 0x400 (64 colorset × 16 pen).
	assign pen_index = spr_active ? (11'h400 + {1'b0, read_color, read_pen}) : 11'd0;
	assign pri_code  = spr_active ? read_pri : 2'd0;

endmodule
