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

// HeatedBarrel (Banpresto/Bandai 1991) - MiSTer core
// Porting base: Darius MiSTer core. MiSTer Template by Sorgelig.

module emu
(
	input         CLK_50M,
	input         RESET,
	inout  [48:0] HPS_BUS,
	output        CLK_VIDEO,
	output        CE_PIXEL,
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,
	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER,
	output        VGA_DISABLE,
	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,
`ifdef MISTER_FB_PALETTE
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,
	output  [1:0] BUTTONS,

	input         CLK_AUDIO,
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,
	output  [1:0] AUDIO_MIX,

	inout   [3:0] ADC_BUS,

	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

///////// Unused ports /////////
assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// DDRAM ora pilotato da u_ddram_spr (sprite ROM in DDR3). CLK = clk_sys.
assign DDRAM_CLK = clk_sys;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
// Pause: toggle on rising edge of joy[12] (standard MiSTer pause bit)
reg pause_toggle;
reg joy_pause_prev;
always @(posedge clk_sys) begin
	if (reset) begin
		pause_toggle <= 1'b0;
		joy_pause_prev <= 1'b0;
	end else begin
		joy_pause_prev <= joy0[12] | joy1[12];
		if ((joy0[12] | joy1[12]) && !joy_pause_prev)
			pause_toggle <= ~pause_toggle;
	end
end
wire pause = pause_toggle | status[17];  // pad OR OSD
assign HDMI_FREEZE = 1'b0;  // overlay pause renderizzato real-time, no freeze scaler
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;  // signed audio
wire signed [15:0] game_audio_l, game_audio_r;
assign AUDIO_L = game_audio_l;
assign AUDIO_R = game_audio_r;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[127:126];

// Volumi audio OSD (Q4.4: 16 = 100%, 32 = 200%, 0 = mute)
wire [2:0] osd_fm_vol  = status[88:86];
wire [2:0] osd_oki_vol = status[91:89];
reg [5:0] fm_vol_q44, oki_vol_q44;
always @(*) begin
	case (osd_fm_vol)
		3'd0: fm_vol_q44 = 6'd16;   // 100%
		3'd1: fm_vol_q44 = 6'd2;    // 12%
		3'd2: fm_vol_q44 = 6'd4;    // 25%
		3'd3: fm_vol_q44 = 6'd8;    // 50%
		3'd4: fm_vol_q44 = 6'd12;   // 75%
		3'd5: fm_vol_q44 = 6'd24;   // 150%
		3'd6: fm_vol_q44 = 6'd32;   // 200%
		3'd7: fm_vol_q44 = 6'd0;    // mute
	endcase
	case (osd_oki_vol)
		3'd0: oki_vol_q44 = 6'd16;
		3'd1: oki_vol_q44 = 6'd2;
		3'd2: oki_vol_q44 = 6'd4;
		3'd3: oki_vol_q44 = 6'd8;
		3'd4: oki_vol_q44 = 6'd12;
		3'd5: oki_vol_q44 = 6'd24;
		3'd6: oki_vol_q44 = 6'd32;
		3'd7: oki_vol_q44 = 6'd0;
	endcase
end

// OSD layer offsets: 6-bit signed 2's complement, default 0 on reset.
// Shift fisso aggiunto per match MAME 1:1 (misurato HW 2026-05-14):
//   BG  X+0  Y+16
//   FG  X+0  Y+16
//   SPR X+0  Y+0
//   TXT X-1  Y+16
// L'OSD resta editabile ±32 attorno al valore corretto.
// Offset per-layer HARDCODATI = match pixel-perfect MAME (trovati via OSD, ora
// fissi; menu P5 rimosso). Valori finali assoluti:
//   BG  X=0  Y=+16   MG X=0 Y=+16   FG X=0 Y=+16
//   SPR X=0  Y=-16   TXT X=-1 Y=+16
// ── OFFSET PER-LAYER esposti nell'OSD (calibrazione 1:1 con MAME) ──────────────
// Azzerati gli hardcoded di Legionnaire/224. Ogni offset = 6-bit signed (-32..+31)
// da status[], sign-extended a 10-bit. Calibrare uno a uno confrontando con MAME,
// Offset per-layer HARDCODATI (calibrati su HW confronto MAME, pagina OSD Calibration rimossa).
wire signed [9:0] osd_bg_xoff  =  10'sd0;
wire signed [9:0] osd_bg_yoff  =  10'sd0;
wire signed [9:0] osd_mg_xoff  =  10'sd0;
wire signed [9:0] osd_mg_yoff  =  10'sd0;
wire signed [9:0] osd_fg_xoff  =  10'sd0;
wire signed [9:0] osd_fg_yoff  =  10'sd0;
wire signed [9:0] osd_spr_xoff = -10'sd1;
wire signed [9:0] osd_spr_yoff =  10'sd0;
wire signed [9:0] osd_txt_xoff = -10'sd1;
wire signed [9:0] osd_txt_yoff =  10'sd0;

`include "build_id.v"
// ─── MAPPA BIT OSD status[] — bit→significato (NO sovrapposizioni, verificato) ───
//  Per hardcodare un valore trovato nell'OSD: leggi il bit qui, poi sostituisci
//  il segnale RTL corrispondente (colonna "segnale RTL") con un literal.
//  Offset = signed 6-bit: 0..31 = +0..+31, valori "-32..-1" = bit5=1 → negativi.
//
//  bit(s)     opzione                  pag  valori                              segnale RTL
//  [0]        Reset / Reset+OSD        -    momentary (condiviso, OK)           reset_cause
//  [7:5]      Scale                    P1   0=Norm,1=Vint,2=Narrow,3=Wide,4=HV  .SCALE
//  [18]       Clean Pause              P1   Off=0/On=1                          .clean
//  [19]       Refresh Rate             P1   59.4Hz=0/60Hz=1                     .mode_60hz
//  [29]       Text layer enable        P4   On=0/Off=1                          ~status[29]
//  [30]       BG layer enable          P4   On=0/Off=1                          ~status[30]
//  [31]       MG layer enable          P4   On=0/Off=1                          ~status[31]
//  [32]       FG layer enable          P4   On=0/Off=1                          ~status[32]
//  [33]       Sprite layer enable      P4   On=0/Off=1                          ~status[33]
//  [88:86]    FM (YM2151) volume       P3   100/12/25/50/75/150/200%/Mute       osd_fm_vol
//  [91:89]    OKI ADPCM volume         P3   100/12/25/50/75/150/200%/Mute       osd_oki_vol
//  [101]      CRT Adjust On/Off        P1   Off/On                              crt_on
//  [100:96]   CRT H-Size               P1   0/+1..+15/-16..-1 (signed)          hsize_s
//  [85:79]    CRT H-Position           P1   0/+1..+48/-48..-1                    hpos_off
//  [78:74]    CRT V-Shift              P1   0/+1..+15/-16..-1 (signed)           vshift_off
//  [127:126]  Aspect ratio             P1   Original/Full/[ARC1]/[ARC2]         ar
//  NB: gli offset per-layer (ex P5) sono HARDCODATI (osd_*_xoff/yoff costanti,
//      match pixel-perfect MAME). I loro vecchi bit sono ora LIBERI.
//  NB: status[17] = PAUSE (riga ~162). NON usarlo per opzioni salvabili: un cfg
//      con bit17=1 mette il 68k in halt al boot -> il gioco non parte.
//  [22:20]    Start Stage (diag)       P4   0=Normal,1..5=Stage N                osd_start_stage
//  LIBERI: [4:1] [8:16] [23:28] [34:85] [92:119] — usabili per nuove opzioni.

localparam CONF_STR = {
	"HeatedBarrel;;",
	"-;",
	"P1,Video;",
	"P1O[127:126],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[7:5],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer,HV-Integer;",
	"P1O[19],Refresh Rate,Original 59.4Hz,60Hz;",
	"P1O[18],Clean Pause,Off,On;",
	"P1O[101],CRT Adjust,Off,On;",
	"H1P1O[100:96],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[85:79],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[78:74],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	"DIP;",
	"-;",
	"P3,Audio;",
	"P3O[88:86],FM (YM2151) volume,100%,12%,25%,50%,75%,150%,200%,Mute;",
	"P3O[91:89],OKI ADPCM volume,100%,12%,25%,50%,75%,150%,200%,Mute;",
	"-;",
	"P4,Debug;",
	"P4O[30],BG layer,On,Off;",
	"P4O[31],MG layer,On,Off;",
	"P4O[32],FG layer,On,Off;",
	"P4O[33],Sprite layer,On,Off;",
	"P4O[29],Text layer,On,Off;",
	"P4-;",
	"P4O[22:20],Start Stage,Normal,Stage 1,Stage 2,Stage 3,Stage 4,Stage 5;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"-;",
	// J1: bit 4=Fire(A), 5=Roll(B), 6=Dynamite(X/C), 7,8,9=unused, 10=Start1, 11=Coin1, 12=Pause
	// 13=Start2, 14=Coin2 (MiSTer arcade convention fissa)
	"J1,Fire,Roll,Dynamite,-,-,-,Start,Coin,Pause,Start 2P,Coin 2P;",
	"jn,A,B,X,,,,Start,R,L,Select,;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire [15:0] joy0, joy1, joy2, joy3;
wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [15:0] ioctl_dout;   // 16-bit: WIDE=1
wire        ioctl_wait;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_menumask({14'd0, ~status[101], 1'b0}),  // H1 group (CRT Adjust) shown only when On
	.ps2_key(ps2_key),
	.joystick_0(joy0),
	.joystick_1(joy1),
	.joystick_2(joy2),
	.joystick_3(joy3),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait)
);

// --- Joystick input mapping ---
// MAME P1_P2 port ($E0002): active low.
// Low byte P1 / high byte P2: bit0=U, bit1=D, bit2=L, bit3=R, bit4=Btn1, bit5=Btn2.
// MiSTer joy bits: joy[0]=R, joy[1]=L, joy[2]=D, joy[3]=U, joy[4]=A, joy[5]=B.
// MAME PLAYERS12 (legionna.cpp:439-444): bit[0]=UP, [1]=DOWN, [2]=LEFT, [3]=RIGHT,
// [4]=BTN1, [5]=BTN2, [6]=BTN3, [7]=unused (active low).
// Mappa direzioni: game bit0=UP=joy0[3], bit1=DOWN=joy0[2], bit2=LEFT=joy0[1],
// bit3=RIGHT=joy0[0]. (Prima erano in ordine joy0[3..0] = U,D,L,R sui bit 3..0
// del game -> rotazione 90gradi su->destra ecc. Corretto invertendo i 4 bit.)
// bit6/7 = UNKNOWN in MAME PLAYERS12 heatbrl -> tied 1 (no input fantasma; joy[6] attivo
// metterebbe bit6=0 = "premuto" -> trigger display DIP al power-on).
wire [7:0] p1_input = {2'b11, ~joy0[5], ~joy0[4], ~joy0[0], ~joy0[1], ~joy0[2], ~joy0[3]};
wire [7:0] p2_input = {2'b11, ~joy1[5], ~joy1[4], ~joy1[0], ~joy1[1], ~joy1[2], ~joy1[3]};
wire [15:0] p1_p2_input = {p2_input, p1_input};
// Heated Barrel: 4 player. PLAYERS34 ($100748) = P3 (low) + P4 (high).
// bit0-3=U/D/L/R, bit4=B1, bit5=B2, bit6/7=UNKNOWN (tied 1, NON input). active-low.
wire [7:0] p3_input = {2'b11, ~joy2[5], ~joy2[4], ~joy2[0], ~joy2[1], ~joy2[2], ~joy2[3]};
wire [7:0] p4_input = {2'b11, ~joy3[5], ~joy3[4], ~joy3[0], ~joy3[1], ~joy3[2], ~joy3[3]};
wire [15:0] p3_p4_input = {p4_input, p3_input};

// Heated Barrel SYSTEM port ($10074C) — MAME heatbrl (4 player):
//   bit0=START1, bit1=START2, bit3=COIN3, bit4=START3, bit5=START4, bit7=COIN4 (active LOW)
wire [15:0] system_input16 = {8'hFF,                          // [15:8] tutti 1
                              ~joy3[11],                       // bit7 = COIN4
                              1'b1,                            // bit6 = unused
                              ~joy3[10],                       // bit5 = START4
                              ~joy2[10],                       // bit4 = START3
                              ~joy2[11],                       // bit3 = COIN3
                              1'b1,                            // bit2 = unused
                              ~joy1[10], ~joy0[10]};           // bit1=START2 bit0=START1

// Seibu coin input (ACTIVE_HIGH per SEIBU_COIN_INPUTS macro): bit0=COIN1, bit1=COIN2.
// Letto dal Z80 a 0x4013 → coin_r → soundlatch sub2main → main 68k legge 0xA0004.
// In sim mode: auto-press coin1 every ~half second (sim accel)
`ifdef MISTER_SIM
reg [27:0] sim_coin_cnt = 0;
reg sim_coin_press = 1'b0;
always @(posedge clk_sys) begin
	sim_coin_cnt <= sim_coin_cnt + 1;
	if (sim_coin_cnt == 28'd50_000_000) sim_coin_press <= 1'b1;   // ~1s @96MHz
	if (sim_coin_cnt == 28'd60_000_000) sim_coin_press <= 1'b0;   // release
	if (sim_coin_cnt == 28'd70_000_000) sim_coin_press <= 1'b1;   // press start
end
wire [7:0] coin_input = {6'd0, sim_coin_press, sim_coin_press};
`else
wire [7:0] coin_input = {6'd0, joy1[11], joy0[11]};
`endif

// DIP switches — loaded from MRA via ioctl (index 254)
// Active-LOW: default "FF,FF" = all OFF = all 1s
reg [15:0] dip_sw = 16'hFFFF;
always @(posedge clk_sys)
	if (ioctl_wr && (ioctl_index == 16'd254) && !ioctl_addr[26:1])
		dip_sw <= ioctl_dout;

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
wire pll_locked /*verilator public_flat_rd*/;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.locked(pll_locked)
);

// Game reset: 17-bit hold (~1.4ms @ 96MHz). Bilancio: troppo corto e
// SDRAM bridge non stabilizza; troppo lungo e vblank IRQ4 si arma prima
// che ROM init code finisca → crash subito dopo reset_release.
// Download stretch: dopo che ioctl_download cade, prolunga reset_cause per N cicli.
// Evita rilascio reset CPU prima che SDRAM bridge abbia finito di propagare gli ultimi
// write delle ROM (download multi-bank: 4 banchi per ogni word del main CPU ROM).
reg [23:0] dl_stretch_cnt = 24'd0;
reg        ioctl_download_prev = 1'b0;
always @(posedge clk_sys) begin
	ioctl_download_prev <= ioctl_download;
	if (ioctl_download) dl_stretch_cnt <= 24'hFFFFFF;
	else if (dl_stretch_cnt != 24'd0) dl_stretch_cnt <= dl_stretch_cnt - 24'd1;
end
wire dl_stretch_active = (dl_stretch_cnt != 24'd0);

wire reset_cause /*verilator public_flat_rd*/ = RESET | status[0] | buttons[1] | ~pll_locked | ioctl_download | dl_stretch_active;
reg [23:0] reset_hold_cnt /*verilator public_flat_rd*/ = 24'hFFFFFF;
always @(posedge clk_sys) begin
	if (reset_cause)                  reset_hold_cnt <= 24'hFFFFFF;
	else if (reset_hold_cnt != 24'd0) reset_hold_cnt <= reset_hold_cnt - 24'd1;
end
wire reset /*verilator public_flat_rd*/ = (reset_hold_cnt != 24'd0);
// Bridge reset: ONLY pll_locked — bridge must run during download, before RESET drops
wire bridge_reset = ~pll_locked;
// Video reset: ONLY pll_locked — CRT needs sync always, even during RESET and download
wire video_reset = ~pll_locked;

///////////////////////   SDRAM   ///////////////////////////////

// Genesis 4-port SDRAM controller (Sorgelig + donor bridge)
// Port 0: graphics ROM + download
// Port 1: main 68000 ROM
// Port 2: temporarily unused donor ROM path
// Port 3: audio/sample ROM path

wire [24:1] sd_addr0, sd_addr1, sd_addr2, sd_addr3;
wire [15:0] sd_din0, sd_din1, sd_din2, sd_din3;
wire        sd_wrl0, sd_wrh0, sd_wrl1, sd_wrh1, sd_wrl2, sd_wrh2, sd_wrl3, sd_wrh3;
wire        sd_req0, sd_req1, sd_req2, sd_req3;
wire        sd_ack0, sd_ack1, sd_ack2, sd_ack3;
wire [15:0] sd_dout0, sd_dout1, sd_dout2, sd_dout3;
wire        sdram_ready;

// OKI ADPCM ROM bridge ↔ jt6295 (via main_top)
wire [17:0] oki_rom_addr;
wire  [7:0] oki_rom_data;
wire        oki_rom_ok;

sdram sdram_ctrl
(
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),

	.init(~pll_locked),
	.clk(clk_sys),
	.prio_mode(2'd0),
	.ready(sdram_ready),

	.addr0(sd_addr0), .wrl0(sd_wrl0), .wrh0(sd_wrh0),
	.din0(sd_din0), .dout0(sd_dout0), .req0(sd_req0), .ack0(sd_ack0),

	.addr1(sd_addr1), .wrl1(sd_wrl1), .wrh1(sd_wrh1),
	.din1(sd_din1), .dout1(sd_dout1), .req1(sd_req1), .ack1(sd_ack1),

	.addr2(sd_addr2), .wrl2(sd_wrl2), .wrh2(sd_wrh2),
	.din2(sd_din2), .dout2(sd_dout2), .req2(sd_req2), .ack2(sd_ack2),

	.addr3(sd_addr3), .wrl3(sd_wrl3), .wrh3(sd_wrh3),
	.din3(sd_din3), .dout3(sd_dout3), .req3(sd_req3), .ack3(sd_ack3)
);

///////////////////////   BRIDGE   ///////////////////////////////

// Bridge between game logic (level protocol) and Genesis SDRAM (toggle protocol)
wire [23:0] game_tile_addr, game_main_addr /*verilator public_flat_rd*/, game_sub_addr;
wire        game_tile_req, game_main_req /*verilator public_flat_rd*/, game_sub_req;
wire  [2:0] game_tile_kind;     // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
wire [31:0] game_tile_data;
wire        game_tile_valid;
wire [15:0] game_main_data /*verilator public_flat_rd*/, game_sub_data;
// Audio Z80 ROM removed from SDRAM — will use BRAM when audio implemented
wire        game_main_ready /*verilator public_flat_rd*/, game_sub_ready;

// ROM instruction cache — between game and SDRAM bridge
wire [23:0] bridge_main_addr /*verilator public_flat_rd*/, bridge_sub_addr;
wire        bridge_main_req  /*verilator public_flat_rd*/, bridge_sub_req;
wire [15:0] bridge_main_data /*verilator public_flat_rd*/, bridge_sub_data;
wire        bridge_main_ready /*verilator public_flat_rd*/, bridge_sub_ready;

rom_cache #(.CACHE_BITS(9)) u_main_cache (
	.clk(clk_sys), .reset(reset),
	.cpu_addr(game_main_addr), .cpu_req(game_main_req),
	.cpu_data(game_main_data), .cpu_ready(game_main_ready),
	.sdram_addr(bridge_main_addr), .sdram_req(bridge_main_req),
	.sdram_data(bridge_main_data), .sdram_ready(bridge_main_ready)
);

rom_cache #(.CACHE_BITS(9)) u_sub_cache (
	.clk(clk_sys), .reset(reset),
	.cpu_addr(game_sub_addr), .cpu_req(game_sub_req),
	.cpu_data(game_sub_data), .cpu_ready(game_sub_ready),
	.sdram_addr(bridge_sub_addr), .sdram_req(bridge_sub_req),
	.sdram_data(bridge_sub_data), .sdram_ready(bridge_sub_ready)
);

sdram_bridge bridge
(
	.clk(clk_sys),
	.reset(bridge_reset),
	.sdram_ready(sdram_ready),

	// HPS download
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	// Game: Tile ROM (32-bit)
	.tile_byte_addr(game_tile_addr),
	.tile_req(game_tile_req),
	.gfx_kind(game_tile_kind),
	.tile_data(game_tile_data),
	.tile_valid(game_tile_valid),

	// Game: Main CPU ROM (16-bit)
	.main_byte_addr(bridge_main_addr),
	.main_req(bridge_main_req),
	.main_data(bridge_main_data),
	.main_ready(bridge_main_ready),

	// Game: temporarily unused donor ROM port
	.sub_byte_addr(bridge_sub_addr),
	.sub_req(bridge_sub_req),
	.sub_data(bridge_sub_data),
	.sub_ready(bridge_sub_ready),

	// OKI ADPCM ROM (port 3)
	.oki_byte_addr(oki_rom_addr),
	.oki_data(oki_rom_data),
	.oki_ok(oki_rom_ok),

	// SDRAM ports
	.sdram_addr0(sd_addr0), .sdram_din0(sd_din0),
	.sdram_wrl0(sd_wrl0), .sdram_wrh0(sd_wrh0),
	.sdram_req0(sd_req0), .sdram_ack0(sd_ack0), .sdram_dout0(sd_dout0),

	.sdram_addr1(sd_addr1), .sdram_din1(sd_din1),
	.sdram_wrl1(sd_wrl1), .sdram_wrh1(sd_wrh1),
	.sdram_req1(sd_req1), .sdram_ack1(sd_ack1), .sdram_dout1(sd_dout1),

	.sdram_addr2(sd_addr2), .sdram_din2(sd_din2),
	.sdram_wrl2(sd_wrl2), .sdram_wrh2(sd_wrh2),
	.sdram_req2(sd_req2), .sdram_ack2(sd_ack2), .sdram_dout2(sd_dout2),

	.sdram_addr3(sd_addr3), .sdram_din3(sd_din3),
	.sdram_wrl3(sd_wrl3), .sdram_wrh3(sd_wrh3),
	.sdram_req3(sd_req3), .sdram_ack3(sd_ack3), .sdram_dout3(sd_dout3)
);

///////////////////////   GAME   ///////////////////////////////

wire [9:0]  render_x;
wire [8:0]  render_y;
wire [15:0] map_xscroll_l0, map_xscroll_l1;
wire [15:0] map_ctrl_l0;
wire [15:0] map_yscroll_l0, map_yscroll_l1;
wire [15:0] map_xscroll_mg, map_yscroll_mg;

HeatedBarrel_main game
(
	.clk(clk_sys),
	.reset(reset),
	.pause(pause),
	.clk_sel(3'd2),              // Main CPU 10 MHz (target reale Gundam)
	.sub_clk_sel(3'd0),          // donor Darius path kept until the sub CPU is removed
	.z80_clk_sel(2'd0),          // Sound CPU default (3.58 MHz)
	.p1_input(p1_input),
	.p2_input(p2_input),
	.pin34_input(p3_p4_input),
	.system_input(system_input16),
	.dsw_input(dip_sw),
	.osd_start_stage(status[22:20]),

	// SDRAM ROM (via bridge)
	.main_rom_rdata(game_main_data),
	.main_rom_ready(game_main_ready),
	.sub_rom_rdata(game_sub_data),
	.sub_rom_ready(game_sub_ready),
	.tilerom_data(game_tile_data),
	.tilerom_valid(game_tile_valid),

	.main_rom_addr(game_main_addr),
	.main_rom_req(game_main_req),
	.sub_rom_addr(game_sub_addr),
	.sub_rom_req(game_sub_req),
	// tilerom_* main_top tied off: arbiter Gundam pilota il bridge direttamente
	.tilerom_addr(),
	.tilerom_req(),
	.tilerom_kind(),

	// Audio ROM download (ioctl → BRAM)
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),

	// Video
	.render_x(render_x),
	.render_y(render_y),
	.vblank_in(VBlank),
	// Scroll esposti dal CRTC Seibu
	.xscroll_l0(map_xscroll_l0),
	.xscroll_l1(map_xscroll_l1),
	.yscroll_l0(map_yscroll_l0),
	.yscroll_l1(map_yscroll_l1),
	.xscroll_mg(map_xscroll_mg),
	.yscroll_mg(map_yscroll_mg),
	.ctrl_l0(map_ctrl_l0),
	// Palette read port (per video pipeline)
	.pal_b_addr(pal_b_addr),
	.pal_b_r(pal_b_r),
	.pal_b_g(pal_b_g),
	.pal_b_b(pal_b_b),
	// Text VRAM read (renderer text legge tile word)
	.text_vram_addr(text_vram_addr),
	.text_vram_data(text_vram_data),
	// BG VRAM read (renderer BG legge tile word)
	.bg_vram_addr(bg_vram_addr),
	.bg_vram_data(bg_vram_data),
	// MG VRAM read
	.mg_vram_addr(mg_vram_addr),
	.mg_vram_data(mg_vram_data),
	// FG VRAM read
	.fg_vram_addr(fg_vram_addr),
	.fg_vram_data(fg_vram_data),
	// Sprite VRAM read (per scanner sprite)
	.spr_vram_addr(spr_vram_addr),
	.spr_vram_data(spr_vram_data),
	// gfx_bank (per MG)
	.gfx_bank(gfx_bank),
	// Coin input (HW button → Z80 → soundlatch → main 0xA0004)
	.coin_input(coin_input),
	// OKI ADPCM ROM bridge (port 3)
	.oki_rom_addr(oki_rom_addr),
	.oki_rom_data(oki_rom_data),
	.oki_rom_ok(oki_rom_ok),
	// Volumi OSD
	.fm_vol_q44(fm_vol_q44),
	.oki_vol_q44(oki_vol_q44),
	// Audio
	.audio_l(game_audio_l),
	.audio_r(game_audio_r)
);

// Palette read-side: indirizzo deciso dal pixel pipeline (priorità tra layer)
wire [10:0] pal_b_addr;
wire [7:0]  pal_b_r, pal_b_g, pal_b_b;

// Text VRAM read wires
wire [10:0] text_vram_addr;
wire [15:0] text_vram_data;

///////////////////////   VIDEO   ///////////////////////////////

// Heated Barrel timing single-screen 256x256 @ 59.4 Hz (original) / 60.1 Hz.
// Pixel clock 6 MHz = clk_sys/16. HTotal=384, VTotal=263 (59.4) o 260 (60Hz).
wire ce_pix;
wire HBlank, VBlank, HSync, VSync, video_de;
wire [9:0] timing_hpos;
wire [9:0] timing_vpos;

HeatedBarrel_video_timing u_video_timing (
	.clk        (clk_sys),
	.reset      (video_reset),
	.mode_60hz  (status[19]),
	.ce_pix     (ce_pix),
	.hpos       (timing_hpos),
	.vpos       (timing_vpos),
	.active_x   (render_x),
	.active_y   (render_y),
	.hblank     (HBlank),
	.vblank     (VBlank),
	.hsync      (HSync),
	.vsync      (VSync),
	.de         (video_de)
);

// ── Flip screen (CRTC reg 0x1A bit 0) ───────────────────────────────────────
// MAME: BIT(reg_1a, 0) → flip_screen. Arcade reale = monitor CRT capovolto;
// game scrive in VRAM convinto che lo schermo sia ruotato 180° → noi vediamo
// flippato finchè non invertiamo.
//
// Strategia:
//  - X flip: il read-side dei line_buffer (dentro tile_layer/text_renderer)
//    legge linebuf[hpos]. Sostituisco hpos con (319-hpos) → mostra mirror H.
//  - Y flip: la prefetch durante display vpos=N riempie il buffer per il
//    display vpos=N+1. In flip ON serve che mostri riga ROM (V_VISIBLE-1-(N+1))
//    = (V_VISIBLE-2-N). Sostituisco vpos del prefetch con (V_VISIBLE-2-vpos)
//    quando flip on, così target_y = V_VISIBLE-2-vpos + 1 = V_VISIBLE-1-vpos.
wire        flip_screen = map_ctrl_l0[5];
// Coordinate LOGICHE display (0..319, 0..223), shiftate dal timing CRTC raw.
// Il timing CRTC ora è SYNC→BP→VISIBLE→FP, quindi VISIBLE inizia a hpos=48,
// vpos=30. I renderer devono vedere coordinate "del gioco" 0..319/0..223.
localparam [9:0] H_VIS_START_TOP = 10'd96;   // = H_SYNC + H_BP del video_timing (HBP 64)
localparam [8:0] V_VIS_START_TOP = 9'd4;     // = V_SYNC(3) + V_BP(1) per 256x256 (era 30 per 224)
wire [9:0]  hpos_logic = timing_hpos - H_VIS_START_TOP;
wire [8:0]  vpos_logic = timing_vpos[8:0] - V_VIS_START_TOP;
// Tile_layer prefetch: con flip serve vpos_for_pf = 223 - vpos_logic (range 0..223,
// come il text a vpos_for_text). Con 222 (vecchio), a vpos_logic=223 -> vpos_for_pf =
// 222-223 = -1 = 511 (9-bit unsigned) >= V_VISIBLE -> vpos_visible=0 -> NESSUN toggle di
// active_buf su quella riga -> 223 toggle/frame (DISPARI) -> il double-buffer del BG si
// INVERTE ogni frame -> l'intero BG alterna buffer -> FLICKER (visibile nelle 2 righe
// basse, dove i due buffer differiscono, in stage 4 con flip attivo dalla schermata-mappa).
// Con 223: range 0..223, tutti < 224 -> 224 toggle (PARI) -> active_buf stabile -> no flicker.
wire [8:0]  vpos_for_pf   = flip_screen ? (9'd255 - vpos_logic) : vpos_logic;  // 256x256
// Text renderer legge tile direttamente da eff_y = vpos (no prefetch ahead).
wire [8:0]  vpos_for_text = flip_screen ? (9'd255 - vpos_logic) : vpos_logic;  // 256x256
// Read path (X): tile_layer linebuf[hpos]. Per text è eff_x = hpos+scroll.
wire [9:0]  hpos_for_read = flip_screen ? (10'd255 - hpos_logic) : hpos_logic;

// ── Text layer renderer (8x8, 4bpp, 64x32 grid) ─────────────────────────────
// Heated Barrel: "char" region PROPRIA 128KB (barrel_6 + barrel_5 interleaved),
// NON dentro user1 come Legionnaire. MRA la carica a ioctl 0x080000..0x0A0000.
// BRAM text = 64KB (32Kw): per ora mappa i primi 64KB (0x080000..0x08FFFF).
wire        text_opaque;
wire [10:0] text_pen;

wire        text_rom_dl_wr =
	ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
	(ioctl_addr >= 27'h080000) && (ioctl_addr < 27'h0A0000);   // char 128KB
// 17-bit address relativo alla region char (= ioctl_addr - 0x080000).
wire [16:0] text_rom_dl_offset = ioctl_addr[16:0];
// Blood Bros Text: no scroll (MAME bloodbro.cpp non chiama set_scroll su tx_tilemap)
HeatedBarrel_text_renderer u_text (
	.clk          (clk_sys),
	.reset        (reset),
	.ce_pix       (ce_pix),
	.hpos         (hpos_for_read),
	.vpos         (vpos_for_text),
	.de           (video_de),
	.layer_en     (map_ctrl_l0[3] & ~status[29]),
	.scroll_x     (16'd0),
	.scroll_y     (16'd0),
	.xoff         (osd_txt_xoff),
	.yoff         (osd_txt_yoff),
	.vram_addr    (text_vram_addr),
	.vram_data    (text_vram_data),
	.rom_dl_wr    (text_rom_dl_wr),
	.rom_dl_addr  (text_rom_dl_offset),
	.rom_dl_data  (ioctl_dout),
	.opaque       (text_opaque),
	.pen_index    (text_pen)
);

// ── BG/MG/FG layer renderer (16x16, 4bpp, 32x32) — fetch SDRAM via arbiter ──
wire        bg_opaque, mg_opaque, fg_opaque;
wire [10:0] bg_pen, mg_pen, fg_pen;
wire [10:0] bg_vram_addr, mg_vram_addr, fg_vram_addr;
wire [15:0] bg_vram_data, mg_vram_data, fg_vram_data;
wire [15:0] gfx_bank;

// new_line pulse: hpos passa da H_TOTAL-1 a 0
reg [9:0] hpos_prev;
always @(posedge clk_sys) if (ce_pix) hpos_prev <= timing_hpos;
wire layer_new_line = ce_pix && (timing_hpos == 10'd0) && (hpos_prev != 10'd0);

// HeatedBarrel CRTC scroll passthrough (MAME legionna_v.cpp:34-49 tile_scroll_w):
//   scroll_ram[0/1] → BG (m_background_layer)
//   scroll_ram[2/3] → MG (m_midground_layer)
//   scroll_ram[4/5] → FG (m_foreground_layer)
// Nel nostro CRTC: xscroll_l0 = ram[0/1], xscroll_mg = ram[2/3], xscroll_l1 = ram[4/5].
wire [15:0] bg_scroll_x = map_xscroll_l0;
wire [15:0] bg_scroll_y = map_yscroll_l0;
wire [15:0] mg_scroll_x = map_xscroll_mg;
wire [15:0] mg_scroll_y = map_yscroll_mg;
wire [15:0] fg_scroll_x = map_xscroll_l1;
wire [15:0] fg_scroll_y = map_yscroll_l1;

// Arbiter wires
wire        arb_bg_req,  arb_mg_req,  arb_fg_req;
wire [23:0] arb_bg_addr, arb_mg_addr, arb_fg_addr;
wire [31:0] arb_bg_data, arb_mg_data, arb_fg_data;
wire        arb_bg_valid, arb_mg_valid, arb_fg_valid;

// HeatedBarrel BG: tilemap 32x32 (legionna_v.cpp:204). MAP_HEIGHT_4=0.
// MAME GFXDECODE "back" color base = 0 (gfx_legionna riga 1168), 32 colorset.
// HAS_TRANSP=1 perché m_background_layer->set_transparent_pen(15) in
// legionna_v.cpp:216 (pen 15 = trasparente, NON opaco come Blood Bros).
HeatedBarrel_tile_layer #(
	.COLOR_BASE   (11'h000),
	.HAS_TRANSP   (1),
	.HAS_GFX_BANK (1),               // heatbrl: back_gfx_bank @$100470 bit14 -> +0x1000
	.TILE_KIND    (3'd0),
	.MAP_HEIGHT_4 (0)
) u_bg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[0] & ~status[30]),
	.new_line(layer_new_line),
	.pen_order_ovr(1'b0),
	.scroll_x(bg_scroll_x), .scroll_y(bg_scroll_y),
	.xoff(osd_bg_xoff), .yoff(osd_bg_yoff),
	.gfx_bank(gfx_bank),
	.vram_addr(bg_vram_addr), .vram_data(bg_vram_data),
	.rom_req(arb_bg_req), .rom_addr(arb_bg_addr),
	.rom_data(arb_bg_data), .rom_valid(arb_bg_valid),
	.opaque(bg_opaque), .pen_index(bg_pen)
);

// Heated Barrel MG: 16x16, 4bpp, 32x32. MAME gfx_heatbrl: "mid" ha ROM INDIPENDENTE
// (get_mid_tile_info_split): tile = (vram & 0xfff) [offset 0, NO +0x1000],
// color = (vram >> 12) & 0xf. GFXDECODE "mid" color base = 16*16 = 0x100.
HeatedBarrel_tile_layer #(
	.COLOR_BASE    (11'h100),
	.HAS_TRANSP    (1),
	.HAS_GFX_BANK  (0),
	.TILE_KIND     (3'd1),
	.TILE_CODE_OFS (13'h0000)
) u_mg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[1] & ~status[31]),
	.new_line(layer_new_line),
	.pen_order_ovr(1'b0),
	.scroll_x(mg_scroll_x), .scroll_y(mg_scroll_y),
	.xoff(osd_mg_xoff), .yoff(osd_mg_yoff),
	.gfx_bank(16'd0),
	.vram_addr(mg_vram_addr), .vram_data(mg_vram_data),
	.rom_req(arb_mg_req), .rom_addr(arb_mg_addr),
	.rom_data(arb_mg_data), .rom_valid(arb_mg_valid),
	.opaque(mg_opaque), .pen_index(mg_pen)
);

// HeatedBarrel FG: tilemap 32x32 (legionna_v.cpp:213).
// get_fore_tile_info: tile = (vram & 0xfff), color = (vram >> 12). No +0x1000.
// MAME GFXDECODE "fore" color base = 32*16 = 0x200, 16 colorset.
// ROM "fore" è user1 1a metà (64KB), descrambled — fetch via sdram_bridge
// gfx_kind=3'd2 con address bitswap.
HeatedBarrel_tile_layer #(
	.COLOR_BASE    (11'h200),
	.HAS_TRANSP    (1),
	.HAS_GFX_BANK  (0),
	.TILE_KIND     (3'd2),
	.MAP_HEIGHT_4  (0),
	.TILE_CODE_OFS (13'h0000),
	.PEN_ORDER     (0)            // FG = DCBA come BG/MG (verificato HW 2026-06-30: logo title pen corretti)
) u_fg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[2] & ~status[32]),  // HeatedBarrel: bit 2 = FG (MAME legionna_v.cpp:344)
	.new_line(layer_new_line),
	.pen_order_ovr(1'b0),
	.scroll_x(fg_scroll_x), .scroll_y(fg_scroll_y),
	.xoff(osd_fg_xoff), .yoff(osd_fg_yoff),
	.gfx_bank(16'd0),
	.vram_addr(fg_vram_addr), .vram_data(fg_vram_data),
	.rom_req(arb_fg_req), .rom_addr(arb_fg_addr),
	.rom_data(arb_fg_data), .rom_valid(arb_fg_valid),
	.opaque(fg_opaque), .pen_index(fg_pen)
);

// ── Sprite renderer (SEI252 / RISE, HeatedBarrel family) ─────────────────────
wire        spr_opaque;
wire [10:0] spr_pen;
wire  [1:0] spr_pri;
wire [10:0] spr_vram_addr_int;  // Sprite renderer: 512 entry × 4 word = 2048 word
wire [10:0] spr_vram_addr = spr_vram_addr_int;
wire [15:0] spr_vram_data;
wire        arb_spr_req;
wire [23:0] arb_spr_addr;
wire [31:0] arb_spr_data;
wire        arb_spr_valid;

// ── Sprite ROM su DDR3 (libera banda SDRAM ai layer tile) ───────────────────
// Pattern BoogieWings/Darius2: sprite ROM (2MB) in DDR3, read 32-bit + write
// durante ioctl_download. SPR range: byte 0x2A0000..0x4A0000 (= word 0x150000).
localparam [27:0] SPR_DDR3_BASE = 28'h0000000;  // base sprite in DDR3
wire [26:0] spr_dl_off = ioctl_addr - 27'h2A0000;
wire        spr_dl_sel = ioctl_download & (ioctl_index == 16'd0) &
                         (ioctl_addr >= 27'h2A0000) & (ioctl_addr < 27'h4A0000);

// DDR3 write (download): handshake we_req/we_ack toggle
reg  [27:0] spr_ddr_waddr;
reg  [15:0] spr_ddr_wdata;
reg         spr_ddr_we_req = 1'b0;
wire        spr_ddr_we_ack;
reg         spr_dl_wr_d = 1'b0;
always @(posedge clk_sys) begin
	spr_dl_wr_d <= ioctl_wr & spr_dl_sel;
	if (ioctl_wr & spr_dl_sel & ~spr_dl_wr_d) begin
		spr_ddr_waddr <= SPR_DDR3_BASE + {1'b0, spr_dl_off};
		spr_ddr_wdata <= ioctl_dout;
		spr_ddr_we_req <= ~spr_ddr_we_req;
	end
end

// DDR3 read (sprite fetch): bridge dal protocollo rom_req/rom_valid del renderer
reg  [27:0] spr_ddr_raddr;
reg         spr_ddr_rd_req = 1'b0;
wire        spr_ddr_rd_ack;
wire [31:0] spr_ddr_rdata;
reg  [1:0]  spr_rd_state = 2'd0;
reg         spr_rom_valid_r = 1'b0;
reg  [31:0] spr_rom_data_r;
always @(posedge clk_sys) begin
	spr_rom_valid_r <= 1'b0;
	case (spr_rd_state)
		2'd0: if (arb_spr_req) begin
			spr_ddr_raddr  <= SPR_DDR3_BASE + {4'd0, arb_spr_addr};
			spr_ddr_rd_req <= ~spr_ddr_rd_req;
			spr_rd_state   <= 2'd1;
		end
		2'd1: if (spr_ddr_rd_ack == spr_ddr_rd_req) begin
			spr_rom_data_r  <= spr_ddr_rdata;
			spr_rom_valid_r <= 1'b1;
			spr_rd_state    <= 2'd2;
		end
		2'd2: if (!arb_spr_req) spr_rd_state <= 2'd0;
	endcase
end
assign arb_spr_data  = spr_rom_data_r;
assign arb_spr_valid = spr_rom_valid_r;

ddram_sprite u_ddram_spr (
	.DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD(DDRAM_RD), .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
	.wraddr(spr_ddr_waddr), .din(spr_ddr_wdata), .we_req(spr_ddr_we_req), .we_ack(spr_ddr_we_ack),
	.rdaddr(spr_ddr_raddr), .dout(spr_ddr_rdata), .rd_req(spr_ddr_rd_req), .rd_ack(spr_ddr_rd_ack)
);

HeatedBarrel_sprite_renderer u_spr (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_for_read), .vpos(vpos_logic),
	.de(video_de), .layer_en(map_ctrl_l0[4] & ~status[33]),    // bit4 = sprite enable, OSD off
	.new_line(layer_new_line),
	.xoff(osd_spr_xoff), .yoff(osd_spr_yoff),
	.spr_addr(spr_vram_addr_int), .spr_data(spr_vram_data),
	.rom_req(arb_spr_req), .rom_addr(arb_spr_addr),
	.rom_data(arb_spr_data), .rom_valid(arb_spr_valid),
	.opaque(spr_opaque), .pen_index(spr_pen), .pri_code(spr_pri)
);

// ── Tile ROM arbiter (BG/MG/FG su SDRAM; sprite su DDR3 = r3 staccato) ──────
tile_rom_arbiter u_arb (
	.clk(clk_sys), .reset(reset), .hblank(HBlank),
	.r0_req(arb_bg_req),  .r0_addr(arb_bg_addr),  .r0_data(arb_bg_data),  .r0_valid(arb_bg_valid),
	.r1_req(arb_mg_req),  .r1_addr(arb_mg_addr),  .r1_data(arb_mg_data),  .r1_valid(arb_mg_valid),
	.r2_req(arb_fg_req),  .r2_addr(arb_fg_addr),  .r2_data(arb_fg_data),  .r2_valid(arb_fg_valid),
	.r3_req(1'b0), .r3_addr(24'd0), .r3_data(), .r3_valid(),   // sprite ora su DDR3
	.r4_req(1'b0), .r4_addr(24'd0), .r4_data(), .r4_valid(),
	.tile_req(game_tile_req), .tile_addr(game_tile_addr), .tile_kind(game_tile_kind),
	.tile_data(game_tile_data), .tile_valid(game_tile_valid)
);

// Pixel pipeline HeatedBarrel (4 layer: MG/BG/FG/Text + sprite a 4 priority).
//
// MAME riferimenti:
//   legionna_v.cpp:226-229  m_sprite_pri_mask[0..3] per HeatedBarrel:
//     pri=0 → 0x0000  (cover bit 0,1,2,4 → mg,bg,fg,text → SOPRA TUTTO)
//     pri=1 → 0xFFF0  (cover bit 0,1,2 → mg,bg,fg → sotto text)
//     pri=2 → 0xFFFC  (cover bit 0,1 → mg,bg → sotto fg,text)
//     pri=3 → 0xFFFE  (cover bit 0 → mg → sotto bg,fg,text)
//   legionna_v.cpp:342-345 screen_update_legionna draw order (back→front):
//     MG (priority code 0) → BG (1) → FG (2) → Text (4)
//   sei021x_sei0220_spr.cpp draw: il pixel sprite vince contro un layer L
//     se (pri_mask & (1 << L)) == 0.
//
// Ordine composite RTL (front→back), derivato dalle pri_mask sopra:
//   Sprite pri=0  (topmost, sopra qualsiasi layer)
//   Text          (bit 4 layer)
//   Sprite pri=1  (sopra mg/bg/fg, sotto text)
//   FG            (bit 2 layer)
//   Sprite pri=2  (sopra mg/bg)
//   BG            (bit 1 layer)
//   Sprite pri=3  (sopra mg)
//   MG            (bit 0 layer, drawn first = in fondo)
//   Backdrop
wire [10:0] backdrop_pen = 11'h000;   // pen 0 (= nero su palette MAME BLACK init)
wire spr_pri0 = spr_opaque & (spr_pri == 2'd0);
wire spr_pri1 = spr_opaque & (spr_pri == 2'd1);
wire spr_pri2 = spr_opaque & (spr_pri == 2'd2);
wire spr_pri3 = spr_opaque & (spr_pri == 2'd3);

// MAME legionna_v.cpp:340 fills bitmap with m_palette->black_pen() before drawing layers.
// Quando nessun layer opaque, output = nero forzato (no pescare palette[X]).
wire any_layer_opaque = spr_pri0 | text_opaque | spr_pri1 | fg_opaque |
                        spr_pri2 | bg_opaque | spr_pri3 | mg_opaque;
// Mixing heatbrl (front-to-back TEXT,SPR0,BG,SPR1,MG,SPR2,FG,SPR3) — stato sfondi-OK.
wire [10:0] pal_b_addr_c = text_opaque  ? text_pen :
                           spr_pri0     ? spr_pen  :
                           bg_opaque    ? bg_pen   :
                           spr_pri1     ? spr_pen  :
                           mg_opaque    ? mg_pen   :
                           spr_pri2     ? spr_pen  :
                           fg_opaque    ? fg_pen   :
                           spr_pri3     ? spr_pen  :
                                          backdrop_pen;
// Register intermediate per ridurre path depth tra layer opaque/pen e palette
// port B addr (era 9 levels comb, ora 1 register stage). Aggiunge 1 ciclo di
// latency che è invisibile a 16 clk/ce_pix.
reg [10:0] pal_b_addr_r;
always @(posedge clk_sys) pal_b_addr_r <= pal_b_addr_c;
assign pal_b_addr = pal_b_addr_r;

// Backdrop = nero forzato a livello video (no pescare palette[0]).
// MAME legionna_v.cpp:340 fa bitmap.fill(black_pen()) prima dei layer.
// Qualunque cosa scriva il game in palette[0] non deve mostrarsi sul backdrop.
wire backdrop_active_c = ~(spr_pri0 | text_opaque | spr_pri1 | fg_opaque |
                           spr_pri2 | bg_opaque  | spr_pri3 | mg_opaque);
// Register backdrop_active/video_de a +2 cicli per allinearli a pal_b_r/g/b:
// pal_b_addr_r (+1) → BRAM read b_dout (+1) → b_r/g/b comb = dato RGB a +2.
// Prima video_de_r era +1 → gate DE 1 pixel AVANTI al dato RGB → sfasamento di
// 1 pixel visibile SOLO sull'uscita analogica (HDMI usa VGA_DE_IN proprio dal
// blanking nativo, non affetto). Allineamento 1:1 con BloodBros c4bc4a1.
reg backdrop_active_rr, video_de_rr;
reg backdrop_active_r,  video_de_r;
always @(posedge clk_sys) begin
	backdrop_active_rr <= backdrop_active_c;
	video_de_rr        <= video_de;
	backdrop_active_r  <= backdrop_active_rr;
	video_de_r         <= video_de_rr;
end
wire [7:0] video_r = video_de_r ? (backdrop_active_r ? 8'h00 : pal_b_r) : 8'h00;
wire [7:0] video_g = video_de_r ? (backdrop_active_r ? 8'h00 : pal_b_g) : 8'h00;
wire [7:0] video_b = video_de_r ? (backdrop_active_r ? 8'h00 : pal_b_b) : 8'h00;

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;

// Pause overlay: dim video + logo + SUPPORTERS + patron scroll.
// Modulo standalone 8-bit RGB. OSD "Clean Pause" (status[18]): ON=raw, OFF=overlay.
// Output su bus intermedi av_r/av_g/av_b (NON piu` diretto ai pin VGA_R/G/B):
// il modulo CRT Stretch si inserisce fra questo overlay e i pin.
wire [7:0] av_r, av_g, av_b;
pause_overlay u_pause_ovl (
	.clk       (clk_sys),
	.pause     (pause),
	.clean     (status[18]),
	.vblank    (VBlank),
	.render_x  (render_x[8:0]),
	.render_y  (render_y),
	.rgb_r_in  (video_r),
	.rgb_g_in  (video_g),
	.rgb_b_in  (video_b),
	.rgb_r_out (av_r),
	.rgb_g_out (av_g),
	.rgb_b_out (av_b)
);

// ── CRT Adjust (Analog H-Size + H-Position + V-Shift) — modulo unico ─────────
// Portato 1:1 da GundamSD. Tutti i controlli spostano/scalano il CONTENUTO
// lasciando i sync nativi (H-Size: read rate; H-Pos: rd_addr; V-Shift: shreg
// VSync). Nessun desync CRT.
localparam int H_TOTAL_BB = 384;
localparam int V_TOTAL_BB = 263;

// ON/OFF (status[101]): OFF = bypass nativo, ON = modulo attivo (i controlli
// funzionano anche con valori a 0).
reg crt_on;
always @(posedge clk_sys) if (ce_pix) crt_on <= status[101];

// H-Size bidirezionale (status[100:96], two's complement 5-bit): 0 = nativo,
// +1..+15 = enlarge (read piu' lento), -1..-16 = shrink (read piu' veloce).
reg signed [4:0] hsize_s;
always @(posedge clk_sys) if (ce_pix) hsize_s <= $signed(status[100:96]);

// H-Position (status[85:79], 7 bit): sposta il contenuto orizzontale. Encoding
// 0..48 = +0..+48 (destra), 79..127 = -48..-1 (sinistra).
reg [6:0] hpos_d;
always @(posedge clk_sys) if (ce_pix) hpos_d <= status[85:79];
wire signed [8:0] hpos_off = (hpos_d <= 7'd48)
	? $signed({2'b0, hpos_d})
	: $signed({2'b0, hpos_d}) - 9'sd128;

// V-Shift (status[78:74], signed 5-bit -16..+15 righe) -> passato al modulo.
reg signed [5:0] vshift_off;
always @(posedge clk_sys) if (ce_pix) vshift_off <= $signed(status[78:74]);

// Read rate a QUARTI di ciclo (step 1.56%), accumulatore. Periodo = (64+hsize)
// quarti. Reset sul RISE di hs_ref (l'HSync di riferimento del read side esposto
// dal modulo: shiftato in HPOS_SYNCSHIFT, nativo altrimenti) cosi` il read-rate
// e il read counter interno ripartono sullo STESSO fronte -> il contenuto
// allargato resta allineato alla finestra e non finisce nel nero a destra.
wire line_tick = ce_pix && (timing_hpos == 10'(H_TOTAL_BB - 1));
wire hs_ref;                      // dal modulo (registrato) -> no loop combinatorio
reg  hs_ref_d;
always @(posedge clk_sys) hs_ref_d <= hs_ref;
wire hs_ref_rise = hs_ref & ~hs_ref_d;
wire [7:0] rd_period = 8'd64 + {{3{hsize_s[4]}}, hsize_s};  // hsize -16..+15 -> 48..79 quarti
reg  [7:0] rd_acc;
wire rd_tick = (rd_acc + 8'd4) >= {1'b0, rd_period};
always @(posedge clk_sys) begin
	if      (hs_ref_rise) rd_acc <= 8'd0;
	else if (rd_tick)     rd_acc <= rd_acc + 8'd4 - {1'b0, rd_period};
	else                  rd_acc <= rd_acc + 8'd4;
end
wire rd_ce = crt_on ? rd_tick : ce_pix;

wire [7:0] str_r, str_g, str_b;
wire       str_hs, str_vs, str_hb, str_vb;
// HeatedBarrel is a NARROW, side-anchored game (256px active at hb1=96): use
// HPOS_SYNCSHIFT so H-Position slides the HSync instead of the content -> no
// black block at screen edges. See crt_adjust.sv HPOS_MODE block.
crt_adjust #(
	.VTOTAL   (V_TOTAL_BB),
	.HTOTAL   (H_TOTAL_BB),
	.HPOS_MODE(0)          // 0 = HPOS_SYNCSHIFT (narrow/side-anchored game)
) u_crt_adjust (
	.clk      (clk_sys),
	.pxl_cen  (ce_pix),
	.pxl2_cen (rd_ce),
	.active   (crt_on),
	.hsize    (hsize_s),
	.hoffset  (hpos_off),
	.voffset  (vshift_off),
	.r_in     (av_r), .g_in (av_g), .b_in (av_b),
	.hs_in    (HSync),           // HSync NATIVO -> no desync
	.vs_in    (VSync),
	.hb_in    (HBlank | VBlank),
	.vb_in    (VBlank),
	.r_out    (str_r), .g_out (str_g), .b_out (str_b),
	.hs_out   (str_hs), .vs_out (str_vs),
	.hb_out   (str_hb), .vb_out (str_vb),
	.hs_ref_out (hs_ref)
);

// Finestra DE per l'OSD: apre all'attivo nativo (ritardato 1 riga), chiude a
// larghezza stretchata piena (pattern Blood Bros).
reg vblank_1l;
always @(posedge clk_sys) if (line_tick) vblank_1l <= VBlank;
wire native_active = ~(HBlank | vblank_1l);
reg  native_active_d;
always @(posedge clk_sys) if (ce_pix) native_active_d <= native_active;
wire native_rise = native_active & ~native_active_d;
wire str_active = ~str_hb;
reg  str_active_d;
always @(posedge clk_sys) if (rd_ce) str_active_d <= str_active;
wire str_fall = str_active_d & ~str_active;
reg de_osd;
always @(posedge clk_sys) begin
	if      (native_rise) de_osd <= 1'b1;
	else if (str_fall)    de_osd <= 1'b0;
end

// Output: ON -> dal modulo; OFF -> nativo.
assign VGA_R  = crt_on ? str_r  : av_r;
assign VGA_G  = crt_on ? str_g  : av_g;
assign VGA_B  = crt_on ? str_b  : av_b;
assign VGA_HS = crt_on ? str_hs : HSync;
assign VGA_VS = crt_on ? str_vs : VSync;

// Aspect ratio: Original = 4:3 arcade display, Full Screen = 0:0
wire [11:0] arx = (!ar) ? 12'd4 : (ar - 1'd1);
wire [11:0] ary = (!ar) ? 12'd3 : 12'd0;

// Integer scaling forzato: Narrower HV-Integer (default), V-Integer, HV-Integer.
// Normal scaling rimosso perché senza setup utente preciso dà sempre risultato sbagliato.
video_freak video_freak
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(crt_on ? rd_ce : ce_pix),
	.VGA_VS(VSync),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(crt_on ? de_osd : ~(HBlank | VBlank)),
	.ARX(arx),
	.ARY(ary),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE(status[7:5])    // 0=Normal,1=V-Int,2=Narrower,3=Wider,4=HV-Integer
);

// LED: blink during download
assign LED_USER = ioctl_download;

// ============================================================
// JTAG Debug Probes (readable via quartus_stp / System Console)
// ============================================================
// JTAG boot trace removed to save M10K for 64KB work RAM

endmodule
