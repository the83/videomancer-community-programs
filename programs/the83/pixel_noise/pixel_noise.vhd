-- Copyright (C) 2026 Theron Humiston
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Program Name:        Pixel Noise
-- Author:              Theron Humiston
-- Overview:
--   Pixel noise synthesis program with independent horizontal and vertical
--   coherence, density, speed, and color controls. Combines features from
--   horiz_pixel_noise, horiz_color_noise, and horiz_palette_noise GLSL
--   shaders (the83/shaderz).
--
--   Modes:
--     Grayscale (color off): Single-channel noise with two-layer mixing.
--     Color Mix (color on, priority off): 3 independent noise channels as
--       R/G/B with additive mixing, converted to YUV. VHS-style chromatic
--       interference. Color knob controls channel spatial decorrelation.
--     Color Priority (color on, priority on): 3 noise layers with priority
--       compositing (no mixing). Highest-priority active layer wins and
--       outputs its palette color. Color knob selects from 16 palettes.
--
--   This is a synthesis program — input video is ignored.
--
-- Resources:
--   0 BRAM, ~2400 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (cell computation):                 1 clock  -> T+1
--   Stage 1 (hash computation, 6 hashes):       1 clock  -> T+2
--   Stage 2 (two-layer mix / passthrough):      1 clock  -> T+3
--   Stage 3 (temporal blend x3):                1 clock  -> T+4
--   Stage 4 (threshold + density x3):           1 clock  -> T+5
--   Stage 5 (compositing + palette lookup):     1 clock  -> T+6
--   Stage 6 (output register):                  1 clock  -> T+7
--   Total: 7 clocks
--
-- Submodules:
--   video_timing_generator: sync edge detection
--   frame_phase_accumulator: animation phase DDS
--
-- Parameters:
--   Pot 1  (registers_in(0)):    H Coherence (cell width)
--   Pot 2  (registers_in(1)):    V Coherence (cell height)
--   Pot 3  (registers_in(2)):    Density
--   Pot 4  (registers_in(3)):    Speed (animation rate)
--   Pot 5  (registers_in(4)):    Color/Palette (channel separation or palette)
--   Tog 7  (registers_in(6)(0)): Color enable (Off=grayscale, On=color)
--   Tog 8  (registers_in(6)(1)): Compositing (Mix / Priority)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture pixel_noise of program_top is

    constant C_SYNC_DELAY_CLKS : integer := 7;

    -- Channel array type (3 channels: R/G/B or layers 0/1/2)
    type t_ch_array is array(0 to 2) of unsigned(9 downto 0);

    -- Video timing
    signal s_timing : t_video_timing_port;

    -- Pixel counters
    signal s_hcount : unsigned(11 downto 0) := (others => '0');
    signal s_vcount : unsigned(11 downto 0) := (others => '0');

    -- Frame phase accumulator
    signal s_phase : unsigned(19 downto 0);

    -- Vsync-latched parameters
    signal r_h_cell_width : unsigned(11 downto 0) := to_unsigned(1, 12);
    signal r_v_cell_width : unsigned(11 downto 0) := to_unsigned(1, 12);
    signal r_threshold    : unsigned(9 downto 0)  := (others => '0');
    signal r_softness     : natural range 0 to 3  := 2;
    signal r_boost_en     : std_logic             := '0';
    signal r_color_en     : std_logic             := '0';
    signal r_priority_en  : std_logic             := '0';
    signal r_ch_offset    : unsigned(5 downto 0)  := (others => '0');
    signal r_palette_idx  : unsigned(3 downto 0)  := (others => '0');
    signal s_prev_vsync   : std_logic             := '1';

    -- Cell tracking counters
    signal s_h_cell_px  : unsigned(11 downto 0) := (others => '0');
    signal s_v_cell_px  : unsigned(11 downto 0) := (others => '0');
    signal s_h_cell     : unsigned(11 downto 0) := (others => '0');
    signal s_v_cell     : unsigned(11 downto 0) := (others => '0');
    signal s_prev_hsync : std_logic             := '1';

    -- Stage 0: registered cell indices
    signal s0_h_cell : unsigned(11 downto 0) := (others => '0');
    signal s0_v_cell : unsigned(11 downto 0) := (others => '0');

    -- Stage 1: 6 hash outputs (3 channels x 2 frames)
    signal s1_ch_n  : t_ch_array := (others => (others => '0'));  -- frame N
    signal s1_ch_n1 : t_ch_array := (others => (others => '0'));  -- frame N+1
    -- Extra layer B for grayscale two-layer mix
    signal s1_b_n   : unsigned(9 downto 0) := (others => '0');
    signal s1_b_n1  : unsigned(9 downto 0) := (others => '0');

    -- Stage 2: after optional two-layer mixing
    signal s2_ch_n  : t_ch_array := (others => (others => '0'));
    signal s2_ch_n1 : t_ch_array := (others => (others => '0'));

    -- Stage 3: temporally blended
    signal s3_ch : t_ch_array := (others => (others => '0'));

    -- Stage 4: after threshold/density
    signal s4_ch : t_ch_array := (others => (others => '0'));

    -- Stage 5: compositing output (YUV directly, no RGB conversion needed)
    signal s5_y : unsigned(9 downto 0) := (others => '0');
    signal s5_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s5_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 6: output register
    signal s6_y : unsigned(9 downto 0) := (others => '0');
    signal s6_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s6_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Hash salts (10-bit constants)
    constant C_SALT_A : unsigned(9 downto 0) := "1010010110";  -- 0x296
    constant C_SALT_B : unsigned(9 downto 0) := "0111110001";  -- 0x1F1
    constant C_SALT_C : unsigned(9 downto 0) := "1100011010";  -- 0x31A
    constant C_SALT_D : unsigned(9 downto 0) := "0011101001";  -- 0x0E9

    -- Channel frame offsets (matching shader's time offsets)
    constant C_FRAME_OFF_0 : unsigned(9 downto 0) := to_unsigned(0, 10);
    constant C_FRAME_OFF_1 : unsigned(9 downto 0) := to_unsigned(31, 10);
    constant C_FRAME_OFF_2 : unsigned(9 downto 0) := to_unsigned(67, 10);

    -- =========================================================================
    -- Hash function
    -- =========================================================================
    function cell_hash(cell_x : unsigned(11 downto 0);
                       cell_y : unsigned(11 downto 0);
                       frame  : unsigned(9 downto 0);
                       salt   : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
        variable vx : unsigned(15 downto 0);
        variable vy : unsigned(15 downto 0);
    begin
        vx := resize(cell_x, 16);
        vy := resize(cell_y, 16);
        v := (vx(11 downto 0) & "0000") + vx + 1;
        v := v xor ("00000" & v(15 downto 5));
        v := v + (v(12 downto 0) & "000");
        v := v + (vy(10 downto 0) & "00000") - vy;
        v := v + resize(frame, 16) + resize(salt, 16);
        v := v xor (v(12 downto 0) & v(15 downto 13));
        v := v + ("00000" & v(15 downto 5));
        v := v xor ("0000000" & v(15 downto 7));
        v := v + (v(12 downto 0) & "000");
        v := v xor ("00000" & v(15 downto 5));
        return v(9 downto 0);
    end function;

    -- =========================================================================
    -- Palette lookup (16 palettes x 3 colors, pre-converted to YUV)
    -- =========================================================================
    type t_yuv_color is record
        y : unsigned(9 downto 0);
        u : unsigned(9 downto 0);
        v : unsigned(9 downto 0);
    end record;

    function palette_lookup(pal_idx : unsigned(3 downto 0);
                            color_idx : natural range 0 to 2) return t_yuv_color is
        variable c : t_yuv_color;
    begin
        -- Default
        c.y := (others => '0');
        c.u := to_unsigned(512, 10);
        c.v := to_unsigned(512, 10);

        case to_integer(pal_idx) is
            when 0 => -- Gray (knob full left = grayscale)
                case color_idx is
                    when 0 => c := (y => to_unsigned(1023, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10));
                    when 1 => c := (y => to_unsigned(583, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10));
                    when 2 => c := (y => to_unsigned(297, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10));
                end case;
            when 1 => -- RGB Primary
                case color_idx is
                    when 0 => c := (y => to_unsigned(306, 10), u => to_unsigned(339, 10), v => to_unsigned(1023, 10));
                    when 1 => c := (y => to_unsigned(601, 10), u => to_unsigned(173, 10), v => to_unsigned(83, 10));
                    when 2 => c := (y => to_unsigned(117, 10), u => to_unsigned(1023, 10), v => to_unsigned(429, 10));
                end case;
            when 2 => -- CMY
                case color_idx is
                    when 0 => c := (y => to_unsigned(717, 10), u => to_unsigned(685, 10), v => to_unsigned(0, 10));
                    when 1 => c := (y => to_unsigned(422, 10), u => to_unsigned(851, 10), v => to_unsigned(941, 10));
                    when 2 => c := (y => to_unsigned(906, 10), u => to_unsigned(0, 10), v => to_unsigned(595, 10));
                end case;
            when 3 => -- Warm Orange
                case color_idx is
                    when 0 => c := (y => to_unsigned(756, 10), u => to_unsigned(85, 10), v => to_unsigned(702, 10));
                    when 1 => c := (y => to_unsigned(456, 10), u => to_unsigned(254, 10), v => to_unsigned(916, 10));
                    when 2 => c := (y => to_unsigned(174, 10), u => to_unsigned(413, 10), v => to_unsigned(804, 10));
                end case;
            when 4 => -- Cool Blue
                case color_idx is
                    when 0 => c := (y => to_unsigned(675, 10), u => to_unsigned(709, 10), v => to_unsigned(345, 10));
                    when 1 => c := (y => to_unsigned(291, 10), u => to_unsigned(925, 10), v => to_unsigned(305, 10));
                    when 2 => c := (y => to_unsigned(66, 10), u => to_unsigned(804, 10), v => to_unsigned(465, 10));
                end case;
            when 5 => -- Tropical
                case color_idx is
                    when 0 => c := (y => to_unsigned(599, 10), u => to_unsigned(584, 10), v => to_unsigned(85, 10));
                    when 1 => c := (y => to_unsigned(631, 10), u => to_unsigned(485, 10), v => to_unsigned(792, 10));
                    when 2 => c := (y => to_unsigned(283, 10), u => to_unsigned(681, 10), v => to_unsigned(412, 10));
                end case;
            when 6 => -- Forest
                case color_idx is
                    when 0 => c := (y => to_unsigned(599, 10), u => to_unsigned(324, 10), v => to_unsigned(209, 10));
                    when 1 => c := (y => to_unsigned(396, 10), u => to_unsigned(387, 10), v => to_unsigned(310, 10));
                    when 2 => c := (y => to_unsigned(203, 10), u => to_unsigned(449, 10), v => to_unsigned(411, 10));
                end case;
            when 7 => -- Magenta Fire
                case color_idx is
                    when 0 => c := (y => to_unsigned(454, 10), u => to_unsigned(337, 10), v => to_unsigned(816, 10));
                    when 1 => c := (y => to_unsigned(224, 10), u => to_unsigned(633, 10), v => to_unsigned(768, 10));
                    when 2 => c := (y => to_unsigned(198, 10), u => to_unsigned(729, 10), v => to_unsigned(685, 10));
                end case;
            when 8 => -- Sunset
                case color_idx is
                    when 0 => c := (y => to_unsigned(732, 10), u => to_unsigned(99, 10), v => to_unsigned(719, 10));
                    when 1 => c := (y => to_unsigned(363, 10), u => to_unsigned(388, 10), v => to_unsigned(880, 10));
                    when 2 => c := (y => to_unsigned(182, 10), u => to_unsigned(658, 10), v => to_unsigned(696, 10));
                end case;
            when 9 => -- Neon
                case color_idx is
                    when 0 => c := (y => to_unsigned(717, 10), u => to_unsigned(685, 10), v => to_unsigned(0, 10));
                    when 1 => c := (y => to_unsigned(389, 10), u => to_unsigned(702, 10), v => to_unsigned(965, 10));
                    when 2 => c := (y => to_unsigned(214, 10), u => to_unsigned(801, 10), v => to_unsigned(673, 10));
                end case;
            when 10 => -- Electric
                case color_idx is
                    when 0 => c := (y => to_unsigned(822, 10), u => to_unsigned(48, 10), v => to_unsigned(655, 10));
                    when 1 => c := (y => to_unsigned(424, 10), u => to_unsigned(440, 10), v => to_unsigned(939, 10));
                    when 2 => c := (y => to_unsigned(126, 10), u => to_unsigned(851, 10), v => to_unsigned(525, 10));
                end case;
            when 11 => -- Ocean
                case color_idx is
                    when 0 => c := (y => to_unsigned(807, 10), u => to_unsigned(634, 10), v => to_unsigned(352, 10));
                    when 1 => c := (y => to_unsigned(341, 10), u => to_unsigned(730, 10), v => to_unsigned(269, 10));
                    when 2 => c := (y => to_unsigned(134, 10), u => to_unsigned(685, 10), v => to_unsigned(416, 10));
                end case;
            when 12 => -- Olive
                case color_idx is
                    when 0 => c := (y => to_unsigned(582, 10), u => to_unsigned(305, 10), v => to_unsigned(410, 10));
                    when 1 => c := (y => to_unsigned(335, 10), u => to_unsigned(386, 10), v => to_unsigned(426, 10));
                    when 2 => c := (y => to_unsigned(200, 10), u => to_unsigned(422, 10), v => to_unsigned(420, 10));
                end case;
            when 13 => -- Amber
                case color_idx is
                    when 0 => c := (y => to_unsigned(749, 10), u => to_unsigned(170, 10), v => to_unsigned(708, 10));
                    when 1 => c := (y => to_unsigned(484, 10), u => to_unsigned(279, 10), v => to_unsigned(685, 10));
                    when 2 => c := (y => to_unsigned(258, 10), u => to_unsigned(367, 10), v => to_unsigned(642, 10));
                end case;
            when 14 => -- Earth
                case color_idx is
                    when 0 => c := (y => to_unsigned(740, 10), u => to_unsigned(343, 10), v => to_unsigned(612, 10));
                    when 1 => c := (y => to_unsigned(457, 10), u => to_unsigned(375, 10), v => to_unsigned(602, 10));
                    when 2 => c := (y => to_unsigned(228, 10), u => to_unsigned(447, 10), v => to_unsigned(561, 10));
                end case;
            when others => -- Pastel
                case color_idx is
                    when 0 => c := (y => to_unsigned(833, 10), u => to_unsigned(539, 10), v => to_unsigned(648, 10));
                    when 1 => c := (y => to_unsigned(850, 10), u => to_unsigned(610, 10), v => to_unsigned(424, 10));
                    when 2 => c := (y => to_unsigned(946, 10), u => to_unsigned(388, 10), v => to_unsigned(464, 10));
                end case;
        end case;
        return c;
    end function;

    -- =========================================================================
    -- Threshold helper (shared across channels)
    -- =========================================================================
    function apply_threshold(noise     : unsigned(9 downto 0);
                             threshold : unsigned(9 downto 0);
                             softness  : natural range 0 to 3;
                             boost_en  : std_logic) return unsigned is
        variable v_diff    : signed(10 downto 0);
        variable v_shifted : unsigned(9 downto 0);
        variable v_boosted : unsigned(10 downto 0);
    begin
        v_diff := signed(resize(noise, 11)) - signed(resize(threshold, 11));
        if v_diff <= 0 then
            return to_unsigned(0, 10);
        end if;
        case softness is
            when 0  => v_shifted := unsigned(v_diff(9 downto 0));
            when 1  => if v_diff > 511 then v_shifted := to_unsigned(1023, 10);
                       else v_shifted := unsigned(v_diff(8 downto 0)) & "0"; end if;
            when 2  => if v_diff > 255 then v_shifted := to_unsigned(1023, 10);
                       else v_shifted := unsigned(v_diff(7 downto 0)) & "00"; end if;
            when 3  => if v_diff > 127 then v_shifted := to_unsigned(1023, 10);
                       else v_shifted := unsigned(v_diff(6 downto 0)) & "000"; end if;
        end case;
        if boost_en = '1' then
            v_boosted := resize(v_shifted, 11) + resize(shift_right(v_shifted, 1), 11);
            if v_boosted > 1023 then
                return to_unsigned(1023, 10);
            else
                return v_boosted(9 downto 0);
            end if;
        end if;
        return v_shifted;
    end function;

begin

    -- =========================================================================
    -- Video Timing Generator
    -- =========================================================================
    timing_gen_inst : entity work.video_timing_generator
        port map(
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- =========================================================================
    -- Pixel Counters
    -- =========================================================================
    p_counters : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.avid = '1' then
                s_hcount <= s_hcount + 1;
            end if;
            if s_timing.hsync_start = '1' then
                s_hcount <= (others => '0');
                s_vcount <= s_vcount + 1;
            end if;
            if s_timing.vsync_start = '1' then
                s_vcount <= (others => '0');
            end if;
        end if;
    end process p_counters;

    -- =========================================================================
    -- Frame Phase Accumulator
    -- =========================================================================
    phase_acc_inst : entity work.frame_phase_accumulator
        generic map(
            G_PHASE_WIDTH => 20,
            G_SPEED_WIDTH => 10
        )
        port map(
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => '1',
            speed   => unsigned(registers_in(3)(9 downto 0)),
            phase   => s_phase
        );

    -- =========================================================================
    -- Parameter latch on vsync
    -- =========================================================================
    p_param_latch : process(clk)
        variable v_density    : unsigned(9 downto 0);
        variable v_h_knob     : unsigned(9 downto 0);
        variable v_v_knob     : unsigned(9 downto 0);
        variable v_h_sq       : unsigned(19 downto 0);
        variable v_v_sq       : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                -- H coherence: quadratic curve
                v_h_knob := unsigned(registers_in(0)(9 downto 0));
                v_h_sq := v_h_knob * v_h_knob;
                r_h_cell_width <= to_unsigned(1, 12) + resize(v_h_sq(19 downto 10), 12);

                -- V coherence: same
                v_v_knob := unsigned(registers_in(1)(9 downto 0));
                v_v_sq := v_v_knob * v_v_knob;
                r_v_cell_width <= to_unsigned(1, 12) + resize(v_v_sq(19 downto 10), 12);

                -- Density -> threshold
                -- Maps density 0->960 (~6% passes), 960+->0 (all passes)
                v_density := unsigned(registers_in(2)(9 downto 0));
                if v_density >= 960 then
                    r_threshold <= to_unsigned(0, 10);
                else
                    r_threshold <= to_unsigned(960, 10) - resize(v_density, 10);
                end if;

                -- Softness (wider at high density for smoother fill)
                if v_density < 240 then
                    r_softness <= 3;
                elsif v_density < 480 then
                    r_softness <= 2;
                elsif v_density < 720 then
                    r_softness <= 1;
                else
                    r_softness <= 0;
                end if;

                -- Boost for sparse noise
                if v_density < 320 then
                    r_boost_en <= '1';
                else
                    r_boost_en <= '0';
                end if;

                -- Color controls
                r_color_en    <= registers_in(6)(0);
                r_priority_en <= registers_in(6)(1);

                -- Knob 5: channel offset (mix mode) or palette index (priority)
                r_ch_offset   <= unsigned(registers_in(4)(9 downto 4));
                r_palette_idx <= unsigned(registers_in(4)(9 downto 6));
            end if;
        end if;
    end process p_param_latch;

    -- =========================================================================
    -- Cell tracking counters
    -- =========================================================================
    p_cell_track : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync <= data_in.hsync_n;

            if s_timing.hsync_start = '1' then
                s_h_cell_px <= (others => '0');
                s_h_cell    <= (others => '0');
            elsif s_timing.avid = '1' then
                if s_h_cell_px >= r_h_cell_width - 1 then
                    s_h_cell_px <= (others => '0');
                    s_h_cell    <= s_h_cell + 1;
                else
                    s_h_cell_px <= s_h_cell_px + 1;
                end if;
            end if;

            if s_timing.vsync_start = '1' then
                s_v_cell_px <= (others => '0');
                s_v_cell    <= (others => '0');
            elsif s_timing.hsync_start = '1' then
                if s_v_cell_px >= r_v_cell_width - 1 then
                    s_v_cell_px <= (others => '0');
                    s_v_cell    <= s_v_cell + 1;
                else
                    s_v_cell_px <= s_v_cell_px + 1;
                end if;
            end if;
        end if;
    end process p_cell_track;

    -- =========================================================================
    -- Stage 0 (T+1): Register cell indices
    -- =========================================================================
    p_stage0 : process(clk)
    begin
        if rising_edge(clk) then
            s0_h_cell <= s_h_cell;
            s0_v_cell <= s_v_cell;
        end if;
    end process p_stage0;

    -- =========================================================================
    -- Stage 1 (T+2): Hash computation — 6 hashes for 3 channels
    --   Color mode: 3 channels with different salts and frame offsets
    --   Grayscale:  ch0 = layer A, ch1 = layer B (for two-layer mix)
    -- =========================================================================
    p_stage1 : process(clk)
        variable v_frame_n  : unsigned(9 downto 0);
        variable v_frame_n1 : unsigned(9 downto 0);
        variable v_hcell_1  : unsigned(11 downto 0);
        variable v_hcell_2  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            v_frame_n  := s_phase(19 downto 10);
            v_frame_n1 := s_phase(19 downto 10) + 1;

            -- Channel 0 (always uses base cell position, salt A)
            s1_ch_n(0)  <= cell_hash(s0_h_cell, s0_v_cell, v_frame_n, C_SALT_A);
            s1_ch_n1(0) <= cell_hash(s0_h_cell, s0_v_cell, v_frame_n1, C_SALT_A);

            if r_color_en = '1' then
                -- Color mode: ch1 and ch2 with spatial offset and frame offset
                v_hcell_1 := s0_h_cell + resize(r_ch_offset, 12);
                v_hcell_2 := s0_h_cell - resize(r_ch_offset, 12);

                s1_ch_n(1)  <= cell_hash(v_hcell_1, s0_v_cell, v_frame_n + C_FRAME_OFF_1, C_SALT_C);
                s1_ch_n1(1) <= cell_hash(v_hcell_1, s0_v_cell, v_frame_n1 + C_FRAME_OFF_1, C_SALT_C);
                s1_ch_n(2)  <= cell_hash(v_hcell_2, s0_v_cell, v_frame_n + C_FRAME_OFF_2, C_SALT_D);
                s1_ch_n1(2) <= cell_hash(v_hcell_2, s0_v_cell, v_frame_n1 + C_FRAME_OFF_2, C_SALT_D);

                s1_b_n  <= (others => '0');
                s1_b_n1 <= (others => '0');
            else
                -- Grayscale mode: ch1 = layer B (for two-layer mix), ch2 unused
                s1_ch_n(1)  <= (others => '0');
                s1_ch_n1(1) <= (others => '0');
                s1_ch_n(2)  <= (others => '0');
                s1_ch_n1(2) <= (others => '0');

                s1_b_n  <= cell_hash(s0_h_cell, s0_v_cell, v_frame_n, C_SALT_B);
                s1_b_n1 <= cell_hash(s0_h_cell, s0_v_cell, v_frame_n1, C_SALT_B);
            end if;
        end if;
    end process p_stage1;

    -- =========================================================================
    -- Stage 2 (T+3): Two-layer mix (grayscale) or passthrough (color)
    -- =========================================================================
    p_stage2 : process(clk)
        variable v_mix0 : unsigned(11 downto 0);
        variable v_mix1 : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if r_color_en = '0' then
                -- Grayscale: two-layer mix 5/8 + 3/8 on channel 0
                v_mix0 := resize(shift_right(s1_ch_n(0), 1), 12) +
                           resize(shift_right(s1_ch_n(0), 3), 12) +
                           resize(shift_right(s1_b_n, 2), 12) +
                           resize(shift_right(s1_b_n, 3), 12);
                if v_mix0 > 1023 then
                    s2_ch_n(0) <= to_unsigned(1023, 10);
                else
                    s2_ch_n(0) <= v_mix0(9 downto 0);
                end if;

                v_mix1 := resize(shift_right(s1_ch_n1(0), 1), 12) +
                           resize(shift_right(s1_ch_n1(0), 3), 12) +
                           resize(shift_right(s1_b_n1, 2), 12) +
                           resize(shift_right(s1_b_n1, 3), 12);
                if v_mix1 > 1023 then
                    s2_ch_n1(0) <= to_unsigned(1023, 10);
                else
                    s2_ch_n1(0) <= v_mix1(9 downto 0);
                end if;

                s2_ch_n(1)  <= (others => '0');
                s2_ch_n1(1) <= (others => '0');
                s2_ch_n(2)  <= (others => '0');
                s2_ch_n1(2) <= (others => '0');
            else
                -- Color: passthrough all 3 channels
                for i in 0 to 2 loop
                    s2_ch_n(i)  <= s1_ch_n(i);
                    s2_ch_n1(i) <= s1_ch_n1(i);
                end loop;
            end if;
        end if;
    end process p_stage2;

    -- =========================================================================
    -- Stage 3 (T+4): Temporal blend x3 channels
    -- =========================================================================
    p_stage3 : process(clk)
        variable v_diff    : signed(10 downto 0);
        variable v_frac    : unsigned(9 downto 0);
        variable v_product : signed(21 downto 0);
        variable v_result  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_frac := s_phase(9 downto 0);
            for i in 0 to 2 loop
                v_diff := signed(resize(s2_ch_n1(i), 11)) -
                          signed(resize(s2_ch_n(i), 11));
                v_product := v_diff * signed('0' & v_frac);
                v_result := signed(resize(s2_ch_n(i), 12)) +
                            resize(v_product(20 downto 10), 12);
                if v_result < 0 then
                    s3_ch(i) <= (others => '0');
                elsif v_result > 1023 then
                    s3_ch(i) <= to_unsigned(1023, 10);
                else
                    s3_ch(i) <= unsigned(v_result(9 downto 0));
                end if;
            end loop;
        end if;
    end process p_stage3;

    -- =========================================================================
    -- Stage 4 (T+5): Threshold + density x3 channels
    -- =========================================================================
    p_stage4 : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to 2 loop
                s4_ch(i) <= apply_threshold(s3_ch(i), r_threshold,
                                            r_softness, r_boost_en);
            end loop;
        end if;
    end process p_stage4;

    -- =========================================================================
    -- Stage 5 (T+6): Compositing — all modes use palette colors
    --   Grayscale: ch0 -> Y, palette color 0 UV as tint
    --   Mix: active channels' palette colors added in YUV (additive overlay)
    --   Priority: highest active channel -> palette color (winner takes all)
    -- =========================================================================
    p_stage5 : process(clk)
        variable v_c0 : t_yuv_color;
        variable v_c1 : t_yuv_color;
        variable v_c2 : t_yuv_color;
        variable v_y_sum  : unsigned(11 downto 0);
        variable v_u_sum  : signed(12 downto 0);
        variable v_v_sum  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_c0 := palette_lookup(r_palette_idx, 0);
            v_c1 := palette_lookup(r_palette_idx, 1);
            v_c2 := palette_lookup(r_palette_idx, 2);

            if r_color_en = '0' then
                -- Grayscale: noise as Y, palette color 0 as UV tint
                s5_y <= s4_ch(0);
                s5_u <= v_c0.u;
                s5_v <= v_c0.v;

            elsif r_priority_en = '0' then
                -- Mix: additive overlay of palette colors in YUV
                v_y_sum := (others => '0');
                v_u_sum := (others => '0');
                v_v_sum := (others => '0');

                if s4_ch(0) > 0 then
                    v_y_sum := v_y_sum + resize(v_c0.y, 12);
                    v_u_sum := v_u_sum + (signed(resize(v_c0.u, 13)) - 512);
                    v_v_sum := v_v_sum + (signed(resize(v_c0.v, 13)) - 512);
                end if;
                if s4_ch(1) > 0 then
                    v_y_sum := v_y_sum + resize(v_c1.y, 12);
                    v_u_sum := v_u_sum + (signed(resize(v_c1.u, 13)) - 512);
                    v_v_sum := v_v_sum + (signed(resize(v_c1.v, 13)) - 512);
                end if;
                if s4_ch(2) > 0 then
                    v_y_sum := v_y_sum + resize(v_c2.y, 12);
                    v_u_sum := v_u_sum + (signed(resize(v_c2.u, 13)) - 512);
                    v_v_sum := v_v_sum + (signed(resize(v_c2.v, 13)) - 512);
                end if;

                -- Clamp Y
                if v_y_sum > 1023 then
                    s5_y <= to_unsigned(1023, 10);
                else
                    s5_y <= v_y_sum(9 downto 0);
                end if;

                -- Clamp U (Cb + 512)
                v_u_sum := v_u_sum + 512;
                if v_u_sum < 0 then
                    s5_u <= to_unsigned(0, 10);
                elsif v_u_sum > 1023 then
                    s5_u <= to_unsigned(1023, 10);
                else
                    s5_u <= unsigned(v_u_sum(9 downto 0));
                end if;

                -- Clamp V (Cr + 512)
                v_v_sum := v_v_sum + 512;
                if v_v_sum < 0 then
                    s5_v <= to_unsigned(0, 10);
                elsif v_v_sum > 1023 then
                    s5_v <= to_unsigned(1023, 10);
                else
                    s5_v <= unsigned(v_v_sum(9 downto 0));
                end if;

            else
                -- Priority: highest active channel wins
                if s4_ch(0) > 0 then
                    s5_y <= v_c0.y;
                    s5_u <= v_c0.u;
                    s5_v <= v_c0.v;
                elsif s4_ch(1) > 0 then
                    s5_y <= v_c1.y;
                    s5_u <= v_c1.u;
                    s5_v <= v_c1.v;
                elsif s4_ch(2) > 0 then
                    s5_y <= v_c2.y;
                    s5_u <= v_c2.u;
                    s5_v <= v_c2.v;
                else
                    s5_y <= (others => '0');
                    s5_u <= to_unsigned(512, 10);
                    s5_v <= to_unsigned(512, 10);
                end if;
            end if;
        end if;
    end process p_stage5;

    -- =========================================================================
    -- Stage 6 (T+7): Output register
    -- =========================================================================
    p_stage6 : process(clk)
    begin
        if rising_edge(clk) then
            s6_y <= s5_y;
            s6_u <= s5_u;
            s6_v <= s5_v;
        end if;
    end process p_stage6;

    -- =========================================================================
    -- Sync delay line
    -- =========================================================================
    p_sync_delay : process(clk)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');
    begin
        if rising_edge(clk) then
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);
        end if;
    end process p_sync_delay;

    -- =========================================================================
    -- Output
    -- =========================================================================
    data_out.y <= std_logic_vector(s6_y);
    data_out.u <= std_logic_vector(s6_u);
    data_out.v <= std_logic_vector(s6_v);

end architecture pixel_noise;
