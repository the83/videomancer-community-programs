-- Copyright (C) 2026 Theron Humiston
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Program Name:        Tangle
-- Author:              Theron Humiston
-- Overview:
--   Integer multiply interference pattern synthesis. Input video is ignored.
--
--   4 formula variants produce self-similar moire patterns from integer
--   arithmetic on pixel coordinates. Rotation via sin/cos LUT, bit-window
--   selector for exploring different slices of the result, negative space,
--   and aspect lock for square patterns on widescreen displays.
--
-- Resources:
--   0 BRAM, estimated ~800 LUTs
--
-- Pipeline:
--   Stage 1: Coordinate mapping (center, zoom, pan, animation)
--   Stage 2: Rotation via sin/cos
--   Stage 3: Fractal formula + bit window + mirror
--   Stage 4: Color mapping + output
--   Sync delay: 4 clocks
--
-- Parameters:
--   Pot 1  (registers_in(0)):    Bit Window (which bits of result to show)
--   Pot 2  (registers_in(1)):    Zoom level
--   Pot 3  (registers_in(2)):    Color palette
--   Pot 4  (registers_in(3)):    Rotation angle
--   Pot 5  (registers_in(4)):    X Pan
--   Pot 6  (registers_in(5)):    Y Pan
--   Tog 7  (registers_in(6)(0)): Negative Space (invert pattern)
--   Tog 8  (registers_in(6)(1)): Animate (drift + slow rotation)
--   Tog 9  (registers_in(6)(2)): Aspect Lock (square patterns on 16:9)
--   Tog 10 (registers_in(6)(3)): Variant A (2-bit formula select with Tog 11)
--   Tog 11 (registers_in(6)(4)): Variant B

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture tangle of program_top is

    constant C_SYNC_DELAY_CLKS : integer := 4;
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Video timing
    signal s_timing : t_video_timing_port;

    -- Pixel counters
    signal s_hcount : unsigned(11 downto 0) := (others => '0');
    signal s_vcount : unsigned(11 downto 0) := (others => '0');

    -- Resolution measurement
    signal s_measured_h : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');

    -- Animation frame counter
    signal r_frame : unsigned(15 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';

    -- Sin/cos LUT
    signal r_angle   : std_logic_vector(9 downto 0) := (others => '0');
    signal s_sin     : signed(9 downto 0);
    signal s_cos     : signed(9 downto 0);

    -- Vsync-latched parameters
    signal r_bit_window  : natural range 0 to 15 := 0;
    signal r_zoom_shift  : natural range 0 to 7 := 0;
    signal r_anim_angle  : unsigned(9 downto 0) := (others => '0');
    signal r_anim_div    : unsigned(5 downto 0) := (others => '0');
    signal r_neg_space   : std_logic := '0';
    signal r_animate     : std_logic := '0';
    signal r_aspect_lock : std_logic := '0';
    signal r_variant     : unsigned(2 downto 0) := (others => '0');
    signal r_palette     : unsigned(3 downto 0) := (others => '0');
    signal r_pan_x       : signed(11 downto 0) := (others => '0');
    signal r_pan_y       : signed(11 downto 0) := (others => '0');

    -- Stage 1: centered + zoomed + panned coords
    signal s1_cx : signed(11 downto 0) := (others => '0');
    signal s1_cy : signed(11 downto 0) := (others => '0');

    -- Stage 2: rotated coords
    signal s2_rx : signed(11 downto 0) := (others => '0');
    signal s2_ry : signed(11 downto 0) := (others => '0');

    -- Stage 3: fractal intensity
    signal s3_intensity : unsigned(7 downto 0) := (others => '0');

    -- Stage 4: output
    signal s4_y : unsigned(9 downto 0) := (others => '0');
    signal s4_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s4_v : unsigned(9 downto 0) := C_CHROMA_MID;

begin

    -- =========================================================================
    -- Video Timing Generator
    -- =========================================================================
    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- =========================================================================
    -- Sin/Cos LUT (combinational)
    -- =========================================================================
    sincos_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => r_angle,
            sin_out  => s_sin,
            cos_out  => s_cos
        );

    -- =========================================================================
    -- Resolution measurement + pixel counters
    -- =========================================================================
    p_counters : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then
                    s_measured_h <= s_h_pixel_counter;
                end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then
                    s_measured_v <= s_v_line_counter;
                end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;

            if s_timing.avid_start = '1' then
                s_hcount <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcount <= s_hcount + 1;
            end if;

            if s_timing.vsync_start = '1' then
                s_vcount <= (others => '0');
            elsif s_timing.avid_end = '1' then
                s_vcount <= s_vcount + 1;
            end if;
        end if;
    end process p_counters;

    -- =========================================================================
    -- Parameter latching on vsync
    -- =========================================================================
    p_params : process(clk)
        variable v_pot1 : unsigned(9 downto 0);
        variable v_pot2 : unsigned(9 downto 0);
        variable v_pot3 : unsigned(9 downto 0);
        variable v_pot5 : unsigned(9 downto 0);
        variable v_pot6 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= s_timing.vsync_start;

            if s_timing.vsync_start = '1' and s_prev_vsync = '0' then
                v_pot1 := unsigned(registers_in(0));
                v_pot2 := unsigned(registers_in(1));
                v_pot3 := unsigned(registers_in(2));
                v_pot5 := unsigned(registers_in(4));
                v_pot6 := unsigned(registers_in(5));

                r_neg_space  <= registers_in(6)(0);
                r_animate    <= registers_in(6)(1);
                r_aspect_lock <= registers_in(6)(2);
                r_variant <= "0" & unsigned(registers_in(6)(4 downto 3));

                r_frame <= r_frame + 1;

                -- Rotation angle: manual from pot 4, plus animation rotation
                if r_animate = '1' then
                    r_anim_div <= r_anim_div + 1;
                    if r_anim_div = 0 then
                        r_anim_angle <= r_anim_angle + 1;  -- ~18 min/rev at 60fps
                    end if;
                end if;
                r_angle <= std_logic_vector(
                    unsigned(registers_in(3)) + r_anim_angle);

                -- Bit window: 0-1023 maps to shift 0-15
                r_bit_window <= to_integer(v_pot1(9 downto 6));

                -- Zoom: 5 levels evenly across knob (0-4)
                -- Each level doubles the pattern scale
                if v_pot2 < 205 then
                    r_zoom_shift <= 0;
                elsif v_pot2 < 410 then
                    r_zoom_shift <= 1;
                elsif v_pot2 < 615 then
                    r_zoom_shift <= 2;
                elsif v_pot2 < 820 then
                    r_zoom_shift <= 3;
                else
                    r_zoom_shift <= 4;
                end if;

                -- Palette: 16 zones across full knob range
                r_palette <= v_pot3(9 downto 6);

                -- Pan: center = 0
                r_pan_x <= resize(signed(resize(v_pot5, 12)) - 512, 12);
                r_pan_y <= resize(signed(resize(v_pot6, 12)) - 512, 12);
            end if;
        end if;
    end process p_params;

    -- =========================================================================
    -- Render pipeline
    -- =========================================================================
    p_render : process(clk)
        -- Stage 1
        variable v_cx : signed(11 downto 0);
        variable v_cy : signed(11 downto 0);
        variable v_anim : signed(11 downto 0);
        -- Stage 2: rotation
        variable v_dx_cos : signed(21 downto 0);
        variable v_dy_sin : signed(21 downto 0);
        variable v_dx_sin : signed(21 downto 0);
        variable v_dy_cos : signed(21 downto 0);
        -- Stage 3: fractal compute
        variable v_mx : unsigned(11 downto 0);
        variable v_my : unsigned(11 downto 0);
        variable v_mul24 : unsigned(23 downto 0);
        variable v_sum12 : unsigned(11 downto 0);
        variable v_xor12 : unsigned(11 downto 0);
        variable v_result : unsigned(23 downto 0);
        variable v_bw : unsigned(7 downto 0);
        -- Stage 4: color
        variable v_final : unsigned(7 downto 0);
        variable v_y_out : unsigned(9 downto 0);
        variable v_u_out : unsigned(9 downto 0);
        variable v_v_out : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- =================================================================
            -- Stage 1: Center, zoom, pan, animate
            -- =================================================================
            v_cx := signed(resize(s_hcount, 12)) -
                    signed('0' & s_measured_h(11 downto 1));
            v_cy := signed(resize(s_vcount, 12)) -
                    signed('0' & s_measured_v(11 downto 1));

            if r_animate = '1' then
                v_anim := resize(signed(r_frame(12 downto 3)), 12);
            else
                v_anim := (others => '0');
            end if;

            -- Aspect lock: scale X by 9/16 so patterns are square on 16:9 displays
            if r_aspect_lock = '1' then
                s1_cx <= resize(shift_right(
                    (v_cx + v_anim) * to_signed(9, 5), r_zoom_shift + 4), 12)
                    + r_pan_x;
            else
                s1_cx <= shift_right(v_cx + v_anim, r_zoom_shift) + r_pan_x;
            end if;
            s1_cy <= shift_right(v_cy + v_anim, r_zoom_shift) + r_pan_y;

            -- =================================================================
            -- Stage 2: Rotation via sin/cos LUT
            -- =================================================================
            v_dx_cos := s1_cx * s_cos;
            v_dy_sin := s1_cy * s_sin;
            v_dx_sin := s1_cx * s_sin;
            v_dy_cos := s1_cy * s_cos;

            s2_rx <= resize(shift_right(v_dx_cos + v_dy_sin, 9), 12);
            s2_ry <= resize(shift_right(v_dy_cos - v_dx_sin, 9), 12);

            -- =================================================================
            -- Stage 3: Fractal formula + bit window + negative space
            -- =================================================================
            v_mx := unsigned(s2_rx);
            v_my := unsigned(s2_ry);

            -- Compute products and combinations using full 12-bit coords
            v_mul24 := resize(v_mx, 12) * resize(v_my, 12);
            v_sum12 := resize(v_mx, 12) + resize(v_my, 12);
            v_xor12 := v_mx xor v_my;

            case to_integer(r_variant) is
                when 0 =>  -- x * y (classic)
                    v_result := v_mul24;
                when 1 =>  -- (x * y) XOR (x + y)
                    v_result := v_mul24 xor resize(v_sum12, 24);
                when 2 =>  -- (x + y) * (x XOR y)
                    v_result := resize(v_sum12 * v_xor12, 24);
                when others =>  -- x*y XOR (x*x + y*y) (moiré)
                    v_result := v_mul24 xor
                                (resize(v_mx * v_mx, 24) +
                                 resize(v_my * v_my, 24));
            end case;

            -- Bit window: select 8 bits from the 24-bit result
            -- Window shifts from LSB (window=0) toward MSB (window=15)
            case r_bit_window is
                when 0  => v_bw := unsigned(std_logic_vector(v_result( 7 downto  0)));
                when 1  => v_bw := unsigned(std_logic_vector(v_result( 8 downto  1)));
                when 2  => v_bw := unsigned(std_logic_vector(v_result( 9 downto  2)));
                when 3  => v_bw := unsigned(std_logic_vector(v_result(10 downto  3)));
                when 4  => v_bw := unsigned(std_logic_vector(v_result(11 downto  4)));
                when 5  => v_bw := unsigned(std_logic_vector(v_result(12 downto  5)));
                when 6  => v_bw := unsigned(std_logic_vector(v_result(13 downto  6)));
                when 7  => v_bw := unsigned(std_logic_vector(v_result(14 downto  7)));
                when 8  => v_bw := unsigned(std_logic_vector(v_result(15 downto  8)));
                when 9  => v_bw := unsigned(std_logic_vector(v_result(16 downto  9)));
                when 10 => v_bw := unsigned(std_logic_vector(v_result(17 downto 10)));
                when 11 => v_bw := unsigned(std_logic_vector(v_result(18 downto 11)));
                when 12 => v_bw := unsigned(std_logic_vector(v_result(19 downto 12)));
                when 13 => v_bw := unsigned(std_logic_vector(v_result(20 downto 13)));
                when 14 => v_bw := unsigned(std_logic_vector(v_result(21 downto 14)));
                when others => v_bw := unsigned(std_logic_vector(v_result(23 downto 16)));
            end case;

            -- Negative space: invert the result before display
            if r_neg_space = '1' then
                s3_intensity <= not v_bw;
            else
                s3_intensity <= v_bw;
            end if;

            -- =================================================================
            -- Stage 4: Color mapping
            -- =================================================================
            v_final := s3_intensity;

            case to_integer(r_palette) is
                -- === Gradient palettes (0-7) ===
                when 0 =>  -- Grayscale
                    v_y_out := v_final & "00";
                    v_u_out := C_CHROMA_MID;
                    v_v_out := C_CHROMA_MID;

                when 1 =>  -- Fire (black → red → yellow → white)
                    if v_final < 64 then
                        v_y_out := resize(v_final & "00", 10);
                        v_u_out := to_unsigned(460, 10);
                        v_v_out := to_unsigned(512, 10) +
                                   resize(v_final & "0", 10);
                    elsif v_final < 128 then
                        v_y_out := to_unsigned(256, 10) +
                                   resize((v_final - 64) & "000", 10);
                        v_u_out := to_unsigned(460, 10) -
                                   resize((v_final - 64) & "0", 10);
                        v_v_out := to_unsigned(640, 10);
                    elsif v_final < 192 then
                        v_y_out := to_unsigned(768, 10) +
                                   resize((v_final - 128) & "00", 10);
                        v_u_out := to_unsigned(332, 10) -
                                   resize((v_final - 128), 10);
                        v_v_out := to_unsigned(640, 10) -
                                   resize((v_final - 128), 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := to_unsigned(512, 10) -
                                   resize(255 - v_final, 10);
                        v_v_out := to_unsigned(512, 10) +
                                   resize(255 - v_final, 10);
                    end if;

                when 2 =>  -- Ocean (black → blue → cyan → white)
                    if v_final < 85 then
                        v_y_out := resize(v_final & "0", 10);
                        v_u_out := to_unsigned(512, 10) +
                                   resize(v_final & "0", 10);
                        v_v_out := to_unsigned(512, 10) -
                                   resize(v_final, 10);
                    elsif v_final < 170 then
                        v_y_out := to_unsigned(170, 10) +
                                   resize((v_final - 85) & "00", 10);
                        v_u_out := to_unsigned(682, 10);
                        v_v_out := to_unsigned(427, 10) -
                                   resize((v_final - 85) & "0", 10);
                    else
                        v_y_out := to_unsigned(512, 10) +
                                   resize((v_final - 170) & "000", 10);
                        v_u_out := to_unsigned(682, 10) -
                                   resize((v_final - 170) & "0", 10);
                        v_v_out := to_unsigned(257, 10) +
                                   resize((v_final - 170) & "00", 10);
                    end if;

                when 3 =>  -- Amber
                    v_y_out := v_final & "00";
                    v_u_out := to_unsigned(480, 10) -
                               shift_right(resize(v_final, 10), 2);
                    v_v_out := to_unsigned(560, 10) +
                               shift_right(resize(v_final, 10), 2);

                when 4 =>  -- Rainbow (3-zone hue)
                    v_y_out := to_unsigned(600, 10);
                    if v_final < 85 then
                        v_u_out := to_unsigned(512, 10) -
                                   resize(v_final & "00", 10);
                        v_v_out := to_unsigned(800, 10);
                    elsif v_final < 170 then
                        v_u_out := to_unsigned(512, 10) +
                                   resize((v_final - 85) & "00", 10);
                        v_v_out := to_unsigned(224, 10);
                    else
                        v_u_out := to_unsigned(800, 10);
                        v_v_out := to_unsigned(224, 10) +
                                   resize((v_final - 170) & "00", 10);
                    end if;

                when 5 =>  -- Green phosphor
                    v_y_out := v_final & "00";
                    v_u_out := to_unsigned(512, 10) -
                               shift_right(resize(v_final, 10), 2);
                    v_v_out := to_unsigned(512, 10) -
                               shift_right(resize(v_final, 10), 1);

                when 6 =>  -- Purple haze
                    v_y_out := v_final & "00";
                    v_u_out := to_unsigned(512, 10) +
                               shift_right(resize(v_final, 10), 2);
                    v_v_out := to_unsigned(512, 10) +
                               shift_right(resize(v_final, 10), 1);

                when 7 =>  -- Cyan tint
                    v_y_out := v_final & "00";
                    v_u_out := to_unsigned(512, 10) +
                               shift_right(resize(v_final, 10), 1);
                    v_v_out := to_unsigned(512, 10) -
                               shift_right(resize(v_final, 10), 1);

                -- === High contrast palettes (8-15) ===
                -- Hard 2-level: black background + solid color foreground

                when 8 =>  -- HC Black on white
                    if v_final(7) = '1' then
                        v_y_out := (others => '0');
                    else
                        v_y_out := to_unsigned(1023, 10);
                    end if;
                    v_u_out := C_CHROMA_MID;
                    v_v_out := C_CHROMA_MID;

                when 9 =>  -- HC Red on white
                    if v_final(7) = '1' then
                        v_y_out := to_unsigned(306, 10);
                        v_u_out := to_unsigned(339, 10);
                        v_v_out := to_unsigned(1023, 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 10 =>  -- HC Green on white
                    if v_final(7) = '1' then
                        v_y_out := to_unsigned(601, 10);
                        v_u_out := to_unsigned(173, 10);
                        v_v_out := to_unsigned(83, 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 11 =>  -- HC Blue on white
                    if v_final(7) = '1' then
                        v_y_out := to_unsigned(117, 10);
                        v_u_out := to_unsigned(1023, 10);
                        v_v_out := to_unsigned(429, 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 12 =>  -- HC Yellow on white
                    if v_final(7) = '1' then
                        v_y_out := to_unsigned(913, 10);
                        v_u_out := to_unsigned(0, 10);
                        v_v_out := to_unsigned(474, 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 13 =>  -- HC Cyan on white
                    if v_final(7) = '1' then
                        v_y_out := to_unsigned(717, 10);
                        v_u_out := to_unsigned(685, 10);
                        v_v_out := to_unsigned(0, 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 14 =>  -- HC Magenta on white
                    if v_final(7) = '1' then
                        v_y_out := to_unsigned(422, 10);
                        v_u_out := to_unsigned(851, 10);
                        v_v_out := to_unsigned(941, 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when others =>  -- HC Orange on white
                    if v_final(7) = '1' then
                        v_y_out := to_unsigned(610, 10);
                        v_u_out := to_unsigned(169, 10);
                        v_v_out := to_unsigned(748, 10);
                    else
                        v_y_out := to_unsigned(1023, 10);
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;
            end case;

            s4_y <= v_y_out;
            s4_u <= v_u_out;
            s4_v <= v_v_out;
        end if;
    end process p_render;

    -- =========================================================================
    -- Sync delay line (4 clocks to match pipeline)
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
    data_out.y <= std_logic_vector(s4_y);
    data_out.u <= std_logic_vector(s4_u);
    data_out.v <= std_logic_vector(s4_v);

end architecture tangle;
