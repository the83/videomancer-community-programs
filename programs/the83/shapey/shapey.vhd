-- Copyright (C) 2026 Theron Humiston
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Program Name:        Shapey
-- Author:              Theron Humiston
-- Overview:
--   Geometric shape synthesis with smooth color, rotation, outline,
--   concentric rings, invert, complementary background, and tiling.
--   Input video is ignored.
--
--   Shapes: square, circle, triangle, diamond, cross.
--   Rotation via sin/cos LUT. Outline renders shape borders only.
--   Concentric mode repeats alternating bands from center.
--   Complementary mode colors the background with the opposite hue.
--
-- Resources:
--   0 BRAM, ~600 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (timing + pixel/tile counters):   1 clock  -> T+1
--   Stage 1 (raw deltas from center):         1 clock  -> T+2
--   Stage 2 (rotate + abs):                   1 clock  -> T+3
--   Stage 3 (distance tests):                 1 clock  -> T+4
--   Stage 4 (combine + color + output):       1 clock  -> T+5
--   Sync delay: 5 clocks
--   Total: 5 clocks
--
-- Parameters:
--   Pot 1  (registers_in(0)):    Shape (square/circle/triangle/diamond/cross)
--   Pot 2  (registers_in(1)):    Size
--   Pot 3  (registers_in(2)):    Color (white + smooth hue wheel)
--   Pot 4  (registers_in(3)):    Rotation angle
--   Pot 5  (registers_in(4)):    H Offset
--   Pot 6  (registers_in(5)):    V Offset
--   Tog 7  (registers_in(6)(0)): Outline
--   Tog 8  (registers_in(6)(1)): Concentric
--   Tog 9  (registers_in(6)(2)): Invert
--   Tog 10 (registers_in(6)(3)): Tiled
--   Tog 11 (registers_in(6)(4)): Complementary background

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture shapey of program_top is

    constant C_SYNC_DELAY_CLKS : integer := 5;
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Video timing
    signal s_timing : t_video_timing_port;

    -- Pixel counters
    signal s_hcount : unsigned(11 downto 0) := (others => '0');
    signal s_vcount : unsigned(11 downto 0) := (others => '0');

    -- Tile counters
    signal s_tile_h : unsigned(11 downto 0) := (others => '0');
    signal s_tile_v : unsigned(11 downto 0) := (others => '0');

    -- Resolution measurement
    signal s_measured_h : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');

    -- Sin/cos LUT signals
    signal r_angle   : std_logic_vector(9 downto 0) := (others => '0');
    signal s_sin     : signed(9 downto 0);
    signal s_cos     : signed(9 downto 0);

    -- Vsync-latched parameters
    signal r_shape_sel     : unsigned(2 downto 0) := (others => '0');
    signal r_radius        : unsigned(11 downto 0) := to_unsigned(120, 12);
    signal r_inner_radius  : unsigned(11 downto 0) := to_unsigned(105, 12);
    signal r_arm_width     : unsigned(11 downto 0) := to_unsigned(30, 12);
    signal r_inner_arm     : unsigned(11 downto 0) := to_unsigned(15, 12);
    signal r_rad_sq        : unsigned(23 downto 0) := (others => '0');
    signal r_inner_rad_sq  : unsigned(23 downto 0) := (others => '0');
    signal r_center_x      : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal r_center_y      : unsigned(11 downto 0) := to_unsigned(270, 12);
    signal r_tile_max      : unsigned(11 downto 0) := to_unsigned(239, 12);
    signal r_ring_shift    : natural range 0 to 7  := 4;
    signal s_prev_vsync    : std_logic := '1';

    -- Switch states
    signal r_outline    : std_logic := '0';
    signal r_concentric : std_logic := '0';
    signal r_invert     : std_logic := '0';
    signal r_tiled      : std_logic := '0';
    signal r_complement : std_logic := '0';

    -- Precomputed foreground and background colors
    signal r_shape_y : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal r_shape_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal r_shape_v : unsigned(9 downto 0) := C_CHROMA_MID;
    signal r_bg_y    : unsigned(9 downto 0) := (others => '0');
    signal r_bg_u    : unsigned(9 downto 0) := C_CHROMA_MID;
    signal r_bg_v    : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Stage 1: raw deltas
    signal s1_dx : signed(12 downto 0) := (others => '0');
    signal s1_dy : signed(12 downto 0) := (others => '0');

    -- Stage 2: rotated + abs
    signal s2_abs_dx  : unsigned(11 downto 0) := (others => '0');
    signal s2_abs_dy  : unsigned(11 downto 0) := (others => '0');
    signal s2_shape   : unsigned(2 downto 0) := (others => '0');
    signal s2_dy_rot  : signed(12 downto 0) := (others => '0');
    signal s2_cheb    : unsigned(11 downto 0) := (others => '0');

    -- Stage 3: distance test results
    signal s3_inside_outer : std_logic := '0';
    signal s3_inside_inner : std_logic := '0';
    signal s3_ring_band    : std_logic := '0';

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
    -- Resolution measurement + pixel counters + tile counters
    -- =========================================================================
    p_counters : process(clk)
    begin
        if rising_edge(clk) then
            -- Horizontal measurement
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then
                    s_measured_h <= s_h_pixel_counter;
                end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;

            -- Vertical measurement
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then
                    s_measured_v <= s_v_line_counter;
                end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;

            -- Pixel position within active area
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

            -- Tile counters
            if s_timing.avid_start = '1' then
                if r_tiled = '1' then
                    s_tile_h <= resize(r_center_x, 12);
                else
                    s_tile_h <= (others => '0');
                end if;
            elsif s_timing.avid = '1' then
                if r_tiled = '1' and s_tile_h >= r_tile_max then
                    s_tile_h <= (others => '0');
                else
                    s_tile_h <= s_tile_h + 1;
                end if;
            end if;

            if s_timing.vsync_start = '1' then
                if r_tiled = '1' then
                    s_tile_v <= resize(r_center_y, 12);
                else
                    s_tile_v <= (others => '0');
                end if;
            elsif s_timing.avid_end = '1' then
                if r_tiled = '1' and s_tile_v >= r_tile_max then
                    s_tile_v <= (others => '0');
                else
                    s_tile_v <= s_tile_v + 1;
                end if;
            end if;
        end if;
    end process p_counters;

    -- =========================================================================
    -- Parameter latching on vsync
    -- =========================================================================
    p_params : process(clk)
        variable v_pot1   : unsigned(9 downto 0);
        variable v_pot2   : unsigned(9 downto 0);
        variable v_pot3   : unsigned(9 downto 0);
        variable v_pot5   : unsigned(9 downto 0);
        variable v_pot6   : unsigned(9 downto 0);
        variable v_radius : unsigned(11 downto 0);
        variable v_inner  : unsigned(11 downto 0);
        variable v_border : unsigned(11 downto 0);
        variable v_arm    : unsigned(11 downto 0);
        variable v_inner_arm : unsigned(11 downto 0);
        -- Color
        variable v_zone   : unsigned(2 downto 0);
        variable v_frac   : unsigned(6 downto 0);
        variable v_y0, v_y1 : unsigned(9 downto 0);
        variable v_u0, v_u1 : unsigned(9 downto 0);
        variable v_v0, v_v1 : unsigned(9 downto 0);
        variable v_dy_c, v_du_c, v_dv_c : signed(10 downto 0);
        variable v_out_y, v_out_u, v_out_v : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= s_timing.vsync_start;
            if s_timing.vsync_start = '1' and s_prev_vsync = '0' then
                v_pot1 := unsigned(registers_in(0));
                v_pot2 := unsigned(registers_in(1));
                v_pot3 := unsigned(registers_in(2));
                v_pot5 := unsigned(registers_in(4));
                v_pot6 := unsigned(registers_in(5));

                -- Switch states
                r_outline    <= registers_in(6)(0);
                r_concentric <= registers_in(6)(1);
                r_invert     <= registers_in(6)(2);
                r_tiled      <= registers_in(6)(3);
                r_complement <= registers_in(6)(4);

                -- Rotation angle
                r_angle <= registers_in(3);

                -- Shape select: 5 shapes across 0-1023
                if v_pot1 < 205 then
                    r_shape_sel <= "000";  -- square
                elsif v_pot1 < 410 then
                    r_shape_sel <= "001";  -- circle
                elsif v_pot1 < 615 then
                    r_shape_sel <= "010";  -- triangle
                elsif v_pot1 < 820 then
                    r_shape_sel <= "011";  -- diamond
                else
                    r_shape_sel <= "100";  -- cross
                end if;

                -- Size
                v_radius := resize(
                    shift_right(v_pot2 * s_measured_v, 10)(11 downto 0), 12);
                if v_radius < 1 then
                    v_radius := to_unsigned(1, 12);
                end if;
                r_radius <= v_radius;

                -- Outline border = radius/8, min 1
                v_border := shift_right(v_radius, 3);
                if v_border < 1 then
                    v_border := to_unsigned(1, 12);
                end if;
                if v_border >= v_radius then
                    v_inner := to_unsigned(0, 12);
                else
                    v_inner := v_radius - v_border;
                end if;
                r_inner_radius <= v_inner;

                -- Cross arm width = radius/4, and inner arm for outline
                v_arm := shift_right(v_radius, 2);
                if v_arm < 1 then
                    v_arm := to_unsigned(1, 12);
                end if;
                r_arm_width <= v_arm;
                if v_border >= v_arm then
                    v_inner_arm := to_unsigned(0, 12);
                else
                    v_inner_arm := v_arm - v_border;
                end if;
                r_inner_arm <= v_inner_arm;

                -- Precompute squared radii for circle tests
                r_rad_sq <= resize(v_radius * v_radius, 24);
                r_inner_rad_sq <= resize(v_inner * v_inner, 24);

                -- Tile max = 2*radius - 1
                if v_radius > 0 then
                    r_tile_max <= resize(v_radius + v_radius - 1, 12);
                else
                    r_tile_max <= (others => '0');
                end if;

                -- Ring spacing from radius magnitude
                if v_radius >= 256 then
                    r_ring_shift <= 6;
                elsif v_radius >= 128 then
                    r_ring_shift <= 5;
                elsif v_radius >= 64 then
                    r_ring_shift <= 4;
                elsif v_radius >= 32 then
                    r_ring_shift <= 3;
                else
                    r_ring_shift <= 2;
                end if;

                -- Position
                r_center_x <= resize(
                    shift_right(v_pot5 * s_measured_h, 10)(11 downto 0), 12);
                r_center_y <= resize(
                    shift_right(v_pot6 * s_measured_v, 10)(11 downto 0), 12);

                -- ============================================================
                -- Color: zone 0 = white, zones 1-7 = smooth hue wheel
                -- Zone select from top 3 bits, interpolate with lower 7 bits
                -- ============================================================
                v_zone := v_pot3(9 downto 7);
                v_frac := v_pot3(6 downto 0);

                -- Select YUV waypoints for interpolation
                -- Pre-computed BT.601 YUV at full saturation
                case to_integer(v_zone) is
                    when 0 =>  -- White (no interpolation)
                        v_y0 := to_unsigned(1023, 10);
                        v_u0 := C_CHROMA_MID;
                        v_v0 := C_CHROMA_MID;
                        v_y1 := v_y0; v_u1 := v_u0; v_v1 := v_v0;
                    when 1 =>  -- Red -> Yellow
                        v_y0 := to_unsigned(306, 10);  v_y1 := to_unsigned(913, 10);
                        v_u0 := to_unsigned(339, 10);  v_u1 := to_unsigned(  0, 10);
                        v_v0 := to_unsigned(1023, 10); v_v1 := to_unsigned(474, 10);
                    when 2 =>  -- Yellow -> Green
                        v_y0 := to_unsigned(913, 10);  v_y1 := to_unsigned(601, 10);
                        v_u0 := to_unsigned(  0, 10);  v_u1 := to_unsigned(173, 10);
                        v_v0 := to_unsigned(474, 10);  v_v1 := to_unsigned( 83, 10);
                    when 3 =>  -- Green -> Cyan
                        v_y0 := to_unsigned(601, 10);  v_y1 := to_unsigned(717, 10);
                        v_u0 := to_unsigned(173, 10);  v_u1 := to_unsigned(685, 10);
                        v_v0 := to_unsigned( 83, 10);  v_v1 := to_unsigned(  0, 10);
                    when 4 =>  -- Cyan -> Blue
                        v_y0 := to_unsigned(717, 10);  v_y1 := to_unsigned(117, 10);
                        v_u0 := to_unsigned(685, 10);  v_u1 := to_unsigned(1023, 10);
                        v_v0 := to_unsigned(  0, 10);  v_v1 := to_unsigned(429, 10);
                    when 5 =>  -- Blue -> Magenta
                        v_y0 := to_unsigned(117, 10);  v_y1 := to_unsigned(422, 10);
                        v_u0 := to_unsigned(1023, 10); v_u1 := to_unsigned(851, 10);
                        v_v0 := to_unsigned(429, 10);  v_v1 := to_unsigned(941, 10);
                    when 6 =>  -- Magenta -> Red
                        v_y0 := to_unsigned(422, 10);  v_y1 := to_unsigned(306, 10);
                        v_u0 := to_unsigned(851, 10);  v_u1 := to_unsigned(339, 10);
                        v_v0 := to_unsigned(941, 10);  v_v1 := to_unsigned(1023, 10);
                    when others =>  -- Red (wrap)
                        v_y0 := to_unsigned(306, 10);  v_y1 := to_unsigned(306, 10);
                        v_u0 := to_unsigned(339, 10);  v_u1 := to_unsigned(339, 10);
                        v_v0 := to_unsigned(1023, 10); v_v1 := to_unsigned(1023, 10);
                end case;

                -- Interpolate using shift-and-add (top 3 bits of frac)
                v_dy_c := signed(resize(v_y1, 11)) - signed(resize(v_y0, 11));
                v_du_c := signed(resize(v_u1, 11)) - signed(resize(v_u0, 11));
                v_dv_c := signed(resize(v_v1, 11)) - signed(resize(v_v0, 11));

                v_out_y := signed(resize(v_y0, 11));
                v_out_u := signed(resize(v_u0, 11));
                v_out_v := signed(resize(v_v0, 11));
                if v_frac(6) = '1' then
                    v_out_y := v_out_y + shift_right(v_dy_c, 1);
                    v_out_u := v_out_u + shift_right(v_du_c, 1);
                    v_out_v := v_out_v + shift_right(v_dv_c, 1);
                end if;
                if v_frac(5) = '1' then
                    v_out_y := v_out_y + shift_right(v_dy_c, 2);
                    v_out_u := v_out_u + shift_right(v_du_c, 2);
                    v_out_v := v_out_v + shift_right(v_dv_c, 2);
                end if;
                if v_frac(4) = '1' then
                    v_out_y := v_out_y + shift_right(v_dy_c, 3);
                    v_out_u := v_out_u + shift_right(v_du_c, 3);
                    v_out_v := v_out_v + shift_right(v_dv_c, 3);
                end if;
                r_shape_y <= unsigned(v_out_y(9 downto 0));
                r_shape_u <= unsigned(v_out_u(9 downto 0));
                r_shape_v <= unsigned(v_out_v(9 downto 0));

                -- Complementary background: invert chroma channels
                -- White (zone 0) has no chroma to invert, so use black
                if r_complement = '1' and v_zone /= 0 then
                    r_bg_y <= unsigned(v_out_y(9 downto 0));
                    r_bg_u <= not unsigned(v_out_u(9 downto 0));
                    r_bg_v <= not unsigned(v_out_v(9 downto 0));
                else
                    r_bg_y <= (others => '0');
                    r_bg_u <= C_CHROMA_MID;
                    r_bg_v <= C_CHROMA_MID;
                end if;
            end if;
        end if;
    end process p_params;

    -- =========================================================================
    -- Render pipeline
    -- =========================================================================
    p_render : process(clk)
        -- Stage 1
        variable v_dx_s : signed(12 downto 0);
        variable v_dy_s : signed(12 downto 0);
        -- Stage 2: rotation
        variable v_dx_cos : signed(22 downto 0);
        variable v_dy_sin : signed(22 downto 0);
        variable v_dx_sin : signed(22 downto 0);
        variable v_dy_cos : signed(22 downto 0);
        variable v_dx_rot : signed(12 downto 0);
        variable v_dy_rot : signed(12 downto 0);
        variable v_abs_dx : unsigned(11 downto 0);
        variable v_abs_dy : unsigned(11 downto 0);
        -- Stage 3: distance tests
        variable v_dist_sq  : unsigned(23 downto 0);
        variable v_tri_sum  : signed(12 downto 0);
        variable v_in_outer : std_logic;
        variable v_in_inner : std_logic;
        variable v_manhattan : unsigned(12 downto 0);
        -- Stage 4
        variable v_in_shape : std_logic;
    begin
        if rising_edge(clk) then
            -- =================================================================
            -- Stage 1: raw deltas from center (or tile center)
            -- =================================================================
            if r_tiled = '1' then
                v_dx_s := signed(resize(s_tile_h, 13)) -
                          signed(resize(r_radius, 13));
                v_dy_s := signed(resize(s_tile_v, 13)) -
                          signed(resize(r_radius, 13));
            else
                v_dx_s := signed(resize(s_hcount, 13)) -
                          signed(resize(r_center_x, 13));
                v_dy_s := signed(resize(s_vcount, 13)) -
                          signed(resize(r_center_y, 13));
            end if;
            s1_dx <= v_dx_s;
            s1_dy <= v_dy_s;

            -- =================================================================
            -- Stage 2: rotate, compute abs and Chebyshev distance
            -- =================================================================
            v_dx_cos := s1_dx * s_cos;
            v_dy_sin := s1_dy * s_sin;
            v_dx_sin := s1_dx * s_sin;
            v_dy_cos := s1_dy * s_cos;

            v_dx_rot := resize(shift_right(v_dx_cos + v_dy_sin, 9), 13);
            v_dy_rot := resize(shift_right(v_dy_cos - v_dx_sin, 9), 13);

            if v_dx_rot < 0 then
                v_abs_dx := unsigned(resize(-v_dx_rot, 12));
            else
                v_abs_dx := unsigned(resize(v_dx_rot, 12));
            end if;
            if v_dy_rot < 0 then
                v_abs_dy := unsigned(resize(-v_dy_rot, 12));
            else
                v_abs_dy := unsigned(resize(v_dy_rot, 12));
            end if;

            s2_abs_dx <= v_abs_dx;
            s2_abs_dy <= v_abs_dy;
            s2_dy_rot <= v_dy_rot;
            s2_shape  <= r_shape_sel;

            if v_abs_dx > v_abs_dy then
                s2_cheb <= v_abs_dx;
            else
                s2_cheb <= v_abs_dy;
            end if;

            -- =================================================================
            -- Stage 3: shape distance tests (outer + inner for outline)
            -- =================================================================
            case s2_shape is
                when "000" =>  -- Square
                    if s2_abs_dx <= r_radius and s2_abs_dy <= r_radius then
                        v_in_outer := '1';
                    else
                        v_in_outer := '0';
                    end if;
                    if s2_abs_dx <= r_inner_radius and
                       s2_abs_dy <= r_inner_radius then
                        v_in_inner := '1';
                    else
                        v_in_inner := '0';
                    end if;

                when "001" =>  -- Circle
                    v_dist_sq := resize(s2_abs_dx * s2_abs_dx, 24) +
                                 resize(s2_abs_dy * s2_abs_dy, 24);
                    if v_dist_sq <= r_rad_sq then
                        v_in_outer := '1';
                    else
                        v_in_outer := '0';
                    end if;
                    if v_dist_sq <= r_inner_rad_sq then
                        v_in_inner := '1';
                    else
                        v_in_inner := '0';
                    end if;

                when "010" =>  -- Triangle
                    v_tri_sum := s2_dy_rot + signed(resize(r_radius, 13));
                    if v_tri_sum < 0 then
                        v_in_outer := '0';
                    elsif s2_dy_rot > signed(resize(r_radius, 13)) then
                        v_in_outer := '0';
                    elsif shift_left(resize(s2_abs_dx, 13), 1) <=
                          unsigned(v_tri_sum) then
                        v_in_outer := '1';
                    else
                        v_in_outer := '0';
                    end if;
                    -- Triangle inner
                    v_tri_sum := s2_dy_rot +
                                 signed(resize(r_inner_radius, 13));
                    if v_tri_sum < 0 then
                        v_in_inner := '0';
                    elsif s2_dy_rot >
                          signed(resize(r_inner_radius, 13)) then
                        v_in_inner := '0';
                    elsif shift_left(resize(s2_abs_dx, 13), 1) <=
                          unsigned(v_tri_sum) then
                        v_in_inner := '1';
                    else
                        v_in_inner := '0';
                    end if;

                when "011" =>  -- Diamond (Manhattan distance)
                    v_manhattan := resize(s2_abs_dx, 13) +
                                   resize(s2_abs_dy, 13);
                    if v_manhattan <= resize(r_radius, 13) then
                        v_in_outer := '1';
                    else
                        v_in_outer := '0';
                    end if;
                    if v_manhattan <= resize(r_inner_radius, 13) then
                        v_in_inner := '1';
                    else
                        v_in_inner := '0';
                    end if;

                when others =>  -- Cross
                    -- Outer: horizontal or vertical arm
                    if (s2_abs_dx <= r_arm_width and
                        s2_abs_dy <= r_radius) or
                       (s2_abs_dy <= r_arm_width and
                        s2_abs_dx <= r_radius) then
                        v_in_outer := '1';
                    else
                        v_in_outer := '0';
                    end if;
                    -- Inner: narrower arms for outline
                    if (s2_abs_dx <= r_inner_arm and
                        s2_abs_dy <= r_inner_radius) or
                       (s2_abs_dy <= r_inner_arm and
                        s2_abs_dx <= r_inner_radius) then
                        v_in_inner := '1';
                    else
                        v_in_inner := '0';
                    end if;
            end case;

            s3_inside_outer <= v_in_outer;
            s3_inside_inner <= v_in_inner;
            s3_ring_band <= s2_cheb(r_ring_shift);

            -- =================================================================
            -- Stage 4: combine modes, select color, output
            -- =================================================================
            if r_outline = '1' then
                v_in_shape := s3_inside_outer and not s3_inside_inner;
            else
                v_in_shape := s3_inside_outer;
            end if;

            if r_concentric = '1' then
                v_in_shape := v_in_shape xor s3_ring_band;
            end if;

            if (v_in_shape xor r_invert) = '1' then
                s4_y <= r_shape_y;
                s4_u <= r_shape_u;
                s4_v <= r_shape_v;
            else
                s4_y <= r_bg_y;
                s4_u <= r_bg_u;
                s4_v <= r_bg_v;
            end if;
        end if;
    end process p_render;

    -- =========================================================================
    -- Sync delay line (5 clocks to match pipeline)
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

end architecture shapey;
