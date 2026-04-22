-- Copyright (C) 2026 Theron Humiston
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Program Name:        Flower
-- Author:              Theron Humiston
-- Overview:
--   Animated flower / rose-curve pattern synthesis. Input video is ignored.
--
--   Renders N-petal roulette curves using CORDIC-based per-pixel polar
--   coordinate computation. The CORDIC uses only additions and shifts
--   (no multipliers) to extract angle and radius for each pixel. A twist
--   parameter adds radius-dependent angular offset, turning petals into
--   spiral arms. Star mode sharpens petals. Fill + Width creates
--   hypotrochoid-like donut shapes with an inner radius cutoff.
--   Aspect ratio is always corrected for square output on 16:9 displays.
--
-- Resources:
--   0 BRAM, estimated ~2200 LUTs
--
-- Pipeline:
--   Stage 1: Coordinate centering + rotation (4 multiplies of 12x10)
--   Stage 2: CORDIC setup (abs, octant swap, quadrant detect)
--   Stages 3-6: CORDIC iterations 0-7 (2 per stage, multiply-free)
--   Stage 7: Angle reconstruction + petal function + color mapping
--   Sync delay: 7 clocks
--
-- Parameters:
--   Pot 1  (registers_in(0)):    Petals (number of lobes, 2-12)
--   Pot 2  (registers_in(1)):    Size (petal radius)
--   Pot 3  (registers_in(2)):    Color palette
--   Pot 4  (registers_in(3)):    Rotation angle
--   Pot 5  (registers_in(4)):    Width (line thickness; inner radius when Fill on)
--   Pot 6  (registers_in(5)):    Twist (spiral arm curvature)
--   Tog 7  (registers_in(6)(0)): Animate (continuous rotation + concentric pulse)
--   Tog 8  (registers_in(6)(1)): Fill (solid petals with inner radius from Width)
--   Tog 9  (registers_in(6)(2)): Concentric (repeating rings)
--   Tog 10 (registers_in(6)(3)): Invert (swap foreground/background)
--   Tog 11 (registers_in(6)(4)): Star (pointy petals, wider gaps)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture flower of program_top is

    constant C_SYNC_DELAY_CLKS : integer := 7;
    constant C_CHROMA_MID      : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- CORDIC atan lookup (10-bit angle: 1024 = 360 degrees)
    type t_atan_lut is array (0 to 7) of unsigned(9 downto 0);
    constant C_ATAN : t_atan_lut := (
        to_unsigned(128, 10), to_unsigned(76, 10), to_unsigned(40, 10), to_unsigned(20, 10),
        to_unsigned(10, 10),  to_unsigned(5, 10),  to_unsigned(3, 10),  to_unsigned(1, 10)
    );

    -- Video timing
    signal s_timing : t_video_timing_port;

    -- Pixel counters
    signal s_hcount : unsigned(11 downto 0) := (others => '0');
    signal s_vcount : unsigned(11 downto 0) := (others => '0');

    -- Resolution measurement
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');

    -- Animation
    signal s_prev_vsync : std_logic := '1';
    signal r_anim_angle : unsigned(9 downto 0) := (others => '0');
    signal r_anim_div   : unsigned(1 downto 0) := (others => '0');
    signal r_frame       : unsigned(15 downto 0) := (others => '0');

    -- Sin/cos LUT
    signal r_angle : std_logic_vector(9 downto 0) := (others => '0');
    signal s_sin   : signed(9 downto 0);
    signal s_cos   : signed(9 downto 0);

    -- Vsync-latched parameters
    signal r_petals      : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal r_size        : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal r_palette     : unsigned(3 downto 0) := (others => '0');
    signal r_width       : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal r_twist       : signed(9 downto 0) := (others => '0');
    signal r_animate     : std_logic := '0';
    signal r_fill        : std_logic := '0';
    signal r_concentric  : std_logic := '0';
    signal r_invert      : std_logic := '0';
    signal r_star        : std_logic := '0';

    -- Stage 1 outputs: rotated centered coordinates
    signal s1_rx : signed(11 downto 0) := (others => '0');
    signal s1_ry : signed(11 downto 0) := (others => '0');

    -- CORDIC pipeline
    type t_cordic is record
        x        : signed(12 downto 0);
        y        : signed(12 downto 0);
        z        : unsigned(9 downto 0);
        quadrant : unsigned(1 downto 0);
        swapped  : std_logic;
    end record;

    constant C_CORDIC_INIT : t_cordic := (
        x => (others => '0'), y => (others => '0'),
        z => (others => '0'), quadrant => (others => '0'),
        swapped => '0'
    );

    type t_cordic_pipe is array (0 to 4) of t_cordic;
    signal cordic : t_cordic_pipe := (others => C_CORDIC_INIT);

    -- Color output
    signal s_out_y : unsigned(9 downto 0) := (others => '0');
    signal s_out_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_out_v : unsigned(9 downto 0) := C_CHROMA_MID;

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
    -- Sin/Cos LUT
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

                r_animate     <= registers_in(6)(0);
                r_fill        <= registers_in(6)(1);
                r_concentric  <= registers_in(6)(2);
                r_invert      <= registers_in(6)(3);
                r_star        <= registers_in(6)(4);

                r_frame <= r_frame + 1;

                -- Rotation: manual + animation (~34 sec/rev at 60fps)
                if r_animate = '1' then
                    r_anim_div <= r_anim_div + 1;
                    if r_anim_div = 0 then
                        r_anim_angle <= r_anim_angle + 1;
                    end if;
                end if;
                r_angle <= std_logic_vector(
                    unsigned(registers_in(3)) + r_anim_angle);

                -- Petals: 2-12 across knob range
                if    v_pot1 < 94  then r_petals <= to_unsigned(2, 4);
                elsif v_pot1 < 187 then r_petals <= to_unsigned(3, 4);
                elsif v_pot1 < 280 then r_petals <= to_unsigned(4, 4);
                elsif v_pot1 < 374 then r_petals <= to_unsigned(5, 4);
                elsif v_pot1 < 467 then r_petals <= to_unsigned(6, 4);
                elsif v_pot1 < 560 then r_petals <= to_unsigned(7, 4);
                elsif v_pot1 < 654 then r_petals <= to_unsigned(8, 4);
                elsif v_pot1 < 747 then r_petals <= to_unsigned(9, 4);
                elsif v_pot1 < 840 then r_petals <= to_unsigned(10, 4);
                elsif v_pot1 < 934 then r_petals <= to_unsigned(11, 4);
                else                    r_petals <= to_unsigned(12, 4);
                end if;

                r_size    <= v_pot2;
                r_palette <= v_pot3(9 downto 6);
                r_width   <= v_pot5(9 downto 4);
                r_twist   <= signed(resize(v_pot6, 10)) - 512;
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
        -- CORDIC setup
        variable v_abs_x, v_abs_y : unsigned(11 downto 0);
        -- CORDIC iteration
        variable v_cx0, v_cx1, v_cx2 : signed(12 downto 0);
        variable v_cy0, v_cy1, v_cy2 : signed(12 downto 0);
        variable v_cz0, v_cz1, v_cz2 : unsigned(9 downto 0);
        -- Angle + petal
        variable v_base_z     : unsigned(9 downto 0);
        variable v_full_angle : unsigned(9 downto 0);
        variable v_n_angle    : unsigned(9 downto 0);
        variable v_twist_mag  : unsigned(9 downto 0);
        variable v_twist_prod : unsigned(19 downto 0);
        variable v_twist_offs : unsigned(9 downto 0);
        variable v_petal_wave : unsigned(7 downto 0);
        variable v_star_sq    : unsigned(15 downto 0);
        variable v_petal_prod : unsigned(17 downto 0);
        variable v_petal_rad  : unsigned(11 downto 0);
        variable v_pixel_rad  : unsigned(11 downto 0);
        variable v_inner      : unsigned(11 downto 0);
        variable v_diff       : unsigned(11 downto 0);
        variable v_on_curve   : std_logic;
        -- Color
        variable v_angle_hue : unsigned(7 downto 0);
        variable v_petal_hue : unsigned(7 downto 0);
        variable v_rad_hue   : unsigned(7 downto 0);
        variable v_hue       : unsigned(7 downto 0);
        variable v_y_out     : unsigned(9 downto 0);
        variable v_u_out     : unsigned(9 downto 0);
        variable v_v_out     : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then

            -- =================================================================
            -- Stage 1: Center + rotate
            -- =================================================================
            v_cx := signed(resize(s_hcount, 12)) -
                    signed('0' & s_measured_h(11 downto 1));
            v_cy := signed(resize(s_vcount, 12)) -
                    signed('0' & s_measured_v(11 downto 1));

            -- Always correct aspect ratio for square output on 16:9
            v_cx := resize(shift_right(v_cx * to_signed(9, 5), 4), 12);

            s1_rx <= resize(shift_right(v_cx * s_cos - v_cy * s_sin, 9), 12);
            s1_ry <= resize(shift_right(v_cx * s_sin + v_cy * s_cos, 9), 12);

            -- =================================================================
            -- Stage 2: CORDIC setup
            -- =================================================================
            if s1_rx >= 0 then v_abs_x := unsigned(s1_rx);
            else               v_abs_x := unsigned(-s1_rx);
            end if;
            if s1_ry >= 0 then v_abs_y := unsigned(s1_ry);
            else               v_abs_y := unsigned(-s1_ry);
            end if;

            if v_abs_y > v_abs_x then
                cordic(0).x <= signed(resize(v_abs_y, 13));
                cordic(0).y <= signed(resize(v_abs_x, 13));
                cordic(0).swapped <= '1';
            else
                cordic(0).x <= signed(resize(v_abs_x, 13));
                cordic(0).y <= signed(resize(v_abs_y, 13));
                cordic(0).swapped <= '0';
            end if;
            cordic(0).z <= (others => '0');
            cordic(0).quadrant <= unsigned(s1_rx(11 downto 11)) &
                                  unsigned(s1_ry(11 downto 11));

            -- =================================================================
            -- Stages 3-6: CORDIC iterations 0-7 (2 per stage)
            -- =================================================================
            for stage in 0 to 3 loop
                v_cx0 := cordic(stage).x;
                v_cy0 := cordic(stage).y;
                v_cz0 := cordic(stage).z;

                if v_cy0 >= 0 then
                    v_cx1 := v_cx0 + shift_right(v_cy0, stage * 2);
                    v_cy1 := v_cy0 - shift_right(v_cx0, stage * 2);
                    v_cz1 := v_cz0 + C_ATAN(stage * 2);
                else
                    v_cx1 := v_cx0 - shift_right(v_cy0, stage * 2);
                    v_cy1 := v_cy0 + shift_right(v_cx0, stage * 2);
                    v_cz1 := v_cz0 - C_ATAN(stage * 2);
                end if;

                if v_cy1 >= 0 then
                    v_cx2 := v_cx1 + shift_right(v_cy1, stage * 2 + 1);
                    v_cy2 := v_cy1 - shift_right(v_cx1, stage * 2 + 1);
                    v_cz2 := v_cz1 + C_ATAN(stage * 2 + 1);
                else
                    v_cx2 := v_cx1 - shift_right(v_cy1, stage * 2 + 1);
                    v_cy2 := v_cy1 + shift_right(v_cx1, stage * 2 + 1);
                    v_cz2 := v_cz1 - C_ATAN(stage * 2 + 1);
                end if;

                cordic(stage + 1).x <= v_cx2;
                cordic(stage + 1).y <= v_cy2;
                cordic(stage + 1).z <= v_cz2;
                cordic(stage + 1).quadrant <= cordic(stage).quadrant;
                cordic(stage + 1).swapped  <= cordic(stage).swapped;
            end loop;

            -- =================================================================
            -- Stage 7: Angle + petal + color
            -- =================================================================

            -- Reconstruct full-circle angle from CORDIC output
            if cordic(4).swapped = '1' then
                v_base_z := to_unsigned(256, 10) - cordic(4).z;
            else
                v_base_z := cordic(4).z;
            end if;

            case to_integer(cordic(4).quadrant) is
                when 0      => v_full_angle := v_base_z;
                when 2      => v_full_angle := to_unsigned(512, 10) - v_base_z;
                when 3      => v_full_angle := to_unsigned(512, 10) + v_base_z;
                when others => v_full_angle := (not v_base_z) + 1;
            end case;

            -- N * angle
            v_n_angle := resize(v_full_angle * resize(r_petals, 10), 10);

            -- Pixel radius from CORDIC (scaled by K ~= 1.647)
            if cordic(4).x >= 0 then
                v_pixel_rad := unsigned(cordic(4).x(11 downto 0));
            else
                v_pixel_rad := (others => '0');
            end if;

            -- Twist: radius-dependent angle offset for spiral arms
            if r_twist >= 0 then
                v_twist_mag := unsigned(r_twist);
            else
                v_twist_mag := unsigned(-r_twist);
            end if;
            v_twist_prod := v_twist_mag * unsigned(v_pixel_rad(11 downto 2));
            v_twist_offs := resize(shift_right(v_twist_prod, 6), 10);
            if r_twist >= 0 then
                v_n_angle := v_n_angle + v_twist_offs;
            else
                v_n_angle := v_n_angle - v_twist_offs;
            end if;

            -- Concentric: wrap radius; animate outward pulse when both on
            if r_concentric = '1' then
                if r_animate = '1' then
                    v_pixel_rad := resize(
                        unsigned(cordic(4).x(8 downto 0)) +
                        resize(r_frame(7 downto 0), 9), 12);
                else
                    v_pixel_rad := resize(unsigned(cordic(4).x(8 downto 0)), 12);
                end if;
            end if;

            -- Petal function: triangle wave ~ |cos(N*theta)|
            if v_n_angle(8) = '0' then
                v_petal_wave := to_unsigned(255, 8) - unsigned(v_n_angle(7 downto 0));
            else
                v_petal_wave := unsigned(v_n_angle(7 downto 0));
            end if;

            -- Star mode: square the wave for pointy petals with wider gaps
            if r_star = '1' then
                v_star_sq := v_petal_wave * v_petal_wave;
                v_petal_wave := unsigned(v_star_sq(15 downto 8));
            end if;

            -- Petal boundary = (size * petal_wave) >> 8
            v_petal_prod := resize(r_size, 10) * resize(v_petal_wave, 8);
            v_petal_rad  := resize(shift_right(v_petal_prod, 8), 12);

            -- Curve hit test
            v_on_curve := '0';
            if r_fill = '1' then
                -- Fill: Width knob sets inner radius (hypotrochoid donut)
                v_inner := resize(shift_left(resize(r_width, 12), 3), 12);
                if v_pixel_rad >= v_inner and v_pixel_rad <= v_petal_rad then
                    v_on_curve := '1';
                end if;
            else
                -- Outline: check distance to petal boundary
                if v_pixel_rad > v_petal_rad then
                    v_diff := v_pixel_rad - v_petal_rad;
                else
                    v_diff := v_petal_rad - v_pixel_rad;
                end if;
                if v_diff <= resize(r_width, 12) + 2 then
                    v_on_curve := '1';
                end if;
            end if;

            -- Invert: swap foreground/background
            if r_invert = '1' then
                v_on_curve := not v_on_curve;
            end if;

            -- Color inputs
            v_angle_hue := unsigned(v_full_angle(9 downto 2));
            v_petal_hue := unsigned(v_n_angle(9 downto 2));
            v_rad_hue   := resize(shift_right(v_pixel_rad, 3), 8);

            -- =============================================================
            -- Color palettes
            -- =============================================================
            case to_integer(r_palette) is

                when 0 =>  -- White on black
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(1023, 10);
                    else
                        v_y_out := (others => '0');
                    end if;
                    v_u_out := C_CHROMA_MID;
                    v_v_out := C_CHROMA_MID;

                when 1 =>  -- Solar flare (yellow-orange, angle-modulated)
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(800, 10) +
                                   shift_right(resize(v_angle_hue, 10), 1);
                        v_u_out := to_unsigned(100, 10) +
                                   shift_right(resize(v_angle_hue, 10), 2);
                        v_v_out := to_unsigned(700, 10) +
                                   shift_right(resize(v_angle_hue, 10), 1);
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 2 =>  -- Copper shimmer (warm metallic, angle modulated)
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(500, 10) +
                                   resize(v_angle_hue, 10);
                        v_u_out := to_unsigned(400, 10);
                        v_v_out := to_unsigned(640, 10);
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 3 =>  -- Petal rainbow (each petal a different hue)
                    if v_on_curve = '1' then
                        v_hue := v_petal_hue;
                        v_y_out := to_unsigned(600, 10);
                        if v_hue < 85 then
                            v_u_out := C_CHROMA_MID - resize(v_hue & "00", 10);
                            v_v_out := to_unsigned(800, 10);
                        elsif v_hue < 170 then
                            v_u_out := C_CHROMA_MID +
                                       resize((v_hue - 85) & "00", 10);
                            v_v_out := to_unsigned(224, 10);
                        else
                            v_u_out := to_unsigned(800, 10);
                            v_v_out := to_unsigned(224, 10) +
                                       resize((v_hue - 170) & "00", 10);
                        end if;
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 4 =>  -- Neon green on black
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(676, 10);
                        v_u_out := to_unsigned(176, 10);
                        v_v_out := to_unsigned(192, 10);
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 5 =>  -- Hot magenta on black
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(422, 10);
                        v_u_out := to_unsigned(851, 10);
                        v_v_out := to_unsigned(941, 10);
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 6 =>  -- Cyan on midnight
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(717, 10);
                        v_u_out := to_unsigned(685, 10);
                        v_v_out := to_unsigned(0, 10);
                    else
                        v_y_out := to_unsigned(16, 10);
                        v_u_out := to_unsigned(524, 10);
                        v_v_out := to_unsigned(508, 10);
                    end if;

                when 7 =>  -- Gold on crimson
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(877, 10);
                        v_u_out := to_unsigned(113, 10);
                        v_v_out := to_unsigned(559, 10);
                    else
                        v_y_out := to_unsigned(50, 10);
                        v_u_out := to_unsigned(490, 10);
                        v_v_out := to_unsigned(570, 10);
                    end if;

                when 8 =>  -- Electric blue on black
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(400, 10);
                        v_u_out := to_unsigned(800, 10);
                        v_v_out := to_unsigned(350, 10);
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 9 =>  -- Plasma (purple-to-pink by angle)
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(400, 10);
                        v_u_out := to_unsigned(650, 10) +
                                   shift_right(resize(v_angle_hue, 10), 2);
                        v_v_out := to_unsigned(600, 10) +
                                   shift_right(resize(v_angle_hue, 10), 1);
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 10 =>  -- Duotone petals (cyan + magenta alternating)
                    if v_on_curve = '1' then
                        if v_n_angle(9) = '0' then
                            v_y_out := to_unsigned(717, 10);
                            v_u_out := to_unsigned(685, 10);
                            v_v_out := to_unsigned(0, 10);
                        else
                            v_y_out := to_unsigned(422, 10);
                            v_u_out := to_unsigned(851, 10);
                            v_v_out := to_unsigned(941, 10);
                        end if;
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 11 =>  -- Warm angle (red -> orange -> yellow by angle)
                    if v_on_curve = '1' then
                        v_hue := v_angle_hue;
                        v_y_out := to_unsigned(400, 10) +
                                   resize(v_hue & "0", 10);
                        v_u_out := to_unsigned(300, 10) -
                                   shift_right(resize(v_hue, 10), 2);
                        v_v_out := to_unsigned(700, 10) +
                                   shift_right(resize(v_hue, 10), 1);
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when 12 =>  -- Emerald tint
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(601, 10);
                        v_u_out := to_unsigned(173, 10);
                        v_v_out := to_unsigned(83, 10);
                    else
                        v_y_out := to_unsigned(20, 10);
                        v_u_out := to_unsigned(490, 10);
                        v_v_out := to_unsigned(470, 10);
                    end if;

                when 13 =>  -- Violet glow
                    if v_on_curve = '1' then
                        v_y_out := to_unsigned(500, 10);
                        v_u_out := to_unsigned(750, 10);
                        v_v_out := to_unsigned(700, 10);
                    else
                        v_y_out := to_unsigned(16, 10);
                        v_u_out := to_unsigned(540, 10);
                        v_v_out := to_unsigned(530, 10);
                    end if;

                when 14 =>  -- Candy stripes (banding within each petal)
                    if v_on_curve = '1' then
                        if v_petal_wave(5) = '0' then
                            v_y_out := to_unsigned(800, 10);
                            v_u_out := to_unsigned(300, 10);
                            v_v_out := to_unsigned(700, 10);
                        else
                            v_y_out := to_unsigned(900, 10);
                            v_u_out := to_unsigned(700, 10);
                            v_v_out := to_unsigned(400, 10);
                        end if;
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;

                when others =>  -- Deep rainbow (darker, more saturated)
                    if v_on_curve = '1' then
                        v_hue := v_angle_hue;
                        v_y_out := to_unsigned(400, 10);
                        if v_hue < 85 then
                            v_u_out := C_CHROMA_MID - resize(v_hue & "00", 10);
                            v_v_out := to_unsigned(900, 10);
                        elsif v_hue < 170 then
                            v_u_out := C_CHROMA_MID +
                                       resize((v_hue - 85) & "00", 10);
                            v_v_out := to_unsigned(124, 10);
                        else
                            v_u_out := to_unsigned(900, 10);
                            v_v_out := to_unsigned(124, 10) +
                                       resize((v_hue - 170) & "00", 10);
                        end if;
                    else
                        v_y_out := (others => '0');
                        v_u_out := C_CHROMA_MID;
                        v_v_out := C_CHROMA_MID;
                    end if;
            end case;

            s_out_y <= v_y_out;
            s_out_u <= v_u_out;
            s_out_v <= v_v_out;
        end if;
    end process p_render;

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
    data_out.y <= std_logic_vector(s_out_y);
    data_out.u <= std_logic_vector(s_out_u);
    data_out.v <= std_logic_vector(s_out_v);

end architecture flower;
