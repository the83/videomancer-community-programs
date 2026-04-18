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
--   coherence, density, and speed controls. Ported from the horiz_pixel_noise
--   GLSL shader (the83/shaderz).
--
--   Generates cell-based pseudo-random noise using combinational XOR hashing
--   with a free-running LFSR for temporal variation. Cell sizes are controlled
--   by coherence knobs via power-of-2 bit shifting. Density is applied as a
--   threshold + softness + boost stage. Animation is driven by a frame phase
--   accumulator with temporal interpolation between frames.
--
--   This is a synthesis program — input video is ignored, output is generated
--   noise (grayscale: Y = noise, U = V = 512).
--
-- Resources:
--   0 BRAM, ~800 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (cell computation):                 1 clock  -> T+1
--   Stage 1 (hash computation):                 1 clock  -> T+2
--   Stage 2 (two-layer mixing):                 1 clock  -> T+3
--   Stage 3 (temporal blend):                   1 clock  -> T+4
--   Stage 4 (threshold + density + output):     1 clock  -> T+5
--   Total: 5 clocks
--
-- Submodules:
--   video_timing_generator: sync edge detection
--   frame_phase_accumulator: animation phase DDS
--   lfsr16: free-running pseudo-random noise
--
-- Parameters:
--   Pot 1  (registers_in(0)):   H Coherence (cell width, 0=per-pixel, 1023=wide)
--   Pot 2  (registers_in(1)):   V Coherence (cell height, 0=per-pixel, 1023=tall)
--   Pot 3  (registers_in(2)):   Density (0=sparse, 1023=full snow)
--   Pot 4  (registers_in(3)):   Speed (animation rate)

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

    constant C_SYNC_DELAY_CLKS : integer := 5;

    -- Video timing
    signal s_timing : t_video_timing_port;

    -- Pixel counters (12-bit, supports up to 4096)
    signal s_hcount : unsigned(11 downto 0) := (others => '0');
    signal s_vcount : unsigned(11 downto 0) := (others => '0');

    -- Frame phase accumulator (20-bit: top 10 = frame number, bottom 10 = fraction)
    signal s_phase : unsigned(19 downto 0);

    -- Free-running LFSR
    signal s_lfsr_q : std_logic_vector(15 downto 0);

    -- Vsync-latched parameters
    signal r_h_shift   : natural range 0 to 10 := 0;
    signal r_v_shift   : natural range 0 to 10 := 0;
    signal r_density   : unsigned(9 downto 0)  := to_unsigned(512, 10);
    signal r_threshold : unsigned(9 downto 0)  := (others => '0');
    signal r_softness  : natural range 0 to 10 := 5;
    signal r_boost_en  : std_logic             := '0';
    signal s_prev_vsync : std_logic            := '1';

    -- Stage 0 outputs: cell indices
    signal s0_h_cell : unsigned(11 downto 0) := (others => '0');
    signal s0_v_cell : unsigned(11 downto 0) := (others => '0');

    -- Stage 1 outputs: hash values (4 values: 2 layers x 2 frames)
    signal s1_noise_a0 : unsigned(9 downto 0) := (others => '0');
    signal s1_noise_b0 : unsigned(9 downto 0) := (others => '0');
    signal s1_noise_a1 : unsigned(9 downto 0) := (others => '0');
    signal s1_noise_b1 : unsigned(9 downto 0) := (others => '0');

    -- Stage 2 outputs: mixed noise for each temporal frame
    signal s2_mixed_0 : unsigned(9 downto 0) := (others => '0');
    signal s2_mixed_1 : unsigned(9 downto 0) := (others => '0');

    -- Stage 3 output: temporally blended noise
    signal s3_blended : unsigned(9 downto 0) := (others => '0');

    -- Stage 4 output: thresholded/density-controlled noise
    signal s4_y : unsigned(9 downto 0) := (others => '0');

    -- Helper function: XOR hash with rotate
    function xor_hash(seed : unsigned(11 downto 0);
                      salt : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        v := seed;
        v := v xor (v(8 downto 0) & v(11 downto 9));   -- rotate left 3
        v := v xor (v(4 downto 0) & v(11 downto 5));   -- rotate left 7
        v := v xor resize(salt, 12);
        v := v xor (v(6 downto 0) & v(11 downto 7));   -- rotate left 5
        return v(9 downto 0);
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
    -- Frame Phase Accumulator (animation speed)
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
    -- Free-running LFSR16
    -- =========================================================================
    lfsr_inst : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr_q
        );

    -- =========================================================================
    -- Parameter latch on vsync (prevents mid-frame tearing)
    -- =========================================================================
    p_param_latch : process(clk)
        variable v_h_raw : natural;
        variable v_v_raw : natural;
        variable v_density : unsigned(9 downto 0);
        variable v_thresh_prod : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                -- H coherence: top 4 bits -> shift 0..10
                v_h_raw := to_integer(unsigned(registers_in(0)(9 downto 6)));
                if v_h_raw > 10 then
                    r_h_shift <= 10;
                else
                    r_h_shift <= v_h_raw;
                end if;

                -- V coherence: top 4 bits -> shift 0..10
                v_v_raw := to_integer(unsigned(registers_in(1)(9 downto 6)));
                if v_v_raw > 10 then
                    r_v_shift <= 10;
                else
                    r_v_shift <= v_v_raw;
                end if;

                -- Density
                v_density := unsigned(registers_in(2)(9 downto 0));
                r_density <= v_density;

                -- Precompute threshold: maps density 0->768, 1023->0
                -- threshold = 768 - density * 3 / 4
                -- Use shift-add: density*3 = density + density<<1, then >>2
                v_thresh_prod := resize(v_density, 20) + resize(v_density & "0", 20);
                r_threshold <= to_unsigned(768, 10) -
                               v_thresh_prod(11 downto 2);

                -- Precompute softness shift: maps density 0->3 (sharp), 1023->0 (wide)
                if v_density < 256 then
                    r_softness <= 3;
                elsif v_density < 512 then
                    r_softness <= 2;
                elsif v_density < 768 then
                    r_softness <= 1;
                else
                    r_softness <= 0;
                end if;

                -- Boost enabled for sparse noise (low density)
                if v_density < 341 then
                    r_boost_en <= '1';
                else
                    r_boost_en <= '0';
                end if;
            end if;
        end if;
    end process p_param_latch;

    -- =========================================================================
    -- Stage 0 (T+1): Cell computation via barrel shifter
    -- =========================================================================
    p_stage0 : process(clk)
        variable v_h_cell : unsigned(11 downto 0);
        variable v_v_cell : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            case r_h_shift is
                when 0  => v_h_cell := s_hcount;
                when 1  => v_h_cell := "0" & s_hcount(11 downto 1);
                when 2  => v_h_cell := "00" & s_hcount(11 downto 2);
                when 3  => v_h_cell := "000" & s_hcount(11 downto 3);
                when 4  => v_h_cell := "0000" & s_hcount(11 downto 4);
                when 5  => v_h_cell := "00000" & s_hcount(11 downto 5);
                when 6  => v_h_cell := "000000" & s_hcount(11 downto 6);
                when 7  => v_h_cell := "0000000" & s_hcount(11 downto 7);
                when 8  => v_h_cell := "00000000" & s_hcount(11 downto 8);
                when 9  => v_h_cell := "000000000" & s_hcount(11 downto 9);
                when 10 => v_h_cell := "0000000000" & s_hcount(11 downto 10);
                when others => v_h_cell := s_hcount;
            end case;

            case r_v_shift is
                when 0  => v_v_cell := s_vcount;
                when 1  => v_v_cell := "0" & s_vcount(11 downto 1);
                when 2  => v_v_cell := "00" & s_vcount(11 downto 2);
                when 3  => v_v_cell := "000" & s_vcount(11 downto 3);
                when 4  => v_v_cell := "0000" & s_vcount(11 downto 4);
                when 5  => v_v_cell := "00000" & s_vcount(11 downto 5);
                when 6  => v_v_cell := "000000" & s_vcount(11 downto 6);
                when 7  => v_v_cell := "0000000" & s_vcount(11 downto 7);
                when 8  => v_v_cell := "00000000" & s_vcount(11 downto 8);
                when 9  => v_v_cell := "000000000" & s_vcount(11 downto 9);
                when 10 => v_v_cell := "0000000000" & s_vcount(11 downto 10);
                when others => v_v_cell := s_vcount;
            end case;

            s0_h_cell <= v_h_cell;
            s0_v_cell <= v_v_cell;
        end if;
    end process p_stage0;

    -- =========================================================================
    -- Stage 1 (T+2): Hash computation
    --   4 hash values: 2 layers x 2 temporal frames
    --   XOR with LFSR for additional randomness
    -- =========================================================================
    p_stage1 : process(clk)
        variable v_seed     : unsigned(11 downto 0);
        variable v_frame_n  : unsigned(9 downto 0);
        variable v_frame_n1 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_frame_n  := s_phase(19 downto 10);
            v_frame_n1 := s_phase(19 downto 10) + 1;

            -- Base seed: cell_x XOR cell_y
            v_seed := s0_h_cell xor s0_v_cell;

            -- Layer A, frame N: hash with frame_n and salt A
            s1_noise_a0 <= xor_hash(v_seed xor resize(v_frame_n, 12),
                                    "1010010110")  -- salt 0xA5B
                           xor unsigned(s_lfsr_q(9 downto 0));

            -- Layer B, frame N: hash with frame_n and salt B
            s1_noise_b0 <= xor_hash(v_seed xor resize(v_frame_n, 12),
                                    "0111110001")  -- salt 0x3E1
                           xor unsigned(s_lfsr_q(15 downto 6));

            -- Layer A, frame N+1: hash with frame_n+1 and salt A
            s1_noise_a1 <= xor_hash(v_seed xor resize(v_frame_n1, 12),
                                    "1010010110")
                           xor unsigned(s_lfsr_q(9 downto 0));

            -- Layer B, frame N+1: hash with frame_n+1 and salt B
            s1_noise_b1 <= xor_hash(v_seed xor resize(v_frame_n1, 12),
                                    "0111110001")
                           xor unsigned(s_lfsr_q(15 downto 6));
        end if;
    end process p_stage1;

    -- =========================================================================
    -- Stage 2 (T+3): Two-layer mixing
    --   Approximates shader's noise * 0.6 + noise2 * 0.4
    --   Using 5/8 * a + 3/8 * b = (a>>1) + (a>>3) + (b>>2) + (b>>3)
    -- =========================================================================
    p_stage2 : process(clk)
        variable v_mix0 : unsigned(11 downto 0);
        variable v_mix1 : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- Frame N mix
            v_mix0 := resize(shift_right(s1_noise_a0, 1), 12) +
                       resize(shift_right(s1_noise_a0, 3), 12) +
                       resize(shift_right(s1_noise_b0, 2), 12) +
                       resize(shift_right(s1_noise_b0, 3), 12);
            if v_mix0 > 1023 then
                s2_mixed_0 <= to_unsigned(1023, 10);
            else
                s2_mixed_0 <= v_mix0(9 downto 0);
            end if;

            -- Frame N+1 mix
            v_mix1 := resize(shift_right(s1_noise_a1, 1), 12) +
                       resize(shift_right(s1_noise_a1, 3), 12) +
                       resize(shift_right(s1_noise_b1, 2), 12) +
                       resize(shift_right(s1_noise_b1, 3), 12);
            if v_mix1 > 1023 then
                s2_mixed_1 <= to_unsigned(1023, 10);
            else
                s2_mixed_1 <= v_mix1(9 downto 0);
            end if;
        end if;
    end process p_stage2;

    -- =========================================================================
    -- Stage 3 (T+4): Temporal blend between frame N and frame N+1
    --   blended = mixed_0 + (mixed_1 - mixed_0) * frame_frac / 1024
    -- =========================================================================
    p_stage3 : process(clk)
        variable v_diff    : signed(10 downto 0);
        variable v_frac    : unsigned(9 downto 0);
        variable v_product : signed(21 downto 0);
        variable v_result  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_diff := signed(resize(s2_mixed_1, 11)) -
                      signed(resize(s2_mixed_0, 11));
            v_frac := s_phase(9 downto 0);
            v_product := v_diff * signed('0' & v_frac);
            v_result := signed(resize(s2_mixed_0, 12)) +
                        resize(v_product(20 downto 10), 12);

            if v_result < 0 then
                s3_blended <= (others => '0');
            elsif v_result > 1023 then
                s3_blended <= to_unsigned(1023, 10);
            else
                s3_blended <= unsigned(v_result(9 downto 0));
            end if;
        end if;
    end process p_stage3;

    -- =========================================================================
    -- Stage 4 (T+5): Threshold + softness + boost -> Y output
    --   Grayscale noise: Y = processed noise, U = V = 512 (neutral chroma)
    -- =========================================================================
    p_stage4 : process(clk)
        variable v_diff   : signed(10 downto 0);
        variable v_shifted : unsigned(9 downto 0);
        variable v_boosted : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- Subtract threshold, clamp to 0
            v_diff := signed(resize(s3_blended, 11)) -
                      signed(resize(r_threshold, 11));

            if v_diff <= 0 then
                s4_y <= (others => '0');
            else
                -- Apply softness: shift left to widen the visible range
                case r_softness is
                    when 0  => v_shifted := unsigned(v_diff(9 downto 0));
                    when 1  => if v_diff > 511 then v_shifted := to_unsigned(1023, 10);
                               else v_shifted := unsigned(v_diff(8 downto 0)) & "0"; end if;
                    when 2  => if v_diff > 255 then v_shifted := to_unsigned(1023, 10);
                               else v_shifted := unsigned(v_diff(7 downto 0)) & "00"; end if;
                    when 3  => if v_diff > 127 then v_shifted := to_unsigned(1023, 10);
                               else v_shifted := unsigned(v_diff(6 downto 0)) & "000"; end if;
                    when others => v_shifted := unsigned(v_diff(9 downto 0));
                end case;

                -- Apply boost for sparse noise (multiply by ~2.5 = x + x>>1)
                if r_boost_en = '1' then
                    v_boosted := resize(v_shifted, 11) +
                                 resize(shift_right(v_shifted, 1), 11);
                    if v_boosted > 1023 then
                        s4_y <= to_unsigned(1023, 10);
                    else
                        s4_y <= v_boosted(9 downto 0);
                    end if;
                else
                    s4_y <= v_shifted;
                end if;
            end if;
        end if;
    end process p_stage4;

    -- =========================================================================
    -- Sync delay line (match processing latency)
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
    -- Output: generated noise (grayscale)
    -- =========================================================================
    data_out.y <= std_logic_vector(s4_y);
    data_out.u <= std_logic_vector(to_unsigned(512, 10));
    data_out.v <= std_logic_vector(to_unsigned(512, 10));

end architecture pixel_noise;
