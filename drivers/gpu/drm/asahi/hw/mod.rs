// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Hardware configuration

use crate::driver::AsahiDevice;
use crate::fw::types::*;
use alloc::vec::Vec;
use kernel::c_str;
use kernel::device::RawDevice;
use kernel::prelude::*;

const MAX_POWERZONES: usize = 5;

pub(crate) mod t600x;
pub(crate) mod t8103;
pub(crate) mod t8112;

/* Note: This is a firmware-relevant ID */
#[derive(Debug, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuCore {
    // Unknown = 0,
    // G5P = 1,
    // G5G = 2,
    // G9P = 3,
    // G9G = 4,
    // G10P = 5,
    // G11P = 6,
    // G11M = 7,
    // G11G = 8,
    // G12P = 9,
    // G13P = 10,
    G13G = 11,
    G13S = 12,
    G13C = 13,
    // G14P = 14,
    G14G = 15,
}

#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuGen {
    G13 = 13,
    G14 = 14,
}

#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuVariant {
    P = 'P' as u32,
    G = 'G' as u32,
    S = 'S' as u32,
    C = 'C' as u32,
    D = 'D' as u32,
}

#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuRevision {
    A0 = 0x00,
    A1 = 0x01,
    B0 = 0x10,
    B1 = 0x11,
    C0 = 0x20,
    C1 = 0x21,
}

#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u32)]
pub(crate) enum GpuRevisionID {
    // Unknown = 0,
    A0 = 1,
    A1 = 2,
    B0 = 3,
    B1 = 4,
    C0 = 5,
    C1 = 6,
}

pub(crate) mod feat {
    pub(crate) mod compat {}
    pub(crate) mod incompat {
        use kernel::bindings;

        pub(crate) const MANDATORY_ZS_COMPRESSION: u64 =
            bindings::drm_asahi_feat_incompat_DRM_ASAHI_FEAT_MANDATORY_ZS_COMPRESSION as u64;
    }
}

#[derive(Debug)]
pub(crate) struct PState {
    pub(crate) volt_mv: Vec<u32>,
    pub(crate) freq_hz: u32,
    pub(crate) pwr_mw: u32,
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct PowerZone {
    pub(crate) target: u32,
    pub(crate) target_offset: u32,
    pub(crate) filter_tc: u32,
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct IOMapping {
    pub(crate) base: usize,
    pub(crate) size: usize,
    pub(crate) range_size: usize,
    pub(crate) writable: bool,
}

impl IOMapping {
    pub(crate) const fn new(
        base: usize,
        size: usize,
        range_size: usize,
        writable: bool,
    ) -> IOMapping {
        IOMapping {
            base,
            size,
            range_size,
            writable,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct HwConfigA {
    pub(crate) unk_87c: i32,
    pub(crate) unk_8cc: u32,
    pub(crate) unk_e24: u32,
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct HwConfigB {
    pub(crate) unk_4e0: u64,
    pub(crate) unk_534: u32,
    pub(crate) unk_ab8: u32,
    pub(crate) unk_abc: u32,
    pub(crate) unk_b30: u32,
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct HwRenderConfig {
    pub(crate) tiling_control: u32,
}

#[derive(Debug)]
pub(crate) struct HwConfig {
    pub(crate) chip_id: u32,
    pub(crate) gpu_gen: GpuGen,
    pub(crate) gpu_variant: GpuVariant,
    pub(crate) gpu_core: GpuCore,
    pub(crate) gpu_feat_compat: u64,
    pub(crate) gpu_feat_incompat: u64,

    pub(crate) base_clock_hz: u32,
    pub(crate) uat_oas: usize,
    pub(crate) max_num_clusters: u32,
    pub(crate) max_num_cores: u32,
    pub(crate) max_num_frags: u32,
    pub(crate) max_num_gps: u32,

    pub(crate) preempt1_size: usize,
    pub(crate) preempt2_size: usize,
    pub(crate) preempt3_size: usize,

    pub(crate) render: HwRenderConfig,

    pub(crate) da: HwConfigA,
    pub(crate) db: HwConfigB,
    pub(crate) shared1_tab: &'static [i32],
    pub(crate) shared1_a4: u32,
    pub(crate) shared2_tab: &'static [i32],
    pub(crate) shared2_unk_508: u32,
    pub(crate) sram_k: F32,
    pub(crate) unk_coef_a: &'static [&'static [F32]],
    pub(crate) unk_coef_b: &'static [&'static [F32]],
    pub(crate) global_tab: Option<&'static [u8]>,

    pub(crate) fast_die0_sensor_mask: u64,
    pub(crate) fast_die0_sensor_mask_alt: u64,
    pub(crate) fast_die0_sensor_present: u32,
    pub(crate) io_mappings: &'static [Option<IOMapping>],
}

#[derive(Debug)]
pub(crate) struct DynConfig {
    pub(crate) uat_ttb_base: u64,
    pub(crate) id: GpuIdConfig,
    pub(crate) pwr: PwrConfig,
}

#[derive(Debug)]
pub(crate) struct GpuIdConfig {
    pub(crate) gpu_gen: GpuGen,
    pub(crate) gpu_variant: GpuVariant,
    pub(crate) gpu_rev: GpuRevision,
    pub(crate) gpu_rev_id: GpuRevisionID,
    pub(crate) max_dies: u32,
    pub(crate) num_clusters: u32,
    pub(crate) num_cores: u32,
    pub(crate) num_active_cores: u32,
    pub(crate) num_frags: u32,
    pub(crate) num_gps: u32,
    pub(crate) core_masks: Vec<u32>,
    pub(crate) core_masks_packed: Vec<u32>,
}

#[derive(Debug)]
pub(crate) struct PwrConfig {
    pub(crate) perf_states: Vec<PState>,
    pub(crate) power_zones: Vec<PowerZone>,

    pub(crate) core_leak_coef: Vec<F32>,
    pub(crate) sram_leak_coef: Vec<F32>,

    pub(crate) max_power_mw: u32,
    pub(crate) max_freq_mhz: u32,

    pub(crate) perf_base_pstate: u32,
    pub(crate) perf_max_pstate: u32,

    pub(crate) min_sram_microvolt: u32,
    pub(crate) avg_power_filter_tc_ms: u32,
    pub(crate) avg_power_ki_only: F32,
    pub(crate) avg_power_kp: F32,
    pub(crate) avg_power_min_duty_cycle: u32,
    pub(crate) avg_power_target_filter_tc: u32,
    pub(crate) fast_die0_integral_gain: F32,
    pub(crate) fast_die0_proportional_gain: F32,
    pub(crate) fast_die0_prop_tgt_delta: u32,
    pub(crate) fast_die0_release_temp: u32,
    pub(crate) fender_idle_off_delay_ms: u32,
    pub(crate) fw_early_wake_timeout_ms: u32,
    pub(crate) idle_off_delay_ms: u32,
    pub(crate) perf_boost_min_util: u32,
    pub(crate) perf_boost_ce_step: u32,
    pub(crate) perf_integral_gain: F32,
    pub(crate) perf_proportional_gain: F32,
    pub(crate) perf_reset_iters: u32,
    pub(crate) perf_filter_drop_threshold: u32,
    pub(crate) perf_filter_time_constant: u32,
    pub(crate) perf_filter_time_constant2: u32,
    pub(crate) perf_integral_gain2: F32,
    pub(crate) perf_integral_min_clamp: u32,
    pub(crate) perf_proportional_gain2: F32,
    pub(crate) perf_tgt_utilization: u32,
    pub(crate) power_sample_period: u32,
    pub(crate) ppm_filter_time_constant_ms: u32,
    pub(crate) ppm_ki: F32,
    pub(crate) ppm_kp: F32,
    pub(crate) pwr_filter_time_constant: u32,
    pub(crate) pwr_integral_gain: F32,
    pub(crate) pwr_integral_min_clamp: u32,
    pub(crate) pwr_min_duty_cycle: u32,
    pub(crate) pwr_proportional_gain: F32,
}

impl PwrConfig {
    pub(crate) fn load(dev: &AsahiDevice, cfg: &HwConfig) -> Result<PwrConfig> {
        let mut perf_states = Vec::new();

        let node = dev.of_node().ok_or(EIO)?;
        let opps = node
            .parse_phandle(c_str!("operating-points-v2"), 0)
            .ok_or(EIO)?;

        let mut max_power_mw: u32 = 0;
        let mut max_freq_mhz: u32 = 0;

        macro_rules! prop {
            ($prop:expr, $default:expr) => {{
                node.get_opt_property(c_str!($prop))
                    .map_err(|e| {
                        dev_err!(dev, "Error reading property {}: {:?}", $prop, e);
                        e
                    })?
                    .unwrap_or($default)
            }};
            ($prop:expr) => {{
                node.get_property(c_str!($prop)).map_err(|e| {
                    dev_err!(dev, "Error reading property {}: {:?}", $prop, e);
                    e
                })?
            }};
        }

        for opp in opps.children() {
            let freq_hz: u64 = opp.get_property(c_str!("opp-hz"))?;
            let mut volt_uv: Vec<u32> = opp.get_property(c_str!("opp-microvolt"))?;
            let pwr_uw: u32 = opp.get_property(c_str!("opp-microwatt"))?;

            if volt_uv.len() != cfg.max_num_clusters as usize {
                dev_err!(
                    dev,
                    "Invalid opp-microvolt length (expected {}, got {})",
                    cfg.max_num_clusters,
                    volt_uv.len()
                );
                return Err(EINVAL);
            }

            volt_uv.iter_mut().for_each(|a| *a /= 1000);
            let volt_mv = volt_uv;

            let pwr_mw = pwr_uw / 1000;
            max_power_mw = max_power_mw.max(pwr_mw);

            let freq_mhz: u32 = (freq_hz / 1_000_000).try_into()?;
            max_freq_mhz = max_freq_mhz.max(freq_mhz);

            perf_states.try_push(PState {
                freq_hz: freq_hz.try_into()?,
                volt_mv,
                pwr_mw,
            })?;
        }

        let pz_data = prop!("apple,power-zones", Vec::new());

        if pz_data.len() > 3 * MAX_POWERZONES || pz_data.len() % 3 != 0 {
            dev_err!(dev, "Invalid apple,power-zones value");
            return Err(EINVAL);
        }

        let pz_count = pz_data.len() / 3;
        let mut power_zones = Vec::new();
        for i in (0..pz_count).step_by(3) {
            power_zones.try_push(PowerZone {
                target: pz_data[i],
                target_offset: pz_data[i + 1],
                filter_tc: pz_data[i + 2],
            })?;
        }

        let core_leak_coef: Vec<F32> = prop!("apple,core-leak-coef");
        let sram_leak_coef: Vec<F32> = prop!("apple,sram-leak-coef");

        if core_leak_coef.len() != cfg.max_num_clusters as usize {
            dev_err!(dev, "Invalid apple,core-leak-coef");
            return Err(EINVAL);
        }
        if sram_leak_coef.len() != cfg.max_num_clusters as usize {
            dev_err!(dev, "Invalid apple,sram_leak_coef");
            return Err(EINVAL);
        }

        Ok(PwrConfig {
            core_leak_coef,
            sram_leak_coef,

            max_power_mw,
            max_freq_mhz,

            perf_base_pstate: prop!("apple,perf-base-pstate", 1),
            perf_max_pstate: perf_states.len() as u32 - 1,
            min_sram_microvolt: prop!("apple,min-sram-microvolt"),

            avg_power_filter_tc_ms: prop!("apple,avg-power-filter-tc-ms"),
            avg_power_ki_only: prop!("apple,avg-power-ki-only"),
            avg_power_kp: prop!("apple,avg-power-kp"),
            avg_power_min_duty_cycle: prop!("apple,avg-power-min-duty-cycle"),
            avg_power_target_filter_tc: prop!("apple,avg-power-target-filter-tc"),
            fast_die0_integral_gain: prop!("apple,fast-die0-integral-gain"),
            fast_die0_proportional_gain: prop!("apple,fast-die0-proportional-gain"),
            fast_die0_prop_tgt_delta: prop!("apple,fast-die0-prop-tgt-delta", 0),
            fast_die0_release_temp: prop!("apple,fast-die0-release-temp", 80),
            fender_idle_off_delay_ms: prop!("apple,fender-idle-off-delay-ms", 40),
            fw_early_wake_timeout_ms: prop!("apple,fw-early-wake-timeout-ms", 5),
            idle_off_delay_ms: prop!("apple,idle-off-delay-ms", 2),
            perf_boost_ce_step: prop!("apple,perf-boost-ce-step", 25),
            perf_boost_min_util: prop!("apple,perf-boost-min-util", 100),
            perf_filter_drop_threshold: prop!("apple,perf-filter-drop-threshold"),
            perf_filter_time_constant2: prop!("apple,perf-filter-time-constant2"),
            perf_filter_time_constant: prop!("apple,perf-filter-time-constant"),
            perf_integral_gain2: prop!("apple,perf-integral-gain2"),
            perf_integral_gain: prop!("apple,perf-integral-gain", f32!(7.8956833)),
            perf_integral_min_clamp: prop!("apple,perf-integral-min-clamp"),
            perf_proportional_gain2: prop!("apple,perf-proportional-gain2"),
            perf_proportional_gain: prop!("apple,perf-proportional-gain", f32!(14.707963)),
            perf_reset_iters: prop!("apple,perf-reset-iters", 6),
            perf_tgt_utilization: prop!("apple,perf-tgt-utilization"),
            power_sample_period: prop!("apple,power-sample-period"),
            ppm_filter_time_constant_ms: prop!("apple,ppm-filter-time-constant-ms"),
            ppm_ki: prop!("apple,ppm-ki"),
            ppm_kp: prop!("apple,ppm-kp"),
            pwr_filter_time_constant: prop!("apple,pwr-filter-time-constant", 313),
            pwr_integral_gain: prop!("apple,pwr-integral-gain", f32!(0.0202129)),
            pwr_integral_min_clamp: prop!("apple,pwr-integral-min-clamp", 0),
            pwr_min_duty_cycle: prop!("apple,pwr-min-duty-cycle"),
            pwr_proportional_gain: prop!("apple,pwr-proportional-gain", f32!(5.2831855)),

            perf_states,
            power_zones,
        })
    }
}
