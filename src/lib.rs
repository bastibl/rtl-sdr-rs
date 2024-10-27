//! # rtlsdr Library
//! Library for interfacing with an RTL-SDR device.

mod device;
pub mod error;
mod rtlsdr;
pub mod tuners;

use device::Device;
use device::KNOWN_DEVICES;
use error::Result;
use rtlsdr::RtlSdr as Sdr;

use rusb::{Context, UsbContext};

pub const DEFAULT_BUF_LENGTH: usize = 16 * 16384;

#[derive(Debug, Clone)]
pub enum TunerGain {
    Auto,
    Manual(i32),
}
#[derive(Debug, Clone, Copy)]
pub enum DirectSampleMode {
    Off,
    On,
    OnSwap, // Swap I and Q ADC, allowing to select between two inputs
}

#[derive(Debug, Clone)]
pub struct DeviceInfo {
    pub index: usize,
    pub serial: String,
}

pub fn enumerate() -> Result<Vec<DeviceInfo>> {
    let context = Context::new()?;
    let devices = context.devices()?;

    let mut devs = Vec::new();
    let mut index = 0;

    for found in devices.iter() {
        let device_desc = found.device_descriptor()?;
        for dev in KNOWN_DEVICES.iter() {
            if device_desc.vendor_id() == dev.vid && device_desc.product_id() == dev.pid {
                let dev = found.open()?;
                let serial = dev.read_serial_number_string_ascii(&device_desc)?;
                devs.push(DeviceInfo { index, serial });
                index += 1;
            }
        }
    }
    Ok(devs)
}

pub struct RtlSdr {
    sdr: Sdr,
}
impl RtlSdr {
    pub fn open(index: usize) -> Result<RtlSdr> {
        let dev = Device::new(index)?;
        let mut sdr = Sdr::new(dev);
        sdr.init()?;
        Ok(RtlSdr { sdr })
    }
    pub fn close(&mut self) -> Result<()> {
        // TODO: wait until async is inactive
        self.sdr.deinit_baseband()
    }
    pub fn reset_buffer(&self) -> Result<()> {
        self.sdr.reset_buffer()
    }
    pub fn read_sync(&self, buf: &mut [u8]) -> Result<usize> {
        self.sdr.read_sync(buf)
    }
    pub fn get_center_freq(&self) -> u32 {
        self.sdr.get_center_freq()
    }
    pub fn set_center_freq(&self, freq: u32) -> Result<()> {
        self.sdr.set_center_freq(freq)
    }
    pub fn get_tuner_gains(&self) -> Result<Vec<i32>> {
        self.sdr.get_tuner_gains()
    }
    pub fn set_tuner_gain(&self, gain: TunerGain) -> Result<()> {
        self.sdr.set_tuner_gain(gain)
    }
    pub fn get_freq_correction(&self) -> i32 {
        self.sdr.get_freq_correction()
    }
    pub fn set_freq_correction(&self, ppm: i32) -> Result<()> {
        self.sdr.set_freq_correction(ppm)
    }
    pub fn get_sample_rate(&self) -> u32 {
        self.sdr.get_sample_rate()
    }
    pub fn set_sample_rate(&self, rate: u32) -> Result<()> {
        self.sdr.set_sample_rate(rate)
    }
    pub fn set_tuner_bandwidth(&self, bw: u32) -> Result<()> {
        self.sdr.set_tuner_bandwidth(bw)
    }
    pub fn set_testmode(&self, on: bool) -> Result<()> {
        self.sdr.set_testmode(on)
    }
    pub fn set_direct_sampling(&self, mode: DirectSampleMode) -> Result<()> {
        self.sdr.set_direct_sampling(mode)
    }
    pub fn set_bias_tee(&self, on: bool) -> Result<()> {
        self.sdr.set_bias_tee(on)
    }
}
