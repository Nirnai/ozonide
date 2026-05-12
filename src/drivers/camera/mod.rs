//! Camera drivers for visual odometry and computer vision
//!
//! Provides interface for camera modules connected via DCMI/SPI.

use crate::types::{FrameMetadata, ImageFormat};

/// Common interface for camera modules
pub trait Camera {
    type Error;

    /// Initialize camera with given configuration
    fn init(&mut self, config: &CameraConfig) -> Result<(), Self::Error>;

    /// Start continuous capture mode
    fn start_capture(&mut self) -> Result<(), Self::Error>;

    /// Stop capture
    fn stop_capture(&mut self) -> Result<(), Self::Error>;

    /// Check if a new frame is available
    fn frame_available(&self) -> bool;

    /// Get pointer to frame buffer and metadata
    /// Returns (buffer_ptr, metadata)
    fn get_frame(&mut self) -> Result<(*const u8, FrameMetadata), Self::Error>;
}

/// Camera configuration
#[derive(Debug, Clone, Copy)]
pub struct CameraConfig {
    pub width: u16,
    pub height: u16,
    pub format: ImageFormat,
    pub fps: u8,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            width: 320,
            height: 240,
            format: ImageFormat::Grayscale,
            fps: 30,
        }
    }
}

// TODO: Implement for specific camera modules
// mod ov2640;
// mod ov7670;
