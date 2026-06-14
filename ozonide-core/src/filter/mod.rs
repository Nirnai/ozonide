mod biquadratic_filter;
mod filter_chain;
mod filter_family;
mod pole_placement;
mod band_filters;

use biquadratic_filter as biquad;
use filter_chain as chain;
use band_filters as band;

pub use biquad::BiquadraticFilter;
pub use chain::FilterChain;
pub use pole_placement::{lowpass, highpass, FilterFamily, MAX_SECTIONS, MAX_ORDER};
pub use band::{notch, bandpass};

/// Convenience alias for the standard filter type used throughout the codebase.
pub type Filter = FilterChain<MAX_SECTIONS>;
