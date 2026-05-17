#[cfg(feature = "board-stm32h743vit6")]
mod stm32h743vit6;

#[cfg(feature = "board-stm32h743vit6")]
pub use stm32h743vit6::Stm32h743vit6 as Board;


// Re-export the trait
mod traits;
pub use traits::*;
