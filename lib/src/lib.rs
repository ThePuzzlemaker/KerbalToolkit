#![warn(clippy::pedantic)]
#![allow(
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::must_use_candidate,
    clippy::many_single_char_names,
    clippy::module_name_repetitions,
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::doc_markdown
)]
pub mod arena;
pub mod bodies;
pub mod ffs;
pub mod kepler;
pub mod krpc;
pub mod maneuver;
pub mod math;
pub mod misc;
pub mod time;
pub mod translunar;
pub mod vessel;
