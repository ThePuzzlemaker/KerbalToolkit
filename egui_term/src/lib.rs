mod backend;
mod bindings;
mod font;
mod theme;
mod types;
mod view;

pub extern crate alacritty_terminal;

pub use backend::settings::BackendSettings;
pub use backend::{PtyEvent, TerminalBackend, TerminalMode};
pub use bindings::{Binding, BindingAction, InputKind, KeyboardBinding};
pub use font::{FontSettings, TerminalFont};
pub use theme::{ColorPalette, TerminalTheme};
pub use view::TerminalView;
