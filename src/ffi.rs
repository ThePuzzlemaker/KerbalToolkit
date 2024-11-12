pub mod bodies;
pub mod missions;
pub mod orbits;
pub mod support;
pub mod sv;
pub mod systems;

#[macro_export]
macro_rules! ffi_defn {
    ($name:ident @ $frame:ident, $module:ident) => {
        let $name = Value::new(&mut *$frame, $name as *mut std::ffi::c_void);
        unsafe {
            $module
                .set_global(&mut *$frame, stringify!($name), $name)
                .into_jlrs_result()?;
        }
    };
}

#[macro_export]
macro_rules! ffi {
    (#[fallible$(($default:expr))?] fn $name:ident($($arg:ident : $ty:ty),*) -> $ret:ty { $($tt:tt)* }) => {
	pub unsafe extern "C" fn $name($($arg: $ty),*) -> $ret {
	    unsafe fn inner($($arg : $ty),*) -> ::color_eyre::eyre::Result<$ret> {
		$($tt)*
	    }
	    match inner($($arg),*) {
		Err(e) => {
		    replace_error(e.to_string());
		    $($default)?
		}
		Ok(v) => v
	    }
	}
    };
    (fn $name:ident($($arg:ident: $ty:ty),*)$(-> $ret:ty)? { $($tt:tt)* }) => {
	pub unsafe extern "C" fn $name($($arg:ty),*)$(-> $ret)? { $($tt)* }
    }
}
