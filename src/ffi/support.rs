#![allow(dead_code)]
use color_eyre::eyre;
use jlrs::{
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, Module, Value},
};
use lock_api::RawRwLock as _;
use parking_lot::RawRwLock;

pub unsafe extern "C" fn ktk_locks_is_locked_read(lck: *const RawRwLock) -> bool {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.is_locked()
}

pub unsafe extern "C" fn ktk_locks_lock_read(lck: *const RawRwLock) {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.lock_shared()
}

pub unsafe extern "C" fn ktk_locks_try_lock_read(lck: *const RawRwLock) -> bool {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.try_lock_shared()
}

pub unsafe extern "C" fn ktk_locks_unlock_read(lck: *const RawRwLock) {
    assert!(!lck.is_null());
    let lck = &*lck;
    assert!(lck.is_locked());
    unsafe { lck.unlock_shared() }
}

pub unsafe extern "C" fn ktk_locks_is_locked_write(lck: *const RawRwLock) -> bool {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.is_locked_exclusive()
}

pub unsafe extern "C" fn ktk_locks_lock_write(lck: *const RawRwLock) {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.lock_exclusive()
}

pub unsafe extern "C" fn ktk_locks_try_lock_write(lck: *const RawRwLock) -> bool {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.try_lock_exclusive()
}

pub unsafe extern "C" fn ktk_locks_unlock_write(lck: *const RawRwLock) {
    assert!(!lck.is_null());
    let lck = &*lck;
    assert!(lck.is_locked_exclusive());
    unsafe { lck.unlock_exclusive() }
}

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    let ktk_locks_is_locked_read = Value::new(
        &mut *frame,
        ktk_locks_is_locked_read as *mut std::ffi::c_void,
    );
    let ktk_locks_lock_read = Value::new(&mut *frame, ktk_locks_lock_read as *mut std::ffi::c_void);
    let ktk_locks_try_lock_read = Value::new(
        &mut *frame,
        ktk_locks_try_lock_read as *mut std::ffi::c_void,
    );
    let ktk_locks_unlock_read =
        Value::new(&mut *frame, ktk_locks_unlock_read as *mut std::ffi::c_void);

    let ktk_locks_is_locked_write = Value::new(
        &mut *frame,
        ktk_locks_is_locked_write as *mut std::ffi::c_void,
    );
    let ktk_locks_lock_write =
        Value::new(&mut *frame, ktk_locks_lock_write as *mut std::ffi::c_void);
    let ktk_locks_try_lock_write = Value::new(
        &mut *frame,
        ktk_locks_try_lock_write as *mut std::ffi::c_void,
    );
    let ktk_locks_unlock_write =
        Value::new(&mut *frame, ktk_locks_unlock_write as *mut std::ffi::c_void);

    unsafe {
        module
            .set_global(
                &mut *frame,
                "ktk_locks_is_locked_read",
                ktk_locks_is_locked_read,
            )
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_locks_lock_read", ktk_locks_lock_read)
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_locks_try_lock_read",
                ktk_locks_try_lock_read,
            )
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_locks_unlock_read", ktk_locks_unlock_read)
            .into_jlrs_result()?;

        module
            .set_global(
                &mut *frame,
                "ktk_locks_is_locked_write",
                ktk_locks_is_locked_write,
            )
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_locks_lock_write", ktk_locks_lock_write)
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_locks_try_lock_write",
                ktk_locks_try_lock_write,
            )
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_locks_unlock_write",
                ktk_locks_unlock_write,
            )
            .into_jlrs_result()?;
    }

    Ok(())
}
