use parking_lot::RwLock;
use std::convert::TryInto;
use std::env;
use std::ffi::CString;
use std::ops::{Deref, DerefMut};
use std::os::raw::c_char;
use thiserror::Error;

struct Ros {
    context: RwLock<RosContext>,
}

#[derive(Error, Debug)]
enum RosInitError {
    #[error("Invalid ROS arguments were provided during initialization")]
    InvalidROSArguments,
}

impl Ros {
    fn init() -> Result<Self, RosInitError> {
        let args: Vec<CString> = env::args()
            .filter_map(|arg| CString::new(arg).ok())
            .collect();
        let c_args: Vec<*const c_char> = args.iter().map(|arg| arg.as_ptr()).collect();

        let options = RosOptions::default();
        let mut context = RosContext::default();

        // Safety based on documentation:
        // 1. This function can be run any number of times, so long as the given
        //    context has been properly prepared.
        //    -> This is fine, because the context is always created from scratch
        //       before this function runs. We should trust that
        //       RosContext::default() properly prepares a context.
        // 2. The given rcl_context_t must be zero initialized with the function
        //    rcl_get_zero_initialized_context() and must not be already initialized by
        //    this function. If the context is already initialized this function will
        //    fail and return the RCL_RET_ALREADY_INIT error code. A context may be
        //    initialized again after it has been finalized with the rcl_shutdown()
        //    function and zero initialized again with
        //    rcl_get_zero_initialized_context().
        //    -> This is fine, because the context is just used for this `rcl_init`
        //       call. RCL_RET_ALREADY_INIT can therefore not be returned. We should
        //       trust that RosContext::default() returns a properly initialized
        //       context.
        // 3. The argc and argv parameters may contain command line arguments for the
        //    program. rcl specific arguments will be parsed, but not removed. If
        //    argc is 0 and argv is NULL no parameters will be parsed.
        //    -> The command line arguments will be passed from the environment.
        //       If the arguments are invalid then InvalidROSArguments will
        //       be returned.
        // 4. The options argument must be non-NULL and must have been initialized
        //    with rcl_init_options_init(). It is unmodified by this function, and
        //    the ownership is not transfered to the context, but instead a copy is
        //    made into the context for later reference. Therefore, the given options
        //    need to be cleaned up with rcl_init_options_fini() after this function
        //    returns.
        //    -> We should trust that RosOptions::default() properly initializes the
        //       options argument
        let return_value = unsafe {
            rcl_sys::rcl_init(
                c_args.len().try_into().unwrap(),
                c_args.as_ptr(),
                &*options,
                &mut *context,
            )
        };

        match return_value.try_into().unwrap() {
            rcl_sys::RCL_RET_OK => Ok(Self {
                context: RwLock::new(context),
            }),
            rcl_sys::RCL_RET_INVALID_ROS_ARGS => Err(RosInitError::InvalidROSArguments),
            _ => panic!(
                "Unspecified error {} occurred while initialzing ROS",
                return_value
            ),
        }
    }
}

impl Drop for Ros {
    fn drop(&mut self) {
        // Safety: The given context must have been initialized with rcl_init().
        //         If not, this function will fail with RCL_RET_ALREADY_SHUTDOWN.
        // Because the Ros instance can only exist if the context was succesfully
        // created, RCL_RET_ALREADY_SHUTDOWN will never be thrown.
        let return_value = unsafe {rcl_sys::rcl_shutdown(&mut **self.context.write())};
        assert_eq!(return_value, rcl_sys::RCL_RET_OK.try_into().unwrap())
    }
}

struct RosContext {
    context: rcl_sys::rcl_context_t,
}

impl RosContext {
    // Unfortunatly, the rcl foxy release marks the called function as non-const
    // even though it is not mutated. This has been fixed in future releases of
    // rcl.
    fn is_valid(&mut self) -> bool {
        // Safety:
        //   If context is NULL, then false is returned.
        //   If context is zero-initialized, then false is returned.
        //   If context is uninitialized, then it is undefined behavior.
        // Context cannot be uninitialized, because it is always initialized
        // by RosContext::default(). No undefined behavior should happen.
        unsafe { rcl_sys::rcl_context_is_valid(&mut self.context) }
    }
}

impl Default for RosContext {
    fn default() -> Self {
        // Safety: this initializes a context. If the implementation is safe, then
        // this call is safe to do.
        let context = unsafe { rcl_sys::rcl_get_zero_initialized_context() };
        Self { context }
    }
}

impl Deref for RosContext {
    type Target = rcl_sys::rcl_context_t;
    fn deref(&self) -> &Self::Target {
        &self.context
    }
}

impl DerefMut for RosContext {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.context
    }
}

impl Drop for RosContext {
    fn drop(&mut self) {
        // Safety:
        // The documentation states that:
        // 1. The context to be finalized must have been previously initialized
        //    with rcl_init(), and then later invalidated with rcl_shutdown().
        // 2. A zero-initialized context that has not been initialized can be finalized.
        // 3. If context is NULL, then RCL_RET_INVALID_ARGUMENT is returned.
        // 4. If context is zero-initialized, then RCL_RET_OK is returned.
        // 5. If context is initialized and valid (rcl_shutdown() was not called on it),
        //    then RCL_RET_INVALID_ARGUMENT is returned.
        //
        // These cases could be handled as follows:
        // 1. If we ensure that RosContext is a private struct and ensure that it
        //    is properly everywhere, this can be handled properly.
        // 2. If the context is not used in rcl_init, then this is fine. However, if
        //    it is then rcl_shutdown should be called first.
        // 3. This should never happen, because it is only initialized with the
        //    rcl_get_zero_initialized_context() call.
        // 4. This is the scenario that we want
        // 5. We have to ensure that rcl_shutdown() is ALWAYS called before RosContext
        //    is dropped.
        let return_value = unsafe { rcl_sys::rcl_context_fini(&mut self.context) };

        // This assert will most likely always succeed, but it is here as
        // a sanity check.
        assert_eq!(return_value, rcl_sys::RCL_RET_OK.try_into().unwrap());
    }
}

struct RosOptions {
    options: rcl_sys::rcl_init_options_t,
}

impl Default for RosOptions {
    fn default() -> Self {
        let mut options = unsafe { rcl_sys::rcl_get_zero_initialized_init_options() };
        let allocator = unsafe { rcl_sys::rcutils_get_default_allocator() };
        let return_value = unsafe { rcl_sys::rcl_init_options_init(&mut options, allocator) };

        assert_eq!(return_value, rcl_sys::RCL_RET_OK.try_into().unwrap());

        Self { options }
    }
}

impl Deref for RosOptions {
    type Target = rcl_sys::rcl_init_options_t;
    fn deref(&self) -> &Self::Target {
        &self.options
    }
}

impl Drop for RosOptions {
    fn drop(&mut self) {
        let return_value = unsafe { rcl_sys::rcl_init_options_fini(&mut self.options) };

        // This should always succeed, because the other option would be
        // an RCL_RET_INVALID_ARGUMENT which can only happen when the provided
        // argument is non-NULL and valid. Because `self.options` is always initialized
        // by the `default()` function and `drop()` is called at most one time, this
        // should never happen.
        assert_eq!(return_value, rcl_sys::RCL_RET_OK.try_into().unwrap())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initialization_of_ros_context() {
        let ros = Ros::init().unwrap();
        let mut raw_context = ros.context.write();
        assert!(raw_context.is_valid());
        drop(raw_context);
        drop(ros)
    }
}
