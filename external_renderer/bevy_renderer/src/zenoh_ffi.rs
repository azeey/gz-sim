use std::ffi::{CString, c_char, c_void};
use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender, unbounded};

#[repr(C)]
struct ZshimSession {
    _private: [u8; 0],
}
#[repr(C)]
struct ZshimPublisher {
    _private: [u8; 0],
}
#[repr(C)]
struct ZshimSubscriber {
    _private: [u8; 0],
}

type SampleCb = extern "C" fn(data: *const u8, len: usize, is_shm: bool, user_ctx: *mut c_void);

unsafe extern "C" {
    fn zshim_open(
        endpoint: *const c_char,
        listen_mode: bool,
        shm_pool_bytes: usize,
    ) -> *mut ZshimSession;
    fn zshim_close(s: *mut ZshimSession);
    fn zshim_declare_publisher(
        s: *mut ZshimSession,
        key_expr: *const c_char,
    ) -> *mut ZshimPublisher;
    fn zshim_publish(
        p: *mut ZshimPublisher,
        data: *const u8,
        len: usize,
        prefer_shm: bool,
    ) -> bool;
    fn zshim_undeclare_publisher(p: *mut ZshimPublisher);
    fn zshim_declare_subscriber(
        s: *mut ZshimSession,
        key_expr: *const c_char,
        cb: SampleCb,
        user_ctx: *mut c_void,
    ) -> *mut ZshimSubscriber;
    fn zshim_undeclare_subscriber(sub: *mut ZshimSubscriber);
}

#[derive(Debug, Clone)]
pub struct ReceivedSample {
    pub data: Vec<u8>,
    pub is_shm: bool,
}

struct SessionInner {
    raw: *mut ZshimSession,
}

unsafe impl Send for SessionInner {}
unsafe impl Sync for SessionInner {}

impl Drop for SessionInner {
    fn drop(&mut self) {
        if !self.raw.is_null() {
            unsafe { zshim_close(self.raw) };
            self.raw = std::ptr::null_mut();
        }
    }
}

#[derive(Clone)]
pub struct ZenohSession {
    inner: Arc<SessionInner>,
}

impl ZenohSession {
    pub fn open(endpoint: &str, listen_mode: bool, shm_pool_bytes: usize) -> Option<Self> {
        let c_ep = CString::new(endpoint).ok()?;
        let raw = unsafe { zshim_open(c_ep.as_ptr(), listen_mode, shm_pool_bytes) };
        if raw.is_null() {
            return None;
        }
        Some(Self {
            inner: Arc::new(SessionInner { raw }),
        })
    }

    pub fn declare_publisher(&self, key_expr: &str) -> Option<ZenohPublisher> {
        let c_ke = CString::new(key_expr).ok()?;
        let raw = unsafe { zshim_declare_publisher(self.inner.raw, c_ke.as_ptr()) };
        if raw.is_null() {
            return None;
        }
        Some(ZenohPublisher {
            _session: self.inner.clone(),
            raw,
        })
    }

    pub fn declare_subscriber(
        &self,
        key_expr: &str,
    ) -> Option<(ZenohSubscriber, Receiver<ReceivedSample>)> {
        let c_ke = CString::new(key_expr).ok()?;
        let (tx, rx) = unbounded::<ReceivedSample>();
        let ctx = Box::into_raw(Box::new(tx));
        let raw = unsafe {
            zshim_declare_subscriber(
                self.inner.raw,
                c_ke.as_ptr(),
                sample_trampoline,
                ctx as *mut c_void,
            )
        };
        if raw.is_null() {
            unsafe { drop(Box::from_raw(ctx)) };
            return None;
        }
        Some((
            ZenohSubscriber {
                _session: self.inner.clone(),
                raw,
                ctx,
            },
            rx,
        ))
    }
}

extern "C" fn sample_trampoline(data: *const u8, len: usize, is_shm: bool, user_ctx: *mut c_void) {
    if data.is_null() || user_ctx.is_null() {
        return;
    }
    let tx = unsafe { &*(user_ctx as *const Sender<ReceivedSample>) };
    let slice = unsafe { std::slice::from_raw_parts(data, len) };
    let _ = tx.send(ReceivedSample {
        data: slice.to_vec(),
        is_shm,
    });
}

pub struct ZenohPublisher {
    _session: Arc<SessionInner>,
    raw: *mut ZshimPublisher,
}

unsafe impl Send for ZenohPublisher {}
unsafe impl Sync for ZenohPublisher {}

impl ZenohPublisher {
    pub fn publish(&self, data: &[u8], prefer_shm: bool) -> bool {
        unsafe { zshim_publish(self.raw, data.as_ptr(), data.len(), prefer_shm) }
    }
}

impl Drop for ZenohPublisher {
    fn drop(&mut self) {
        if !self.raw.is_null() {
            unsafe { zshim_undeclare_publisher(self.raw) };
            self.raw = std::ptr::null_mut();
        }
    }
}

pub struct ZenohSubscriber {
    _session: Arc<SessionInner>,
    raw: *mut ZshimSubscriber,
    ctx: *mut Sender<ReceivedSample>,
}

unsafe impl Send for ZenohSubscriber {}
unsafe impl Sync for ZenohSubscriber {}

impl Drop for ZenohSubscriber {
    fn drop(&mut self) {
        if !self.raw.is_null() {
            unsafe { zshim_undeclare_subscriber(self.raw) };
            self.raw = std::ptr::null_mut();
        }
        if !self.ctx.is_null() {
            unsafe { drop(Box::from_raw(self.ctx)) };
            self.ctx = std::ptr::null_mut();
        }
    }
}
