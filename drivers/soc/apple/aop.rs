// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple AOP driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::{arch::asm, mem, ptr, slice};

use kernel::{
    bindings, c_str, device, dma,
    error::from_err_ptr,
    io_mem::IoMem,
    module_platform_driver, new_condvar, new_mutex, of, platform,
    prelude::*,
    soc::apple::aop::{from_fourcc, EPICService, FakehidListener, AOP},
    soc::apple::rtkit,
    sync::{Arc, ArcBorrow, CondVar, Mutex},
    types::{ARef, ForeignOwnable},
    workqueue::{self, impl_has_work, new_work, Work, WorkItem},
};

const AOP_MMIO_SIZE: usize = 0x1e0000;
const ASC_MMIO_SIZE: usize = 0x4000;
const BOOTARGS_OFFSET: usize = 0x22c;
const BOOTARGS_SIZE: usize = 0x230;
const CPU_CONTROL: usize = 0x44;
const CPU_RUN: u32 = 0x1 << 4;
const AFK_ENDPOINT_START: u8 = 0x20;
const AFK_ENDPOINT_COUNT: u8 = 0xc;
const AFK_OPC_GET_BUF: u64 = 0x89;
const AFK_OPC_INIT: u64 = 0x80;
const AFK_OPC_INIT_RX: u64 = 0x8b;
const AFK_OPC_INIT_TX: u64 = 0x8a;
const AFK_OPC_INIT_UNK: u64 = 0x8c;
const AFK_OPC_SEND: u64 = 0xa2;
const AFK_OPC_START_ACK: u64 = 0x86;
const AFK_OPC_SHUTDOWN_ACK: u64 = 0xc1;
const AFK_OPC_RECV: u64 = 0x85;
const AFK_MSG_GET_BUF_ACK: u64 = 0xa1 << 48;
const AFK_MSG_INIT: u64 = AFK_OPC_INIT << 48;
const AFK_MSG_INIT_ACK: u64 = 0xa0 << 48;
const AFK_MSG_START: u64 = 0xa3 << 48;
const AFK_MSG_SHUTDOWN: u64 = 0xc0 << 48;
const AFK_RB_BLOCK_STEP: usize = 0x40;
const EPIC_TYPE_NOTIFY: u32 = 0;
const EPIC_CATEGORY_REPORT: u8 = 0x00;
const EPIC_CATEGORY_NOTIFY: u8 = 0x10;
const EPIC_CATEGORY_REPLY: u8 = 0x20;
const EPIC_SUBTYPE_STD_SERVICE: u16 = 0xc0;
const EPIC_SUBTYPE_FAKEHID_REPORT: u16 = 0xc4;
const EPIC_SUBTYPE_RETCODE: u16 = 0x84;
const EPIC_SUBTYPE_RETCODE_PAYLOAD: u16 = 0xa0;
const QE_MAGIC1: u32 = from_fourcc(b" POI");
const QE_MAGIC2: u32 = from_fourcc(b" POA");

fn align_up(v: usize, a: usize) -> usize {
    (v + a - 1) & !(a - 1)
}

#[inline(always)]
fn mem_sync() {
    unsafe {
        asm!("dsb sy");
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct QEHeader {
    magic: u32,
    size: u32,
    channel: u32,
    ty: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct EPICHeader {
    version: u8,
    seq: u16,
    _pad0: u8,
    _unk0: u32,
    timestamp: u64,
    // Subheader
    length: u32,
    sub_version: u8,
    category: u8,
    subtype: u16,
    tag: u16,
    _unk1: u16,
    _pad1: u64,
    inline_len: u32,
}

#[repr(C, packed)]
struct EPICServiceAnnounce {
    name: [u8; 20],
    _unk0: u32,
    retcode: u32,
    _unk1: u32,
    channel: u32,
    _unk2: u32,
    _unk3: u32,
}

#[pin_data]
struct FutureValue<T> {
    #[pin]
    val: Mutex<Option<T>>,
    #[pin]
    completion: CondVar,
}

impl<T: Clone> FutureValue<T> {
    fn pin_init() -> impl PinInit<FutureValue<T>> {
        pin_init!(
            FutureValue {
                val <- new_mutex!(None),
                completion <- new_condvar!()
            }
        )
    }
    fn complete(&self, val: T) {
        *self.val.lock() = Some(val);
        self.completion.notify_all();
    }
    fn wait(&self) -> T {
        let mut ret_guard = self.val.lock();
        while ret_guard.is_none() {
            self.completion.wait(&mut ret_guard);
        }
        ret_guard.as_ref().unwrap().clone()
    }
    fn reset(&self) {
        *self.val.lock() = None;
    }
}

struct AFKRingBuffer {
    offset: usize,
    block_size: usize,
    buf_size: usize,
}

struct AFKEndpoint {
    index: u8,
    iomem: Option<dma::CoherentAllocation<u8, dma::CoherentAllocator>>,
    txbuf: Option<AFKRingBuffer>,
    rxbuf: Option<AFKRingBuffer>,
    seq: u16,
    calls: [Option<Arc<FutureValue<u32>>>; 8],
}

unsafe impl Send for AFKEndpoint {}

impl AFKEndpoint {
    fn new(index: u8) -> AFKEndpoint {
        AFKEndpoint {
            index,
            iomem: None,
            txbuf: None,
            rxbuf: None,
            seq: 0,
            calls: [const { None }; 8],
        }
    }

    fn start(&self, rtkit: &mut rtkit::RtKit<AopData>) -> Result<()> {
        rtkit.send_message(self.index, AFK_MSG_INIT)
    }

    fn stop(&self, rtkit: &mut rtkit::RtKit<AopData>) -> Result<()> {
        rtkit.send_message(self.index, AFK_MSG_SHUTDOWN)
    }

    fn recv_message(
        &mut self,
        client: ArcBorrow<'_, AopData>,
        rtkit: &mut rtkit::RtKit<AopData>,
        msg: u64,
    ) -> Result<()> {
        let opc = msg >> 48;
        match opc {
            AFK_OPC_INIT => {
                rtkit.send_message(self.index, AFK_MSG_INIT_ACK)?;
            }
            AFK_OPC_GET_BUF => {
                self.recv_get_buf(client.dev.clone(), rtkit, msg)?;
            }
            AFK_OPC_INIT_UNK => {} // no-op
            AFK_OPC_START_ACK => {}
            AFK_OPC_INIT_RX => {
                if self.rxbuf.is_some() {
                    dev_err!(
                        client.dev,
                        "Got InitRX message with existing rxbuf at endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                self.rxbuf = Some(self.parse_ring_buf(&client.dev, msg)?);
                if self.txbuf.is_some() {
                    rtkit.send_message(self.index, AFK_MSG_START)?;
                }
            }
            AFK_OPC_INIT_TX => {
                if self.txbuf.is_some() {
                    dev_err!(
                        client.dev,
                        "Got InitTX message with existing txbuf at endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                self.txbuf = Some(self.parse_ring_buf(&client.dev, msg)?);
                if self.rxbuf.is_some() {
                    rtkit.send_message(self.index, AFK_MSG_START)?;
                }
            }
            AFK_OPC_RECV => {
                self.recv_rb(client)?;
            }
            AFK_OPC_SHUTDOWN_ACK => {
                client.shutdown_complete();
            }
            _ => dev_err!(
                client.dev,
                "AFK endpoint {} got unknown message {}",
                self.index,
                msg
            ),
        }
        Ok(())
    }

    fn parse_ring_buf(&self, dev: &ARef<device::Device>, msg: u64) -> Result<AFKRingBuffer> {
        let msg = msg as usize;
        let size = ((msg >> 16) & 0xFFFF) * AFK_RB_BLOCK_STEP;
        let offset = ((msg >> 32) & 0xFFFF) * AFK_RB_BLOCK_STEP;
        let buf_size = self.iomem_read32(dev, offset)? as usize;
        let block_size = (size - buf_size) / 3;
        Ok(AFKRingBuffer {
            offset,
            block_size,
            buf_size,
        })
    }
    fn iomem_write32(&mut self, dev: &ARef<device::Device>, off: usize, data: u32) -> Result<()> {
        let iomem = self.iomem.as_mut().unwrap();
        if off + mem::size_of::<u32>() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem write");
            return Err(EIO);
        }
        unsafe {
            let ptr = iomem.first_ptr_mut().offset(off as isize) as *mut u32;
            *ptr = data;
        }
        Ok(())
    }
    fn iomem_read32(&self, dev: &ARef<device::Device>, off: usize) -> Result<u32> {
        let iomem = self.iomem.as_ref().unwrap();
        if off + mem::size_of::<u32>() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem read");
            return Err(EIO);
        }
        // SAFETY: all bit patterns are valid u32s
        unsafe {
            let ptr = iomem.first_ptr().offset(off as isize) as *const u32;
            Ok(*ptr)
        }
    }
    fn memcpy_from_iomem(
        &self,
        dev: &ARef<device::Device>,
        off: usize,
        target: &mut [u8],
    ) -> Result<()> {
        let iomem = self.iomem.as_ref().unwrap();
        if off + target.len() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem read");
            return Err(EIO);
        }
        // SAFETY: We checked that it is in bounds above
        unsafe {
            let ptr = iomem.first_ptr().offset(off as isize);
            let src = slice::from_raw_parts(ptr, target.len());
            target.copy_from_slice(src);
        }
        Ok(())
    }

    fn memcpy_to_iomem(&self, dev: &ARef<device::Device>, off: usize, src: &[u8]) -> Result<()> {
        let iomem = self.iomem.as_ref().unwrap();
        if off + src.len() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem write");
            return Err(EIO);
        }
        // SAFETY: We checked that it is in bounds above
        unsafe {
            let ptr = iomem.first_ptr_mut().offset(off as isize);
            let target = slice::from_raw_parts_mut(ptr, src.len());
            target.copy_from_slice(src);
        }
        Ok(())
    }

    fn recv_get_buf(
        &mut self,
        dev: ARef<device::Device>,
        rtkit: &mut rtkit::RtKit<AopData>,
        msg: u64,
    ) -> Result<()> {
        let size = ((msg & 0xFFFF0000) >> 16) as usize * AFK_RB_BLOCK_STEP;
        if self.iomem.is_some() {
            dev_err!(
                dev,
                "Got GetBuf message with existing buffer on endpoint {}",
                self.index
            );
            return Err(EIO);
        }
        let iomem = dma::try_alloc_coherent(dev, size, false)?;
        rtkit.send_message(self.index, AFK_MSG_GET_BUF_ACK | iomem.dma_handle)?;
        self.iomem = Some(iomem);
        Ok(())
    }

    fn recv_rb(&mut self, client: ArcBorrow<'_, AopData>) -> Result<()> {
        let (buf_offset, block_size, buf_size) = match self.rxbuf.as_ref() {
            Some(b) => (b.offset, b.block_size, b.buf_size),
            None => {
                dev_err!(
                    client.dev,
                    "Got Recv message with no rxbuf at endpoint {}",
                    self.index
                );
                return Err(EIO);
            }
        };
        let mut rptr = self.iomem_read32(&client.dev, buf_offset + block_size)? as usize;
        let mut wptr = self.iomem_read32(&client.dev, buf_offset + block_size * 2)?;
        mem_sync();
        let base = buf_offset + block_size * 3;
        let mut msg_buf = KVec::new();
        const QEH_SIZE: usize = mem::size_of::<QEHeader>();
        while wptr as usize != rptr {
            let mut qeh_bytes = [0; QEH_SIZE];
            self.memcpy_from_iomem(&client.dev, base + rptr, &mut qeh_bytes)?;
            let mut qeh = unsafe { &*(qeh_bytes.as_ptr() as *const QEHeader) };
            if qeh.magic != QE_MAGIC1 && qeh.magic != QE_MAGIC2 {
                let magic = qeh.magic;
                dev_err!(
                    client.dev,
                    "Invalid magic on ep {}, got {:x}",
                    self.index,
                    magic
                );
                return Err(EIO);
            }
            if qeh.size as usize > (buf_size - rptr - QEH_SIZE) {
                rptr = 0;
                self.memcpy_from_iomem(&client.dev, base + rptr, &mut qeh_bytes)?;
                qeh = unsafe { &*(qeh_bytes.as_ptr() as *const QEHeader) };

                if qeh.magic != QE_MAGIC1 && qeh.magic != QE_MAGIC2 {
                    let magic = qeh.magic;
                    dev_err!(
                        client.dev,
                        "Invalid magic on ep {}, got {:x}",
                        self.index,
                        magic
                    );
                    return Err(EIO);
                }
            }
            msg_buf.resize(qeh.size as usize, 0, GFP_KERNEL)?;
            self.memcpy_from_iomem(&client.dev, base + rptr + QEH_SIZE, &mut msg_buf)?;
            let (hdr_bytes, msg) = msg_buf.split_at(mem::size_of::<EPICHeader>());
            let header = unsafe { &*(hdr_bytes.as_ptr() as *const EPICHeader) };
            self.handle_ipc(client, qeh, header, msg)?;
            rptr = align_up(rptr + QEH_SIZE + qeh.size as usize, block_size) % buf_size;
            mem_sync();
            self.iomem_write32(&client.dev, buf_offset + block_size, rptr as u32)?;
            wptr = self.iomem_read32(&client.dev, buf_offset + block_size * 2)?;
            mem_sync();
        }
        Ok(())
    }
    fn handle_ipc(
        &mut self,
        client: ArcBorrow<'_, AopData>,
        qhdr: &QEHeader,
        ehdr: &EPICHeader,
        data: &[u8],
    ) -> Result<()> {
        let subtype = ehdr.subtype;
        if ehdr.category == EPIC_CATEGORY_REPORT {
            if subtype == EPIC_SUBTYPE_STD_SERVICE {
                let announce = unsafe { &*(data.as_ptr() as *const EPICServiceAnnounce) };
                let chan = announce.channel;
                let name_len = announce
                    .name
                    .iter()
                    .position(|x| *x == 0)
                    .unwrap_or(announce.name.len());
                return Into::<Arc<_>>::into(client).register_service(
                    self,
                    chan,
                    &announce.name[..name_len],
                );
            } else if subtype == EPIC_SUBTYPE_FAKEHID_REPORT {
                return client.process_fakehid_report(self, qhdr.channel, data);
            } else {
                dev_err!(
                    client.dev,
                    "Unexpected EPIC report subtype {:x} on endpoint {}",
                    subtype,
                    self.index
                );
                return Err(EIO);
            }
        } else if ehdr.category == EPIC_CATEGORY_REPLY {
            if subtype == EPIC_SUBTYPE_RETCODE_PAYLOAD || subtype == EPIC_SUBTYPE_RETCODE {
                if data.len() < mem::size_of::<u32>() {
                    dev_err!(
                        client.dev,
                        "Retcode data too short on endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                let retcode = u32::from_ne_bytes(data[..4].try_into().unwrap());
                let tag = ehdr.tag as usize;
                if tag == 0 || tag - 1 > self.calls.len() || self.calls[tag - 1].is_none() {
                    dev_err!(
                        client.dev,
                        "Got a retcode with invalid tag {:?} on endpoint {}",
                        tag,
                        self.index
                    );
                    return Err(EIO);
                }
                self.calls[tag - 1].take().unwrap().complete(retcode);
                return Ok(());
            } else {
                dev_err!(
                    client.dev,
                    "Unexpected EPIC reply subtype {:x} on endpoint {}",
                    subtype,
                    self.index
                );
                return Err(EIO);
            }
        }
        dev_err!(
            client.dev,
            "Unexpected EPIC category {:x} on endpoint {}",
            ehdr.category,
            self.index
        );
        Err(EIO)
    }
    fn send_rb(
        &mut self,
        client: &AopData,
        rtkit: &mut rtkit::RtKit<AopData>,
        channel: u32,
        ty: u32,
        header: &[u8],
        data: &[u8],
    ) -> Result<()> {
        let (buf_offset, block_size, buf_size) = match self.txbuf.as_ref() {
            Some(b) => (b.offset, b.block_size, b.buf_size),
            None => {
                dev_err!(
                    client.dev,
                    "Attempting to send message with no txbuf at endpoint {}",
                    self.index
                );
                return Err(EIO);
            }
        };
        let base = buf_offset + block_size * 3;
        mem_sync();
        let rptr = self.iomem_read32(&client.dev, buf_offset + block_size)? as usize;
        let mut wptr = self.iomem_read32(&client.dev, buf_offset + block_size * 2)? as usize;
        const QEH_SIZE: usize = mem::size_of::<QEHeader>();
        if wptr < rptr && wptr + QEH_SIZE >= rptr {
            dev_err!(client.dev, "Tx buffer full at endpoint {}", self.index);
            return Err(EIO);
        }
        let payload_len = header.len() + data.len();
        let qeh = QEHeader {
            magic: QE_MAGIC1,
            size: payload_len as u32,
            channel,
            ty,
        };
        let qeh_bytes = unsafe {
            slice::from_raw_parts(
                &qeh as *const QEHeader as *const u8,
                mem::size_of::<QEHeader>(),
            )
        };
        self.memcpy_to_iomem(&client.dev, base + wptr, qeh_bytes)?;
        if payload_len > buf_size - wptr - QEH_SIZE {
            wptr = 0;
            self.memcpy_to_iomem(&client.dev, base + wptr, qeh_bytes)?;
        }
        self.memcpy_to_iomem(&client.dev, base + wptr + QEH_SIZE, header)?;
        self.memcpy_to_iomem(&client.dev, base + wptr + QEH_SIZE + header.len(), data)?;
        wptr = align_up(wptr + QEH_SIZE + payload_len, block_size) % buf_size;
        self.iomem_write32(&client.dev, buf_offset + block_size * 2, wptr as u32)?;
        let msg = wptr as u64 | (AFK_OPC_SEND << 48);
        rtkit.send_message(self.index, msg)
    }
    fn epic_notify(
        &mut self,
        client: &AopData,
        rtkit: &mut rtkit::RtKit<AopData>,
        channel: u32,
        subtype: u16,
        data: &[u8],
    ) -> Result<Arc<FutureValue<u32>>> {
        let mut tag = 0;
        for i in 0..self.calls.len() {
            if self.calls[i].is_none() {
                tag = i + 1;
                break;
            }
        }
        if tag == 0 {
            dev_err!(
                client.dev,
                "Too many inflight calls on endpoint {}",
                self.index
            );
            return Err(EIO);
        }
        let call = Arc::pin_init(FutureValue::pin_init(), GFP_KERNEL)?;
        let hdr = EPICHeader {
            version: 2,
            seq: self.seq,
            length: data.len() as u32,
            sub_version: 2,
            category: EPIC_CATEGORY_NOTIFY,
            subtype,
            tag: tag as u16,
            ..EPICHeader::default()
        };
        self.send_rb(
            client,
            rtkit,
            channel,
            EPIC_TYPE_NOTIFY,
            unsafe {
                slice::from_raw_parts(
                    &hdr as *const EPICHeader as *const u8,
                    mem::size_of::<EPICHeader>(),
                )
            },
            data,
        )?;
        self.seq = self.seq.wrapping_add(1);
        self.calls[tag - 1] = Some(call.clone());
        Ok(call)
    }
}

struct ListenerEntry {
    svc: EPICService,
    listener: Arc<dyn FakehidListener>,
}

unsafe impl Send for ListenerEntry {}

#[pin_data]
struct AopData {
    dev: ARef<device::Device>,
    aop_mmio: IoMem<AOP_MMIO_SIZE>,
    asc_mmio: IoMem<ASC_MMIO_SIZE>,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<AopData>>>,
    #[pin]
    endpoints: [Mutex<AFKEndpoint>; AFK_ENDPOINT_COUNT as usize],
    #[pin]
    ep_shutdown: FutureValue<()>,
    #[pin]
    hid_listeners: Mutex<KVec<ListenerEntry>>,
    #[pin]
    subdevices: Mutex<KVec<*mut bindings::platform_device>>,
}

unsafe impl Send for AopData {}
unsafe impl Sync for AopData {}

#[pin_data]
struct AopServiceRegisterWork {
    name: &'static CStr,
    data: Arc<AopData>,
    service: EPICService,
    #[pin]
    work: Work<AopServiceRegisterWork>,
}

impl_has_work! {
    impl HasWork<Self, 0> for AopServiceRegisterWork { self.work }
}

impl AopServiceRegisterWork {
    fn new(name: &'static CStr, data: Arc<AopData>, service: EPICService) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(AopServiceRegisterWork {
                name, data, service,
                work <- new_work!("AopServiceRegisterWork::work"),
            }),
            GFP_KERNEL,
        )
    }
}

impl WorkItem for AopServiceRegisterWork {
    type Pointer = Arc<AopServiceRegisterWork>;

    fn run(this: Arc<AopServiceRegisterWork>) {
        let info = bindings::platform_device_info {
            parent: this.data.dev.as_raw(),
            name: this.name.as_ptr() as *const _,
            id: bindings::PLATFORM_DEVID_AUTO,
            res: ptr::null_mut(),
            num_res: 0,
            data: &this.service as *const EPICService as *const _,
            size_data: mem::size_of::<EPICService>(),
            dma_mask: 0,
            fwnode: ptr::null_mut(),
            properties: ptr::null_mut(),
            of_node_reused: false,
        };
        let pdev = unsafe { from_err_ptr(bindings::platform_device_register_full(&info)) };
        match pdev {
            Err(e) => {
                dev_err!(
                    this.data.dev,
                    "Failed to create device for service {:?}: {:?}",
                    this.name,
                    e
                );
            }
            Ok(pdev) => {
                let res = this.data.subdevices.lock().push(pdev, GFP_KERNEL);
                if res.is_err() {
                    dev_err!(this.data.dev, "Failed to store subdevice");
                }
            }
        }
    }
}

impl AopData {
    fn new(pdev: &mut platform::Device) -> Result<Arc<AopData>> {
        let aop_mmio = unsafe { pdev.ioremap_resource(0)? };
        let asc_mmio = unsafe { pdev.ioremap_resource(1)? };
        Arc::pin_init(
            pin_init!(
                AopData {
                    dev: pdev.get_device(),
                    aop_mmio,
                    asc_mmio,
                    rtkit <- new_mutex!(None),
                    endpoints <- init::pin_init_array_from_fn(|i| {
                        new_mutex!(AFKEndpoint::new(AFK_ENDPOINT_START + i as u8))
                    }),
                    ep_shutdown <- FutureValue::pin_init(),
                    hid_listeners <- new_mutex!(KVec::new()),
                    subdevices <- new_mutex!(KVec::new()),
                }
            ),
            GFP_KERNEL,
        )
    }
    fn start(&self) -> Result<()> {
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.wake()?;
        }
        for ep in 0..AFK_ENDPOINT_COUNT {
            let rtk_ep_num = AFK_ENDPOINT_START + ep;
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            if !rtk.has_endpoint(rtk_ep_num) {
                continue;
            }
            rtk.start_endpoint(rtk_ep_num)?;
            let ep_guard = self.endpoints[ep as usize].lock();
            ep_guard.start(rtk)?;
        }
        Ok(())
    }
    fn register_service(
        self: Arc<Self>,
        ep: &mut AFKEndpoint,
        channel: u32,
        name: &[u8],
    ) -> Result<()> {
        let svc = EPICService {
            channel,
            endpoint: ep.index,
        };
        let dev_name = match name {
            b"aop-audio" => c_str!("snd_soc_apple_aop"),
            b"las" => c_str!("iio_aop_las"),
            b"als" => c_str!("iio_aop_als"),
            _ => {
                return Ok(());
            }
        };
        // probe can call back into us, run it with locks dropped.
        let work = AopServiceRegisterWork::new(dev_name, self, svc)?;
        workqueue::system().enqueue(work).map_err(|_| ENOMEM)
    }

    fn process_fakehid_report(&self, ep: &AFKEndpoint, ch: u32, data: &[u8]) -> Result<()> {
        let guard = self.hid_listeners.lock();
        for entry in &*guard {
            if entry.svc.endpoint == ep.index && entry.svc.channel == ch {
                return entry.listener.process_fakehid_report(data);
            }
        }
        Ok(())
    }

    fn shutdown_complete(&self) {
        self.ep_shutdown.complete(());
    }

    fn stop(&self) -> Result<()> {
        for ep in 0..AFK_ENDPOINT_COUNT {
            {
                let rtk_ep_num = AFK_ENDPOINT_START + ep;
                let mut guard = self.rtkit.lock();
                let rtk = guard.as_mut().unwrap();
                if !rtk.has_endpoint(rtk_ep_num) {
                    continue;
                }
                let ep_guard = self.endpoints[ep as usize].lock();
                ep_guard.stop(rtk)?;
            }
            self.ep_shutdown.wait();
            self.ep_shutdown.reset();
        }
        Ok(())
    }

    fn aop_read32(&self, off: usize) -> u32 {
        self.aop_mmio.readl_relaxed(off)
    }

    fn patch_bootargs(&self, patches: &[(u32, u32)]) -> Result<()> {
        let offset = self.aop_read32(BOOTARGS_OFFSET) as usize;
        let size = self.aop_read32(BOOTARGS_SIZE) as usize;
        let mut arg_bytes = KVec::with_capacity(size, GFP_KERNEL)?;
        for _ in 0..size {
            arg_bytes.push(0, GFP_KERNEL).unwrap();
        }
        self.aop_mmio.try_memcpy_fromio(&mut arg_bytes, offset)?;
        let mut idx = 0;
        while idx < size {
            let key = u32::from_le_bytes(arg_bytes[idx..idx + 4].try_into().unwrap());
            let size = u32::from_le_bytes(arg_bytes[idx + 4..idx + 8].try_into().unwrap()) as usize;
            idx += 8;
            for (k, v) in patches.iter() {
                if *k != key {
                    continue;
                }
                arg_bytes[idx..idx + size].copy_from_slice(&(*v as u64).to_le_bytes()[..size]);
                break;
            }
            idx += size;
        }
        self.aop_mmio.try_memcpy_toio(offset, &arg_bytes)
    }

    fn start_cpu(&self) {
        let val = self.asc_mmio.readl_relaxed(CPU_CONTROL);
        self.asc_mmio.writel_relaxed(val | CPU_RUN, CPU_CONTROL);
    }
}

impl AOP for AopData {
    fn epic_call(&self, svc: &EPICService, subtype: u16, msg_bytes: &[u8]) -> Result<u32> {
        let ep_idx = svc.endpoint - AFK_ENDPOINT_START;
        let call = {
            let mut rtk_guard = self.rtkit.lock();
            let rtk = rtk_guard.as_mut().unwrap();
            let mut ep_guard = self.endpoints[ep_idx as usize].lock();
            ep_guard.epic_notify(self, rtk, svc.channel, subtype, msg_bytes)?
        };
        Ok(call.wait())
    }
    fn add_fakehid_listener(
        &self,
        svc: EPICService,
        listener: Arc<dyn FakehidListener>,
    ) -> Result<()> {
        let mut guard = self.hid_listeners.lock();
        Ok(guard.push(ListenerEntry { svc, listener }, GFP_KERNEL)?)
    }
    fn remove_fakehid_listener(&self, svc: &EPICService) -> bool {
        let mut guard = self.hid_listeners.lock();
        for i in 0..guard.len() {
            if guard[i].svc == *svc {
                guard.swap_remove(i);
                return true;
            }
        }
        false
    }
    fn remove(&self) {
        if let Err(e) = self.stop() {
            dev_err!(self.dev, "Failed to stop AOP {:?}", e);
        }
        *self.rtkit.lock() = None;
        let guard = self.subdevices.lock();
        for pdev in &*guard {
            unsafe {
                bindings::platform_device_unregister(*pdev);
            }
        }
    }
}

struct NoBuffer;
impl rtkit::Buffer for NoBuffer {
    fn iova(&self) -> Result<usize> {
        unreachable!()
    }
    fn buf(&mut self) -> Result<&mut [u8]> {
        unreachable!()
    }
}

#[vtable]
impl rtkit::Operations for AopData {
    type Data = Arc<AopData>;
    type Buffer = NoBuffer;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, ep: u8, msg: u64) {
        let mut rtk = data.rtkit.lock();
        let mut ep_guard = data.endpoints[(ep - AFK_ENDPOINT_START) as usize].lock();
        let ret = ep_guard.recv_message(data, rtk.as_mut().unwrap(), msg);
        if let Err(e) = ret {
            dev_err!(data.dev, "Failed to handle rtkit message, error: {:?}", e);
        }
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _crashlog: Option<&[u8]>) {
        dev_err!(data.dev, "AOP firmware crashed");
    }
}

#[repr(transparent)]
struct AopDriver(Arc<dyn AOP>);

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c_str!("apple,aop")), ())]
);

impl platform::Driver for AopDriver {
    type IdInfo = ();

    const ID_TABLE: platform::IdTable<()> = &OF_TABLE;

    fn probe(pdev: &mut platform::Device, _info: Option<&()>) -> Result<Pin<KBox<AopDriver>>> {
        let dev = pdev.get_device();
        let data = AopData::new(pdev)?;
        let of = dev.of_node().ok_or(EIO)?;
        let alig = of.get_property(c_str!("apple,aop-alignment"))?;
        let aopt = of.get_property(c_str!("apple,aop-target"))?;
        data.patch_bootargs(&[
            (from_fourcc(b"EC0p"), 0x20000),
            (from_fourcc(b"nCal"), 0x0),
            (from_fourcc(b"alig"), alig),
            (from_fourcc(b"AOPt"), aopt),
        ])?;
        let rtkit = rtkit::RtKit::<AopData>::new(&dev, None, 0, data.clone())?;
        *data.rtkit.lock() = Some(rtkit);
        data.start_cpu();
        data.start()?;
        let data = data as Arc<dyn AOP>;
        Ok(KBox::pin(AopDriver(data), GFP_KERNEL)?)
    }
}

impl Drop for AopDriver {
    fn drop(&mut self) {
        self.0.remove();
    }
}

module_platform_driver! {
    type: AopDriver,
    name: "apple_aop",
    license: "Dual MIT/GPL",
}
