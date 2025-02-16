// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP lid angle sensor driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings, c_str,
    iio::common::aop_sensors::{AopSensorData, IIORegistration, MessageProcessor},
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::aop::{EPICService, AOP},
    sync::Arc,
    types::ForeignOwnable,
};

struct MsgProc;

impl MessageProcessor for MsgProc {
    fn process(&self, message: &[u8]) -> u32 {
        message[1] as u32
    }
}

#[repr(transparent)]
struct IIOAopLasDriver(IIORegistration<MsgProc>);

kernel::of_device_table!(OF_TABLE, MODULE_OF_TABLE, (), [] as [(of::DeviceId, ()); 0]);

impl platform::Driver for IIOAopLasDriver {
    type IdInfo = ();

    const ID_TABLE: platform::IdTable<()> = &OF_TABLE;

    fn probe(
        pdev: &mut platform::Device,
        _info: Option<&()>,
    ) -> Result<Pin<KBox<IIOAopLasDriver>>> {
        let dev = pdev.get_device();
        let parent = dev.parent().unwrap();
        // SAFETY: our parent is AOP, and AopDriver is repr(transparent) for Arc<dyn Aop>
        let adata_ptr = unsafe { Pin::<KBox<Arc<dyn AOP>>>::borrow(parent.get_drvdata()) };
        let adata = (&*adata_ptr).clone();
        // SAFETY: AOP sets the platform data correctly
        let service = unsafe { *((*dev.as_raw()).platform_data as *const EPICService) };

        let ty = bindings::BINDINGS_IIO_ANGL;
        let data = AopSensorData::new(dev, ty, MsgProc)?;
        adata.add_fakehid_listener(service, data.clone())?;
        let info_mask = 1 << bindings::BINDINGS_IIO_CHAN_INFO_RAW;
        Ok(KBox::pin(
            IIOAopLasDriver(IIORegistration::<MsgProc>::new(
                data,
                c_str!("aop-sensors-las"),
                ty,
                info_mask,
                &THIS_MODULE,
            )?),
            GFP_KERNEL,
        )?)
    }
}

module_platform_driver! {
    type: IIOAopLasDriver,
    name: "iio_aop_las",
    license: "Dual MIT/GPL",
    alias: ["platform:iio_aop_las"],
}
