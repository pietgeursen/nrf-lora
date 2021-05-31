//! Defines a custom `AttributeValueProvider`.

use nrf52840_hal as hal;

use hal::{
    gpio::{Output, Pin, PushPull},
    prelude::OutputPin,
};
use rubble::{Error, att::{AttUuid, Attribute, AttributeAccessPermissions, AttributeProvider, Handle, HandleRange}, bytes::{ByteWriter, ToBytes}, l2cap::Sender, uuid::{Uuid128, Uuid16}};

pub struct LoraMessagingAttrs {
    // Attributes exposed to clients that don't change.
    // This includes the "primary service" and "characteristic" attributes.
    // Some attributes are copied from the declaration of `BatteryServiceAttrs` in the gatt module.
    static_attributes: [Attribute<&'static [u8]>; 6],
    // State and resources to be modified/queried when packets are received.
    // The `AttributeValueProvider` interface allows attributes to be generated lazily; those
    // attributes should use these fields.
    led_pin: Pin<Output<PushPull>>,
    led_buf: [u8; 1],
}

const PRIMARY_SERVICE_UUID16: Uuid16 = Uuid16(0x2800);
const CHARACTERISTIC_UUID16: Uuid16 = Uuid16(0x2803);
const GENERIC_ATTRIBUTE_UUID16: Uuid16 = Uuid16(0x1801);
const BATTERY_LEVEL_UUID16: Uuid16 = Uuid16(0x2A19);

// Randomly generated
/* UUID string: 56596701-5E0B-4C20-883C-981FA47E91F3 */
const LED_UUID128: [u8; 16] = [
    0xF3,0x91,0x7E,0xA4,0x1F,0x98,0x3C,0x88,0x20,0x4C,0x0B,0x5E,0x01,0x67,0x59,0x56
];
// Replace bytes 12/13 (0x6701) of the 128-bit UUID with 6702 
const LED_STATE_CHAR_UUID128: [u8; 16] = [
    0xF3,0x91,0x7E,0xA4,0x1F,0x98,0x3C,0x88,0x20,0x4C,0x0B,0x5E,0x02,0x67,0x59,0x56
    //0xC9, 0x15, 0x15, 0x96, 0x54, 0x56, 0x64, 0xB3, 0x38, 0x45, 0x26, 0x5D, 0xF1, 0x62, 0x6A, 0xA8,
];

const LED_CHAR_DECL_VALUE: [u8; 19] = [
    0x02 | 0x08, // 0x02 = read, 0x08 = write with response
    // 2 byte handle pointing to characteristic value
    0x03,
    0x00,
    // 128-bit UUID of characteristic value (copied from above constant)
    0xF3,0x91,0x7E,0xA4,0x1F,0x98,0x3C,0x88,0x20,0x4C,0x0B,0x5E,0x02,0x67,0x59,0x56
];

impl LoraMessagingAttrs {
    pub fn new(mut led_pin: Pin<Output<PushPull>>) -> Self {
        // Turn off by default (active low)
        led_pin.set_high().unwrap();
        Self {
            static_attributes: [
                Attribute::new(
                    PRIMARY_SERVICE_UUID16.into(),
                    Handle::from_raw(0x0001),
                    &LED_UUID128,
                ),
                Attribute::new(
                    CHARACTERISTIC_UUID16.into(),
                    Handle::from_raw(0x0002),
                    &LED_CHAR_DECL_VALUE,
                ),
                // 0x0003 is skipped because it's lazily generated
                // Dummy ending attribute
                // This needs to come after our lazily generated data attribute because group_end()
                // needs to return a reference
                Attribute::new(
                    GENERIC_ATTRIBUTE_UUID16.into(),
                    Handle::from_raw(0x0004),
                    &[],
                ),
                // Below is copied from `gatt::BatteryServiceAttrs`
                Attribute::new(
                    PRIMARY_SERVICE_UUID16.into(),
                    Handle::from_raw(0x0005),
                    &[0x0F, 0x18], // "Battery Service" = 0x180F
                ),
                Attribute::new(
                    CHARACTERISTIC_UUID16.into(),
                    Handle::from_raw(0x0006),
                    &[
                        0x02, // 1 byte properties: READ = 0x02
                        0x07, 0x00, // 2 bytes handle = 0x0007
                        0x19, 0x2A, // 2 bytes UUID = 0x2A19 (Battery Level)
                    ],
                ),
                // Characteristic value (Battery Level)
                Attribute::new(
                    BATTERY_LEVEL_UUID16.into(),
                    Handle::from_raw(0x0007),
                    &[48u8],
                ),
            ],
            led_pin,
            led_buf: [0u8],
        }
    }
}

impl LoraMessagingAttrs {
    // Lazily produces an attribute to be read/written, representing the LED state.
    fn led_data_attr(&self) -> Attribute<[u8; 1]> {
        Attribute::new(
            Uuid128::from_bytes(LED_STATE_CHAR_UUID128).into(),
            Handle::from_raw(0x0003),
            self.led_buf,
        )
    }
}

impl AttributeProvider for LoraMessagingAttrs {
    /// Retrieves the permissions for attribute with the given handle.
    fn attr_access_permissions(&self, handle: Handle) -> AttributeAccessPermissions {
        match handle.as_u16() {
            0x0003 => AttributeAccessPermissions::ReadableAndWriteable,
            _ => AttributeAccessPermissions::Readable,
        }
    }

    /// Attempts to write data to the attribute with the given handle.
    /// If any of your attributes are writeable, this function must be implemented.
    fn write_attr(&mut self, handle: Handle, data: &[u8]) -> Result<(), Error> {
        match handle.as_u16() {
            0x0003 => {
                if data.is_empty() {
                    return Err(Error::InvalidLength);
                }
                // If we receive a 1, activate the LED; otherwise deactivate it
                // Assumes LED is active low
                if data[0] == 1 {
                    self.led_pin.set_low().unwrap();
                } else {
                    self.led_pin.set_high().unwrap();
                }
                // Copy written value into buffer to display back for reading
                self.led_buf.copy_from_slice(&data[..1]);
                Ok(())
            }
            _ => panic!("Attempted to write an unwriteable attribute"),
        }
    }

    fn is_grouping_attr(&self, uuid: AttUuid) -> bool {
        uuid == PRIMARY_SERVICE_UUID16 || uuid == CHARACTERISTIC_UUID16
    }

    fn group_end(&self, handle: Handle) -> Option<&Attribute<dyn AsRef<[u8]>>> {
        match handle.as_u16() {
            // Handles for the LED primary service and characteristic
            // The group end is a dummy attribute; strictly speaking it's not required
            // but we can't use the lazily generated attribute because this funtion requires
            // returning a reference
            0x0001 | 0x0002 => Some(&self.static_attributes[2]),
            // Handles for Battery Service
            0x0005 | 0x0006 => Some(&self.static_attributes[5]),
            _ => None,
        }
    }

    /// Applies a function to all attributes with handles within the specified range
    fn for_attrs_in_range(
        &mut self,
        range: HandleRange,
        mut f: impl FnMut(&Self, &Attribute<dyn AsRef<[u8]>>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        // Handles start at 1, not 0, but we're not directly indexing
        let start = range.start().as_u16();
        let end = range.end().as_u16();
        let range_u16 = start..=end;
        // Can't just iterate from start to end because of the presence of lazy attributes
        // Ranges are empty if start >= end
        for attr in &self.static_attributes {
            if range_u16.contains(&attr.handle.as_u16()) {
                f(self, attr)?;
            }
        }
        // Check lazy attributes
        // Note that with this implementation, if a static attribute has handle greater than a
        // lazy attribute, the order in which f() is applied is not preserved.
        // This may matter for the purposes of short-circuiting an operation if it cannot be applied
        // to a particular attribute
        if range_u16.contains(&0x0003) {
            f(self, &self.led_data_attr())?;
        };
        Ok(())
    }


    fn find_information(
        &mut self,
        range: HandleRange,
        responder: &mut Sender<'_>,
    ) -> Result<(), Error> {
        self.for_attrs_in_range(range, |_slf, attr|{
            let mut buffer = [0u8; 20];

            buffer[0] = 0x05; // The "find_information" response magic number.

            buffer[1] = match attr.att_type{
                AttUuid::Uuid16(_) => 1,
                AttUuid::Uuid128(_) => 2 
            };
            let handle = attr.handle.as_u16();
            buffer[2] = (handle >> 8) as u8;
            buffer[3] = (handle & 0xFF) as u8;

            let mut byte_write = ByteWriter::new(&mut buffer[..]);
            attr.att_type.to_bytes(&mut byte_write)?;

            
            match attr.att_type{
                AttUuid::Uuid16(_) => {
                    responder.send(&buffer[..6])
                }
                AttUuid::Uuid128(_) => {
                    responder.send(&buffer[..20])
                } 
            }
        })

    }
    
}
