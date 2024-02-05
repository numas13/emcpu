pub trait Extract: Sized {
    fn zextract(&self, pos: u32, len: u32) -> Self;
    fn sextract(&self, pos: u32, len: u32) -> Self;
}

macro_rules! impl_extract {
    ($($uint:ty = $sint:ty),+ $(,)?) => (
        $(impl Extract for $uint {
            fn zextract(&self, pos: u32, len: u32) -> Self {
                ((*self as $uint << (32 - pos - len)) >> (32 - len)) as Self
            }

            fn sextract(&self, pos: u32, len: u32) -> Self {
                ((*self as $uint << (32 - pos - len)) as $sint >> (32 - len)) as Self
            }
        })+
    );
}

impl_extract! {
    u8 = i8,
    u16 = i16,
    u32 = i32,
    u64 = i64,
    u128 = i128,
}

pub fn zextract<T: Extract>(value: T, pos: u32, len: u32) -> T {
    value.zextract(pos, len)
}

pub fn sextract<T: Extract>(value: T, pos: u32, len: u32) -> T {
    value.sextract(pos, len)
}

pub trait Deposit: Sized {
    fn deposit<F: Into<Self>>(&self, pos: u32, len: u32, field: F) -> Self;
}

macro_rules! impl_deposit {
    ($($uint:ty = $sint:ty),+ $(,)?) => {
        $(
            impl Deposit for $uint {
                fn deposit<F: Into<Self>>(&self, pos: u32, len: u32, field: F) -> Self {
                    let mask = (1 as $uint << len).wrapping_sub(1) << pos;
                    (*self & !mask) | ((field.into() << pos) & mask)
                }
            }
         )+
    };
}

impl_deposit! {
    u8 = i8,
    u16 = i16,
    u32 = i32,
    u64 = i64,
    u128 = i128,
}

pub fn deposit<T: Deposit, F: Into<T>>(value: T, pos: u32, len: u32, field: F) -> T {
    value.deposit(pos, len, field)
}
