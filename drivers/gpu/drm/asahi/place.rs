// SPDX-License-Identifier: Apache-2.0 OR MIT

//! "Placement new" macro
//!
//! This cursed abomination of a declarative macro is used to emulate a "placement new" feature,
//! which allows initializing objects directly in a user-provided memory region without first
//! going through the stack.
//!
//! This driver needs to manage several large GPU objects of a fixed layout. Linux kernel stacks are
//! very small, so it is impossible to create these objects on the stack. While the compiler can
//! sometimes optimize away the stack copy and directly instantiate in target memory, this is not
//! guaranteed and not reliable. Therefore, we need some mechanism to ergonomically initialize
//! complex structures directly in a pre-allocated piece of memory.
//!
//! This issue also affects some driver-internal structs which are large/complex enough to overflow
//! the stack. While this can be solved by breaking them up into pieces and using `Box` more
//! liberally, this has performance implications and still isn't very nice. This macro can also be
//! used to solve this issue.
//!
//! # Further reading
//! https://github.com/rust-lang/rust/issues/27779#issuecomment-378416911
//! https://internals.rust-lang.org/t/removal-of-all-unstable-placement-features/7223

/// Initialize a `MaybeUninit` in-place, without constructing the value on the stack first.
///
/// This macro is analogous to `MaybeUninit::write()`. In other words,
/// `place!(foo, bar)` is equivalent to `MaybeUninit::write(foo, bar)`, except that `bar` is not
/// constructed first, but rather its fields (if it is a structure constructor) are copied one by
/// one into the correct location in the `MaybeUninit`.
///
/// The macro supports most Rust initialization syntax including type paths, generic arguments,
/// and nested structures. Nested structures are themselves initialized in-place field by field.
/// `..Default::default()` is supported, but this macro converts it to `..Zeroed::zeroed()`, as it
/// initializes those structs by zero-initializing the underlying memory. Usage of
/// `..Default::default()` with a type not implementing `Zeroed` will result in a compile error.
///
/// Usage:
/// ```
/// let mut buf = MaybeUninit::uninit();
/// let mut_ref = place!(&mut buf, MyStruct {
///     b: true,
///     s: String::from("works"),
///     i: str::parse::<i32>("123").unwrap(),
///     v: vec![String::from("works")],
///     x: foo::MyOtherCoolStruct {
///         a: false,
///         b: String::from("Hello, world!"),
///     },
///     y: foo::MyOtherCoolStruct {
///         a: false,
///         b: String::from("Hello, world!"),
///     },
///     z: foo::MyCoolGenericStruct::<bool, String> {
///         a: false,
///         b: String::from("Hello, world!"),
///     },
/// };
/// // `mut_ref` is now a mutable reference to the `buf`, which is now safely initialized.
/// ```
///
/// Based on https://crates.io/crates/place by DianaNites, with contributions by Joshua Barretto.
#[macro_export]
macro_rules! place {
    // Top-level struct
    (@STRUCT $ptr:ident, _TOP, $typ:path, {$($typ_init:tt)*} { $($fields:tt)* }) => {{
        place!(@STRUCT_ZERO $ptr, {$($typ_init)*} { $($fields)* });
        place!(@STRUCT_CHECK $ptr, {$($typ_init)*} { $($fields)* } {
            place!(@FIELDS $ptr, $($fields)*);
        });
    }};
    // Nested structure
    (@STRUCT $ptr:ident, $f_struct:ident, $typ:path, {$($typ_init:tt)*} { $($fields:tt)* }) => {{
        use core::ptr::addr_of_mut;
        let buf = unsafe { addr_of_mut!((*$ptr).$f_struct) };
        place!(@STRUCT_ZERO buf, {$($typ_init)*} { $($fields)* });
        place!(@STRUCT_CHECK $ptr, {$($typ_init)*} { $($fields)* } {
            place!(@FIELDS buf, $($fields)*);
        });
    }};

    // Zero-initialize structure if the initializer ends in ..default::Default()
    (@STRUCT_ZERO $ptr:ident, {$($typ_init:tt)*} { $($f:ident $(: $v:expr)?),* $(,)? }) => {};
    (@STRUCT_ZERO $ptr:ident, {$($typ_init:tt)*} { $($($f:ident $(: $v:expr)?),*,)? ..Default::default() }) => {{
        // Check that the structure actually implements Zeroed
        const _: () = {
            fn _check_default() {
                let _ = $($typ_init)* {
                    ..Zeroed::zeroed()
                };
            }
        };
        use core::ptr;
        unsafe { ptr::write_bytes($ptr, 0, 1) };

    }};

    // Check that all fields are specified
    (@STRUCT_CHECK $ptr:ident, {$($typ_init:tt)*} { $($($f:ident $(: $v:expr)?),*,)? ..Default::default() } {$($body:tt)*}) => {
        if false {
            #[allow(clippy::redundant_field_names)]
            let _x = $($typ_init)* {
                $($(
                    $f $(: $v)?
                ),*
                ,)?
                ..Zeroed::zeroed()
            };
        } else {
            {$($body)*}
        }
    };
    (@STRUCT_CHECK $ptr:ident, {$($typ_init:tt)*} { $($f:ident $(: $v:expr)?),* $(,)? } {$($body:tt)*}) => {
        if false {
            #[allow(clippy::redundant_field_names)]
            let _x = $($typ_init)* {
                $(
                    $f $(: $v)?
                ),*
            };
        } else {
            {$($body)*}
        }
    };
    // Top-level scalar
    (@SCALAR $ptr:ident, _TOP, $val:expr) => {
        let tmp = $val;
        unsafe { $ptr.write(tmp); }
    };
    // Regular field
    (@SCALAR $ptr:ident, $f:ident, $val:expr) => {{
        use core::ptr::addr_of_mut;
        let tmp = $val;
        unsafe { addr_of_mut!((*$ptr).$f).write(tmp); }
    }};
    // Type-like name followed by braces is a nested structure
    (@PARTIAL $ptr:ident, $f:ident, {$($head:tt)*}, {{ $($fields:tt)* } $($tail:tt)*}) => {
        place!(@STRUCT $ptr, $f, $($head)*, {$($head)*} { $($fields)* });
        place!(@FIELDS $ptr $($tail)*)
    };
    // Type-like name followed by ::ident, append to head
    (@PARTIAL $ptr:ident, $f:ident, {$($head:tt)*}, {::$id:ident $($tail:tt)*}) => {
        place!(@PARTIAL $ptr, $f, {$($head)* :: $id}, {$($tail)*});
    };
    // Type-like name followed by ::<args>, append to head
    (@PARTIAL $ptr:ident, $f:ident, {$($head:tt)*}, {::<$($gen:ty),*> $($tail:tt)*}) => {
        place!(@PARTIAL $ptr, $f, {$($head)* :: <$($gen),*>}, {$($tail)*});
    };
    // Type-like name followed by ::<'lifetime>, append to head
    (@PARTIAL $ptr:ident, $f:ident, {$($head:tt)*}, {::<$li:lifetime> $($tail:tt)*}) => {
        place!(@PARTIAL $ptr, $f, {$($head)* :: <$li>}, {$($tail)*});
    };
    // Anything else, parse it as an expression
    (@PARTIAL $ptr:ident, $f:ident, {$($head:tt)*}, {$($tail:tt)*}) => {
        place!(@EXPR $ptr, $f, $($head)* $($tail)*)
    };
    // Expression followed by more fields
    (@EXPR $ptr:ident, $f:ident, $val:expr, $($tail:tt)*) => {
        place!(@SCALAR $ptr, $f, $val);
        place!(@FIELDS $ptr, $($tail)*)
    };
    // Last field expression, without a trailing comma
    (@EXPR $ptr:ident, $f:ident, $val:expr) => {
        place!(@SCALAR $ptr, $f, $val);
    };
    // Field with a value starting with an ident, start incremental type parsing
    (@FIELDS $ptr:ident, $f:ident : $id:ident $($tail:tt)*) => {
        place!(@PARTIAL $ptr, $f, {$id}, {$($tail)*});
    };
    // Same, but starting with ::ident
    (@FIELDS $ptr:ident, $f:ident : ::$id:ident $($tail:tt)*) => {
        place!(@PARTIAL $ptr, $f, {::$id}, {$($tail)*});
    };
    // Otherwise, parse it as an expression
    (@FIELDS $ptr:ident, $f:ident : $($tail:tt)*) => {
        place!(@EXPR $ptr, $f, $($tail)*)
    };
    // Default terminating case
    (@FIELDS $ptr:ident, ..Default::default() ) => {};
    // Terminating case
    (@FIELDS $ptr:ident $(,)? ) => {};
    (
        $buf:expr,
        $($val:tt)*
    ) => {{
        use core::mem::MaybeUninit;
        // Ensures types are correct
        let obj: &mut MaybeUninit<_> = $buf;
        let top_ptr = obj.as_mut_ptr();
        place!(@FIELDS top_ptr, _TOP: $($val)*);
        // SAFETY: All fields have been initialized above
        // The compiler ensures that all fields were used, all types were correct,
        // and that size and alignment are correct.
        unsafe { obj.assume_init_mut() }
    }};
}

/// Helper macro to get the struct type part of a struct initialization expression.
#[macro_export]
#[doc(hidden)]
macro_rules! get_type {
    ($t:ty { $($val:tt)* }) => {
        $t
    };
}

/// Like `Box::try_new(...)`, but with in-place initialization.
#[macro_export]
macro_rules! box_in_place {
    ($($val:tt)*) => {{
        use $crate::place;
        let b = Box::<$crate::get_type!($($val)*)>::try_new_uninit();
        match b {
            Ok(mut p) => {
                place!((&mut *p), $($val)*);
                Ok(unsafe { p.assume_init() })
            }
            Err(e) => Err(e)
        }
    }};
}

// TODO: figure out how to make this run
#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::MaybeUninit;

    #[derive(Debug, PartialEq)]
    struct MyCoolStruct {
        b: bool,
        s: String,
        i: i32,
        v: Vec<String>,
        x: MyOtherCoolStruct,
        y: MyOtherCoolStruct,
        z: foo::MyCoolGenericStruct<bool, String>,
    }

    #[derive(Debug, PartialEq)]
    struct MyDefaultStruct {
        b: bool,
        i: i32,
        j: i16,
    }
    default_zeroed!(MyDefaultStruct);

    mod foo {
        #[derive(Debug, PartialEq)]
        pub struct MyOtherCoolStruct {
            pub a: bool,
            pub b: String,
        }
        #[derive(Debug, PartialEq)]
        pub struct MyCoolGenericStruct<T, U> {
            pub a: T,
            pub b: U,
        }
    }

    use foo::MyOtherCoolStruct;

    #[test]
    fn test_initialized() {
        let mut buf: MaybeUninit<MyCoolStruct> = MaybeUninit::uninit();

        let x: &mut MyCoolStruct = place!(
            &mut buf,
            MyCoolStruct {
                b: true,
                s: String::from("works"),
                i: str::parse::<i32>("123").unwrap(),
                v: vec![String::from("works")],
                x: MyOtherCoolStruct {
                    a: false,
                    b: String::from("Hello, world!"),
                },
                y: foo::MyOtherCoolStruct {
                    a: false,
                    b: String::from("Hello, world!"),
                },
                z: foo::MyCoolGenericStruct::<bool, String> {
                    a: false,
                    b: String::from("Hello, world!"),
                }
            }
        );
        //dbg!(x);

        assert_eq!(
            x,
            &MyCoolStruct {
                b: true,
                s: String::from("works"),
                i: str::parse::<i32>("123").unwrap(),
                v: vec![String::from("works")],
                x: foo::MyOtherCoolStruct {
                    a: false,
                    b: String::from("Hello, world!"),
                },
                y: foo::MyOtherCoolStruct {
                    a: false,
                    b: String::from("Hello, world!"),
                },
                z: foo::MyCoolGenericStruct::<bool, String> {
                    a: false,
                    b: String::from("Hello, world!"),
                },
            },
        );
    }

    #[test]
    fn test_default() {
        let mut buf: MaybeUninit<MyDefaultStruct> = MaybeUninit::uninit();

        let x: &mut MyDefaultStruct = place!(
            &mut buf,
            MyDefaultStruct {
                b: true,
                i: 1,
                ..Default::default()
            }
        );

        assert_eq!(
            x,
            &MyDefaultStruct {
                b: true,
                i: 1,
                j: 0,
            },
        );
    }

    #[test]
    fn test_scalar() {
        let mut buf: MaybeUninit<u32> = MaybeUninit::uninit();

        let x: &mut u32 = place!(&mut buf, 1234);

        assert_eq!(x, &mut 1234u32);
    }
}
