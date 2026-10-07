//! A fixed-capacity, stack-allocated vector for `Copy` lane data.
//!
//! The kernels keep per-call scratch data (link frames, bounding sphere centers) on the stack.
//! Zero-initializing fixed-size arrays of eight-lane vectors costs kilobytes of stores per
//! kernel call, which profiling showed to be a quarter of the kernel's instructions. This
//! buffer leaves the storage uninitialized and only hands out elements that were written.

use core::mem::MaybeUninit;

/// Stack vector with capacity `N`. Elements `0..len` are always initialized.
pub(crate) struct StackVec<T: Copy, const N: usize> {
    len: usize,
    data: [MaybeUninit<T>; N],
}

impl<T: Copy, const N: usize> StackVec<T, N> {
    /// An empty vector.
    #[inline(always)]
    pub fn new() -> Self {
        StackVec {
            len: 0,
            data: [const { MaybeUninit::uninit() }; N],
        }
    }

    /// Appends an element.
    ///
    /// # Panics
    /// Panics if the vector is full.
    #[inline(always)]
    pub fn push(&mut self, v: T) {
        assert!(self.len < N, "StackVec capacity exceeded");
        self.data[self.len] = MaybeUninit::new(v);
        self.len += 1;
    }

    /// Number of elements.
    #[inline(always)]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Element `i`.
    ///
    /// # Panics
    /// Panics if `i >= len`.
    #[inline(always)]
    pub fn get(&self, i: usize) -> &T {
        assert!(i < self.len, "StackVec index out of bounds");
        // SAFETY: elements below `len` were written by `push` and never moved out (`T: Copy`).
        unsafe { self.data[i].assume_init_ref() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_and_get() {
        let mut v = StackVec::<[f32; 3], 4>::new();
        v.push([1.0, 2.0, 3.0]);
        v.push([4.0, 5.0, 6.0]);
        assert_eq!(v.len(), 2);
        assert_eq!(*v.get(1), [4.0, 5.0, 6.0]);
    }

    #[test]
    #[should_panic]
    fn get_beyond_len_panics() {
        let v = StackVec::<f32, 4>::new();
        v.get(0);
    }
}
