#![no_std]

//! The Sharp memory LCD's pixel bus as this board wires it, and the timing the driver keeps
//! (`HARDWARE.md`): the LS012B7DD06A (LCP-2619063C) the watch ships with, or with the `ls014`
//! feature the larger LS014B7DD01 (LCP-0620031B), whose 21-pin FPC carries the same signals on
//! the same pins.

pub mod dma;

/// Gate lines, one a screen column; the panel is square, so also the pixels a line holds.
#[cfg(not(feature = "ls014"))]
pub const LINES: usize = 240;
#[cfg(feature = "ls014")]
pub const LINES: usize = 280;
/// Pixel pairs a line shifts in, top pair first.
pub const PAIRS: usize = LINES / 2;
/// The last `GCK` edge that writes a plane: line `l`'s MSB plane follows edge `2l + 2`, its LSB
/// plane edge `2l + 3`.
#[allow(clippy::cast_possible_truncation)]
pub const LAST_WRITTEN: u32 = 2 * LINES as u32 + 1;
/// The first `GCK` edge after the last line's, which keeps a full half-period like edges 1 and
/// 2; the edge between is fast-forwarded (LS014 Figure 6-5-3).
pub const TAIL_FROM: u32 = LAST_WRITTEN + 2;
/// `GCK` edges a frame makes (Figure 6-5-1): the LS012's ends at 483, the LS014's runs on to 568.
#[cfg(not(feature = "ls014"))]
pub const GCK_EDGES: u32 = TAIL_FROM;
#[cfg(feature = "ls014")]
pub const GCK_EDGES: u32 = TAIL_FROM + 5;
/// `BCK` edges a written plane makes: `BSP`'s, then one for each pair and three more, which
/// latch too (`HARDWARE.md`; the LS014's Figure 6-5-2 numbers them 1 to 144).
pub const BCK_EDGES: usize = PAIRS + 4;
/// Bus words a plane latches, on `BCK` edges 2 onwards.
pub const LATCHED: usize = BCK_EDGES - 1;

const _: () = assert!(GCK_EDGES == if LINES == 240 { 483 } else { 568 } && BCK_EDGES == if LINES == 240 { 124 } else { 144 });

/// The panel's minimums in nanoseconds, from Tables 6-3-2 and 6-3-3 of its datasheet. The
/// LS014's tables also give maximums; they are the typical values plus about 1.5 %, and its
/// own partial update (Figure 6-5-3) holds `INTB` high for less than "the maximum", so only the
/// minimums are kept.
pub mod limits {
    #[cfg(not(feature = "ls014"))]
    mod panel {
        pub const TS_RGB: u32 = 200;
        pub const TH_RGB: u32 = 200;
        pub const TW_BCK: u32 = 400;
        pub const TS_BSP: u32 = 200;
        pub const THW_GEN: u32 = 9_670;
        pub const THH_GCK: u32 = 4_000;
        pub const THW_GCK: u32 = 62_670;
        pub const THS_INTB: u32 = 10_000;
        pub const THS_GSP: u32 = 10_000;
        pub const THH_INTB: u32 = 62_670;
        pub const THW_INTB: u32 = 30_300_000;
        pub const TLW_INTB: u32 = 10_000;
    }
    #[cfg(feature = "ls014")]
    mod panel {
        pub const TS_RGB: u32 = 330;
        pub const TH_RGB: u32 = 330;
        pub const TW_BCK: u32 = 660;
        pub const TS_BSP: u32 = 330;
        pub const THW_GEN: u32 = 29_000;
        /// thGCK1, `GEN` to `GCK`.
        pub const THH_GCK: u32 = 19_100;
        pub const THW_GCK: u32 = 95_040;
        pub const THS_INTB: u32 = 23_760;
        pub const THS_GSP: u32 = 47_520;
        /// From `GCK` 563.
        pub const THH_INTB: u32 = 308_880;
        pub const THW_INTB: u32 = 53_790_000;
        pub const TLW_INTB: u32 = 190_080;
    }
    /// Data set up before a `BCK` edge, and held after it.
    pub use panel::{TH_RGB, TS_RGB};
    /// A `BCK` half-period, and `BSP` either side of `BCK`'s first edge.
    pub use panel::{TS_BSP, TW_BCK};
    /// `GEN` high, then low before the next `GCK` edge.
    pub use panel::{THH_GCK, THW_GEN};
    /// A `GCK` half-period over a written plane, or edge 1, 2 or the tail's.
    pub use panel::THW_GCK;
    /// `INTB` to `GSP`, then `GSP` either side of `GCK`'s first edges.
    pub use panel::{THS_GSP, THS_INTB};
    /// `INTB` high after [`super::TAIL_FROM`], high for the frame, then low.
    pub use panel::{THH_INTB, THW_INTB, TLW_INTB};
    /// A fast-forwarded `GCK` half-period (Note 6-3-1).
    pub const TFAST_GCK: u32 = 1_000;
}

/// Port E: the odd pixel of a pair on bits 0, 2 and 4 (R, G, B), the even one 8 bits up.
pub const DATA_MASK: u32 = 0x1515;
/// Port E's `BSRR` word for a black pair.
pub const BLACK: u32 = DATA_MASK << 16;

/// Port E's `BSRR` word for the pair `odd` above `even`, both ABGR2222, on `plane` (0 the LSB).
#[must_use]
pub const fn word(odd: u8, even: u8, plane: u32) -> u32 {
    let set = ((odd as u32 >> plane) & 0x15) | (((even as u32 >> plane) & 0x15) << 8);
    set | ((!set & DATA_MASK) << 16)
}

/// The gate line that writes screen column `x`.
#[must_use]
pub const fn line(x: usize) -> usize {
    LINES - 1 - x
}

/// A cycle counter, and code run with interrupts masked.
pub trait Clock {
    fn cycles(&mut self) -> u32;
    fn masked<R>(&mut self, f: impl FnOnce(&mut Self) -> R) -> R;
}

/// Waits from the last write: [`Pace::after`]'s final counter read and its write run masked, so an
/// interrupt only lengthens a wait.
pub struct Pace<C> {
    clock: C,
    mark: u32,
}

impl<C: Clock> Pace<C> {
    pub fn new(mut clock: C) -> Self {
        let mark = clock.cycles();
        Self { clock, mark }
    }

    pub fn wait(&mut self, cycles: u32) {
        while self.clock.cycles().wrapping_sub(self.mark) < cycles {}
    }

    /// `write`, once `cycles` have passed since the last write.
    pub fn after(&mut self, cycles: u32, write: impl FnOnce()) {
        self.wait(cycles.saturating_sub(MASKED_SPIN));
        let mark = self.mark;
        self.mark = self.clock.masked(|clock| {
            let now = loop {
                let now = clock.cycles();
                if now.wrapping_sub(mark) >= cycles {
                    break now;
                }
            };
            write();
            now
        });
    }
}

/// The end of a wait [`Pace::after`] spins masked, so a short wait ends on time.
const MASKED_SPIN: u32 = 48;

/// Nanoseconds the bit-banged driver keeps, each at or above its [`limits`].
pub mod timing {
    use super::limits;

    #[cfg(not(feature = "ls014"))]
    mod panel {
        pub const DATA_HOLD: u32 = 300;
        pub const DATA_SETUP: u32 = 300;
        pub const GEN_HIGH: u32 = 11_000;
        pub const GEN_HOLD: u32 = 5_000;
        pub const GCK_HALF: u32 = 70_000;
        pub const GSP_SETUP: u32 = 12_000;
        pub const INTB_HIGH_MS: u32 = 31;
        pub const INTB_LOW: u32 = 12_000;
    }
    #[cfg(feature = "ls014")]
    mod panel {
        pub const DATA_HOLD: u32 = 350;
        pub const DATA_SETUP: u32 = 350;
        pub const GEN_HIGH: u32 = 31_000;
        pub const GEN_HOLD: u32 = 20_000;
        pub const GCK_HALF: u32 = 100_000;
        pub const GSP_SETUP: u32 = 50_000;
        pub const INTB_HIGH_MS: u32 = 55;
        pub const INTB_LOW: u32 = 200_000;
    }
    /// Data held after a `BCK` edge, then set up before the next: a half-period is both.
    pub use panel::{DATA_HOLD, DATA_SETUP};
    /// `GEN` high, then low before the next `GCK` edge.
    pub use panel::{GEN_HIGH, GEN_HOLD};
    /// A `GCK` half-period with no line written, and the last edge to `INTB` low.
    pub use panel::GCK_HALF;
    /// `INTB` to `GSP`, and `GSP` either side of `GCK`'s first edges.
    pub use panel::GSP_SETUP;
    pub use panel::{INTB_HIGH_MS, INTB_LOW};
    /// `BSP` high before `BCK`'s first edge.
    pub const BSP_SETUP: u32 = 1_000;
    /// `BCK` parked low, then `GEN` high.
    pub const GEN_SETUP: u32 = 1_000;
    /// A `GCK` half-period over a line a partial frame keeps.
    pub const GCK_FAST: u32 = 2_000;
    /// What one plane of a written line holds `GCK` for, at the least.
    #[allow(clippy::cast_possible_truncation)]
    pub const WRITTEN_HALF: u32 =
        BSP_SETUP + super::LATCHED as u32 * (DATA_HOLD + DATA_SETUP) + GEN_SETUP + GEN_HIGH + GEN_HOLD;
    /// From [`super::TAIL_FROM`] to `INTB` low: the tail's half-periods, then two more.
    pub const TAIL: u32 = (super::GCK_EDGES - super::TAIL_FROM + 2) * GCK_HALF;

    const _: () = {
        assert!(DATA_HOLD >= limits::TH_RGB && DATA_SETUP >= limits::TS_RGB, "thRGB, tsRGB");
        assert!(DATA_HOLD + DATA_SETUP >= limits::TW_BCK, "thwBCK, tlwBCK");
        assert!(BSP_SETUP >= limits::TS_BSP && DATA_HOLD >= limits::TS_BSP, "thsBSP, tlsBSP");
        assert!(GEN_HIGH >= limits::THW_GEN, "thwGEN");
        assert!(GEN_HOLD >= limits::THH_GCK, "thhGCK");
        assert!(GCK_HALF >= limits::THW_GCK && WRITTEN_HALF >= limits::THW_GCK, "thwGCK, tlwGCK");
        assert!(TAIL >= limits::THH_INTB, "thhINTB");
        assert!(GCK_FAST >= limits::TFAST_GCK, "Note 6-3-1");
        assert!(GSP_SETUP >= limits::THS_GSP && GSP_SETUP >= limits::THS_INTB, "thsINTB, thsGSP, tlsGSP");
        assert!(INTB_HIGH_MS * 1_000_000 >= limits::THW_INTB, "thwINTB");
        assert!(INTB_LOW >= limits::TLW_INTB, "tlwINTB");
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bus as `HARDWARE.md` maps it, one channel at a time.
    fn by_channel(odd: u8, even: u8, plane: u32) -> u32 {
        let bit = |p: u8, channel: u32| (u32::from(p) >> (2 * channel + plane)) & 1;
        let mut set = 0;
        for (channel, pin) in [(0, 0), (1, 2), (2, 4)] {
            set |= bit(odd, channel) << pin;
            set |= bit(even, channel) << (pin + 8);
        }
        set | ((!set & DATA_MASK) << 16)
    }

    #[test]
    fn every_pair_on_both_planes() {
        for odd in 0..=u8::MAX {
            for even in 0..=u8::MAX {
                for plane in 0..2 {
                    assert_eq!(word(odd, even, plane), by_channel(odd, even, plane), "{odd:#04x} {even:#04x} plane {plane}");
                }
            }
        }
    }

    #[test]
    fn a_word_sets_or_clears_every_data_line_and_nothing_else() {
        for odd in 0..=u8::MAX {
            let w = word(odd, !odd, 1);
            assert_eq!((w | (w >> 16)) & 0xFFFF, DATA_MASK);
            assert_eq!(w & (w >> 16), 0);
            assert_eq!(w & !(DATA_MASK | (DATA_MASK << 16)), 0);
        }
        assert_eq!(word(0, 0, 0), BLACK);
        assert_eq!(word(0xC0, 0xC0, 1), BLACK, "alpha is not on the bus");
    }

    /// One tick a counter read or a write; an interrupt of `x` ticks before step `at`, or at
    /// the unmask after it when it falls in masked code.
    struct Model {
        now: u32,
        step: usize,
        at: usize,
        x: u32,
        masked: bool,
        pending: bool,
    }

    impl Model {
        fn tick(&mut self) -> u32 {
            if self.step == self.at {
                if self.masked {
                    self.pending = true;
                } else {
                    self.now += self.x;
                }
            }
            self.step += 1;
            self.now += 1;
            self.now
        }
    }

    struct Shared<'a>(&'a core::cell::RefCell<Model>);

    impl Clock for Shared<'_> {
        fn cycles(&mut self) -> u32 {
            self.0.borrow_mut().tick()
        }

        fn masked<R>(&mut self, f: impl FnOnce(&mut Self) -> R) -> R {
            self.0.borrow_mut().masked = true;
            let r = f(self);
            let mut m = self.0.borrow_mut();
            m.masked = false;
            if m.pending {
                m.pending = false;
                m.now += m.x;
            }
            r
        }
    }

    #[test]
    fn an_interrupt_anywhere_only_lengthens_a_wait() {
        extern crate std;
        use core::cell::RefCell;
        use std::vec::Vec;
        let gaps = [3, 50, 1, 200, 7];
        for x in [1, 2, 9, 60, 10_000] {
            for at in 0..400 {
                let model = RefCell::new(Model { now: 0, step: 0, at, x, masked: false, pending: false });
                let mut writes = Vec::new();
                let mut pace = Pace::new(Shared(&model));
                let start = model.borrow().now;
                for gap in gaps {
                    pace.after(gap, || writes.push(model.borrow_mut().tick()));
                }
                for (i, gap) in gaps.into_iter().enumerate() {
                    let from = if i == 0 { start } else { writes[i - 1] };
                    let took = writes[i] - from;
                    assert!(took >= gap, "interrupt of {x} at step {at}: write {i} {took} after the last, wanted {gap}");
                }
            }
        }
    }

    #[test]
    fn line_zero_is_the_right_edge() {
        assert_eq!(line(LINES - 1), 0);
        assert_eq!(line(0), LINES - 1);
    }
}
