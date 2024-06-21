use std::{fmt, ops};

use serde::{Deserialize, Serialize};
use time::Duration;

#[derive(Copy, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(transparent)]
pub struct GET(Duration);

impl GET {
    pub fn new_dhms(days: u32, hours: u8, minutes: u8, seconds: u8, millis: u16) -> Self {
        Self(Duration::new(
            seconds as i64
                + 60 * minutes as i64
                + 60 * 60 * hours as i64
                + 60 * 60 * 24 * days as i64,
            millis as i32 * 1_000_000,
        ))
    }

    pub fn new_hms(hours: u32, minutes: u8, seconds: u8, millis: u16) -> Self {
        Self(Duration::new(
            seconds as i64 + 60 * minutes as i64 + 60 * 60 * hours as i64,
            millis as i32 * 1_000_000,
        ))
    }

    pub fn is_negative(self) -> bool {
        self.0.is_negative()
    }

    #[must_use]
    pub fn negate(self) -> Self {
        Self(-self.0)
    }

    pub fn days(self) -> i64 {
        self.0.whole_days()
    }

    pub fn hours(self) -> u8 {
        (self.0.whole_hours() % 24).unsigned_abs() as u8
    }

    pub fn whole_hours(self) -> i64 {
        self.0.whole_hours()
    }

    pub fn minutes(self) -> u8 {
        (self.0.whole_minutes() % 60).unsigned_abs() as u8
    }

    pub fn seconds(self) -> u8 {
        (self.0.whole_seconds() % 60).unsigned_abs() as u8
    }

    pub fn millis(self) -> u16 {
        (self.0.whole_milliseconds() % 1000).unsigned_abs() as u16
    }

    pub fn into_duration(self) -> Duration {
        self.0
    }

    pub fn from_duration(duration: Duration) -> Self {
        Self(duration)
    }
}

impl ops::Sub<GET> for GET {
    type Output = Duration;

    fn sub(self, rhs: GET) -> Self::Output {
        self.0 - rhs.0
    }
}

impl ops::Sub<Duration> for GET {
    type Output = GET;

    fn sub(self, rhs: Duration) -> Self::Output {
        GET(self.0 - rhs)
    }
}

impl ops::Add<Duration> for GET {
    type Output = GET;

    fn add(self, rhs: Duration) -> Self::Output {
        GET(self.0 + rhs)
    }
}

impl fmt::Display for GET {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.days() > 0 && !f.alternate() {
            write!(
                f,
                "GET({}:{:02}:{:02}:{:02}.{:>03})",
                self.days(),
                self.hours(),
                self.minutes(),
                self.seconds(),
                self.millis()
            )
        } else {
            write!(
                f,
                "GET({:02}:{:02}:{:02}.{:>03})",
                self.whole_hours(),
                self.minutes(),
                self.seconds(),
                self.millis()
            )
        }
    }
}

impl fmt::Debug for GET {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}

#[derive(Copy, Clone, Default, PartialEq, Eq, PartialOrd, Hash, Serialize, Deserialize)]
#[repr(transparent)]
pub struct UT(Duration);

impl UT {
    pub fn new_dhms(days: u32, hours: u8, minutes: u8, seconds: u8, millis: u16) -> Self {
        Self(Duration::new(
            seconds as i64
                + 60 * minutes as i64
                + 60 * 60 * hours as i64
                + 60 * 60 * 24 * days as i64,
            millis as i32 * 1_000_000,
        ))
    }

    pub fn new_hms(hours: u32, minutes: u8, seconds: u8, millis: u16) -> Self {
        Self(Duration::new(
            seconds as i64 + 60 * minutes as i64 + 60 * 60 * hours as i64,
            millis as i32 * 1_000_000,
        ))
    }

    pub fn is_negative(self) -> bool {
        self.0.is_negative()
    }

    #[must_use]
    pub fn negate(self) -> Self {
        Self(-self.0)
    }

    pub fn days(self) -> i64 {
        self.0.whole_days()
    }

    pub fn hours(self) -> u8 {
        (self.0.whole_hours() % 24).unsigned_abs() as u8
    }

    pub fn whole_hours(self) -> i64 {
        self.0.whole_hours()
    }

    pub fn minutes(self) -> u8 {
        (self.0.whole_minutes() % 60).unsigned_abs() as u8
    }

    pub fn seconds(self) -> u8 {
        (self.0.whole_seconds() % 60).unsigned_abs() as u8
    }

    pub fn millis(self) -> u16 {
        (self.0.whole_milliseconds() % 1000).unsigned_abs() as u16
    }

    pub fn into_duration(self) -> Duration {
        self.0
    }

    pub fn from_duration(duration: Duration) -> Self {
        Self(duration)
    }

    pub fn new_seconds(sec: f64) -> UT {
        UT::from_duration(Duration::seconds_f64(sec))
    }
}

impl ops::Sub<UT> for UT {
    type Output = Duration;

    fn sub(self, rhs: UT) -> Self::Output {
        self.0 - rhs.0
    }
}

impl ops::Sub<Duration> for UT {
    type Output = UT;

    fn sub(self, rhs: Duration) -> Self::Output {
        UT(self.0 - rhs)
    }
}

impl ops::Add<Duration> for UT {
    type Output = UT;

    fn add(self, rhs: Duration) -> Self::Output {
        UT(self.0 + rhs)
    }
}

impl fmt::Display for UT {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "UT({}s)", self.0.as_seconds_f64())
    }
}

impl fmt::Debug for UT {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}
