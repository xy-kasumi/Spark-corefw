// Motor pin assignments for BTT Octopus Pro v1.1.
// motor7 (PA14 DIR) is omitted because PA14 doubles as SWCLK.
//
// |  m# | step | dir  | en   | uart | diag |
// | --- | ---- | ---- | ---- | ---- | ---- |
// |  m0 | PF13 | PF12 | PF14 | PC4  | PG6  |
// |  m1 | PG0  | PG1  | PF15 | PD11 | PG9  |
// |  m2 | PF11 | PG3  | PG5  | PC6  | PG10 |
// |  m3 | PG4  | PC1  | PA0  | PC7  | PG11 |
// |  m4 | PF9  | PF10 | PG2  | PF2  | PG12 |
// |  m5 | PC13 | PF0  | PF1  | PE4  | PG13 |
// |  m6 | PE2  | PE3  | PD4  | PE1  | PG14 |

pub const NUM_MOTORS: usize = 7;
pub const MOTOR_NAMES: [&str; NUM_MOTORS] = ["m0", "m1", "m2", "m3", "m4", "m5", "m6"];
