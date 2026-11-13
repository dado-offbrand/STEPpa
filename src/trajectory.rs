#[cfg(test)]
mod test {
    use std::fs;
    use super::*;

    #[test]
    pub fn debug_dump() {
        let mut step_points: [(f32, f32, f32); 199] = [(0.0, 0.0, 0.0); 199];
        let mut trajectory: CycloidTrajectory3D = CycloidTrajectory3D::new(0.0, 0.0, 0.1);

        for i in 0..199 {
            let point: (f32, f32, f32) = trajectory.cycle(-0.09, 0.09, -0.045435, 0.07, -0.22, -0.22);
            
            if point.0 > 2.0 {
                println!("OUTLIER!: {}, {}, {}", point.0, point.1, point.2);
            }
            step_points[i] = point;
        }

        fs::write("points.txt", format!("{:?}", step_points));
    }
}

struct CycloidTrajectory3D {
    start_x: f32,
    end_x: f32,
    start_y: f32,
    end_y: f32,
    z_base: f32,
    step_height: f32,
    dt: f32,
    phase: f32,
    duty_cycle: f32,
}

impl CycloidTrajectory3D {
    pub const START_X: f32 = -0.09;
    
    pub fn new(phase_offset: f32, forward_offset: f32, height_offset: f32) -> Self {
        CycloidTrajectory3D {
            start_x: -0.09,
            end_x: 0.09 + forward_offset,
            start_y: 0.0,
            end_y: 0.0,
            z_base: -0.22 + height_offset,
            step_height: 0.03,
            dt: 0.01,
            phase: 0.0 + phase_offset,
            duty_cycle: 0.6
        }
    }

    pub fn set_offsets(&mut self, phase_offset: f32, forward_offset: f32, height_offset:f32) {
        self.phase = 0.0 + phase_offset;
        self.end_x = 0.0 + forward_offset;
        self.z_base = -0.22 + height_offset
        // the 0.0 + offset originated from python to assure it was a float. probably not needed in rust, right?
    }

    pub fn cycle (&mut self, x1: f32, x2: f32, y1: f32, y2: f32, z1: f32, z2: f32) -> (f32, f32, f32) {
        let t: f32 = self.phase % 1.0;
        let x: f32;
        let y: f32;
        let z: f32;

        // swing phase (straight line)
        if t < self.duty_cycle {
            let u: f32 = t / self.duty_cycle;
            x = x2 + (x1 - x2) * u;
            y = y2 + (y1 - y2) * u;
            z = z2 + (z1 - z2) * u;
        } else {
            // swing phase (cycloidal step)
            let u: f32 = (t - self.duty_cycle) / (1.0 - self.duty_cycle);
            x = x1 + (x2 - x1) * u;
            y = y1 + (y2 - y1) * u;

            let A: f32 = self.step_height;

            z = (1.0 - u) * z1 
            + u * z2 
            + A * (1.0 - (2.0 * std::f32::consts::PI * u).cos()) / 2.0;
        }

        self.phase += self.dt;
        return (x, y, z);
    }
}