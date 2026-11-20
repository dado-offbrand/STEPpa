#![allow(unused)] 
// this entire class is meant to be a separate file with instantiable
// util instances, so ignore all unused warnings
#[cfg(test)]
mod test {
    use super::*;

    #[test]
    pub fn test_targs() {
        let mut targs: Target = Target::new(0.0, 0.0, 0.0);
        println!("Targets initialized to: {}", format!("{:?}",targs.get()));
        assert!(targs.get()==[0.0, 0.0, 0.0]);

        targs.set_x(1.0);
        println!("Setting x to 1.0.. Result: {}", format!("{:?}", targs.get()));
        assert!(targs.get()[0]==1.0);

        targs.set_y(1.0);
        println!("Setting y to 1.0.. Result: {}", format!("{:?}", targs.get()));
        assert!(targs.get()[1]==1.0);

        targs.set_z(1.0);
        println!("Setting z to 1.0.. Result: {}", format!("{:?}", targs.get()));
        assert!(targs.get()[2]==1.0);

        targs.set(2.0, 2.0, 2.0);
        println!("Setting xyz to 2.0.. Result {}", format!("{:?}", targs.get()));
        assert!(targs.get()==[2.0, 2.0, 2.0]);
    }

    #[test]
    pub fn test_pose() {
        let mut pose: Pose = Pose::new(90.0, 90.0, 90.0);
        println!("Pose initialized to: {}", format!("{:?}",pose.get_pose_deg()));
        assert!(pose.get_pose_deg()==[90.0, 90.0, 90.0]);

        pose.set_shoulder(45.0);
        println!("Setting shoulder to 45.0.. Result: {}", format!("{:?}", pose.get_pose_deg()));
        assert!(pose.get_pose_deg()[0]==45.0);

        pose.set_upper(45.0);
        println!("Setting upper to 45.0.. Result: {}", format!("{:?}", pose.get_pose_deg()));
        assert!(pose.get_pose_deg()[1]==45.0);

        pose.set_lower(45.0);
        println!("Setting lower to 45.0.. Result: {}", format!("{:?}", pose.get_pose_deg()));
        assert!(pose.get_pose_deg()[2]==45.0);

        pose.set_pose_deg(45.0, 45.0, 45.0);
        println!("Setting entire pose to 45.0, 45.0, 45.0.. Result: {}", format!("{:?}", pose.get_pose_deg()));
        assert!(pose.get_pose_deg()==[45.0, 45.0, 45.0])
    }
}

pub struct Pose {
    shoulder:f32,
    upper:f32,
    lower:f32,
}

impl Pose {
    // Constructor that creates a new pose
    pub fn new (shoulder:f32, upper:f32, lower:f32) -> Self {
        Pose {
            shoulder:shoulder,
            upper:upper,
            lower:lower,
        }
    }

    // Set shoulder pose specifically
    pub fn set_shoulder(&mut self, shoulder:f32) {
        self.shoulder = shoulder;
    }

    // Set upper leg pose specifically
    pub fn set_upper(&mut self, upper:f32) {
        self.upper = upper;
    }

    // Set lower leg pose specifically
    pub fn set_lower(&mut self, lower:f32) {
        self.lower = lower;
    }

    // Set entire pose (DEGREES)
    pub fn set_pose_deg(&mut self, shoulder:f32, upper:f32, lower:f32) {
        self.set_shoulder(shoulder);
        self.set_upper(upper);
        self.set_lower(lower);
    }

    // Set entire pose (RADIANS)
    pub fn set_pose_rad(&mut self, shoulder:f32, upper:f32, lower:f32) {
        self.set_shoulder(shoulder.to_radians());
        self.set_upper(upper.to_radians());
        self.set_lower(lower.to_radians());
    }

    // Get entire pose as shoulder, upper, lower (DEGREES)
    pub fn get_pose_deg(&self) -> [f32; 3] {
        let array: [f32; 3] = [
            self.shoulder, self.upper, self.lower,
        ];

        return array;
    }

    // Get entire pose as shoulder, upper, lower (RADIAN)
    pub fn get_pose_rad(&self) -> [f32; 3] {
        let array: [f32; 3] = [
            self.shoulder.to_radians(),
            self.upper.to_radians(),
            self.lower.to_radians(),
        ];

        return array;
    }
}

pub struct Target {
    x:f32,
    y:f32,
    z:f32,
}

impl Target {
    // Constructor that creates new set of targets
    pub fn new(x:f32, y:f32, z:f32) -> Self {
        Target {
            x:x,
            y:y,
            z:z,
        }
    }

    // Set x targ specifically
    pub fn set_x(&mut self, x:f32) {
        self.x = x;
    }

    // Set y targ specifically
    pub fn set_y(&mut self, y:f32) {
        self.y = y;
    }

    // Set z targ specifically
    pub fn set_z(&mut self, z:f32) {
        self.z = z;
    }

    // Set all targs as x, y, z
    pub fn set(&mut self, x:f32, y:f32, z:f32) {
        self.set_x(x);
        self.set_y(y);
        self.set_z(z);
    }

    // Get all targs as x, y, z
    pub fn get(&self) -> [f32; 3] {
        let array: [f32; 3] = [
            self.x, self.y, self.z,
        ];

        return array;
    }
}