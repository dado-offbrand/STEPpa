mod ik;
mod gait;

struct Quadruped {
    back_left: Leg,
    front_left: Leg,
    front_right: Leg,
    back_right: Leg,
    legs: [Leg; 4],
    gait_planner: Gait,
    movement_vector: i32,
}

impl Quadruped {
    pub fn new() -> Self {
        Quadruped {
            back_left: Leg::new("Back Left"),
            front_left: Leg::new("Front Left"),
            front_right: Leg::new("Front Right"),
            back_right: Leg::new("Back Right"),
            gait_planner: Gait::new(&self) // can I do this?? probably not. 
            movement_vector: 1,
        }
    }

    pub fn update_all_poses(&mut self, targets: [f32; 12]) {
        // Wacky technique. You can unpack an array into 
        // a bunch of variables (for clarity)
        let [
            fl_s, fr_s, bl_s, br_s, // Shoulders
            fl_u, fr_u, bl_u, br_u, // Uppers
            fl_l, fr_l, bl_l, br_l  // Lowers
        ] = pose_array;

        self.front_left.update_pose(fl_s, fl_u, fl_l);
        self.front_right.update_pose(fr_s, fr_u, fr_l);
        self.back_left.update_pose(bl_s, bl_u, bl_l);
        self.back_right.update_pose(br_s, br_u, br_l);
    }

    pub fn get_target_tuples(&self) -> .. {
        // DO NOT IMPLEMENT.
        // Phase out this function, it was unused in the python script.
    }

    pub fn get_pose_array(self) -> [f32; 12] {
        let fl_p = self.front_left.get_pose();
        let fr_p = self.front_right.get_pose();
        let bl_p = self.back_left.get_pose();
        let br_p = self.back_right.get_pose();

        let pose: [f32; 12] = [
            fl_p.0, fr_p.0, bl_p.0, br_p.0,
            fl_p.1, fr_p.1, bl_p.1, br_p.1,
            fl_p.2, fr_p.2, bl_p.2, br_p.2,
        ]

        return pose
    }
}

struct Leg {
    pose:[f32; 3],
    targ_x:f32,
    targ_y:f32,
    targ_z:f32,
    group:String,
}

impl Leg {
    // &str allows raw literals and dynamic
    // str instead of String. raw literals used for
    // new values
    pub fn new(group: &str) -> Self {
        let pose: [f32; 3] = [0, ik::UN, ik::LN];
        let targs = ik::leg_ik(pose[0], pose[1], pose[2]);
        
        Leg {
            pose: pose,
            targ_x:targs.0,
            targ_y:targs.1,
            targ_z:targs.2,
            group:group.to_string(),
        }
    }

    pub fn set_foot_target(&mut self, x:f32, y:f32, z:f32) {
        self.targ_x = x;
        self.targ_y = y;
        self.targ_z = z;
    }

    // Sets foot target position
    pub fn get_foot_target(&self) -> (f32, f32, f32) {
        return (self.targ_x, self.targ_y, self.targ_z);
    }

    // Updates pose based off of target position
    pub fn update_pose(&mut self) {
        let targs = ik::leg_ik(self.targ_x, self.targ_y, self.targ_z);
        self.pose[0] = targs.0;
        self.pose[1] = targs.1;
        self.pose[2] = targs.2;
    }

    // Get (a copy of) current pose
    //pub fn get_pose(&self) -> .. {
    //    
    //}
    // edit code to omit this, it's useless in rust as arrays are treated differently
}