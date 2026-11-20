#![allow(unused)] // this code is in development, ofc some of it is unused

mod types;
mod trajectory;
mod ik;

use crate::types::*;
use crate::trajectory::*;

pub fn main() {
    println!("blah");
}

struct Quadruped {
    back_left: Leg,
    front_left: Leg,
    front_right: Leg,
    back_right: Leg,
    movement_vector: i32,
    forward_vector: i32,
    side_vector: i32,
    rotational_vector: i32,
    stride: f32,
    rotational_stride: f32,
}

impl Quadruped {
    pub fn new() -> Self {
        Quadruped {
            back_left: Leg::new("Back Left", 0.0, 0.0, 0.01),
            front_left: Leg::new("Front Left", 0.5, 0.0, 0.01),
            front_right: Leg::new("Front Right", 0.0, 0.0, 0.01),
            back_right: Leg::new("Back Right", 0.5, 0.0, 0.01),
            movement_vector: 1,
            forward_vector: 0,
            side_vector: 0,
            rotational_vector: 0,
            stride: 0.03,
            rotational_stride: 0.03,
        }
    }

    // Updates every leg in the quadruped for a tick of its state
    pub fn update(&mut self) {
        self.back_left.update();
        self.front_left.update();
        self.front_right.update();
        self.back_right.update();
    }

    
    // Allows the pose of the entire robot to be gotten for simulation, logging,
    // etc.
    pub fn get_pose_array(self) -> [f32; 12] {
        let fl_p = self.front_left.pose.get_pose_rad();
        let fr_p = self.front_right.pose.get_pose_rad();
        let bl_p = self.back_left.pose.get_pose_rad();
        let br_p = self.back_right.pose.get_pose_rad();

        let pose: [f32; 12] = [
            fl_p[0], fr_p[0], bl_p[0], br_p[0],
            fl_p[1], fr_p[1], bl_p[1], br_p[1],
            fl_p[2], fr_p[2], bl_p[2], br_p[2],
        ];

        return pose;
    }

    // OLD GAIT CODE THAT NEEDS IMPLEMENTED?
    // Returns the x/y beginning/end target xy position for the feet
    // to account for quadruped rotation. This is a cheap alternate
    // to a full IK model.
    // pub fn get_xy_ctargs(&self) -> {
    //     // returns an array of 4-tuple pairs for each leg in order
    //     // figure out how to define that.... tuple([f32;4]) somehow?
    // }

    // pub fn cycle(&self) -> [f32; 12] {
    //     // code is less simple here because tp's are no longer
    //     // stored in an array. that is probably fine for now
    // }
}

struct Leg {
    pose: Pose,
    targs: Target,
    group:String,
    trajectory:CycloidTrajectory3D,
}

impl Leg {
    pub fn new(group: &str, phase_offset: f32, forward_offset: f32, height_offset: f32) -> Self {
        let pose: Pose = Pose::new(0.0, ik::UN, ik::LN);
        let pose_arr = pose.get_pose_rad();
        let temp_targs: (f32, f32, f32) = ik::leg_ik(pose_arr[0], pose_arr[1], pose_arr[2]).unwrap();
        // ^ if this fails and panics, I want the entire program to crash and stop sending info
        // to the motor.
        let targs: Target = Target::new(temp_targs.0, temp_targs.1, temp_targs.2);
        let trajectory: CycloidTrajectory3D = CycloidTrajectory3D::new(phase_offset, forward_offset, height_offset);

        Leg {
            pose: pose,
            targs: targs,
            group: group.to_string(),
            trajectory: trajectory,
        }
    }

    // Updates the entire leg 
    pub fn update(&mut self) {
        // current targs
        //let ctargs: (f32, f32, f32) = self.trajectory.cycle(ARGS HERE PUT ARGS HERE);
        let ctargs: (f32, f32, f32) = (0.0, 0.0, 0.0);

        // update self.targs based off of new targs
        self.targs.set(ctargs.0, ctargs.1, ctargs.2);

        // get new current pose based off of new targs
        let cpose: (f32, f32, f32) = ik::leg_ik(ctargs.0, ctargs.1, ctargs.2).unwrap();
        // ^ again, if this panics, I would prefer it crashes the program as its the fastest
        // way to stop sending packets to the motor

        // update the pose to the current pose
        self.pose.set_pose_rad(cpose.0, cpose.1, cpose.2);
    }
}