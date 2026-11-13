mod trajectory;

struct Gait {
    front_left_tp: CycloidTrajectory3D,
    front_right_tp: CycloidTrajectory3D,
    back_left_tp: CycloidTrajectory3D,
    back_right_tp: CycloidTrajectory3D,
    forward_vector: i32,
    side_vector: i32,
    rotational_vector: i32,
    stride: f32,
    rotational_stride: f32,
    quad: Quadruped,
}

impl Gait {
    // can I pass the quadruped instance in as an alias here?? uh oh...
    // if not, just pass the object through at every function ig? would
    // that work?
    pub fn new(quad: Quadruped) -> self {
        let fl_tp = CycloidTrajectory3D::new(0.5, 0, 0.01);
        let fr_tp = CycloidTrajectory3D::new(0, 0, 0.01);
        let bl_tp = CycloidTrajectory3D::new(0, 0, 0.01);
        let br_tp = CycloidTrajectory3D::new(0.5, 0, 0.01);
        
        Gait {
            front_left_tp: fl_tp,
            front_right_tp: fr_tp,
            back_left_tp: bl_tp,
            back_right_tp: br_tp,
            forward_vector: 0,
            side_vector: 0,
            rotational_vector: 0,
            stride: 0.03,
            rotational_stride: 0.03,
            quad: quad,
        }
    }

    // Returns the x/y beginning/end target xy position for the feet
    // to account for quadruped rotation. This is a cheap alternate
    // to a full IK model.
    pub fn get_xy_ctargs(&self) -> {
        // returns an array of 4-tuple pairs for each leg in order
        // figure out how to define that.... tuple([f32;4]) somehow?
    }

    pub fn cycle(&self) -> [f32; 12] {
        // code is less simple here because tp's are no longer
        // stored in an array. that is probably fine for now
    }
}