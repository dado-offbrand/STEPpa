/*
 * Leg IK CONSTANTS
*/
const LOWER_LEG_LENGTH: f32 = 0.180; // constant because lower leg length was accidentally left too long (subtract foot length!)

const D: f32  = 0.045435;        // shoulder lateral offset
const L1: f32 = 0.134;           // upper leg pulley distance
const L2: f32 = LOWER_LEG_LENGTH; // pulley to center of foot distance

// TODO convert from python np.deg2rad
pub const UN: f32 = -45.0_f32.to_radians(); // upper neutral angle
pub const LN: f32 = -90.0_f32.to_radians(); // lower neutral angle

pub fn leg_ik(x:f32, y:f32, z:f32) -> Result<(f32, f32, f32), String> {
    /// Input: (x, y, z) foot position in the shoulder joint's local frame (metres).
    ///        +x  forward,  +y  outward (positive for left, negative for right),  +z  up
    /// Output: (theta_shoulder, theta_upper, theta_lower) in radians.

    // Shoulder (abduction / adduction)
    let r_yz = y.hypot(z);
    if r_yz < D {
        return Err(format!("Target too close to body axis: r_yz={:.4f} < d={:.4f}", r_yz, D));
    }
    let theta_shoulder = y.atan2(-z) - D.atan2((r_yz.powi(2) - D.powi(2)).sqrt());

    // Project into sagittal plane
    // .max() requires two matching float types; 0.0_f32 explicitly states it is a 32-bit float
    // ^ suggested alternative for python .max
    let l_vert = (r_yz.powi(2) - D.powi(2)).max(0.0_f32).sqrt();
    let r = x.hypot(l_vert);
    
    if r > L1 + L2 {
        return Err(format!("Target unreachable: R={:.4f} > max reach {:.4f}", r, L1 + L2));
    }
    if r < (L1 - L2).abs() {
        return Err(format!("Target unreachable: R={:.4f} < min reach {:.4f}", r, (L1 - L2).abs()));
    }

    // Knee angle (law of cosines); negate for the -X lower-joint axis
    let numerator = r.powi(2) - L1.powi(2) - L2.powi(2);
    let denominator = 2.0 * L1 * L2;
    // Suggested native clamping equivalent to Python's max(-1.0, min(1.0, val))
    let cos_knee = (numerator / denominator).clamp(-1.0, 1.0);
    let theta_lower = -cos_knee.acos();

    // Upper angle
    let theta_knee = -theta_lower;
    let theta_upper = x.atan2(l_vert) - (L2 * theta_knee.sin()).atan2(L1 + L2 * theta_knee.cos());

    return Ok((theta_shoulder, theta_upper, theta_lower))
}

pub fn leg_fk(theta_shoulder:f32, theta_upper:f32, theta_lower:f32) -> (f32, f32, f32) {
    let theta_lower_global = theta_upper - theta_lower;          
    let x = L1 * theta_upper.sin() + L2 * theta_lower_global.sin();
    let down = L1 * theta_upper.cos() + L2 * theta_lower_global.cos();
    let r = D.hypot(down);
    let total_angle = D.atan2(down) + theta_shoulder;
    let y = r * total_angle.sin();
    let z = -r * total_angle.cos();
    
    return (x, y, z)
}

pub fn get_pose_neutral() -> [f32; 12] {
    /// Returns angles for neutral pose
    /// Joint order: Back Left, Front Left, Front Right, Back Right (per group)
    /// Indices:     0          1            2             3           (shoulders)
    ///              4          5            6             7           (uppers)
    ///              8          9            10            11          (lowers)

    let angles: [f32; 12] = [
        0, 0, 0, 0,
        UN, UN, UN, UN,
        LN, LN, LN, LN,
    ]

    return angles
}