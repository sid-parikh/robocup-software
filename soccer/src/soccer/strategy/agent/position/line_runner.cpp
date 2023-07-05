#include "line_runner.hpp"

namespace strategy {
std::optional<RobotIntent> LineRunner::get_task(RobotIntent intent, const WorldState* world_state,
                                             FieldDimensions field_dimensions) {
                                                                                  
    double y = 2.0;
    if (going_up_) {
        y = 4.0;
    }
    rj_geometry::Point target_pt{(6.0 * (line_num_ * 0.1) - 2.0), y};

    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // face ball on way up, face path on way down
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    if (going_up_) {
        face_option = planning::FaceTarget{};
    }

    // avoid ball
    bool ignore_ball = false;

    planning::LinearMotionInstant goal{target_pt, target_vel};

    intent.motion_command = 
        planning::MotionCommand{"path_target", goal, face_option, ignore_ball};
    return intent;                                   
                                             }
}