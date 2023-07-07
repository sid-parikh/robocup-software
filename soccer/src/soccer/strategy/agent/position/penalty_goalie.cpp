#include "penalty_goalie.hpp"

namespace strategy {

PenaltyGoalie::PenaltyGoalie(int r_id) : Position(r_id) { position_name_ = "PenaltyGoalie"; }

std::optional<RobotIntent> PenaltyGoalie::derived_get_task(RobotIntent intent) {
    // calculate intercept pt
    WorldState* world_state = this->world_state();

    auto ball_pt = world_state->ball.position;
    auto ball_vel = world_state->ball.velocity;

    auto ball_dir = ball_vel.norm();
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    auto ball_to_bot = ball_pt - robot_position;
    rj_geometry::Point intercept_pt = ball_to_bot.dot(ball_dir) * ball_dir;

    // Stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // Face ball
    planning::PathTargetFaceOption face_option{planning::FaceBall{}};
    // Avoid ball
    bool ignore_ball{false};

    // Create PTMC
    planning::LinearMotionInstant target{intercept_pt, target_vel};
    intent.motion_command =
        planning::MotionCommand{"path_target", target, face_option, ignore_ball};
    return intent;
}

}  // namespace strategy
