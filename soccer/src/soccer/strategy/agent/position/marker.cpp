#include "marker.hpp"

namespace strategy {
Marker::Marker() {}
std::optional<RobotIntent> Marker::get_task(RobotIntent intent, const WorldState* world_state,
                                            FieldDimensions field_dimensions) {
    auto ball_position = world_state->ball.position;

    int closestId = 0;
    int closestDist = 9999999;
    // FInds closest enemy to goal
    for (int i = 0; i < kNumShells; i++) {
        RobotState robot = world_state->get_robot(false, i);
        if (robot.visible) {
            if ((robot.pose.position() - ball_position).mag() < closestDist) {
                closestId = i;
                closestDist = (robot.pose.position() - ball_position).mag();
            }
        }
    }
    float box_w{field_dimensions.penalty_long_dist()};
    float box_h{field_dimensions.penalty_short_dist()};
    float line_w{field_dimensions.line_width()};
    double min_wall_rad{(kRobotRadius * 4.0f) + line_w +
                        hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)))};

    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // planning::LinearMotionInstant goal{
    //     (world_state->ball.position - ball_position) * factor + ball_position,
    //     rj_geometry::Point{0.0, 0.0}};
    auto target_position =
        (world_state->get_robot(false, closestId).pose.position() - ball_position) * factor +
        ball_position;

    if (target_position.mag() < 1.2 * min_wall_rad) {
        target_position = target_position.normalized(1.2 * min_wall_rad);
    }
    planning::LinearMotionInstant goal{target_position, rj_geometry::Point{0.0, 0.0}};

    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, false};
    return intent;
}
}  // namespace strategy