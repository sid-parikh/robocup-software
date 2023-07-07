#include "penalty_goalie.hpp"

namespace strategy {

PenaltyGoalie::PenaltyGoalie(int r_id) : Position(r_id) {
    position_name_ = "PenaltyGoalie";
}

std::optional<RobotIntent> PenaltyGoalie::derived_get_task(RobotIntent intent) {
    // Create Motion Command
    intent.motion_command =
        planning::MotionCommand{"goalie_idle"};
    return intent;
}

}  // namespace strategy
