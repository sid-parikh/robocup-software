#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/*
 * Stateless position that idles only on the baseline
 */
class PenaltyGoalie : public Position {
public:
    PenaltyGoalie(int r_id);
    ~PenaltyGoalie() override = default;

    // Do not particpate in passing
    void derived_acknowledge_pass() override{};
    void derived_pass_ball() override{};
    void derived_acknowledge_ball_in_transit() override{};

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy
