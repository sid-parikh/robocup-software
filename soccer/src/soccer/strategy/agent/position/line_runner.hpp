
#include <rclcpp/rclcpp.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "role_interface.hpp"

namespace strategy {

class LineRunner : public RoleInterface {
public:
    LineRunner(int line_num, bool going_up)
        : line_num_(line_num),
          going_up_(going_up){

          };
    ~LineRunner() = default;

    std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* world_state,
                                        FieldDimensions field_dimensions) override;

private:
    int line_num_;
    bool going_up_;
};
}  // namespace strategy