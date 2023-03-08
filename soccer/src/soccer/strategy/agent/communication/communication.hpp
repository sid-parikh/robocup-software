#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/acknowledge.hpp"
#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/msg/agent_response.hpp"
#include "rj_msgs/msg/agent_response_variant.hpp"
#include "rj_msgs/msg/pass_request.hpp"
#include "rj_msgs/msg/pass_response.hpp"
#include "rj_msgs/msg/position_request.hpp"
#include "rj_msgs/msg/position_response.hpp"
#include "rj_msgs/msg/test_request.hpp"
#include "rj_msgs/msg/test_response.hpp"

namespace strategy::communication {

// BEGIN REQUEST TYPES //

struct PassRequest {
    u_int32_t request_uid;
};
bool operator==(const PassRequest& a, const PassRequest& b);

struct PositionRequest {
    u_int32_t request_uid;
};
bool operator==(const PositionRequest& a, const PositionRequest& b);

struct TestRequest {
    u_int32_t request_uid;
};
bool operator==(const TestRequest& a, const TestRequest& b);

using AgentRequest = std::variant<PassRequest, TestRequest, PositionRequest>;

// END REQUEST TYPES //

// BEGIN RESPONSE TYPES //

struct Acknowledge {
    u_int32_t response_uid;
};
bool operator==(const Acknowledge& a, const Acknowledge& b);

struct PassResponse {
    u_int32_t response_uid;
};
bool operator==(const PassResponse& a, const PassResponse& b);

struct PositionResponse {
    u_int32_t response_uid;
    std::string position;
};
bool operator==(const PositionResponse& a, const PositionResponse& b);

struct TestResponse {
    u_int32_t response_uid;
    std::string message;
};
bool operator==(const TestResponse& a, const TestResponse& b);

using AgentResponseVariant =
    std::variant<Acknowledge, PassResponse, PositionResponse, TestResponse>;

struct AgentResponse {
    AgentRequest associated_request;
    AgentResponseVariant response;
};
bool operator==(const AgentResponse& a, const AgentResponse& b);

// END RESPONSE TYPES //

/**
 * @brief Wraps a communication request by giving the intended destination of the communication.
 *
 */
struct PosAgentRequestWrapper {
    AgentRequest request;  // The request to be send
    std::vector<u_int8_t>
        target_agents;  // The target receivers of the message (unnecessary in broadcast)
    bool broadcast;     // Denotes whether or not the message should be sent to all robots.
    bool urgent;        // If urgent, first response is sent through (others are dropped)
};

/**
 * @brief Wraps a communication response to ensure symmetry for agent-to-agent communication.
 *
 */
struct PosAgentResponseWrapper {
    AgentResponseVariant response;
};

/**
 * @brief Wraps a communication request to ensure symmetry for agent-to-agent communication.
 *
 */
struct AgentPosRequestWrapper {
    AgentRequest request;
};

/**
 * @brief Wraps a communication response by giving the robot the communication is from.
 *
 */
struct AgentPosResponseWrapper {
    AgentRequest associated_request;
    std::vector<u_int8_t> from_robot_ids;
    std::vector<u_int8_t> received_robot_ids;
    bool broadcast;
    bool urgent;
    RJ::Time created;
    std::vector<AgentResponseVariant> responses;
};

// TODO (https://app.clickup.com/t/8677c0tqe): Make this templated and less ugly
void generate_uid(PassRequest& request);
void generate_uid(PositionRequest& request);
void generate_uid(TestRequest& request);

void generate_uid(Acknowledge& response);
void generate_uid(PassResponse& response);
void generate_uid(PositionResponse& response);
void generate_uid(TestResponse& response);

}  // namespace strategy::communication

namespace rj_convert {

// BEGIN REQUEST TYPES //

template <>
struct RosConverter<strategy::communication::PassRequest, rj_msgs::msg::PassRequest> {
    static rj_msgs::msg::PassRequest to_ros(const strategy::communication::PassRequest& from) {
        rj_msgs::msg::PassRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static strategy::communication::PassRequest from_ros(const rj_msgs::msg::PassRequest& from) {
        return strategy::communication::PassRequest{from.request_uid};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassRequest, rj_msgs::msg::PassRequest);

template <>
struct RosConverter<strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest> {
    static rj_msgs::msg::PositionRequest to_ros(
        const strategy::communication::PositionRequest& from) {
        rj_msgs::msg::PositionRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static strategy::communication::PositionRequest from_ros(
        const rj_msgs::msg::PositionRequest& from) {
        return strategy::communication::PositionRequest{from.request_uid};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest);

template <>
struct RosConverter<strategy::communication::TestRequest, rj_msgs::msg::TestRequest> {
    static rj_msgs::msg::TestRequest to_ros(const strategy::communication::TestRequest& from) {
        rj_msgs::msg::TestRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static strategy::communication::TestRequest from_ros(const rj_msgs::msg::TestRequest& from) {
        return strategy::communication::TestRequest{from.request_uid};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::TestRequest, rj_msgs::msg::TestRequest);

template <>
struct RosConverter<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest> {
    static rj_msgs::msg::AgentRequest to_ros(const strategy::communication::AgentRequest& from) {
        rj_msgs::msg::AgentRequest result;
        if (const auto* test_request = std::get_if<strategy::communication::TestRequest>(&from)) {
            result.test_request.emplace_back(convert_to_ros(*test_request));
        } else if (const auto* position_request =
                       std::get_if<strategy::communication::PositionRequest>(&from)) {
            result.position_request.emplace_back(convert_to_ros(*position_request));
        } else if (const auto* pass_request =
                       std::get_if<strategy::communication::PassRequest>(&from)) {
            result.pass_request.emplace_back(convert_to_ros(*pass_request));
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
        return result;
    }

    static strategy::communication::AgentRequest from_ros(const rj_msgs::msg::AgentRequest& from) {
        strategy::communication::AgentRequest result;
        if (!from.test_request.empty()) {
            result = convert_from_ros(from.test_request.front());
        } else if (!from.position_request.empty()) {
            result = convert_from_ros(from.position_request.front());
        } else if (!from.pass_request.empty()) {
            result = convert_from_ros(from.pass_request.front());
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest);

// END REQUEST TYPES //

// BEGIN RESPONSE TYPES //

template <>
struct RosConverter<strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge> {
    static rj_msgs::msg::Acknowledge to_ros(const strategy::communication::Acknowledge& from) {
        rj_msgs::msg::Acknowledge result;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::Acknowledge from_ros(const rj_msgs::msg::Acknowledge& from) {
        return strategy::communication::Acknowledge{from.response_uid};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge);

template <>
struct RosConverter<strategy::communication::PassResponse, rj_msgs::msg::PassResponse> {
    static rj_msgs::msg::PassResponse to_ros(const strategy::communication::PassResponse& from) {
        rj_msgs::msg::PassResponse result;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::PassResponse from_ros(const rj_msgs::msg::PassResponse& from) {
        strategy::communication::PassResponse result{};
        result.response_uid = from.response_uid;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassResponse, rj_msgs::msg::PassResponse);

template <>
struct RosConverter<strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse> {
    static rj_msgs::msg::PositionResponse to_ros(
        const strategy::communication::PositionResponse& from) {
        rj_msgs::msg::PositionResponse result;
        result.position = from.position;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::PositionResponse from_ros(
        const rj_msgs::msg::PositionResponse& from) {
        strategy::communication::PositionResponse result;
        result.position = from.position;
        result.response_uid = from.response_uid;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse);

template <>
struct RosConverter<strategy::communication::TestResponse, rj_msgs::msg::TestResponse> {
    static rj_msgs::msg::TestResponse to_ros(const strategy::communication::TestResponse& from) {
        rj_msgs::msg::TestResponse result;
        result.message = from.message;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::TestResponse from_ros(const rj_msgs::msg::TestResponse& from) {
        strategy::communication::TestResponse result;
        result.message = from.message;
        result.response_uid = from.response_uid;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::TestResponse, rj_msgs::msg::TestResponse);

template <>
struct RosConverter<strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse> {
    static rj_msgs::msg::AgentResponse to_ros(const strategy::communication::AgentResponse& from) {
        rj_msgs::msg::AgentResponse result;
        result.associated_request = convert_to_ros(from.associated_request);
        if (const auto* acknowledge_response =
                std::get_if<strategy::communication::Acknowledge>(&(from.response))) {
            result.response.acknowledge_response.emplace_back(
                convert_to_ros(*acknowledge_response));
        } else if (const auto* test_response =
                       std::get_if<strategy::communication::TestResponse>(&(from.response))) {
            result.response.test_response.emplace_back(convert_to_ros(*test_response));
        } else if (const auto* position_response =
                       std::get_if<strategy::communication::PositionResponse>(&(from.response))) {
            result.response.position_response.emplace_back(convert_to_ros(*position_response));
        } else if (const auto* pass_response =
                       std::get_if<strategy::communication::PassResponse>(&(from.response))) {
            result.response.pass_response.emplace_back(convert_to_ros(*pass_response));
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
        return result;
    }

    static strategy::communication::AgentResponse from_ros(
        const rj_msgs::msg::AgentResponse& from) {
        strategy::communication::AgentResponse result;
        result.associated_request = convert_from_ros(from.associated_request);
        if (!from.response.acknowledge_response.empty()) {
            result.response = convert_from_ros(from.response.acknowledge_response.front());
        } else if (!from.response.test_response.empty()) {
            result.response = convert_from_ros(from.response.test_response.front());
        } else if (!from.response.position_response.empty()) {
            result.response = convert_from_ros(from.response.position_response.front());
        } else if (!from.response.pass_response.empty()) {
            result.response = convert_from_ros(from.response.pass_response.front());
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse);

// END RESPONSE TYPES //

}  // namespace rj_convert