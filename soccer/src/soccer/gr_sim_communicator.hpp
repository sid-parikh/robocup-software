#pragma once

#include <rj_protos/grSim_Commands.pb.h>
#include <rj_protos/grSim_Packet.pb.h>
#include <rj_protos/grSim_Replacement.pb.h>

#include <context.hpp>
#include <rj_geometry/transform_matrix.hpp>
#include <node.hpp>
#include <boost/asio.hpp>

class GrSimCommunicator : public Node {
public:
    explicit GrSimCommunicator(Context* context);

    void send_sim_command(const grSim_Packet& cmd);

    // Places the ball at a position on the screen
    void place_ball(QPointF pos, rj_geometry::TransformMatrix screen_to_world);

    void run() override;

private:
    Context* context_;
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket asio_socket_;
    boost::asio::ip::udp::endpoint grsim_endpoint_;
};