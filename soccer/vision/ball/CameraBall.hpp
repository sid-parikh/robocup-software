#pragma once

#include <list>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

/**
 * Wrapper for the protobuf observation
 */
class CameraBall {
public:
    /**
     * @param timeCaptured Time that the picture was taken
     * @param pos Position of the ball at that time
     */
    CameraBall(RJ::Time timeCaptured, Geometry2d::Point pos)
        : timeCaptured(timeCaptured), pos(pos) {}

    RJ::Time getTimeCaptured();
    Geometry2d::Point getPos();

    /**
     * Combines all the balls in the list and returns a ball
     * with the average pos and time
     *
     * @param balls The list of balls to combine
     */
    static CameraBall CombineBalls(std::list<CameraBall> balls);

private:
    RJ::Time timeCaptured;
    Geometry2d::Point pos;
};