#pragma once

#include <deque>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <Configuration.hpp>

#include "vision/robot/CameraRobot.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "vision/filter/KalmanFilter3D.hpp"

class WorldRobot;

/**
 * Filtered robot estimation for a single camera
 */
class KalmanRobot {
public:
    /**
     * Checks previousWorldRobot to see if it's valid
     *
     * @param cameraID ID of the camera this filter is applied to
     * @param creationTime Time this filter was created
     * @param initMeasurement Initial robot measurement
     */
    KalmanRobot(unsigned int cameraID, RJ::Time creationTime,
                CameraRobot initMeasurement, WorldRobot& previousWorldRobot);

    /**
     * Predicts one time step forward
     */
    void predict(RJ::Time currentTime);

    /**
     * Predicts one time step forward then triangulates toward the measurement
     *
     * @param currentTime Current time of the prediction/update step
     * @param updateRobot Robot measurement that we are using as feedback
     */
    void predictAndUpdate(RJ::Time currentTime, CameraRobot updateRobot);

    /**
     * Returns true when the filter hasn't been updated in a while and should be deteled
     */
    bool isUnhealthy();

    /**
     * @return The camera id this belongs to
     */
    unsigned int getCameraID();

    int getRobotID();

    /**
     * @return How healthy this filter is. AKA How often it's been updated
     */
    int getHealth();

    /**
     * @return Best estimate of the linear position of the robot
     */
    Geometry2d::Point getPos();

    /**
     * @return Best estimate of the heading. Not bounded
     */
    double getTheta();

    /**
     * @return Best estimate of the linear velocity of the robot
     */
    Geometry2d::Point getVel();

    /**
     * @return Best estimate of the angular velocity
     */
    double getOmega();

    /**
     * @return Covariance in X and Y linear direction of the position of the robot
     */
    Geometry2d::Point getPosCov();

    /**
     * @return Covariance of theta of the robot
     */
    double getThetaCov();

    /**
     * @return Covariance in X and Y linear direction of the velocity of the robot
     */
    Geometry2d::Point getVelCov();

    /**
     * @return Covariance of omega of the robot
     */
    double getOmegaCov();

    /**
     * @return List of previous camera robot measurements for kick detection
     */
    std::deque<CameraRobot> getPrevMeasurements();

    static void createConfiguration(Configuration* cfg);

private:
    static ConfigDouble* max_time_outside_vision;

    RJ::Time lastUpdateTime;
    RJ::Time lastPredictTime;

    std::deque<CameraRobot> previousMeasurements;

    KalmanFilter3D filter;

    double previousTheta;
    int unwrapThetaCtr;
    int health;

    int robotID;

    unsigned int cameraID;
};