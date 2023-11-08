#include "RobusPosition.h"

namespace RobotPosition
{
    /**
     * @brief Update the robot position based on its velocity and orientation.
     *
     * This function updates the robot's position based on its current velocity and orientation.
     * If the robot is set to follow a target, it adjusts its velocity and angular velocity accordingly.
     */
    void update() {
        static unsigned long oldTime;
        unsigned long time = micros();
        float dt = (time - oldTime) / 1000000.0;
        oldTime = time;

        RobusMovement::update();
        float velocity = RobusMovement::getVelocity();
        float orientation = RobusMovement::computeOrientation();

        float xVelocity = cos(orientation) * velocity;
        float yVelocity = sin(orientation) * velocity;

        position.x += xVelocity * dt;
        position.y += yVelocity * dt;

        if (followingTarget) {
            float targetDistance = dist(position.x, position.y, target.x, target.y);
            if (targetDistance != 0) {
                Vector targetDirection;
                targetDirection.x = (target.x - position.x) / targetDistance;
                targetDirection.y = (target.y - position.y) / targetDistance;

                float targetAngle = atan2(targetDirection.y, targetDirection.x);

                float robusOrientation = RobusMovement::computeOrientation();
                Vector robusDirection;
                robusDirection.x = cos(robusOrientation);
                robusDirection.y = sin(robusOrientation);

                float directionDot = robusDirection.x * targetDirection.x + robusDirection.y * targetDirection.y;
                float velocity = pow(directionDot, curveTightness) * velocity;
                
                float deltaOrientation = smallestAngleDifference(robusOrientation, targetAngle);
                float angularVelocity = deltaOrientation * followAngularVelocityScale;

                RobusMovement::setVelocity(velocity);
                RobusMovement::setAngularVelocity(angularVelocity);
            } else {
                RobusMovement::stop();
            }
        }
    }

    /**
     * @brief Get the current orientation of the robot.
     * @return The orientation angle of the robot in radians.
     */
    float getOrientation() {
        return RobusMovement::computeOrientation();
    }

    /**
     * @brief Get the current position of the robot.
     * @return The current position of the robot as a Vector.
     */
    Vector getPosition() {
        return position;
    }

    /**
     * @brief Set the position of the robot to the specified coordinates.
     * @param x The X-coordinate of the new position.
     * @param y The Y-coordinate of the new position.
     */
    void setPosition(float x, float y) {
        position.x = x;
        position.y = y;
    }

    /**
     * @brief Set the position of the robot to the specified Vector.
     * @param position The new position of the robot.
     */
    void setPosition(Vector position) {
        setPosition(position.x, position.y);
    }

    Vector getTarget() {
        return target;
    }

    /**
     * @brief Set the target position for the robot to follow.
     * @param x The X-coordinate of the target position.
     * @param y The Y-coordinate of the target position.
     */
    void setTarget(float x, float y) {
        target.x = x;
        target.y = y;
    }

    /**
     * @brief Set the target position for the robot to follow using a Vector.
     * @param position The target position for the robot to follow.
     */
    void setTarget(Vector position) {
        setTarget(position.x, position.y);
    }

    /**
     * @brief Check if the robot is currently following a target.
     * @return true if the robot is following a target; otherwise, false.
     */
    bool isFollowingTarget() {
        return followingTarget;
    }

    /**
     * @brief Set whether the robot should follow a target or not.
     * @param followingTarget true to make the robot follow a target; false to stop following.
     */
    void setFollowingTarget(bool followingTarget) {
        followingTarget = followingTarget;
    }

    /**
     * @brief Start following the target by setting the following state to true.
     */
    void startFollowingTarget() {
        setFollowingTarget(true);
    }

    /**
     * @brief Stop following the target by setting the following state to false.
     */
    void stopFollowingTarget() {
        setFollowingTarget(false);
    }

    /**
     * @brief Set the scale factor for the robot's angular velocity when following a target.
     * @param scale The scale factor for the angular velocity.
     */
    void setFollowAngularVelocityScale(float scale) {
        followAngularVelocityScale = scale;
    }

    /**
     * @brief Set the velocity at which the robot follows a target.
     * @param velocity The velocity at which the robot follows the target.
     */
    void setFollowVelocity(float velocity) {
        followVelocity = velocity;
    }

    /**
     * @brief Set the tightness of the curve when following a target.
     * @param tightness The tightness of the curve, affecting the robot's path.
     */
    void setCurveTightness(float tightness) {
        curveTightness = tightness;
    }

    /**
     * @brief Get the scale factor for the robot's angular velocity when following a target.
     * @return The current angular velocity scale factor.
     */
    float getFollowAngularVelocityScale() {
        return followAngularVelocityScale;
    }

    /**
     * @brief Get the velocity at which the robot follows a target.
     * @return The current target following velocity.
     */
    float getFollowVelocity() {
        return followVelocity;
    }

     /**
     * @brief Get the tightness of the curve when following a target.
     * @return The current curve tightness value.
     */
    float getCurveTightness() {
        return curveTightness;
    }

    namespace {
        Vector position = {}; /**< The current position of the robot. */
        Vector target = {}; /**< The target position for the robot to follow. */
        bool followingTarget = false; /**< Flag indicating whether the robot is following a target. */

        float followAngularVelocityScale = 0.1; /**< Scale factor for angular velocity when following a target. */
        float followVelocity = 10; /**< Velocity at which the robot follows a target. */
        float curveTightness = 1; /**< Tightness of the curve when following a target. */
    }
}
