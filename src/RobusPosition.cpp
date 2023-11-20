#include "RobusPosition.h"

#ifndef INTEGRATION_ITERATION
#define INTEGRATION_ITERATION 1
#endif

namespace RobusPosition
{
    // Helper function to normalize an angle to the range [0, 2*PI]
    float normalizeAngle(float angle) {
        while (angle < 0) {
            angle += TWO_PI;
        }
        while (angle >= TWO_PI) {
            angle -= TWO_PI;
        }
        return angle;
    }

    float smallestSignedAngle(float currentAngle, float targetAngle) {
        // Ensure both angles are in the range [0, 2*PI]
        currentAngle = normalizeAngle(currentAngle);
        targetAngle = normalizeAngle(targetAngle);

        // Calculate the difference between the angles
        float angleDifference = targetAngle - currentAngle;

        // Normalize the angle difference to be in the range [-PI, PI]
        if (angleDifference > PI) {
            angleDifference -= TWO_PI;
        } else if (angleDifference < -PI) {
            angleDifference += TWO_PI;
        }

        return angleDifference;
    }

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

        const double robusVelocity = (double) RobusMovement::getVelocity() * (inverted ? -1 : 1);
        const double robusAngularVelocity = (double) RobusMovement::getAngularVelocity();
        double robusOrientation = (double) RobusMovement::computeOrientation();

        double iterationDt = dt / INTEGRATION_ITERATION;
        for (int i = 0; i < INTEGRATION_ITERATION; i++) {
            position.x += cos(robusOrientation) * robusVelocity * iterationDt;
            position.y += sin(robusOrientation) * robusVelocity * iterationDt;
            robusOrientation += robusAngularVelocity * iterationDt;
        }

        if (followingTarget) {
            float targetDistance = dist(position.x, position.y, target.x, target.y);
            if (targetDistance > 0.01) {
                Vector targetDirection = Vector(0, 0);
                targetDirection.x = (target.x - position.x) / targetDistance;
                targetDirection.y = (target.y - position.y) / targetDistance;

                float targetAngle = atan2(targetDirection.y, targetDirection.x);

                float robusOrientation = RobusMovement::computeOrientation();
                Vector robusDirection = Vector(0, 0);
                robusDirection.x = cos(robusOrientation);
                robusDirection.y = sin(robusOrientation);

                float directionDot = robusDirection.x * targetDirection.x + robusDirection.y * targetDirection.y;
                float velocity = pow(directionDot, curveTightness) * followVelocity * (inverted ? -1 : 1);
                
                float deltaOrientation = smallestSignedAngle(robusOrientation, targetAngle);
                float angularVelocity = deltaOrientation * followAngularVelocityScale;
                
                //float distanceError = directionDot * distanceError
                //float velocity = pow(directionDot, curveTightness) * targetDistance; //  higher = higher error when in the right direction (will turn more before moving forward);
                //velocity = clamp(velocity * 10, -followVelocity, followVelocity);
                
                RobusMovement::setVelocity(velocity);
                RobusMovement::setAngularVelocity(clamp(angularVelocity, -0.5, 0.5));
            } else {
                RobusMovement::stop();
            }
        }
        
        RobusMovement::update();
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
        followingTarget = true;
        //setFollowingTarget(true);
    }

    /**
     * @brief Stop following the target by setting the following state to false.
     */
    void stopFollowingTarget() {
        followingTarget = false;
        //setFollowingTarget(false);
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

     /**
     * @brief Get if the robot direction is inverted.
     * @return If the robot direction is inverted.
     */
    bool isInverted() {
        return inverted;
    }

    /**
     * @brief Set if the robot direction is inverted.
     * @param invert if the robot direction will be inverted or not.
     */
    void setInverted(bool invert) {
        inverted = invert;
    }

    namespace {
        Vector position = Vector(); /**< The current position of the robot. */
        Vector target = Vector(); /**< The target position for the robot to follow. */
        bool followingTarget = false; /**< Flag indicating whether the robot is following a target. */
        //5, 10, 5
        float followAngularVelocityScale = 3.0; /**< Scale factor for angular velocity when following a target. */
        float followVelocity = 3; /**< Velocity at which the robot follows a target. */
        float curveTightness = 50; /**< Tightness of the curve when following a target. */
        bool inverted = false;
    }
}
