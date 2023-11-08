#include "RobusPosition.h"

namespace RobotPosition
{
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
        }
    }

    float getOrientation() {
        return RobusMovement::computeOrientation();
    }

    Vector getPosition() {
        return position;
    }

    void setPosition(float x, float y) {
        target.x = x;
        target.y = y;
    }

    void setPosition(Vector position) {
        setPosition(position.x, position.y);
    }

    bool isFollowingTarget() {
        return followingTarget;
    }

    bool setFollowingTarget(bool followingTarget) {
        followingTarget = followingTarget;
    }

    bool startFollowingTarget() {
        setFollowingTarget(true);
    }

    bool stopFollowingTarget() {
        setFollowingTarget(false);
    }

    void setFollowAngularVelocityScale(float scale) {
        followAngularVelocityScale = scale;
    }

    void setFollowVelocity(float velocity) {
        followVelocity = velocity;
    }

    void setCurveTightness(float tightness) {
        curveTightness = tightness;
    }

    float getFollowAngularVelocityScale() {
        return followAngularVelocityScale;
    }

    float getFollowVelocity() {
        return followVelocity;
    }

    float getCurveTightness() {
        return curveTightness;
    }

    namespace {
        Vector position = {};
        Vector target = {};
        bool followingTarget = false;

        float followAngularVelocityScale = 0.1;
        float followVelocity = 10;
        float curveTightness = 1;
    }
}
