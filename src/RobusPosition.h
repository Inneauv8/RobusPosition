#ifndef ROBUS_MOVEMENT_H
#define ROBUS_MOVEMENT_H

#include <RobusPosition.h> 
#include <RobusMovement.h>
#include <mathX.h>

namespace RobotPosition
{
    struct Vector {
        Vector() : x(0), y(0) {}
        float x;
        float y;
    };

    void update();

    float getOrientation();

    Vector getPosition();
    void setPosition(float x, float y);
    void setPosition(Vector position);

    bool isFollowingTarget();
    bool setFollowingTarget(boolean followingTarget);

    bool startFollowingTarget();
    bool stopFollowingTarget();

    void setFollowAngularVelocityScale(float scale);
    void setFollowVelocity(float velocity);
    void setCurveTightness(float tightness);

    float getFollowAngularVelocityScale();
    float getFollowVelocity();
    float getCurveTightness();

    namespace {
        extern Vector position;
        extern Vector target;
        extern bool followingTarget;

        extern float followAngularVelocityScale;
        extern float followVelocity;
        extern float curveTightness;
    }
}

#endif // ROBUS_MOVEMENT_H