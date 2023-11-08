#ifndef ROBUS_MOVEMENT_H
#define ROBUS_MOVEMENT_H

#include <RobusPosition.h> 
#include <RobusMovement.h>
#include <mathX.h>

namespace RobotPosition
{   
    /**
     * @brief Vector structure to represent position and direction.
     */
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

    Vector getTarget();
    void setTarget(float x, float y);
    void setTarget(Vector position);

    bool isFollowingTarget();
    void setFollowingTarget(boolean followingTarget);

    void startFollowingTarget();
    void stopFollowingTarget();

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