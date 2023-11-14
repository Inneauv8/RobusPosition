#ifndef ROBUS_MOVEMENT_H
#define ROBUS_MOVEMENT_H

#include <RobusPosition.h> 
#include <RobusMovement.h>
#include <mathX.h>

namespace RobusPosition
{   
    /**
     * @brief Vector structure to represent position and direction.
     */
    struct Vector {
        Vector(float x = 0, float y = 0) : x(0), y(0) {
            this->x = x;
            this->y = y;
        }
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
    void setFollowingTarget(bool followingTarget);

    void startFollowingTarget();
    void stopFollowingTarget();

    void setFollowAngularVelocityScale(float scale);
    void setFollowVelocity(float velocity);
    void setCurveTightness(float tightness);

    float getFollowAngularVelocityScale();
    float getFollowVelocity();
    float getCurveTightness();

    bool isInverted();
    void setInverted(bool invert);

    namespace {
        extern Vector position;
        extern Vector target;
        extern bool followingTarget;

        extern float followAngularVelocityScale;
        extern float followVelocity;
        extern float curveTightness;

        extern bool inverted;
    }
}

#endif // ROBUS_MOVEMENT_H