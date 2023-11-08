#ifndef ROBUS_MOVEMENT_H
#define ROBUS_MOVEMENT_H

#include <RobusPosition.h> 
#include <RobusMovement.h>

namespace RobotPosition
{
    struct Point {
        Point() : x(0), y(0) {}
        float x;
        float y;
    };

    void update();

    float getOrientation();

    Point getPosition();
    void setPosition(float x, float y);
    void setPosition(Point position);

    namespace {
        extern Point position;
        extern Point target;
    }
}

#endif // ROBUS_MOVEMENT_H