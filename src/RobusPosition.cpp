#include "RobusPosition.h"

namespace RobotPosition
{
    void update() {
        
        RobusMovement::update();
    }

    float getOrientation() {
        return RobusMovement::computeOrientation();
    }

    Point getPosition() {
        return position;
    }

    void setPosition(float x, float y) {
        target.x = x;
        target.y = y;
    }

    void setPosition(Point position) {
        setPosition(position.x, position.y);
    }

    namespace {
        Point position = {};
        Point target = {};
    }
}
