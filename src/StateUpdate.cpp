#include "../include/StateUpdate.h"
#include <cmath>

void StateUpdate::updateLinearState(Vector3& position, Vector3& velocity, const Vector3& acceleration, double deltaT) {
    // Update position
    position.x += velocity.x * deltaT + 0.5 * acceleration.x * deltaT * deltaT;
    position.y += velocity.y * deltaT + 0.5 * acceleration.y * deltaT * deltaT;
    position.z += velocity.z * deltaT + 0.5 * acceleration.z * deltaT * deltaT;

    // Update velocity
    velocity.x += acceleration.x * deltaT;
    velocity.y += acceleration.y * deltaT;
    velocity.z += acceleration.z * deltaT;
}

void StateUpdate::updateOrientation(Orientation& orientation, const Vector3& angularRate, double deltaT) {
    // Assuming angularRate is in radians per second
    orientation.roll += angularRate.x * deltaT;
    orientation.pitch += angularRate.y * deltaT;
    orientation.yaw += angularRate.z * deltaT;
}

double StateUpdate::degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double StateUpdate::radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

