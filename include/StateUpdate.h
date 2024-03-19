#ifndef STATEUPDATE_H
#define STATEUPDATE_H

class StateUpdate {
public:
    struct Vector3 {
        double x, y, z;
    };

    struct Orientation {
        double roll, pitch, yaw; // Assuming radians
    };

    // Method to update linear state (position and velocity)
    static void updateLinearState(Vector3& position, Vector3& velocity, const Vector3& acceleration, double deltaT);

    // Method to update orientation based on angular rates
    static void updateOrientation(Orientation& orientation, const Vector3& angularRate, double deltaT);

    // Optionally, method to convert degrees to radians and vice versa if needed
    static double degreesToRadians(double degrees);
    static double radiansToDegrees(double radians);
};

#endif // STATEUPDATE_H

