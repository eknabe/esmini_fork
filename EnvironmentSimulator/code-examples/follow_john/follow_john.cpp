#include "stdio.h"
#include "math.h"
#include <string>
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    SE_SimpleVehicleState  vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;
    double                 throttleWeight = 0.5;  // weight for throttle control

    if (SE_Init("../../../../EnvironmentSimulator/code-examples/follow_john/follow_john.xosc", 0, 1, 0, 0) != 0)
    {
        SE_LogMessage("Failed to initialize the scenario, quit\n");
        return -1;
    }

    // Initialize the vehicle model, fetch initial state from the scenario
    SE_GetObjectState(0, &objectState);
    void* vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);
    SE_SimpleVehicleSteeringRate(vehicleHandle, 6.0f);

    // show some road features, including road sensor
    SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

    // Run for specified duration or until 'Esc' button is pressed
    while (SE_GetQuitFlag() != 1)
    {
        // Get simulation delta time since last call (first will be minimum timestep)
        float dt = SE_GetSimTimeStep();

        // Get road information at a point some speed dependent distance ahead
        double targetSpeed = 4.0;

        // Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
        double throttle = throttleWeight * (targetSpeed - static_cast<double>(vehicleState.speed));

        // Step vehicle model with driver input, but wait until time > 0
        if (SE_GetSimulationTime() > 0 && !SE_GetPauseFlag())
        {
            // Steer towards the point
            double steerAngle = 0.0;
            SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
        }

        // Fetch updated state and report to scenario engine
        SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

        // Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
        SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h);

        // wheel status (revolution and steering angles)
        SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation, vehicleState.wheel_angle);

        // speed (along vehicle longitudinal (x) axis)
        SE_ReportObjectSpeed(0, vehicleState.speed);

        // Finally, update scenario using same time step as for vehicle model
        SE_StepDT(dt);
    }

    SE_Close();

    return 0;
}