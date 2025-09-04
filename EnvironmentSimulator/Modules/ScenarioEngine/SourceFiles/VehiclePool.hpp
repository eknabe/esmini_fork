/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once

#include <map>

namespace scenarioengine
{

    class ScenarioReader;
    class Vehicle;

    class VehiclePool
    {
    public:
        VehiclePool() = default;
        VehiclePool(ScenarioReader*                  reader,
                    const std::vector<unsigned int>* categories,
                    bool                             accept_disconnected_trailers,
                    bool                             accept_cars_with_trailer);
        ~VehiclePool();

        int                          Initialize(ScenarioReader*                  reader,
                                                const std::vector<unsigned int>* categories,
                                                bool                             accept_disconnected_trailers,
                                                bool                             accept_cars_with_trailer);
        int                          AddVehicle(Vehicle* vehicle);
        void                         DeleteVehicleAndTrailers(Vehicle* vehicle);
        const std::vector<Vehicle*>& GetVehicles() const;
        const std::vector<Vehicle*>& GetVehicles(std::string category) const;
        Vehicle*                     GetVehicle(unsigned int index, std::string category = "");
        Vehicle*                     GetRandomVehicle(std::string category = "");

    private:
        std::map<std::string, std::vector<Vehicle*>> vehicle_pool_;
    };

}  // namespace scenarioengine