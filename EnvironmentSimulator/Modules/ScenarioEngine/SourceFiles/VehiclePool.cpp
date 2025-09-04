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

#include <random>
#include "VehiclePool.hpp"
#include "ScenarioReader.hpp"

using namespace scenarioengine;

VehiclePool::VehiclePool(ScenarioReader*                  reader,
                         const std::vector<unsigned int>* categories,
                         bool                             accept_disconnected_trailers,
                         bool                             accept_cars_with_trailer)
{
    Initialize(reader, categories, accept_disconnected_trailers, accept_cars_with_trailer);
}

void VehiclePool::DeleteVehicleAndTrailers(Vehicle* vehicle)
{
    auto RecursiveDeleteTrailers = [](Vehicle* v, auto& recurseLambda) -> void
    {
        if (v->trailer_hitch_ && v->trailer_hitch_->trailer_vehicle_)
        {
            recurseLambda(static_cast<Vehicle*>(v->trailer_hitch_->trailer_vehicle_), recurseLambda);
        }

        delete v;
    };

    if (vehicle->trailer_hitch_ && vehicle->trailer_hitch_->trailer_vehicle_)
    {
        RecursiveDeleteTrailers(static_cast<Vehicle*>(vehicle->trailer_hitch_->trailer_vehicle_), RecursiveDeleteTrailers);
    }

    delete vehicle;
}

const std::vector<Vehicle*>& scenarioengine::VehiclePool::GetVehicles() const
{
    if (vehicle_pool_.empty())
    {
        static std::vector<Vehicle*> empty;
        return empty;
    }
    else if (vehicle_pool_.size() > 1)
    {
        LOG_WARN("VehiclePool::GetVehicles(): Multiple categories in list, picking first");
    }

    return vehicle_pool_.begin()->second;
}

const std::vector<Vehicle*>& scenarioengine::VehiclePool::GetVehicles(std::string category) const
{
    auto it = vehicle_pool_.find(category);

    if (it == vehicle_pool_.end())
    {
        // Key not found
        LOG_ERROR("Vehicle category {} not found!", category);
        static std::vector<Vehicle*> empty;
        return empty;
    }
    else
    {
        return it->second;
    }
}

VehiclePool::~VehiclePool()
{
    for (auto& category : vehicle_pool_)
    {
        for (const auto& entry : category.second)
        {
            DeleteVehicleAndTrailers(entry);
        }
        category.second.clear();
    }

    vehicle_pool_.clear();
}

int VehiclePool::Initialize(ScenarioReader*                  reader,
                            const std::vector<unsigned int>* categories,
                            bool                             accept_disconnected_trailers,
                            bool                             accept_cars_with_trailer)
{
    Catalogs* catalogs = reader->GetCatalogs();

    std::vector<unsigned int> categories_tmp;
    if (categories->empty())
    {
        // set default categories
        categories_tmp = {Vehicle::Category::CAR,
                          Vehicle::Category::BUS,
                          Vehicle::Category::TRUCK,
                          Vehicle::Category::VAN,
                          Vehicle::Category::MOTORBIKE};
    }
    else
    {
        categories_tmp = *categories;
    }
    categories = &categories_tmp;

    // Register model filesnames from first vehicle catalog
    // if no catalog loaded, use same model as central object
    for (size_t i = 0; i < catalogs->catalog_.size(); i++)
    {
        const Catalog* catalog = catalogs->catalog_[i];
        if (catalog->GetType() == CatalogType::CATALOG_VEHICLE)
        {
            for (size_t j = 0; j < catalog->entry_.size(); j++)
            {
                Vehicle* vehicle = reader->parseOSCVehicle(catalog->entry_[j]->GetNode());
                if (  // skip unwanted disconnected trailers
                    !(accept_disconnected_trailers == false && vehicle->type_ == Vehicle::Category::TRAILER && vehicle->TowVehicle() == nullptr) &&
                    // skip vehicles with trailers, unless trailer category specified
                    !(vehicle->TrailerVehicle() != nullptr &&
                      std::find(categories->begin(), categories->end(), Vehicle::Category::TRAILER) == categories->end()) &&
                    // skip cars with trailers, unless accepted
                    !(accept_cars_with_trailer == false && vehicle->TrailerVehicle() != nullptr && vehicle->category_ == Vehicle::Category::CAR) &&
                    // include only vehicles of specified categories
                    (std::find(categories->begin(), categories->end(), vehicle->category_) != categories->end()))
                {
                    vehicle_pool_[Vehicle::Category2String(vehicle->category_)].push_back(vehicle);
                }
                else
                {
                    DeleteVehicleAndTrailers(vehicle);
                }
            }
        }
    }

    return 0;
}

int VehiclePool::AddVehicle(Vehicle* vehicle)
{
    if (vehicle != nullptr)
    {
        vehicle_pool_[Vehicle::Category2String(vehicle->category_)].push_back(vehicle);
        return static_cast<int>(vehicle_pool_.size() - 1);
    }
    return -1;
}

Vehicle* VehiclePool::GetVehicle(unsigned int index, std::string category)
{
    std::vector<Vehicle*>* vehicle_group = nullptr;

    if (category.empty())
    {
        if (vehicle_pool_.empty())
        {
            return nullptr;
        }
        else if (vehicle_pool_.size() > 1)
        {
            LOG_WARN("VehiclePool::GetVehicle(): Multiple categories in list, picking first");
        }
        vehicle_group = &vehicle_pool_.begin()->second;
    }
    else
    {
        auto it = vehicle_pool_.find(category);
        if (it == vehicle_pool_.end())
        {
            // Key not found
            LOG_ERROR("Vehicle category {} not found!", category);
            return nullptr;
        }
        vehicle_group = &it->second;
    }

    if (vehicle_group == nullptr || vehicle_group->empty() || index >= vehicle_group->size())
    {
        return nullptr;
    }

    return (*vehicle_group)[index];
}

Vehicle* VehiclePool::GetRandomVehicle(std::string category)
{
    std::vector<Vehicle*>* vehicle_group = nullptr;

    auto it = vehicle_pool_.find(category);
    if (it == vehicle_pool_.end())
    {
        // pick random category
        it = vehicle_pool_.begin();
        std::advance(it, SE_Env::Inst().GetRand().GetNumberBetween(0, static_cast<int>(vehicle_pool_.size() - 1)));
    }
    vehicle_group = &it->second;

    if (vehicle_group == nullptr || vehicle_group->empty())
    {
        return nullptr;
    }

    // pick random vehicle from category
    return (*vehicle_group)[SE_Env::Inst().GetRand().GetNumberBetween(0, static_cast<int>(vehicle_group->size() - 1))];
}
