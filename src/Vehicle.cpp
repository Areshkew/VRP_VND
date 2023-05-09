#include <iostream>
#include "Vehicle.h"
#include "VRP.h"

Vehicle::Vehicle() : remainingCap( Vehicle::maxCap ){
    VRP::vehicleCounter.fetch_add(1);
    this->VehicleID = VRP::vehicleCounter;
}

// Getters Setters
int Vehicle::getVehID(){
    return this->VehicleID;
}

void Vehicle::setVehID(int newVehId){
    this->VehicleID = newVehId;
}

int Vehicle::getRemainingCap(){
    return this->remainingCap;
}

void Vehicle::setRemainingCap(int newRemainingCap){
    this->remainingCap = newRemainingCap;
}

int Vehicle::getMaxCap(){
    return this->maxCap;
}

void Vehicle::printVehData(){
    std::cout << "Vehicle ID: " << this->VehicleID << std::endl;
    std::cout << "Remaining Cap: " << this->remainingCap << std::endl;
    std::cout << "Speed (KM/H): " << Vehicle::speedKMH << std::endl;
    std::cout << "MaxCap: " << this->maxCap << std::endl;
}

