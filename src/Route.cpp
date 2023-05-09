#include <iostream>

#include "Route.h"
#include "Vehicle.h"
#include "Node.h"

Route::Route(Vehicle c_veh) : 
    veh( c_veh ), routeID( c_veh.getVehID() ), routeNodes(), totalRouteTimeInHrs(0), finalized(false) { }

bool Route::equals(Route otherRoute){
    return (this->routeID == otherRoute.getRouteID());
}

int Route::getRouteSize(){
    return this->routeNodes.size();
}

void Route::addNodeToRoute(Node *new_node, double cost){
    this->routeNodes.emplace_back(new_node);
    new_node->updateServiceStatus(true);
    this->updateTotalRouteTime(cost, *new_node);
}

Node * Route::getLastNode(){
    return this->routeNodes[  this->routeNodes.size() - 1 ];
}

void Route::updateVehCap(Node addedNode){
    int cap = this->veh.getRemainingCap();
    veh.setRemainingCap(cap - addedNode.getDemand());
}

void Route::updateTotalRouteTime(double cost, Node node){
    this->setTotalRouteTimeInHrs( 
        this->getTotalRouteTimeInHrs() + cost + node.getServiceTime()
    );
}

//
int Route::getRouteID(){
    return this->routeID;
}

void Route::setRouteID(int new_routeID){
    this->routeID = new_routeID;
}

Vehicle Route::getVeh(){
    return this->veh;
}

void Route::setVeh(Vehicle new_veh){
    this->veh = new_veh;
}

std::vector<Node*> Route::getRouteNodes(){
    return this->routeNodes;
}

void Route::setRouteNodes(std::vector<Node*> new_routeNodes){
    this->routeNodes = new_routeNodes;
}

double Route::getTotalRouteTimeInHrs(){
    return this->totalRouteTimeInHrs;
}

void Route::setTotalRouteTimeInHrs(double new_totalRouteTimeInHrs){
    this->totalRouteTimeInHrs = new_totalRouteTimeInHrs;
}

bool Route::isFinalized(){
    return this->finalized;
}

void Route::setFinalized(bool new_finalized){
    this->finalized = new_finalized;
}
