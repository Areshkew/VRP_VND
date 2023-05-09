#include <iostream>

#include "Node.h"
#include "VRP.h"

std::string Node::toString(){
    return std::to_string(this->nodeID);
}

void Node::printNodeData(){
    std::cout << "NodeId: " << this->nodeID << std::endl;
    std::cout << "x: " << this->coords.x << std::endl;
    std::cout << "y: " << this->coords.y << std::endl;
    std::cout << "demand: " << this->demand << std::endl;
    std::cout << "serviced: " << this->serviced << std::endl;
    std::cout << "service Time: " << this->serviceTime << std::endl;
}

// Storage Constructor
Node::Node(float x, float y, int demand) : demand(demand), serviceTime(0), serviced(true) {
    VRP::nodeCounter.fetch_add(1);
    this->nodeID = VRP::nodeCounter;
    this->coords.x = x;
    this->coords.y = y;
    this->coords.NodeId = this->nodeID;
}

Node::Node() :  demand( 100 * (1 + ran.nextInt(5)) ),
                serviceTime(0.25), serviced(false)
    {
    VRP::nodeCounter.fetch_add(1);
    this->nodeID = VRP::nodeCounter;
    this->coords.x = ran.nextInt(80);
    this->coords.y = ran.nextInt(80);
    this->coords.NodeId = this->nodeID;
}

// Getters settersf
XY Node::getCoords(){
    return this->coords;
}

int Node::getNodeID(){
    return this->nodeID;
}

void Node::setNodeID(int new_nodeID){
    this->nodeID = new_nodeID;
}

int Node::getX(){
    return this->coords.x;
}

void Node::setX(int new_X){
    this->coords.x = new_X;
}

int Node::getY(){
    return this->coords.y;
}

void Node::setY(int new_Y){
    this->coords.y = new_Y;
}

int Node::getDemand(){
    return this->demand;
}

void Node::setDemand(int new_Demand){
    this->demand = new_Demand;
}

double Node::getServiceTime(){
    return this->serviceTime;
}

bool * Node::isServiced(){
    return &this->serviced;
}

void Node::updateServiceStatus(bool new_Serviced){
    this->serviced = new_Serviced;
}