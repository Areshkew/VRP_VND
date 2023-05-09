
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <time.h>

#include "VRP.h"
#include "Node.h"
#include "Vehicle.h"
#include "LocalSearch.h"

// ------------------------------------------------------ CONSTRUCTOR
VRP::VRP(int totalServicePoints, int totalVehicles) : 
        totalServicePoints(totalServicePoints), totalVehicles(totalVehicles), storage(50, 50, 0),
        distanceMatrix( totalServicePoints + 1, std::vector<double>( totalServicePoints + 1, 0 ) ),
        timeMatrix( totalServicePoints + 1, std::vector<double>( totalServicePoints + 1, 0 ) ),
        allNodes(), allVehicles(), allRoutes(), allPoints(), QT_COUNT(0)
{
    this->initializeAlgorithmData();
    this->nearestNeighboor();
    std::cout << "Se inicio la ejecución de DVNS..." << std::endl;
    this->DVNS(3);
    std::cout << "Termino la ejecución de DVNS..." << std::endl;
}

// ------------------------------------------------------ INITIALIZING METHODS
void VRP::initializeAlgorithmData() {
    // Generating Nodes and Distance Matrix
    this->allNodes.reserve(this->totalServicePoints + 1);
    this->generateRandomNodes();
    this->euclideanDistanceMatrix();
    // Generating Random Vehicles
    this->allVehicles.reserve(this->totalVehicles);
    this->generateVehicles();
    // Initializing  Routes
    this->allRoutes.reserve(this->totalVehicles);
    this->routesInit();
}

void VRP::generateRandomNodes(){
    this->allNodes.emplace_back( this->storage );
    for(int i = 0; i < this->totalServicePoints; i++){
        Node *customer = new Node();
        this->allNodes.emplace_back(*customer);

        XY point(customer->getX(), customer->getY(), customer->getNodeID(), this->allNodes.back().isServiced());
        this->allPoints.insert( point );

        if(this->allPoints.insert( point )){
            this->QT_COUNT++;
        }
    }

}

void VRP::euclideanDistanceMatrix(){
    for (int i = 0; i < allNodes.size(); i++) {

        for (int j = 0; j < allNodes.size(); j++) {

            double deltaX = (allNodes.at(i).getX() - allNodes.at(j).getX());
            double deltaY = (allNodes.at(i).getY() - allNodes.at(j).getY());
            double distance = sqrt( pow(deltaX, 2) + pow(deltaY, 2));

            distance = round(distance);
            distanceMatrix[i][j] = distance;
            timeMatrix[i][j] = distance / Vehicle::speedKMH;
        }
    }
}

void VRP::generateVehicles(){
    for(int i = 0; i < this->totalVehicles; i++) {
         Vehicle veh;
         this->allVehicles.emplace_back(veh);
    }
}

void VRP::routesInit(){
    for(auto &veh : this->allVehicles){
        Route route(veh);
        route.addNodeToRoute(&this->storage, 0);

        this->allRoutes.emplace_back(route);
    }
}

// ------------------------------------------------------ UTILITY METHODS
void VRP::PrintDM(){
    for(auto &r : this->distanceMatrix){
        for(auto &c : r){
            std::cout << c << " ";
        }
        std::cout << std::endl;
    }
    std::cout << " ------------ " << std::endl;
    for(auto &r : this->timeMatrix){
        for(auto &c : r){
            std::cout << c << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<Node> VRP::getAllNodes(){
    return this->allNodes;
}

std::vector<Vehicle> VRP::getAllVehs(){
    return this->allVehicles;
}

std::vector<Route> VRP::getAllRoutes(){
    return this->allRoutes;
}

bool compareRoute(Route a, Route b) {
    return a.getTotalRouteTimeInHrs() < b.getTotalRouteTimeInHrs();
}

Route VRP::findSlowestRoute(std::vector<Route> routes){
    return *std::max_element(routes.begin(), routes.end(), compareRoute);
}

// ------------------------------------------------------ Nearest Neighbourhood Method (DVNS Solution Input)
void VRP::nearestNeighboor(){
    size_t nodesQuantity = this->QT_COUNT;
    float nodesPerVehicle = trunc( static_cast<float>(nodesQuantity) / this->totalVehicles);
    float missingNodes = nodesQuantity - (nodesPerVehicle*this->totalVehicles);

    for(int i = 0; i < this->allRoutes.size(); i++){ // Route It
        Node *currentNode = &this->storage; 

        for(int j = 0; j < (nodesPerVehicle + missingNodes); j++){ // Route creation
            int min = std::numeric_limits<int>::max();
            int nearestNodeId = 0;
            int _time = 0;
            std::vector<XY> NearestPoints;
            this->expandSearch(&NearestPoints, 15, currentNode);
            
            for(auto &it : NearestPoints){
                int distance = this->distanceMatrix[ currentNode->getNodeID() - 1 ][ it.NodeId - 1 ];
                _time = this->timeMatrix[ currentNode->getNodeID() - 1 ][ it.NodeId - 1 ];
                if(distance < min){
                    min = distance;
                    nearestNodeId = it.NodeId;
                }
            }

            currentNode = &this->allNodes.at(nearestNodeId - 1);
            this->allRoutes.at(i).addNodeToRoute(currentNode, _time);
        }
        missingNodes = 0;

    }
}

void VRP::expandSearch(std::vector<XY> *found, int range, Node *currentNode){
    if(found->size() > 0 ) //&& range > 50
        return;

    AABB storageRange(currentNode->getCoords(), range);
    this->allPoints.query(storageRange, found);

    this->expandSearch(found, range + 5, currentNode);    
}

// ------------------------------------------------------ DVNS
std::vector<Route> VRP::CloneRoutes(){
    std::vector<Route> routesClone;
    for(auto &it : this->allRoutes){
        Route new_route( it.getVeh() );
        new_route.setTotalRouteTimeInHrs( it.getTotalRouteTimeInHrs() );

        for(auto &jt : it.getRouteNodes()){
            Node new_node(jt->getX(), jt->getY(), jt->getDemand());
            new_node.setNodeID( jt->getNodeID() );
            new_route.addNodeToRoute( &new_node, 0 );
        }
        routesClone.emplace_back(new_route);
    }
}

void VRP::DVNS(int kmax){
    unsigned short int k = 1;
    auto timeStart = clock();

    while (k <= kmax && (clock() - timeStart) / CLOCKS_PER_SEC <= 10) {
        std::vector<Route> routes = this->allRoutes;

        if (k == 3) {
            RelocationMove rm = findBestRelocationMove(routes);
            if(rm.getMoveCostFrom() < 0) {
                rm.applyRelocationMove();
                updateRouteCost(rm.getFromRoute());
                updateRouteCost(rm.getToRoute());
                this->allRoutes = routes;
                k = 1;
            } else {
                k += 1;
            }
        } else if (k == 1) {
            SwapMove sm = findBestSwapMove(routes);
            if(sm.getMoveCostFrom() < 0) {
                sm.applySwapMove();
                updateRouteCost(sm.getExaminedRouteFrom());
                updateRouteCost(sm.getExaminedRouteTo());
                this->allRoutes = routes;
                k = 1;
            } else {
                k += 1;
            }
        } else if(k == 2) {
            TwoOpt twoOpt = findBestTwoOptMove(routes);
            twoOpt.applyTwoOptMove();
            updateRouteCost(twoOpt.getFromRoute());
            updateRouteCost(twoOpt.getToRoute());
            if(objectiveFunctionIsImproved(this->allRoutes, routes)) {
                this->allRoutes = routes;
                k = 1;
            } else {
                k += 1;
            }
        }
    }

}

bool VRP::objectiveFunctionIsImproved(std::vector<Route> scurrent,std::vector<Route> snew) {
    return (findSlowestRoute(snew).getTotalRouteTimeInHrs() < findSlowestRoute(scurrent).getTotalRouteTimeInHrs());
}

void VRP::updateRouteCost(Route route) {
    double timeCost = 0.0;
    for(int i = 0; i < route.getRouteSize() - 1; i++) {
        Node a  = *route.getRouteNodes().at(i);
        Node b = *route.getRouteNodes().at(i + 1);
        timeCost += timeMatrix[a.getNodeID()][b.getNodeID()] + b.getServiceTime();
    }
    route.setTotalRouteTimeInHrs(timeCost);

}

RelocationMove VRP::findBestRelocationMove(std::vector<Route> routes){
    RelocationMove rm;
    double bestMoveCostFrom = std::numeric_limits<double>::max();
    int totalRoutes = routes.size();

    Route examinedRouteFrom = findSlowestRoute(routes);
    int from = examinedRouteFrom.getRouteID() - 1;
    int examinedRouteFromSize = examinedRouteFrom.getRouteSize();
    double maxTime = examinedRouteFrom.getTotalRouteTimeInHrs();

    for(int to = 0; to < totalRoutes; to++) {
            Route examinedRouteTo = routes.at(to);
            
            for(int sourceIndex = 1; sourceIndex < routes.at(from).getRouteSize(); sourceIndex++) {
                Node a = *examinedRouteFrom.getRouteNodes().at(sourceIndex - 1);
                Node b = *examinedRouteFrom.getRouteNodes().at(sourceIndex);
                Node *c = nullptr;

                //Not examining the last Node    
                if(sourceIndex + 1 != examinedRouteFromSize) { 
                    c = examinedRouteFrom.getRouteNodes().at(sourceIndex + 1);
                }

                //Intervals
                for(int targetIndex = 0; targetIndex < routes.at(to).getRouteSize() - 1; targetIndex++) {

                    if(to == from) {
                        if(sourceIndex == targetIndex + 1 || sourceIndex == targetIndex) {
                            continue;
                        }
                    }

                    Node f = *examinedRouteTo.getRouteNodes().at(targetIndex);
                    Node g = *examinedRouteTo.getRouteNodes().at(targetIndex + 1);

                    double costAdded1, costRemoved1;
                    if(c != nullptr) {
                        costRemoved1 = this->timeMatrix[a.getNodeID() - 1][b.getNodeID() - 1] 
                                        + timeMatrix[b.getNodeID() - 1][c->getNodeID() - 1] 
                                        + b.getServiceTime();

                        costAdded1 = timeMatrix[a.getNodeID() - 1][c->getNodeID() - 1];
                    } else {
                        costRemoved1 = timeMatrix[a.getNodeID() - 1][b.getNodeID() - 1] + b.getServiceTime(); //arc B-C does not exist, becuase B is the last Node
                        costAdded1 = 0;
                    }

                    double costRemoved2 = timeMatrix[f.getNodeID() - 1][g.getNodeID()  - 1];
                    double costAdded2 = timeMatrix[f.getNodeID()  - 1][b.getNodeID() - 1] 
                                        + timeMatrix[b.getNodeID() - 1][g.getNodeID() - 1] + b.getServiceTime();

                    double moveCostFrom = costAdded1 - costRemoved1;
                    double moveCostTo = costAdded2 - costRemoved2;

                    double moveCost = moveCostFrom + moveCostTo;

                    double criterion = std::numeric_limits<double>::max();
                    if(to == from) {
                        criterion = examinedRouteTo.getTotalRouteTimeInHrs() + moveCost;
                    } else {
                        criterion = examinedRouteTo.getTotalRouteTimeInHrs() + moveCostTo;
                    }

                    if(criterion < maxTime) {
                        if (moveCostFrom < bestMoveCostFrom) {
                            int relocatedDemand = 0;
                            if(from != to) {
                                //checking if the route receiving the relocated node can accomodate its demand
                                relocatedDemand = examinedRouteFrom.getRouteNodes().at(sourceIndex)->getDemand();
                                if( RelocationMove::capConstrainsAreViolated(examinedRouteTo, relocatedDemand) ) {
                                    continue;
                                }
                            }

                            bestMoveCostFrom = moveCostFrom;
                            rm.setSourcePosition(sourceIndex);
                            rm.setTargetPosition(targetIndex);
                            rm.setFromRoute(examinedRouteFrom);
                            rm.setToRoute(examinedRouteTo);
                            rm.setMoveCostFrom(moveCostFrom);
                            rm.setMoveCostTo(moveCostTo);
                            rm.setMoveCost(moveCost);
                            rm.setFromRemainingCap(examinedRouteFrom.getVeh().getRemainingCap() + relocatedDemand);
                            rm.setToRemainingCap(examinedRouteTo.getVeh().getRemainingCap() - relocatedDemand);
                        }
                    }
                }
            }
        }
        return rm;
}

SwapMove VRP::findBestSwapMove(std::vector<Route> routes) {
        SwapMove sm;
        double bestMoveCostFrom = std::numeric_limits<double>::max();
        int totalRoutes = routes.size();

        Route examinedRouteFrom = findSlowestRoute(routes);
        int from = examinedRouteFrom.getRouteID() - 1;
        double maxTime = examinedRouteFrom.getTotalRouteTimeInHrs();

        //iterate through Routes
        for(int to = 0; to < totalRoutes; to++) {
            Route examinedRouteTo = routes.at(to);

            for(int sourceIndex = 1; sourceIndex < routes.at(from).getRouteSize(); sourceIndex++) {          
                Node a  = *examinedRouteFrom.getRouteNodes().at(sourceIndex - 1);
                Node b = *examinedRouteFrom.getRouteNodes().at(sourceIndex);
                Node *c = nullptr;
                
                if(sourceIndex + 1 < examinedRouteFrom.getRouteSize()) { //if you are not examining the last Node
                    c = examinedRouteFrom.getRouteNodes().at(sourceIndex + 1);
                }
                int secondIndex = 1;
                if(from == to) {
                    secondIndex = sourceIndex + 1;
                }
                
                for(int targetIndex = secondIndex; targetIndex < routes.at(to).getRouteSize(); targetIndex++) {

                    if(to == from) {
                        if(sourceIndex == targetIndex) {
                            continue;
                        }
                    }

                    //cap constraints
                    if(examinedRouteFrom.getVeh().getRemainingCap() + examinedRouteFrom.getRouteNodes().at(sourceIndex)->getDemand() < examinedRouteTo.getRouteNodes().at(targetIndex)->getDemand()) {
                        continue;
                    }

                    if(examinedRouteTo.getVeh().getRemainingCap() + examinedRouteTo.getRouteNodes().at(targetIndex)->getDemand() < examinedRouteFrom.getRouteNodes().at(sourceIndex)->getDemand()) {
                        continue;
                    }


                    Node e  = *examinedRouteTo.getRouteNodes().at(targetIndex - 1);
                    Node f = *examinedRouteTo.getRouteNodes().at(targetIndex);
                    Node *g = nullptr;

                    if(targetIndex + 1 != examinedRouteTo.getRouteSize()) { // not examining the last Node
                        g = examinedRouteTo.getRouteNodes().at(targetIndex + 1);
                    }

                    double costRemoved1 = 0.0, costAdded1 = 0.0;
                    double costRemoved2 = 0.0, costAdded2 = 0.0;
                    double moveCostFrom = 0.0, moveCostTo = 0.0, moveCost = 0.0;

                    if(from != to || (from == to && targetIndex != sourceIndex + 1)) {
                        if(c != nullptr) {
                            costRemoved1 = timeMatrix[a.getNodeID() - 1][b.getNodeID() - 1] + timeMatrix[b.getNodeID()  - 1][c->getNodeID()  - 1];
                            costAdded1 = timeMatrix[a.getNodeID() - 1][f.getNodeID() - 1] + timeMatrix[f.getNodeID() - 1][c->getNodeID()  - 1];
                        } else {
                            costRemoved1 = timeMatrix[a.getNodeID() - 1][b.getNodeID() - 1];
                            costAdded1 = timeMatrix[a.getNodeID() - 1][f.getNodeID() - 1];
                        }

                        if(g != nullptr) {
                            costRemoved2 = timeMatrix[e.getNodeID() - 1][f.getNodeID() - 1] + timeMatrix[f.getNodeID() - 1][g->getNodeID() - 1];
                            costAdded2 = timeMatrix[e.getNodeID() - 1][b.getNodeID() - 1] + timeMatrix[b.getNodeID() - 1][g->getNodeID() - 1];
                        } else {
                            costRemoved2 = timeMatrix[e.getNodeID() - 1][f.getNodeID() - 1];
                            costAdded2 = timeMatrix[e.getNodeID() - 1][b.getNodeID() - 1];
                        }


                    } else { //target = source + 1
                        if(targetIndex == sourceIndex + 1) {
                            costRemoved1 = timeMatrix[a.getNodeID() - 1][b.getNodeID() - 1];
                            costAdded1 = timeMatrix[a.getNodeID() - 1][f.getNodeID() - 1];

                            if(g != nullptr) {
                                costRemoved2 = timeMatrix[f.getNodeID() - 1][g->getNodeID() - 1];
                                costAdded2 = timeMatrix[b.getNodeID() - 1][g->getNodeID() - 1];
                            }

                        }
                    }

                    moveCostFrom = costAdded1 - costRemoved1;
                    moveCostTo = costAdded2 - costRemoved2;
                    moveCost = moveCostFrom + moveCostTo;

                    //decongestion
                    double criterion = std::numeric_limits<double>::max();
                    if(to == from) {
                        criterion = examinedRouteTo.getTotalRouteTimeInHrs() + moveCost;
                    } else {
                        criterion = examinedRouteTo.getTotalRouteTimeInHrs() + moveCostTo;
                    }

                    if(criterion < maxTime) {
                        if(moveCostFrom < bestMoveCostFrom && moveCost < 0.0) {
                            bestMoveCostFrom = moveCostFrom;
                            sm.setSourceIndex(sourceIndex);
                            sm.setTargetIndex(targetIndex);
                            sm.setExaminedRouteFrom(examinedRouteFrom);
                            sm.setExaminedRouteTo(examinedRouteTo);
                            sm.setMoveCostFrom(moveCostFrom);
                            sm.setMoveCostTo(moveCostTo);
                            sm.setMoveCost(moveCost);


                            int demandAbsoluteDifference = abs(b.getDemand() - f.getDemand());
                            if(b.getDemand() == f.getDemand()) {
                                sm.setFromRemainingCap(examinedRouteFrom.getVeh().getRemainingCap()); //
                                sm.setToRemainingCap(examinedRouteTo.getVeh().getRemainingCap());
                            } else if (b.getDemand() > f.getDemand()) {
                                sm.setFromRemainingCap(examinedRouteFrom.getVeh().getRemainingCap() + demandAbsoluteDifference);
                                sm.setToRemainingCap(examinedRouteTo.getVeh().getRemainingCap() - demandAbsoluteDifference);
                            } else {
                                sm.setFromRemainingCap(examinedRouteFrom.getVeh().getRemainingCap() - demandAbsoluteDifference);
                                sm.setToRemainingCap(examinedRouteTo.getVeh().getRemainingCap() + demandAbsoluteDifference);
                            }
                        }
                    }
                }
            }
        }
        return sm;
    }


    TwoOpt VRP::findBestTwoOptMove(std::vector<Route> routes) {

        TwoOpt twoOpt;
        double bestMoveCost = std::numeric_limits<double>::max();

        for(int from = 0; from < routes.size(); from++) {
            Route fromRoute = routes.at(from);

            for(int to = 0; to < routes.size(); to++) {
                Route toRoute = routes.at(to);

                for(int fromIndex = 0; fromIndex < fromRoute.getRouteSize(); fromIndex++) {
                    int startTo = 0;
                    if(from == to) {
                        startTo = fromIndex + 2;
                    }

                    Node a = *fromRoute.getRouteNodes().at(fromIndex);
                    Node *b = nullptr;
                    if(fromIndex + 1 < fromRoute.getRouteSize()) {
                        b = fromRoute.getRouteNodes().at(fromIndex + 1);
                    }

                    for(int toIndex = startTo; toIndex < toRoute.getRouteSize(); toIndex++) {
                        Node k = *toRoute.getRouteNodes().at(toIndex);
                        Node *l = nullptr;
                        if(toIndex + 1 < toRoute.getRouteSize()) {
                            l = toRoute.getRouteNodes().at(toIndex + 1);
                        }

                        //start
                        double costAdded = 0.0, costRemoved = 0.0, moveCost = 0.0;
                        if(from == to) {
                            if(fromIndex == 0 && toIndex == fromRoute.getRouteSize() - 1) {
                                continue;
                            }

                            if(fromIndex + 1 == toIndex) {
                                continue;
                            }

                            if( l != nullptr) {
                                costAdded = timeMatrix[a.getNodeID() - 1][k.getNodeID() - 1] + timeMatrix[b->getNodeID() - 1][l->getNodeID() - 1];
                                costRemoved = timeMatrix[a.getNodeID()- 1][b->getNodeID() - 1] + timeMatrix[k.getNodeID() - 1][l->getNodeID() - 1];
                            } else { //k is the last node of the route
                                costAdded = timeMatrix[a.getNodeID() - 1][k.getNodeID() - 1];
                                costRemoved = timeMatrix[a.getNodeID() - 1][b->getNodeID() - 1];
                            }


                        } else { //if routes are different
                            if(fromIndex == 0 && toIndex == 0) {
                                continue;
                            }

                            //if b and l are null
                            if(fromIndex == fromRoute.getRouteSize() - 1 && toIndex == toRoute.getRouteSize() - 1) {
                                continue;
                            }

                            if(capacityConstraintsAreViolated(fromRoute, fromIndex, toRoute, toIndex)) {
                                continue;
                            }

                            if(b != nullptr && l != nullptr) {
                                costAdded = timeMatrix[a.getNodeID() - 1][l->getNodeID() - 1] + timeMatrix[k.getNodeID() - 1][b->getNodeID() - 1];
                                costRemoved = timeMatrix[a.getNodeID() - 1][b->getNodeID() - 1] + timeMatrix[k.getNodeID() - 1][l->getNodeID() - 1];
                            } else if (b == nullptr) {
                                costAdded = timeMatrix[a.getNodeID()  - 1][l->getNodeID()  - 1];
                                costRemoved = timeMatrix[k.getNodeID()  - 1][l->getNodeID()  - 1];
                            } else if (l == nullptr) {
                                costAdded = timeMatrix[k.getNodeID()  - 1][b->getNodeID()  - 1];
                                costRemoved = timeMatrix[a.getNodeID()  - 1][b->getNodeID()  - 1];
                            }

                        }
                        moveCost = costAdded - costRemoved;
                        if(moveCost < bestMoveCost) {
                            bestMoveCost = moveCost;
                            twoOpt.setFromRoute(fromRoute);
                            twoOpt.setToRoute(toRoute);
                            twoOpt.setFromIndex(fromIndex);
                            twoOpt.setToIndex(toIndex);
                            twoOpt.setMoveCost(moveCost);
                        }
                    }

                }
            }
        }

        return twoOpt;
    }

    bool VRP::capacityConstraintsAreViolated(Route route1, int index1, Route route2, int index2) {
        double firstRouteFirstSegmentLoad = 0;
        for(int i = 0; i <= index1; i++) {
            firstRouteFirstSegmentLoad += route1.getRouteNodes().at(i)->getDemand();
        }
        double firstRouteLoad = route1.getVeh().getMaxCap() - route1.getVeh().getRemainingCap();
        double firstRouteSecondSegmentLoad = firstRouteLoad - firstRouteFirstSegmentLoad;

        double secondRouteFirstSegmentLoad = 0;
        for(int i = 0; i <= index2; i++) {
            secondRouteFirstSegmentLoad += route2.getRouteNodes().at(i)->getDemand();
        }
        double secondRouteLoad = route2.getVeh().getMaxCap() - route2.getVeh().getRemainingCap();
        double secondRouteSecondSegmentLoad = secondRouteLoad - secondRouteFirstSegmentLoad;

        if(firstRouteFirstSegmentLoad + secondRouteSecondSegmentLoad > route1.getVeh().getMaxCap()) {
            return true;
        }

        if(secondRouteFirstSegmentLoad + firstRouteSecondSegmentLoad > route2.getVeh().getMaxCap()) {
            return true;
        }

        return false;
    }
// ----------------------------------------------------- Graphing Methods
std::vector< std::pair<size_t, size_t> > VRP::getGraphEdges(){
    std::vector< std::pair<size_t, size_t> > vectorEdges;

    for(int i = 0; i < this->allRoutes.size(); i++){ // Route It
        std::vector<Node*> routeNodes = this->allRoutes.at(i).getRouteNodes();
        vectorEdges.reserve( routeNodes.size() );

        for(size_t j = 0; j < routeNodes.size()  - 1; j++){
            std::pair<size_t, size_t> pair(routeNodes[j]->getNodeID() - 1, routeNodes[j+1]->getNodeID() - 1);
            vectorEdges.emplace_back(pair);
        }
    }
    return vectorEdges;
}

