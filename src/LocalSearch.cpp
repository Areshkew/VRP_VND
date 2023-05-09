#include "LocalSearch.h"
#include "Node.h"

// ----------------------- RelocationMove
// Take a Node from another route and put in the actual route.
bool RelocationMove::capConstrainsAreViolated(Route targetRoute, int incomingDemand){
    return targetRoute.getVeh().getRemainingCap() < incomingDemand;
}

void RelocationMove::applyRelocationMove(){
    //Node from Source Position
    Node relocatedNode = *fromRoute.getRouteNodes().at(sourcePosition);

    this->fromRoute.getRouteNodes().erase( fromRoute.getRouteNodes().begin() + sourcePosition );

    if(this->fromRoute.equals(toRoute)) {
        if(this->sourcePosition > targetPosition) {
            this->toRoute.getRouteNodes()
                         .emplace( fromRoute.getRouteNodes().begin() + targetPosition + 1, &relocatedNode);
        } else {
            this->toRoute.getRouteNodes()
                         .emplace( fromRoute.getRouteNodes().begin() + targetPosition, &relocatedNode);
        }

    } else {
        this->toRoute.getRouteNodes()
                     .emplace(fromRoute.getRouteNodes().begin() + targetPosition + 1, &relocatedNode);
    }
    this->fromRoute.getVeh().setRemainingCap(fromRemainingCap);
    this->toRoute.getVeh().setRemainingCap(toRemainingCap);
}

bool RelocationMove::isCostImproving(){
    return this->moveCost < 0;
}

int RelocationMove::getSourcePosition(){
    return this->sourcePosition;
}

void RelocationMove::setSourcePosition(int sourcePosition){
    this->sourcePosition = sourcePosition;
}

int RelocationMove::getTargetPosition(){
    return this->targetPosition;
}

void RelocationMove::setTargetPosition(int targetPosition){
    this->targetPosition = targetPosition;
}

double RelocationMove::getMoveCost(){
    return this->moveCost;
}

void RelocationMove::setMoveCost(double moveCost){
    this->moveCost = moveCost;
}

Route RelocationMove::getFromRoute(){
    return this->fromRoute;
}

void RelocationMove::setFromRoute(Route fromRoute){
    this->fromRoute = fromRoute;
}

Route RelocationMove::getToRoute(){
    return this->toRoute;
}

void RelocationMove::setToRoute(Route toRoute){
    this->toRoute = toRoute;
}

double RelocationMove::getMoveCostFrom() {
    return this->moveCostFrom;
}

void RelocationMove::setMoveCostFrom(double moveCostFrom){
    this->moveCostFrom = moveCostFrom;
}

double RelocationMove::getMoveCostTo(){
    return this->moveCostTo;
}

void RelocationMove::setMoveCostTo(double moveCostTo){
    this->moveCostTo = moveCostTo;
}

int RelocationMove::getFromRemainingCap(){
    return this->fromRemainingCap;
}

void RelocationMove::setFromRemainingCap(int fromRemainingCap){
    this->fromRemainingCap = fromRemainingCap;
}

int RelocationMove::getToRemainingCap(){
    return this->toRemainingCap;
}

void RelocationMove::setToRemainingCap(int toRemainingCap){
    this->toRemainingCap = toRemainingCap;
}

// ----------------------- SwapMove
// Change two elements from the actual route if they're better.
void SwapMove::applySwapMove(){
    if(examinedRouteFrom.equals(examinedRouteTo)) {
        Node b = *examinedRouteFrom.getRouteNodes().at(sourceIndex);
        Node f = *examinedRouteTo.getRouteNodes().at(targetIndex);

        examinedRouteFrom.getRouteNodes().at(sourceIndex) = &f;
        examinedRouteTo.getRouteNodes().at(targetIndex) = &b;

    } else {
        Node b = *examinedRouteFrom.getRouteNodes().at(sourceIndex);
        Node f = *examinedRouteTo.getRouteNodes().at(targetIndex);


        examinedRouteFrom.getVeh().setRemainingCap(fromRemainingCap);
        examinedRouteTo.getVeh().setRemainingCap(toRemainingCap);

        examinedRouteFrom.getRouteNodes().at(sourceIndex) = &f;
        examinedRouteTo.getRouteNodes().at(targetIndex) = &b;
    }
}

int SwapMove::getSourceIndex(){
    return this->sourceIndex;
}

void SwapMove::setSourceIndex(int sourceIndex){
    this->sourceIndex = sourceIndex;
}

int SwapMove::getTargetIndex(){
    return this->targetIndex;
}

void SwapMove::setTargetIndex(int targetIndex){
    this->targetIndex = targetIndex;
}

double SwapMove::getMoveCostFrom(){
    return this->moveCostFrom;
}

void SwapMove::setMoveCostFrom(double moveCostFrom){
    this->moveCostFrom = moveCostFrom;
}

double SwapMove::getMoveCostTo(){
    return this->moveCostTo;
}

void SwapMove::setMoveCostTo(double moveCostTo){
    this->moveCostTo = moveCostTo;
}

double SwapMove::getMoveCost(){
    return this->moveCost;
}

void SwapMove::setMoveCost(double moveCost){
    this->moveCost = moveCost;
}

int SwapMove::getFromRemainingCap(){
    return this->fromRemainingCap;
}

void SwapMove::setFromRemainingCap(int fromRemainingCap){
    this->fromRemainingCap = fromRemainingCap;
}

int SwapMove::getToRemainingCap(){
    return this->toRemainingCap;
}

void SwapMove::setToRemainingCap(int toRemainingCap){
    this->toRemainingCap = toRemainingCap;
}

Route SwapMove::getExaminedRouteFrom(){
    return this->examinedRouteFrom;
}

void SwapMove::setExaminedRouteFrom(Route examinedRouteFrom){
    this->examinedRouteFrom = examinedRouteFrom;
}

Route SwapMove::getExaminedRouteTo(){
    return this->examinedRouteTo;
}

void SwapMove::setExaminedRouteTo(Route examinedRouteTo){
    this->examinedRouteTo = examinedRouteTo;
}

// ----------------------- TwoOpt
// Change node between two routes.
void TwoOpt::applyTwoOptMove(){
    if(fromRoute.equals(toRoute)) {
        std::vector<Node*> modifiedRoute;

        for(int i = 0; i <= fromIndex; i++) {
            modifiedRoute.emplace_back( this->fromRoute.getRouteNodes().at(i) );
        }

        for(int i = toIndex; i > fromIndex; i--) {
            modifiedRoute.emplace_back( this->fromRoute.getRouteNodes().at(i) );
        }

        for(int i = toIndex + 1; i < fromRoute.getRouteSize(); i++) {
            modifiedRoute.emplace_back( this->fromRoute.getRouteNodes().at(i) );
        }

        int load = 0;
        for(Node* node : modifiedRoute) {
            load += node->getDemand();
        }
        this->fromRoute.setRouteNodes( modifiedRoute );
        this->fromRoute.getVeh().setRemainingCap(fromRoute.getVeh().getMaxCap() - load);
        this->fromRoute.setTotalRouteTimeInHrs(fromRoute.getTotalRouteTimeInHrs() + moveCost);
    } else {
        std::vector<Node*> modifiedRoute1;
        std::vector<Node*> modifiedRoute2;
        for(int i = 0; i <= fromIndex; i++) {
            modifiedRoute1.emplace_back( this->fromRoute.getRouteNodes().at(i) );
        }
        for(int i = toIndex + 1; i < toRoute.getRouteSize(); i++) {
            modifiedRoute1.emplace_back( this->toRoute.getRouteNodes().at(i) );
        }

        for (int i = 0 ; i <= toIndex; i++) {
            modifiedRoute2.emplace_back( this->toRoute.getRouteNodes().at(i) );
        }
        for (int i = fromIndex + 1 ; i < fromRoute.getRouteSize(); i++) {
            modifiedRoute2.emplace_back( this->fromRoute.getRouteNodes().at(i) );
        }
        this->fromRoute.setRouteNodes(modifiedRoute1);
        this->toRoute.setRouteNodes(modifiedRoute2);
        this->updateRouteCap(&fromRoute);
        this->updateRouteCap(&toRoute);
    }
}

void TwoOpt::updateRouteCap(Route *route){
    int load = 0;
    for(Node* node : route->getRouteNodes()) {
        load += node->getDemand();
    }
    route->getVeh().setRemainingCap(route->getVeh().getMaxCap() - load);
}

Route TwoOpt::getFromRoute(){
    return this->fromRoute;
}

void TwoOpt::setFromRoute(Route fromRoute){
    this->fromRoute = fromRoute;
}

Route TwoOpt::getToRoute(){
    return this->toRoute;
}

void TwoOpt::setToRoute(Route toRoute){
    this->toRoute = toRoute;
}

int TwoOpt::getFromIndex(){
    return this->fromIndex;
}

void TwoOpt::setFromIndex(int fromIndex){
    this->fromIndex = fromIndex;
}

int TwoOpt::getToIndex(){
    return this->toIndex;
}

void TwoOpt::setToIndex(int toIndex){
    this->toIndex = toIndex;
}

double TwoOpt::getMoveCost(){
    return this->moveCost;
}   

void TwoOpt::setMoveCost(double moveCost){
    this->moveCost = moveCost;
}