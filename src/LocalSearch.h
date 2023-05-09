#ifndef LocalSearch_H
#define LocalSearch_H

#include "Route.h"
#include "Vehicle.h"

// ----------------------- RelocationMove
// Take a Node from another route and put in the actual route.
class RelocationMove{
    private:
        int sourcePosition, targetPosition;
        Route fromRoute, toRoute;
        double moveCostFrom, moveCostTo;
        double moveCost;
        int fromRemainingCap, toRemainingCap;
        const Vehicle veh;

    public:
        RelocationMove() : sourcePosition(0), targetPosition(0), fromRoute(veh), toRoute(veh),
                           moveCostFrom(0.0), moveCostTo(0.0), moveCost(0.0), fromRemainingCap(0), toRemainingCap(0)  {
            Vehicle veh;
            veh.setVehID(9999);
            fromRoute = *new Route(veh);   
            toRoute  = *new Route(veh); 
        };

        static bool capConstrainsAreViolated(Route targetRoute, int incomingDemand);
        void applyRelocationMove();
        bool isCostImproving();
        int getSourcePosition();
        void setSourcePosition(int sourcePosition);
        int getTargetPosition();
        void setTargetPosition(int targetPosition);
        double getMoveCost();
        void setMoveCost(double moveCost);
        Route getFromRoute();
        void setFromRoute(Route fromRoute);
        Route getToRoute();
        void setToRoute(Route toRoute);
        double getMoveCostFrom();
        void setMoveCostFrom(double moveCostFrom);
        double getMoveCostTo();
        void setMoveCostTo(double moveCostTo);
        int getFromRemainingCap();
        void setFromRemainingCap(int fromRemainingCap);
        int getToRemainingCap();
        void setToRemainingCap(int toRemainingCap);
};


// ----------------------- SwapMove
// Change two elements from the actual route if they're better.
class SwapMove {
    private:
        int sourceIndex, targetIndex;
        double moveCostFrom, moveCostTo, moveCost;
        int fromRemainingCap, toRemainingCap;
        Route examinedRouteFrom, examinedRouteTo;
        const Vehicle veh;

    public:
        SwapMove() : sourceIndex(0), targetIndex(0), moveCostFrom(0.0), moveCostTo(0.0), moveCost(0),
                    fromRemainingCap(0), toRemainingCap(0), examinedRouteFrom(veh), examinedRouteTo(veh) {
            Vehicle veh;
            veh.setVehID(9999);
            examinedRouteFrom = *new Route(veh);   
            examinedRouteTo  = *new Route(veh);       
        };

        void applySwapMove();
        int getSourceIndex();
        void setSourceIndex(int sourceIndex);
        int getTargetIndex();
        void setTargetIndex(int targetIndex);
        double getMoveCostFrom();
        void setMoveCostFrom(double moveCostFrom);
        double getMoveCostTo();
        void setMoveCostTo(double moveCostTo);
        double getMoveCost();
        void setMoveCost(double moveCost);
        int getFromRemainingCap();
        void setFromRemainingCap(int fromRemainingCap);
        int getToRemainingCap();
        void setToRemainingCap(int toRemainingCap);
        Route getExaminedRouteFrom();
        void setExaminedRouteFrom(Route examinedRouteFrom);
        Route getExaminedRouteTo();
        void setExaminedRouteTo(Route examinedRouteTo);
};

// ----------------------- TwoOpt
// Change node between two routes.
class TwoOpt {
    private:
        Route fromRoute, toRoute;
        int fromIndex, toIndex;
        double moveCost;
        const Vehicle veh;

    public:
        TwoOpt() : fromRoute(veh), toRoute(veh), fromIndex(0), toIndex(0), moveCost(0.0){
            Vehicle veh;
            veh.setVehID(9999);
            fromRoute = *new Route(veh);   
            toRoute  = *new Route(veh); 
        };

        void updateRouteCap(Route *route);
        void applyTwoOptMove();
        Route getFromRoute();
        void setFromRoute(Route fromRoute);
        Route getToRoute();
        void setToRoute(Route toRoute);
        int getFromIndex();
        void setFromIndex(int fromIndex);
        int getToIndex();
        void setToIndex(int toIndex);
        double getMoveCost();
        void setMoveCost(double moveCost);
};

#endif