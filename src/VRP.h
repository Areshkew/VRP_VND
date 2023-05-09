#ifndef VRP_H
#define VRP_H

#include <vector>
#include <atomic>
#include "Vehicle.h"
#include "Route.h"
#include "Node.h"
#include "QuadTree.h"
#include "LocalSearch.h"

class VRP{

    private:
        int totalVehicles, totalServicePoints;
        Node storage;
        std::vector<Node> allNodes;
        QuadTree allPoints;
        int QT_COUNT;
        std::vector<Vehicle> allVehicles;
        std::vector<Route> allRoutes;
        std::vector< std::vector<double> > distanceMatrix;
        std::vector< std::vector<double> > timeMatrix;

    public:
        inline static std::atomic<int> vehicleCounter = 0;
        inline static std::atomic<int> nodeCounter = 0;

        // Constructor
        VRP(int totalServicePoints, int totalVehicles);

        // Initializing Methods
        void initializeAlgorithmData();
        void generateRandomNodes();
        void euclideanDistanceMatrix();
        void getTimeMatrix();
        void generateVehicles();
        void routesInit();

        // Utility Methods
        void PrintDM();
        std::vector<Node> getAllNodes();
        std::vector<Vehicle> getAllVehs();
        std::vector<Route> getAllRoutes();
        Route findSlowestRoute(std::vector<Route> routes);
        
        // Nearest Neighboor
        void nearestNeighboor();
        void expandSearch(std::vector<XY> *found, int range, Node *currentNode);

        // DVNS
        std::vector<Route> CloneRoutes();
        void DVNS(int kmax);
        void updateRouteCost(Route route);
        bool objectiveFunctionIsImproved(std::vector<Route> scurrent, std::vector<Route> snew);
        RelocationMove findBestRelocationMove(std::vector<Route> routes);
        SwapMove findBestSwapMove(std::vector<Route> routes);
        TwoOpt findBestTwoOptMove(std::vector<Route> routes);
        bool capacityConstraintsAreViolated(Route route1, int index1, Route route2, int index2);

        //Graphing Methods
        std::vector< std::pair<size_t, size_t> > getGraphEdges();
};

#endif