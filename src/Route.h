#ifndef Route_H
#define Route_H

#include <vector>
#include "Vehicle.h"
#include "Node.h"

class Route{
    private:
        int routeID;
        Vehicle veh;
        std::vector<Node*> routeNodes;
        double totalRouteTimeInHrs;
        bool finalized;

    public:
        Route(Vehicle veh);

        //Methods
        bool equals(Route otherRoute);
        int getRouteSize();
        void addNodeToRoute(Node *new_node, double cost);
        Node * getLastNode();
        void updateVehCap(Node addedNode);
        void updateTotalRouteTime(double cost, Node node);
        //
        int getRouteID();
        void setRouteID(int new_routeID);
        Vehicle getVeh();
        void setVeh(Vehicle new_veh);
        std::vector<Node*> getRouteNodes();
        void setRouteNodes(std::vector<Node*> new_routeNodes);
        double getTotalRouteTimeInHrs();
        void setTotalRouteTimeInHrs(double new_totalRouteTimeInHrs);
        bool isFinalized();
        void setFinalized(bool new_finalized);   
};

#endif