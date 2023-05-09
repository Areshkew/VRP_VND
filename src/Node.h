#ifndef Node_H
#define Node_H

#include <random>
#include <string>
#include <atomic>
#include "QuadTree.h"

class Random{
    public:

        int nextInt(int n){
            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_int_distribution<int> dist(1,n);

            return dist(mt);
        }
};

class Node{
    private:
        int nodeID, demand;
        XY coords;
        double serviceTime;
        bool serviced;
        Random ran;

    public:
        //storage Constructor
        Node(float x, float y, int demand);

        Node();

        // Methods
        std::string toString();
        void printNodeData();

        // Getters Setters
        int getNodeID();
        void setNodeID(int nodeID);
        XY getCoords();

        int getX();
        void setX(int new_X);
        int getY();
        void setY(int new_Y);
        int getDemand();
        void setDemand(int new_Demand);
        double getServiceTime();
        bool * isServiced();
        void updateServiceStatus(bool serviced);
};
#endif