#include <cmath>
#include <iostream>

#include "QuadTree.h"

// ----------------------------------------------- axis-aligned bounding box
bool AABB::containsPoint(XY point)
{
    return !(fabs(point.x - this->center.x) > this->halfDimension 
            || fabs(point.y - this->center.y) > this->halfDimension) && !*point.serviced;
}

bool AABB::intersectsAABB(AABB other)
{
    if(fabs(this->center.x - other.center.x) == (this->halfDimension + other.halfDimension)
            && fabs(this->center.y - other.center.y) == (this->halfDimension + other.halfDimension))
        return false;

    return (fabs(this->center.x - other.center.x) < (this->halfDimension + other.halfDimension) 
        && fabs(this->center.y - other.center.y) < (this->halfDimension + other.halfDimension));
}

// ----------------------------------------------- QuadTree Methods
std::vector<XY> QuadTree::getPoints(){
    return this->points;
}

bool QuadTree::insert(XY _point){
    if(!this->boundary.containsPoint(_point))
        return false;
    
    if(this->points.size() < this->QT_CAPACITY ){
        this->points.emplace_back(_point);
        return true;
    }else{
        if(!this->divided){            
            this->subdivide();
        }

        if ( this->northWest->insert(_point) ) {
            return true;
        } else if (this->northEast->insert(_point)) {
            return true;
        } else if (this->southWest->insert(_point)) {
            return true;
        } else if (this->southEast->insert(_point)) {
            return true;
        }
    }
    return false;
}

void QuadTree::subdivide(){
    auto x = this->boundary.center.x;
    auto y = this->boundary.center.y;
    auto halfDim = this->boundary.halfDimension;

    auto se = new AABB(XY(x + halfDim / 2, y - halfDim / 2, 0, &this->TRUE_VALUE), halfDim / 2);
    this->southEast = new QuadTree(*se); //sureste

    auto sw = new AABB(XY(x - halfDim / 2, y - halfDim / 2, 0, &this->TRUE_VALUE), halfDim / 2);
    this->southWest = new QuadTree(*sw); //suroeste
    
    auto ne = new AABB(XY(x + halfDim / 2, y + halfDim / 2, 0, &this->TRUE_VALUE), halfDim / 2);
    this->northEast = new QuadTree(*ne); //Noreste

    auto nw = new AABB(XY(x - halfDim / 2, y + halfDim / 2, 0, &this->TRUE_VALUE), halfDim / 2);
    this->northWest = new QuadTree(*nw); //Noroeste

    this->divided = true;
}

void QuadTree::query(AABB range, std::vector<XY> *found){

    if(!this->boundary.intersectsAABB(range))
        return;

    for (int i = 0; i < this->points.size(); i++) {
        if (range.containsPoint( this->points[i] )) {
            found->emplace_back( this->points[i] );
        }
    }

    if (this->divided) {
        this->northWest->query(range, found);
        this->northEast->query(range, found);
        this->southWest->query(range, found);
        this->southEast->query(range, found);
    }

}