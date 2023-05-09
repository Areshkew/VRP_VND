#include <iostream>
#include <cmath>
#include <matplot/matplot.h>
#include "src/VRP.h"

#define ARGS ;

void getCoords(VRP *VRP, matplot::vector_1d *X, matplot::vector_1d *Y){
    for(auto &it : VRP->getAllNodes()){
        if(*it.isServiced()){
            X->emplace_back( it.getX() );
            Y->emplace_back( it.getY() );
        }else{
            X->emplace_back( 50 );
            Y->emplace_back( 50 );
        }
    }
}

int main( int argc, char *argv[])
{
   // ACCEPT ARGUMENTS THROUGH COMMAND LINE
   #ifdef ARGS
       int totalServicePoints = atoi(argv[1]);
       int totalVehicles = atoi(argv[2]);
   #else
       int totalServicePoints = 0;
       int totalVehicles = 0;
   #endif

    // VRP Solution
    VRP vrp(totalServicePoints, totalVehicles);
    std::cout << "----------------" << std::endl;
    for(auto &it : vrp.getAllRoutes()){ 
        std::cout << "Ruta: " << it.getRouteID() << std::endl;
        std::cout << "Tiempo (Hrs): " << it.getTotalRouteTimeInHrs() << std::endl;
        for(int j = 0; j < it.getRouteNodes().size(); j++){
            if(j == it.getRouteNodes().size() - 1){
                std::cout << it.getRouteNodes().at(j)->getNodeID() - 1;
            }else
                std::cout << it.getRouteNodes().at(j)->getNodeID() - 1 << " -> "; 
        }
        std::cout << std::endl;
        std::cout << "----------------" << std::endl;
    }

    // Graph 
    using namespace matplot;
    std::vector<std::pair<size_t, size_t>> edges = vrp.getGraphEdges();
    matplot::vector_1d x_data;
    matplot::vector_1d y_data;
    getCoords(&vrp, &x_data, &y_data);

    auto g = digraph(edges);
    g->marker_size(7);
    g->line_width(1);
    g->x_data(x_data);
    g->y_data(y_data);
    // g->show_labels(false);

    show();
    return 0;
}