#ifndef NETWORKMGR_H
#define NETWORKMGR_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "../detailed/RGraph.h"
#include "NetworkILP.h"
#include "../base/SVGPlot.h"

class NetworkMgr {
    // public:
    //     NetworkMgr(DB& db, SVGPlot& plot): _db(db), _plot(plot), _rGraph(db.numNets(), db.numLayers(), 6) {}
    //     ~NetworkMgr() {}

    //     void genRGraph();   // generate a routing graph from the given database
    //     void distrNet();    // distribute the space to each net with a network-flow-based ILP formulation
    //     // post processing stage

    //     void drawRGraph(bool distrDone);
    //     void drawResult();
    //     void drawDB();

    // private:
    //     DB& _db;
    //     SVGPlot& _plot;
    //     RGraph _rGraph;
    //     // NetworkILP _flowSolver; // defined in distrNet()


};

#endif
