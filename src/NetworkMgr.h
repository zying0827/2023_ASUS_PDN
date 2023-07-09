#ifndef NETWORKMGR_H
#define NETWORKMGR_H

#include "Include.h"
#include "DB.h"
#include "RGraph.h"
#include "NetworkILP.h"

class NetworkMgr {
    public:
        NetworkMgr(DB& db): _db(db), _rGraph(db.numNets(), db.numLayers(), 6) {}
        ~NetworkMgr() {}

        void genRGraph();   // generate a routing graph from the given database
        void distrNet();    // distribute the space to each net with a network-flow-based ILP formulation
        // post processing stage

    private:
        DB& _db;
        RGraph _rGraph;
        // NetworkILP _flowSolver; // defined in distrNet()


};

#endif
