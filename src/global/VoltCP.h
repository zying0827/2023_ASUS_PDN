#ifndef VOLT_CP_H
#define VOLT_CP_H

#include <gurobi_c++.h>
#include "../base/Include.h"
#include "RGraph.h"
#include "../base/DB.h"
using namespace std;

class VoltCP {
    public:
        VoltCP(DB& db, RGraph& rGraph);
        ~VoltCP() {}

        void setObjective(double areaWeight, double viaWeight);
        void setVoltConstraints(double threshold);
        void addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width);
        void addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, double width);
        void relaxCapacityConstraints(vector<double> vLambda);
        void solve();
        void collectResult();
        // void printResult();

    private:
        DB& _db;
        RGraph& _rGraph;
        GRBEnv _env;
        GRBModel _model;
        GRBVar** _vVoltage;    // non-port node voltage, index = [netId] [nPortnodeId]
        GRBVar*** _vPEdgeInV;   // inverse of the (plane) edge voltage difference, index = [netId] [layPairId] [pEdgeId]
        GRBVar*** _vVEdgeInV;   // inverse of the (via) edge voltage difference, index = [netId] [layPairId] [vEdgeId]
        GRBVar**  _vMaxViaCost;      // the maximum flow on an OASGEdge, index = [netId] [vEdgeId]
        int _numCapConstrs;          // number of capacity constraints
};

#endif