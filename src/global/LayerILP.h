#ifndef LAYER_ILP
#define LAYER_ILP

#include <gurobi_c++.h>
#include "../base/Include.h"
#include "RGraph.h"
using namespace std;

class LayerILP {
    public:
        LayerILP(RGraph& rGraph, vector< vector<double> > vNetWeight, vector<double> vAccuViaLength);
        ~LayerILP() {}

        void formulate();
        void solve();
        void collectResult();
        void printResult();

    private:
        void clear();
        void setObjective();
        void setConflictConstraints();
        
        RGraph& _rGraph;
        GRBEnv _env;
        GRBModel _model;
        // gurobi variables
        GRBVar*** _vFlow;  // index = [twoPinNetId] [layId] [RGEdgeId]
        GRBVar _currentLB;  // gamma
        
        // input constants
        vector< vector<double> > _vNetWeight; // index = [netId] [netTPortId]
        vector<double> _vAccuViaLength;     // index = [layId]

};

#endif