#ifndef NETWORKILP_H
#define NETWORKILP_H

#include <gurobi_c++.h>
#include "../base/Include.h"
// #include "OASG.h"
using namespace std;

class NetworkILP {
    // public:
    //     NetworkILP(RGraph& rGraph, vector<double> netCrit, double areaCost, double widthCost, double viaCost);
    //     ~NetworkILP() {}

    //     void formulate();
    //     void solve();
    //     void collectResult();
    //     void printResult();

    // private:
    //     void clear();
    //     void setObjective();
    //     void setFlowConstraints();
    //     void setViaConstraints();
    //     void setNodeConstraints();

    //     RGraph& _rGraph;
    //     GRBEnv _env;
    //     GRBModel _model;
    //     // gurobi variables
    //     // Index Order: _v[netId][EdgeTypeId]
    //     // vector< vector<GRBVar> > _vFh;   // horizontal flow of non-fixed-via nodes
    //     // vector< vector<GRBVar> > _vFf;   // vertical flow of 2D feasible nodes
    //     // vector< vector<GRBVar> > _vFs;   // vertical flow of source-via nodes
    //     // vector< vector<GRBVar> > _vFt;   // vertical flow of target-via nodes
    //     // vector< vector<GRBVar> > _vFr;   // horizontal flow of fixed-via nodes

    //     // vector< vector< vector<GRBVar> > > _vOtherApproval; // whether an otherNode is approved to flow Net[netId], index = [netId][layId][otherNodeId]
    //     // vector< vector< vector<GRBVar> > > _vF2DApproval; // whether a F2DNode is approved to flow Net[netId], index = [netId][layId][F2DNodeId]
    //     // vector< vector<GRBVar> > _vF2DVia;  // whether to add via of Net on 2D Feasible Node, index = [netId][F2DNodeId]

    //     // vector< vector< vector<GRBVar> > > _vHFlow;     // horizontal flow of non-fixed-via nodes, index = [netId] [layId] [HEdgeId]
    //     // vector< vector< vector<GRBVar> > > _vF2DFlow;     // vertical flow of 2D feasible nodes from layId to layId+1, index = [netId] [layPairId] [F2DNodeId]
    //     // vector< vector<GRBVar> > _vSViaFlow;     // vertical flow of source-via nodes from layId+1 to layId, index = [netId] [layPairId]
    //     // vector< vector< vector<GRBVar> > > _vTViaFlow;     // vertical flow of target-via nodes from layId to layId+1, index = [netId] [layPairId] [T2DNodeId]
    //     // vector< vector< vector<GRBVar> > > _vSRFlow;     // horizontal flow of source-via nodes, index = [netId] [layId] [SREdgeId]
    //     // vector< vector< vector<GRBVar> > > _vTRFlow;     // horizontal flow of target-via nodes, index = [netId] [layId] [TREdgeId]

    //     GRBVar*** _vOtherApproval;  // whether an otherNode is approved to flow Net[netId], index = [netId][layId][otherNodeId]
    //     GRBVar*** _vF2DApproval;    // whether a F2DNode is approved to flow Net[netId], index = [netId][layId][F2DNodeId]
    //     GRBVar** _vF2DVia;          // whether to add via of Net on 2D Feasible Node, index = [netId][F2DNodeId]
    //     GRBVar*** _vHFlow;      // horizontal flow of non-fixed-via nodes, index = [netId] [layId] [HEdgeId]
    //     GRBVar*** _vF2DFlow;    // vertical flow of 2D feasible nodes from layId to layId+1, index = [netId] [layPairId] [F2DNodeId]
    //     GRBVar** _vSViaFlow;    // vertical flow of source-via nodes from layId+1 to layId, index = [netId] [layPairId]
    //     GRBVar*** _vTViaFlow;   // vertical flow of target-via nodes from layId to layId+1, index = [netId] [layPairId] [T2DNodeId]
    //     GRBVar*** _vSRFlow;     // horizontal flow of source-via nodes, index = [netId] [layId] [SREdgeId]
    //     GRBVar*** _vTRFlow;     // horizontal flow of target-via nodes, index = [netId] [layId] [TREdgeId]

    //     GRBVar _ubResistance;   // gamma

    //     int _numNets;
    //     vector< double > _netCrit;
    //     double _areaCost;
    //     double _widthCost;
    //     double _viaCost;

};

#endif