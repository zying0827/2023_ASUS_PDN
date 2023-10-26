#ifndef VOLT_SLP_H
#define VOLT_SLP_H

#include <gurobi_c++.h>
#include "../base/Include.h"
#include "RGraph.h"
#include "../base/DB.h"
using namespace std;

class VoltSLP {
    public:
        VoltSLP(DB& db, RGraph& rGraph, vector< vector< double > > vOldVoltage);
        ~VoltSLP() {}

        void setObjective(double areaWeight, double viaWeight);
        void setVoltConstraints(double threshold);
        void setLimitConstraint(double ratio);
        void addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width);
        void addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, double width);
        void relaxCapacityConstraints(vector<double> vLambda);
        void solve();
        void collectResult();
        // void printResult();
        void collectTempVoltage();
        void solveRelaxed();
        void collectRelaxedTempVoltage();
        void collectRelaxedResult();
        void printRelaxedResult();

        vector< vector< double > > vNewVoltage() { return _vNewVoltage; }
        double area() const { return _area; }
        double viaArea() const { return _viaArea; }
        double overlap() const { return _overlap; }

    private:
        GRBLinExpr linApprox(double cost, OASGEdge* edge);
        // input
        DB& _db;
        RGraph& _rGraph;
        vector< vector< double > > _vOldVoltage;    // non-port node voltage from the last iteration, index = [netId] [nPortnodeId]
        // vector< vector< vector< double > > > _vPOldInV;     // inverse of the (plane) edge voltage difference from the last iteration, index = [netId] [layId] [pEdgeId]
        // vector< vector< vector< double > > > _vVOldInV;     // inverse of the (via) edge voltage difference from the last iteration, index = [netId] [layPairId] [vEdgeId]

        // gurobi model
        GRBEnv _env;
        GRBModel _model;
        GRBModel* _modelRelaxed;

        // gurobi variable
        GRBVar** _vVoltage;    // non-port node voltage, index = [netId] [nPortnodeId]
        GRBVar*** _vPEdgeInV;   // inverse of the (plane) edge voltage difference, index = [netId] [layId] [pEdgeId]
        GRBVar*** _vVEdgeInV;   // inverse of the (via) edge voltage difference, index = [netId] [layPairId] [vEdgeId]
        GRBVar**  _vMaxViaCost;      // the maximum flow on an OASGEdge, index = [netId] [vEdgeId]
        int _numCapConstrs;          // number of non-obstacle/boundary capacity constraints

        // output
        vector< vector< double > > _vNewVoltage;    // non-port node voltage in this iteration, index = [netId] [nPortnodeId]
        double _area;       // the resulting area, assigned in collectRelaxedResult
        double _viaArea;
        double _overlap;    // the resulting overlapped width, assigned in collectRelaxedResult
};

#endif