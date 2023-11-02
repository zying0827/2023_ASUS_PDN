#ifndef VOLT_EIGEN_H
#define VOLT_EIGEN_H

#include <gurobi_c++.h>
#include "../base/Include.h"
using namespace std;

class VoltEigen {
    public:
        VoltEigen(size_t numNodes) : _model(_env), _numNodes(numNodes) {
            vector<double> GRow(numNodes, 0.0);
            for (size_t rowId = 0; rowId < numNodes; ++ rowId) {
                _G.push_back(GRow);
                _I.push_back(0.0);
                _V.push_back(0.0);
            }
            _vVoltage = new GRBVar [numNodes];
            for (size_t nodeId = 0; nodeId < numNodes; ++ nodeId) {
                _vVoltage[nodeId] = _model.addVar(0.0, std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, "V_n" + to_string(nodeId) );
            }
        }
        ~VoltEigen() {}
        void setMatrix(size_t rowNodeId, double conductance);
        void setMatrix(size_t rowNodeId, size_t colNodeId, double conductance);
        void setInputVector(size_t rowNodeId, double inputVolt, double conductance);
        void setInputVector(size_t rowNodeId, double inputCurrent);
        void solve();
        vector< vector< double > > G() { return _G; }
        vector<double> I() { return _I; }
        double G(size_t rowId, size_t colId) const { return _G[rowId][colId]; }
        double I(size_t rowId) const { return _I[rowId]; }
        double V(size_t rowId) const { return _V[rowId]; }
        size_t numNodes() const { return _numNodes; }
    private:

        GRBEnv _env;
        GRBModel _model;
        GRBVar* _vVoltage;
        vector< vector< double > > _G;  // the conductance matrix
        vector<double> _I;  // the input current vector
        vector<double> _V;  // the voltage vector to be solved
        size_t _numNodes;
};

#endif