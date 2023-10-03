#ifndef VOLT_EIGEN_H
#define VOLT_EIGEN_H

#include "../base/Include.h"
using namespace std;

class VoltEigen {
    public:
        VoltEigen(size_t numNodes) : _numNodes(numNodes) {
            vector<double> GRow(numNodes, 0.0);
            for (size_t rowId = 0; rowId < numNodes; ++ rowId) {
                _G.push_back(GRow);
                _I.push_back(0.0);
            }

        }
        ~VoltEigen() {}
        void setMatrix(size_t rowNodeId, size_t colNodeId, double resistance);
        void setInputVector(size_t rowNodeId, double inputVolt, double resistance);
        vector< vector< double > > G() { return _G; }
        vector<double> I() { return _I; }
        double G(size_t rowId, size_t colId) const { return _G[rowId][colId]; }
        double I(size_t rowId) const { return _I[rowId]; }
        double V(size_t rowId) const { return _V[rowId]; }
        size_t numNodes() const { return _numNodes; }
    private:
        vector< vector< double > > _G;  // the conductance matrix
        vector<double> _I;  // the input current vector
        vector<double> _V;  // the voltage vector to be solved
        size_t _numNodes;
};

#endif