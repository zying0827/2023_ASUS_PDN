#ifndef DETAILED_DB_H
#define dETAILED_DB_H

#include "../base/Include.h"
using namespace std;

class Grid {
    public:
        Grid() {
            _congestion = 0;
            _congestCur = 0;
            _congestHis = 0;
        }
        ~Grid() {}
        int congestion() const { return _congestion; }
        int congestCur() const { return _congestCur; }
        int congestHis() const { return _congestHis; }
        size_t vNetId(size_t i) const { return _vNetId[i]; }
        size_t numNets() const { return _vNetId.size(); }
        void incCongestCur() { _congestCur ++; _congestion ++; }
        void decCongestCur() { _congestCur --; _congestion --; }
        void addNet(size_t netId) { _vNetId.push_back(netId); }
    private:
        int _congestion;    // _congestHis + _congestCur
        int _congestCur;    // current congestion cost
        int _congestHis;    // history congestion cost
        vector<size_t> _vNetId;
};

enum GNodeStatus {
    Init,
    InQueue,
    InPath
};

class GNode {
    public:
        GNode(int xId, int yId) : _xId(xId), _yId(yId) {
            _status = GNodeStatus::Init;
            _parent = NULL;
            _cost = numeric_limits<double>::max();
            _curDist = numeric_limits<double>::max();
            _estDist = numeric_limits<double>::max();
            _congest = numeric_limits<double>::max();
        }
        ~GNode() {}

        // get function
        int xId() const { return _xId; }
        int yId() const { return _yId; }
        GNodeStatus status() const { return _status; }
        GNode* parent() { return _parent; }
        double cost() const { return _cost; }
        double curDist() const { return _curDist; }
        double estDist() const { return _estDist; }
        double congest() const { return _congest; }

        // set function
        void setStatus(GNodeStatus status) { _status = status; }
        void setParent(GNode* parentNode) { _parent = parentNode; }
        void setCost(double cost) { _cost = cost; }
        void setCurDist(double curDist) { _curDist = curDist; }
        void setEstDist(double estDist) { _estDist = estDist; }
        void setCongest(double congest) { _congest = congest; }

    private:
        int _xId;
        int _yId;
        GNodeStatus _status;
        GNode* _parent;
        double _cost;
        double _curDist;    // the current distance cost (from the source)
        double _estDist;    // the estimated distance cost (to the target)
        double _congest;    // the (accumulated) congestion cost
};

#endif