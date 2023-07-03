#ifndef VIA_H
#define VIA_H

#include "Include.h"

using namespace std;

class Via{
    public:
        Via(unsigned int rowId, unsigned int colId, unsigned int netId, bool fixed): _rowId(rowId), _colId(colId), _netId(netId), _fixed(fixed) {}
        ~Via() {}
        
        unsigned int rowId() const { return _rowId; }
        unsigned int colId() const { return _colId; }
        unsigned int netId() const {return _netId; }
        bool fixed() const {return _fixed; }
    private:
        unsigned int _rowId;
        unsigned int _colId;
        unsigned int _netId;
        bool _fixed;
};

class ViaCluster{
    public:
        ViaCluster() {}
        ~ViaCluster() {}

        Via* vVia(int viaId) const { return _vVia[viaId]; }
        void addVia(Via* v) { _vVia.push_back(v); }
    private:
        vector<Via*> _vVia;
};

#endif