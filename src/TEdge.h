#ifndef TEDGE_H
#define TEDGE_H

#include "Include.h"

using namespace std;

class TEdge {
    public:
        TEdge(): _sId(0), _tId(0), _capacity(0), _cost(0) {}
        ~TEdge() {}

        unsigned int sId() const { return _sId; }
        unsigned int tId() const { return _tId; }
        unsigned int capacity() const { return _capacity; }
        unsigned int cost() const { return _cost; }

        void setSId(unsigned int sId) { _sId = sId; }   
    private:
        unsigned int _sId;   // source node Id
        unsigned int _tId;   // target node Id
        unsigned int _capacity;
        unsigned int _cost;
};

#endif