#ifndef TILE_H
#define TILE_H

#include "Include.h"

using namespace std;

class Tile{
    public:
        Tile(unsigned int layId, unsigned int rowId, unsigned int colId): _layId(layId), _rowId(rowId), _colId(colId) {
            _netId = 0;
            _hasNet = false;
            _obstacle = false;
        }
        ~Tile() {}

        // get functions
        unsigned int layId() const { return _layId; }
        unsigned int rowId() const { return _rowId; }
        unsigned int colId() const { return _colId; }
        unsigned int netId() const {
            if (!_hasNet) {
                cerr << "ERROR: Tile(" << _layId << "," << _rowId << "," << _colId << ") has no net!" << endl;
                return 0;
            } else {
                return _netId;
            }
        }
        unsigned int nodeId() const { return _nodeId; }
        bool hasNet() const { return _hasNet; }
        bool obstacle() const { return _obstacle; }

        // set functions
        void setObstacle() {_obstacle = true; }
        void setNet(unsigned int netId) {
            if (_hasNet || _obstacle) {
                cerr << "setNet fails! ERROR: Tile(" << _layId << "," << _rowId << "," << _colId << ") is full!" << endl;
            }
            _netId = netId; 
            _hasNet = true;
        }
    private:
        unsigned int _layId;
        unsigned int _rowId;
        unsigned int _colId;
        unsigned int _netId;
        unsigned int _nodeId;
        bool _hasNet;
        bool _obstacle;
};

#endif