#ifndef DB_H
#define DB_H

#include "Include.h"
#include "Tile.h"
#include "Via.h"

class DB {
    public:
        DB() {}
        ~DB() {}

        Tile* vTile(int layId, int rowId, int colId) { return &_vTile[layId][rowId][colId]; }
        Via* vVia(int viaId) { return &_vVia[viaId]; }
        ViaCluster* vViaCluster(int clusterId) { return &_vViaCluster[clusterId]; }
        size_t numNets() const { return _numNets; }
        size_t numLayers() const { return _numLayers; }
        
    private:
        vector< vector< vector< Tile > > > _vTile;
        vector<Via> _vVia;
        vector<ViaCluster> _vViaCluster;
        size_t _numNets;
        size_t _numLayers;
};

#endif