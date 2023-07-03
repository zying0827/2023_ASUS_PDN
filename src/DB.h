#ifndef DB_H
#define DB_H

#include "Include.h"
#include "Tile.h"
#include "Via.h"

class DB {
    public:
        DB() {}
        ~DB() {}

        Tile* vTile(int layId, int rowId, int colId) const { return &_vTile[layId][rowId][colId]; }
        Via* vVia(int viaId) const { return &_vVia[viaId]; }
        ViaCluster* vViaCluster(int clusterId) const { return &_vViaCluster[clusterId]; }
        
    private:
        vector< vector< vector< Tile > > > _vTile;
        vector<Via> _vVia;
        vector<ViaCluster> _vViaCluster;
};

#endif