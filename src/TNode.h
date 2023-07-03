#ifndef TNODE_H
#define TNODE_H

#include "Include.h"
#include "TEdge.h"
#include "Tile.h"

using namespace std;

class TNode {
    public:
        TNode(Tile* tile): _tile(tile) {}
        ~TNode() {}

        Tile* tile() const { return _tile; }
        TEdge* vTEdge(int edgeId) const { return _vTEdge[edgeId]; }

        void addTEdge(TEdge* e) { _vTEdge.push_back(e); }

    private:
        Tile* _tile;
        vector<TEdge*> _vTEdge;  // the neighboring edges
};

#endif