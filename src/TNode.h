#ifndef TNODE_H
#define TNODE_H

#include "Include.h"
#include "TEdge.h"
#include "Tile.h"

using namespace std;

enum TNodeType {
    SVia,
    TVia,
    F2D,
    Other
};

class TNode {
    public:
        TNode(Tile* tile): _tile(tile) {}
        ~TNode() {}

        Tile* tile() const { return _tile; }
        // TEdge* vTEdge(int edgeId) const { return _vTEdge[edgeId]; }
        // size_t numTEdges() const { return _vTEdge.size(); }

        TEdge* vOutEdge(int oEdgeId) const { return _vOutEdge[oEdgeId]; }
        TEdge* vInEdge(int iEdgeId) const { return _vInEdge[iEdgeId]; }
        size_t numOutEdges() const { return _vOutEdge.size(); }
        size_t numInEdges() const { return _vInEdge.size(); }

        // void addTEdge(TEdge* e) { _vTEdge.push_back(e); }
        void addOutEdge(TEdge* e) { _vOutEdge.push_back(e); }
        void addInEdge(TEdge* e) { _vInEdge.push_back(e); }

    private:
        Tile* _tile;
        // vector<TEdge*> _vTEdge;  // the neighboring edges
        vector<TEdge*> _vOutEdge; // edges that start from this node
        vector<TEdge*> _vInEdge; // edges that go to this node
        TNodeType _nodeType;
        unsigned int _nodeTypeId;
};

#endif