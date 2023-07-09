#ifndef RGRAPH_H
#define RGRAPH_H

#include "Include.h"
#include "TNode.h"
#include "TEdge.h"

using namespace std;

class RGraph {
    friend class NetworkILP;
    public:
        RGraph(size_t numNets, size_t numLayers, size_t numNeighborsUB): _numNets(numNets), _numLayers(numLayers), _numNeighborsUB(numNeighborsUB) {}
        ~RGraph() {}
        
        size_t numNets() const { return _numNets; }
        size_t numLayers() const { return _numLayers; }
        size_t numLayerPairs() const { return _numLayers-1; }
        size_t numNeighborsUB() const { return _numNeighborsUB; }

        size_t numNodes() const { return _vNode.size(); }
        size_t numT2DNodes(int netId) const { return _vTViaNode[netId][0].size(); }    // number of 2D target nodes of Net[netId]
        size_t numF2DNodes() const { return _vF2DNode[0].size(); }
        size_t numOtherNodes(int layId) const { return _vOtherNode[layId].size(); }

        TNode* vSViaNode(int netId, int layId) { return _vSViaNode[netId][layId]; }
        TNode* vTViaNode(int netId, int layId, int T2DNodeId) { return _vTViaNode[netId][layId][T2DNodeId]; }
        TNode* vF2DNode(int layId, int F2DNodeId) { return _vF2DNode[layId][F2DNodeId]; }
        TNode* vOtherNode(int layId, int otherNodeId) { return _vOtherNode[layId][otherNodeId]; }

        size_t numEdges() const { return _vEdge.size(); }
        size_t numHEdges(int layId) const { return _vHEdge[layId].size(); }
        size_t numSREdges(int netId, int layId) const { return _vSREdge[netId][layId].size(); }
        size_t numTREdges(int netId, int layId) const { return _vTREdge[netId][layId].size(); }

        TEdge* vHEdge(int layId, int HEdgeId) { return _vHEdge[layId][HEdgeId]; }
        TEdge* vF2DEdge(int layPairId, int F2DNodeId) { return _vF2DEdge[layPairId][F2DNodeId]; }
        TEdge* vSViaEdge(int netId, int layPairId) { return _vSViaEdge[netId][layPairId]; }
        TEdge* vTViaEdge(int netId, int layPairId, int T2DNodeId) { return _vTViaEdge[netId][layPairId][T2DNodeId]; }
        TEdge* vSREdge(int netId, int layId, int SREdgeId) { return _vSREdge[netId][layId][SREdgeId]; }
        TEdge* vTREdge(int netId, int layId, int TREdgeId) { return _vTREdge[netId][layId][TREdgeId]; }

    private:
        vector<TNode> _vNode;
        vector< vector<TNode*> > _vSViaNode;  // nodes occupied by source via clusters, index = [netId] [layId]
        vector< vector< vector<TNode*> > > _vTViaNode;   // nodes occupied by target via clusters, index = [netId] [layId] [T2DNodeId]
        vector< vector<TNode*> > _vF2DNode;   // 2d-feasible nodes (without obstacles on all layers), index = [layId] [F2NodeId]
        vector< vector<TNode*> > _vOtherNode;   // nodes other than source/target via and 2d-feasible ones, index = [layId] [otherNodeId]

        vector<TEdge> _vEdge;
        vector< vector<TEdge*> > _vHEdge;     // edges with TEdgeType == Horizontal, index = [layId] [HEdgeId], same nodeType: from lowId to highId; different nodeType: from F2DNode to OtherNode
        vector< vector<TEdge*> > _vF2DEdge;     // edges with TEdgeType == Feasible2D, index = [layPairId] [F2DNodeId], from layId to layId+1
        vector< vector<TEdge*> > _vSViaEdge;     // edges with TEdgeType == SourceVia, index = [netId] [layPairId], from layId+1 to layId
        vector< vector< vector<TEdge*> > > _vTViaEdge;     // edges with TEdgeType == TargetVia, index = [netId] [layPairId] [T2DNodeId], from layId to layId+1
        vector< vector< vector<TEdge*> > > _vSREdge;     // edges with TEdgeType == SRoundVia, index = [netId] [layId] [SREdgeId], from sViaNode
        vector< vector< vector<TEdge*> > > _vTREdge;     // edges with TEdgeType == TRoundVia, index = [netId] [layId] [TREdgeId], to tViaNode

        size_t _numNets;
        size_t _numLayers;
        size_t _numNeighborsUB;

};

#endif

