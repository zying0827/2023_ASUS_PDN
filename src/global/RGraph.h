#ifndef RGRAPH_H
#define RGRAPH_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "../base/Net.h"
// #include "OASG.h"
using namespace std;

//         topological relation        physical relation
// RGraph <--------------------> OASG <-----------------> DB

enum OASGNodeType {
    SOURCE,
    TARGET,
    MIDDLE
};

class OASGNode {
    public:
        OASGNode(double x, double y, OASGNodeType type, Port* port = NULL)
        : _x(x), _y(y), _nodeType(type), _port(port) {}
        ~OASGNode() {}

        double x() const { return _x; }
        double y() const { return _y; }
        double voltage() const { return _voltage; }
        OASGNodeType nodeType() const { return _nodeType; }
        size_t outEdgeId(size_t i) const { return _vOutEdgeId[i]; }
        size_t inEdgeId(size_t i) const { return _vInEdgeId[i]; }
        size_t numOutEdges() const {return _vOutEdgeId.size(); }
        size_t numInEdges() const { return _vInEdgeId.size(); }
        Port* port() const { return _port; }

        void addOutEdge(size_t OASGEdgeId) { _vOutEdgeId.push_back(OASGEdgeId); }
        void addInEdge(size_t OASGEdgeId) { _vInEdgeId.push_back(OASGEdgeId); }
    private:
        double _x;
        double _y;
        double _voltage;
        OASGNodeType _nodeType;
        vector<size_t> _vOutEdgeId;    // the OASGEdges starting from this node
        vector<size_t> _vInEdgeId;    // the OASGEdges going to this node
        Port* _port;                     // the port of the via cluster to which this node belongs, NULL if no via cluster
};

class OASGEdge {
    public:
        OASGEdge(size_t edgeId, size_t netId, size_t layId, size_t typeEdgeId, OASGNode* sNode, OASGNode* tNode, bool viaEdge)
        : _OASGEdgeId(edgeId), _netId(netId), _layId(layId), _typeEdgeId(typeEdgeId), _sNode(sNode), _tNode(tNode), _viaEdge(viaEdge) {
            _length = sqrt( pow(_sNode->x() - _tNode->x(), 2) + pow(_sNode->y() - _tNode->y(), 2) );
        }
        ~OASGEdge() {}

        OASGNode* sNode() { return _sNode; }
        OASGNode* tNode() { return _tNode; }

        double length() const { return _length; }
        bool viaEdge() const { return _viaEdge; }
        size_t layId() const { return _layId; }
        size_t netId() const { return _netId; }
        size_t typeEdgeId() const { return _typeEdgeId; }

        // double cost() {
        //     if (_viaEdge) {

        //     }
        // }

        bool cross(OASGEdge* e) {
            // reference: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
            
            auto onSegment = [] (OASGNode* p, OASGNode* q, OASGNode* r) -> bool {
                if (q->x() <= max(p->x(), r->x()) && q->x() >= min(p->x(), r->x()) &&
                    q->y() <= max(p->y(), r->y()) && q->y() >= min(p->y(), r->y()))
                return true;
            
                return false;
            };
            auto orientation = [] (OASGNode* p, OASGNode* q, OASGNode* r) -> int {
                int val = (q->y() - p->y()) * (r->x() - q->x()) -
                        (q->x() - p->x()) * (r->y() - q->y());
            
                if (val == 0) return 0;  // collinear
            
                return (val > 0)? 1: 2; // clock or counterclock wise
            };

            OASGNode* p1 = _sNode;
            OASGNode* q1 = _tNode;
            OASGNode* p2 = e->sNode();
            OASGNode* q2 = e->tNode();
            int o1 = orientation(p1, q1, p2);
            int o2 = orientation(p1, q1, q2);
            int o3 = orientation(p2, q2, p1);
            int o4 = orientation(p2, q2, q1);
        
            // General case
            if (o1 != o2 && o3 != o4)
                return true;
        
            // Special Cases
            // p1, q1 and p2 are collinear and p2 lies on segment p1q1
            if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        
            // p1, q1 and q2 are collinear and q2 lies on segment p1q1
            if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        
            // p2, q2 and p1 are collinear and p1 lies on segment p2q2
            if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        
            // p2, q2 and q1 are collinear and q1 lies on segment p2q2
            if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        
            return false; // Doesn't fall in any of the above cases
        }

        void setNode(OASGNode* sNode, OASGNode* tNode) {
            _sNode = sNode;
            _tNode = tNode;
            _length = sqrt( pow(_sNode->x() - _tNode->x(), 2) + pow(_sNode->y() - _tNode->y(), 2) );
        }

        void setViaEdge(bool viaEdge) { _viaEdge = viaEdge; }

        void print() {
            cerr << "OASGEdge[" << _OASGEdgeId << "], length=" << _length << ", (" << _sNode->x()/40 << " " << _sNode->y()/40 << ") -> (" << _tNode->x()/40 << " " << _tNode->y()/40 << ")" << endl;
        }

        // void setNode(size_t sNodeId, size_t tNodeId) { _sNodeId = sNodeId; _tNodeId = tNodeId; }
        // void setLength(double length) { _length = length; }
    private:
        double _length;
        OASGNode* _sNode;   // the node with higher voltage
        OASGNode* _tNode;   // the node with lower voltage
        size_t _OASGEdgeId;
        bool _viaEdge;
        size_t _layId;
        size_t _netId;
        size_t _typeEdgeId;

        // size_t _sNodeId;    // the node with higher voltage
        // size_t _tNodeId;    // the node with lower voltage
        // pair<double, double> _sPos;
        // pair<double, double> _tPos; 

        
};

class RGEdge {
    public:
        RGEdge(vector<OASGEdge*> vEdge) : _vEdge(vEdge) { _selected = false; updateLength(); }
        ~RGEdge() {}

        size_t numEdges() const { return _vEdge.size(); }
        OASGEdge* vEdge(size_t edgeId) const { return _vEdge[edgeId]; }
        double length() const { return _length; }
        bool selected() const { return _selected; }
        bool cross(RGEdge* e) {
            for (size_t edgeId1 = 0; edgeId1 < _vEdge.size(); ++ edgeId1) {
                for (size_t edgeId2 = 0; edgeId2 < e->numEdges(); ++ edgeId2) {
                    if (_vEdge[edgeId1]->cross(e->vEdge(edgeId2))) {
                        return true;
                    }
                }
            }
            return false;
        }
        void updateLength() {
            for (size_t edgeId = 0; edgeId < _vEdge.size(); ++ edgeId) {
                _length += _vEdge[edgeId] -> length();
            }
        }
        void addEdge(OASGEdge* e) { _vEdge.push_back(e); } 
        void select() { _selected = true; }
        void print() {
            cerr << "RGEdge { _length=" << _length << ", _vEdge = " << endl;
            for (size_t edgeId = 0; edgeId < _vEdge.size(); ++ edgeId) {
                _vEdge[edgeId]->print();
            }
            cerr << "}" << endl;
        }

    private:
        vector<OASGEdge*> _vEdge;
        double _length;     // exclude the length of vias
        bool _selected;
};

enum RGraphType {
    CROSS,      // merge edges from the OASG
    NO_CROSS,   // after layer distribution, before pseudo routing
    DETOURED    // after pseudo routing, before current distribution
};

class RGraph {
    public:
        // RGraph(DB& db);
        RGraph() {}
        ~RGraph() {}

        // get functions
        size_t num2PinNets() const { return _num2PinNets; }
        size_t numLayers() const { return _numLayers; }
        size_t numRGEdges(size_t twoPinNetId, size_t layId) const { return _vRGEdge[twoPinNetId][layId].size(); }
        RGEdge* vEdge(size_t twoPinNetId, size_t layId, size_t RGEdgeId) { return _vRGEdge[twoPinNetId][layId][RGEdgeId]; }
        size_t twoPinNetId(size_t sPortId, size_t tPortId) { return _portPair2Edge[make_pair(sPortId, tPortId)]; } // tPortId1 < tPortId
        // size_t sNodeId(size_t netId) const { return _vSNodeId[netId]; }
        // size_t tNodeId(size_t netId, size_t netTNodeId) const { return _vTNodeId[netId][netTNodeId]; }
        // size_t numTNodes(size_t netId) const { return _vTNodeId[netId].size(); }
        Port* sPort(size_t sPortId) const { return _vSPort[sPortId]; }
        Port* tPort(size_t netId, size_t netTPortId) const { return _vTPort[netId][netTPortId]; }
        size_t numTPorts(size_t netId) const { return _vTPort[netId].size(); }
        size_t numNets() const { return _numNets; }
        OASGNode* sourceOASGNode(size_t netId, size_t layId) { return _vSourceOASGNode[netId][layId]; }
        OASGNode* targetOASGNode(size_t netId, size_t netTPortId, size_t layId) { return _vTargetOASGNode[netId][netTPortId][layId]; }
        OASGEdge* vOASGEdge(size_t OASGEdgeId) { return _vOASGEdge[OASGEdgeId]; }
        OASGEdge* vPlaneOASGEdge(size_t netId, size_t layId, size_t planeEdgeId) { return _vPlaneOASGEdge[netId][layId][planeEdgeId]; }
        OASGEdge* vViaOASGEdge(size_t netId, size_t layId, size_t viaEdgeId) { return _vViaOASGEdge[netId][layId][viaEdgeId]; }
        size_t numPlaneOASGEdges(size_t netId, size_t layId) const { return _vPlaneOASGEdge[netId][layId].size(); }
        size_t numViaOASGEdges(size_t netId, size_t layId) const { return _vViaOASGEdge[netId][layId].size(); }

        // set functions
        void initRGraph(DB db);
        // after OASG coonstruction, before layer distribution
        void constructRGraph();
        vector< vector<OASGEdge*> > DFS(OASGNode* node, size_t netId);
        OASGNode* addOASGNode(double x, double y, OASGNodeType type, Port* port = NULL);
        void addOASGEdge(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode, bool viaEdge);
    private:
        RGraphType _type;
        // vector<size_t> _vSNodeId;   // index = [netId]
        // vector< vector<size_t> > _vTNodeId;     // index = [netId][netTPortId]
        vector<Port*> _vSPort;  // index = [netId]
        vector< vector< Port* > > _vTPort;     // index = [netId][netTPortId], the ports of the same net are sorted in descending order
        vector< vector< vector<RGEdge*> > > _vRGEdge;   // index = [twoPinNetId] [layId] [RGEdgeId]
        size_t _num2PinNets;
        size_t _numLayers;
        map< pair<size_t, size_t>, size_t > _portPair2Edge;
        size_t _numNets;

        vector<OASGNode*> _vOASGNode;   // all OASGNodes of all nets
        vector< vector<OASGNode*> > _vSourceOASGNode;     // nodes of the source via clusters, index = [netId] [layId]
        vector< vector< vector<OASGNode*> > > _vTargetOASGNode;     // nodes of the target via clusters index = [netId] [netTPortId] [layId]
        // vector< vector<OASGNode*> > _vMiddleOASGNode;   // nodes other than the source or target ones, index = [layId] [middleNodeId]

        vector<OASGEdge*> _vOASGEdge;   // all OASGEdges of all nets
        vector< vector< vector<OASGEdge*> > > _vPlaneOASGEdge;   // horizontal OASGEdges, index = [netId] [layId] [typeEdgeId]
        vector< vector< vector<OASGEdge*> > > _vViaOASGEdge;   // vertical OASGEdges between Layer[layId, layId+1], index = [netId] [layId] [typeEdgeId]
};

#endif