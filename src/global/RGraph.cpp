#include "RGraph.h"

void RGraph::initRGraph(DB db) {
    // database info
    _type = RGraphType::CROSS;
    _numLayers = db.numLayers();
    _numNets = db.numNets();

    // OASG info (ports and nodes)
    for (size_t netId = 0; netId < db.numNets(); ++ netId) {
        Net* net = db.vNet(netId);
        // construct source ports
        _vSPort.push_back(net->sourcePort());
        // construct source nodes
        vector<OASGNode*> layerSNode;
        for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
            OASGNode* node = new OASGNode(net->sourceViaCstr()->centerX(), net->sourceViaCstr()->centerY(), OASGNodeType::SOURCE, net->sourcePort());
            layerSNode.push_back(node);
            _vOASGNode.push_back(node);
        }
        _vSourceOASGNode.push_back(layerSNode);

        // construct target ports and nodes
        vector< vector<OASGNode*> > netLayerTNode;
        vector<Port*> netPort;
        for (size_t netTPortId = 0; netTPortId < net->numTPorts(); ++ netTPortId) {
            netPort.push_back(net->targetPort(netTPortId));
            vector<OASGNode*> layerTNode;
            for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
                OASGNode* node = new OASGNode(net->vTargetViaCstr(netTPortId)->centerX(), net->vTargetViaCstr(netTPortId)->centerY(), OASGNodeType::TARGET, net->targetPort(netTPortId));
                layerTNode.push_back(node);
                _vOASGNode.push_back(node);
            }
            netLayerTNode.push_back(layerTNode);
        }
        _vTPort.push_back(netPort);
        _vTargetOASGNode.push_back(netLayerTNode);
    }

    // cerr << "tartget OASG nodes:" << endl;
    // cerr << "numNets = " << db.numNets() << endl;
    // for (size_t netId = 0; netId < db.numNets(); ++ netId) {
    //     Net* net = db.vNet(netId);
    //     net->print();
    //     for (size_t netTPortId = 0; netTPortId < net->numTPorts(); ++ netTPortId) {
    //         for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
    //             cerr << "_vTargetOASGNode[" << netId << "][" << netTPortId << "][" << layId << "] = " << _vTargetOASGNode[netId][netTPortId][layId]->x() << endl;
    //         }
    //     }
    // }

    // OASG info (edges)
    for (size_t netId = 0; netId < db.numNets(); ++ netId) {
        vector< vector<OASGEdge*> > netLayerEdge;
        for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
            vector<OASGEdge*> layerEdge;
            netLayerEdge.push_back(layerEdge);
        }
        _vViaOASGEdge.push_back(netLayerEdge);
        _vPlaneOASGEdge.push_back(netLayerEdge);
    }
    for (size_t netId = 0; netId < db.numNets(); ++ netId) {
        Net* net = db.vNet(netId);
        // construct source and target edges
        for (size_t layId = 0; layId < db.numLayers()-1; ++ layId) {
            addOASGEdge(netId, layId, _vSourceOASGNode[netId][layId], _vSourceOASGNode[netId][layId+1], true);
            for (size_t netTPortId = 0; netTPortId < net->numTPorts(); ++ netTPortId) {
                addOASGEdge(netId, layId, _vTargetOASGNode[netId][netTPortId][layId+1], _vTargetOASGNode[netId][netTPortId][layId], true);
            }
        }
    }
}

OASGNode* RGraph::addOASGNode(double x, double y, OASGNodeType type, Port* port){
    // _vOutEdgeId and _vInEdgeId will be set later through addOASGEdge()
    // _voltage will be set in the preporcessing of currentDistribution()
    OASGNode* node = new OASGNode(x, y, type, port);
    _vOASGNode.push_back(node);
    return node;
}

void RGraph::addOASGEdge(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode, bool viaEdge){
    size_t OASGEdgeId = _vOASGEdge.size();
    size_t typeEdgeId = viaEdge ? _vViaOASGEdge[netId][layId].size() : _vPlaneOASGEdge[netId][layId].size();
    OASGEdge* edge = new OASGEdge(OASGEdgeId, netId, layId, typeEdgeId, sNode, tNode, viaEdge);
    sNode->addOutEdge(OASGEdgeId);
    tNode->addInEdge(OASGEdgeId);
    _vOASGEdge.push_back(edge);
    if (viaEdge) {
        _vViaOASGEdge[netId][layId].push_back(edge);
    } else {
        _vPlaneOASGEdge[netId][layId].push_back(edge);
    }
}

void RGraph::constructRGraph() {
    // build _vRGEdge and _portPair2Edge
    size_t twoPinNetId = 0;
    for (size_t netId = 0; netId < numNets(); ++ netId) {
        // for each layer, apply DFS
        for (size_t layId = 0; layId < numLayers(); ++ layId) {
            // build RGEdges and the map between sPort and tPort
            OASGNode* sNode = sourceOASGNode(netId, layId);
            size_t sPortId = sNode->port()->portId();
            vector< vector<OASGEdge*> > paths;
            paths = DFS(sNode, netId);
            for (size_t pathId = 0; pathId < paths.size(); ++ pathId) {
                size_t tPortId = paths[pathId][0]->tNode()->port()->portId();
                pair<size_t, size_t> portPairId = make_pair(sPortId, tPortId);
                RGEdge* e = new RGEdge(paths[pathId]);
                e->updateLength();
                if (_portPair2Edge.count(portPairId) == 0) {
                    _portPair2Edge[portPairId] = twoPinNetId;
                    vector<RGEdge*> temp;
                    vector< vector< RGEdge* > > temp2(numLayers(), temp);
                    temp2[layId].push_back(e);
                    _vRGEdge.push_back(temp2);
                    twoPinNetId ++;
                } else {
                    _vRGEdge[_portPair2Edge[portPairId]][layId].push_back(e);
                }
            }
            // build RGEdges and the map between tPorts
            for (size_t netTPortId = 0; netTPortId < _vTargetOASGNode[netId].size(); ++ netTPortId) {
                OASGNode* tNode = targetOASGNode(netId, netTPortId, layId);
                size_t tPortId = tNode->port()->portId();
                paths = DFS(tNode, netId);
                for (size_t pathId = 0; pathId < paths.size(); ++ pathId) {
                    size_t tPort1Id = paths[pathId][0]->tNode()->port()->portId();
                    pair<size_t, size_t> portPairId = make_pair(tPortId, tPort1Id);
                    RGEdge* e = new RGEdge(paths[pathId]);
                    e->updateLength();
                    if (_portPair2Edge.count(portPairId) == 0) {
                        _portPair2Edge[portPairId] = twoPinNetId;
                        vector<RGEdge*> temp;
                        vector< vector< RGEdge* > > temp2(numLayers(), temp);
                        temp2[layId].push_back(e);
                        _vRGEdge.push_back(temp2);
                        twoPinNetId ++;
                    } else {
                        _vRGEdge[_portPair2Edge[portPairId]][layId].push_back(e);
                    }
                }
            }
        }
    }
    _num2PinNets = twoPinNetId;
}

vector< vector<OASGEdge*> > RGraph::DFS(OASGNode* node, size_t netId) {
    vector< vector<OASGEdge*> > paths;
    for (size_t edgeId = 0; edgeId < node->numOutEdges(); ++ edgeId) {
        OASGEdge* succEdge = vOASGEdge(node->outEdgeId(edgeId));
        if (succEdge->viaEdge() == false && succEdge->netId() == netId) {
            if (succEdge->tNode()->nodeType() == OASGNodeType::TARGET) {
                vector<OASGEdge*> temp;
                temp.push_back(succEdge);
                paths.push_back(temp);
            }
            else {
                vector< vector<OASGEdge*> > tempPaths;
                tempPaths = DFS(succEdge->tNode(), netId);
                for (size_t tempPathId = 0; tempPathId < tempPaths.size(); ++ tempPathId) {
                    tempPaths[tempPathId].push_back(succEdge);
                    paths.push_back(tempPaths[tempPathId]);
                }
            }
        }
    }
    return paths;
}

