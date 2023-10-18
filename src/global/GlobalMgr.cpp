#include "GlobalMgr.h"
#include "LayerILP.h"
#include "VoltEigen.h"
#include "FlowLP.h"
#include "AddCapacity.h"
#include <utility>

// public functions

void GlobalMgr::plotDB() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t viaId = 0; viaId < _db.numVias(); ++ viaId) {
            Via* via = _db.vVia(viaId);
            via->shape()->plot(via->netId(), layId);
            // _plot.drawCircle(via->shape()->ctrX(), via->shape()->ctrY(), via->shape()->radius(), via->netId());
        }
        for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
            Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
            for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                obs->vShape(shapeId)->plot(SVGPlotColor::gray, layId);
            }
        }
    }
    
}

void GlobalMgr::buildTestOASG() {
    cerr << "buildTestOASG..." << endl;
    // layer0
    cerr << "layer0..." << endl;
    OASGNode* lay0_detNode1 = _rGraph.addOASGNode(1, 400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay0_detNode2 = _rGraph.addOASGNode(1, 400, 520, OASGNodeType::MIDDLE);
    _rGraph.addOASGEdge(0, 0, _rGraph.sourceOASGNode(0,0), _rGraph.targetOASGNode(0,0,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.sourceOASGNode(1,0), _rGraph.targetOASGNode(1,0,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.sourceOASGNode(1,0), _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.targetOASGNode(1,0,0), lay0_detNode1, false);
    _rGraph.addOASGEdge(1, 0, lay0_detNode1, _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.targetOASGNode(1,0,0), lay0_detNode2, false);
    _rGraph.addOASGEdge(1, 0, lay0_detNode2, _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.sourceOASGNode(2,0), _rGraph.targetOASGNode(2,0,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.sourceOASGNode(2,0), _rGraph.targetOASGNode(2,1,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.targetOASGNode(2,0,0), _rGraph.targetOASGNode(2,1,0), false);

    // layer1
    cerr << "layer1..." << endl;
    OASGNode* lay1_obsNode1 = _rGraph.addOASGNode(2, 320, 560, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode2 = _rGraph.addOASGNode(2, 480, 560, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode3 = _rGraph.addOASGNode(2, 480, 600, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode4 = _rGraph.addOASGNode(2, 320, 600, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode1 = _rGraph.addOASGNode(1, 400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode2 = _rGraph.addOASGNode(1, 400, 520, OASGNodeType::MIDDLE);

    _rGraph.addOASGEdge(0, 1, _rGraph.sourceOASGNode(0,1), _rGraph.targetOASGNode(0,0,1), false);

    _rGraph.addOASGEdge(1, 1, _rGraph.sourceOASGNode(1,1), _rGraph.targetOASGNode(1,0,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.sourceOASGNode(1,1), _rGraph.targetOASGNode(1,1,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.targetOASGNode(1,0,1), lay1_detNode1, false);
    _rGraph.addOASGEdge(1, 1, lay1_detNode1, _rGraph.targetOASGNode(1,1,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.targetOASGNode(1,0,1), lay1_detNode2, false);
    _rGraph.addOASGEdge(1, 1, lay1_detNode2, _rGraph.targetOASGNode(1,1,1), false);

    _rGraph.addOASGEdge(2, 1, _rGraph.sourceOASGNode(2,1), _rGraph.targetOASGNode(2,0,1), false);
    _rGraph.addOASGEdge(2, 1, _rGraph.sourceOASGNode(2,1), lay1_obsNode1, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode1, lay1_obsNode4, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode4, _rGraph.targetOASGNode(2,1,1), false);
    _rGraph.addOASGEdge(2, 1, _rGraph.targetOASGNode(2,0,1), lay1_obsNode1, false);
    _rGraph.addOASGEdge(2, 1, _rGraph.targetOASGNode(2,0,1), lay1_obsNode2, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode2, lay1_obsNode3, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode3, _rGraph.targetOASGNode(2,1,1), false);

    // layer2
    cerr << "layer2..." << endl;
    OASGNode* lay2_net0_obsNode1 = _rGraph.addOASGNode(0, 80, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode2 = _rGraph.addOASGNode(0, 320, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode3 = _rGraph.addOASGNode(0, 320, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode4 = _rGraph.addOASGNode(0, 80, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode1 = _rGraph.addOASGNode(1, 80, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode2 = _rGraph.addOASGNode(1, 320, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode3 = _rGraph.addOASGNode(1, 320, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode4 = _rGraph.addOASGNode(1, 80, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode1 = _rGraph.addOASGNode(1, 400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode2 = _rGraph.addOASGNode(1, 400, 520, OASGNodeType::MIDDLE);

    _rGraph.addOASGEdge(0, 2, _rGraph.sourceOASGNode(0,2), lay2_net0_obsNode1, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode1, lay2_net0_obsNode4, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode4, _rGraph.targetOASGNode(0,0,2), false);
    _rGraph.addOASGEdge(0, 2, _rGraph.sourceOASGNode(0,2), lay2_net0_obsNode2, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode2, lay2_net0_obsNode3, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode3, _rGraph.targetOASGNode(0,0,2), false);
    
    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), lay2_net1_obsNode1, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode1, lay2_net1_obsNode4, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode4, _rGraph.targetOASGNode(1,0,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), lay2_net1_obsNode2, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode2, lay2_net1_obsNode3, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode3, _rGraph.targetOASGNode(1,0,2), false);

    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), _rGraph.targetOASGNode(1,1,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.targetOASGNode(1,0,2), lay2_detNode1, false);
    _rGraph.addOASGEdge(1, 2, lay2_detNode1, _rGraph.targetOASGNode(1,1,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.targetOASGNode(1,0,2), lay2_detNode2, false);
    _rGraph.addOASGEdge(1, 2, lay2_detNode2, _rGraph.targetOASGNode(1,1,2), false);

    _rGraph.addOASGEdge(2, 2, _rGraph.sourceOASGNode(2,2), _rGraph.targetOASGNode(2,0,2), false);
    _rGraph.addOASGEdge(2, 2, _rGraph.sourceOASGNode(2,2), _rGraph.targetOASGNode(2,1,2), false);
    _rGraph.addOASGEdge(2, 2, _rGraph.targetOASGNode(2,0,2), _rGraph.targetOASGNode(2,1,2), false);

    // layer3
    cerr << "layer3..." << endl;
    OASGNode* lay3_detNode1 = _rGraph.addOASGNode(1, 400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay3_detNode2 = _rGraph.addOASGNode(1, 400, 520, OASGNodeType::MIDDLE);
    _rGraph.addOASGEdge(0, 3, _rGraph.sourceOASGNode(0,3), _rGraph.targetOASGNode(0,0,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.sourceOASGNode(1,3), _rGraph.targetOASGNode(1,0,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.sourceOASGNode(1,3), _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.targetOASGNode(1,0,3), lay3_detNode1, false);
    _rGraph.addOASGEdge(1, 3, lay3_detNode1, _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.targetOASGNode(1,0,3), lay3_detNode2, false);
    _rGraph.addOASGEdge(1, 3, lay3_detNode2, _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.sourceOASGNode(2,3), _rGraph.targetOASGNode(2,0,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.sourceOASGNode(2,3), _rGraph.targetOASGNode(2,1,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.targetOASGNode(2,0,3), _rGraph.targetOASGNode(2,1,3), false);
}

void GlobalMgr::buildOASG() {
    // TODO for Luo:
    // for each layer, for each net, use addOASGNode() and addOASGEdge() to construct a crossing OASG
    // in the later stage, all possible paths from the source to target ports and from target ports to lower-voltage target ports will be searched by DFS
    // so the OASGEdges should point from the source to the target ports or from higher-voltage to lower-voltage target ports all along
}

void GlobalMgr::plotOASG() {
    //_plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId);
            }
        }
    }
}

void GlobalMgr::layerDistribution() {
    // construct the routing graph
    _rGraph.constructRGraph();
    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     _rGraph.vOASGNode(nodeId) -> print();
    // }
    // cerr << "constructRGraph DONE" << endl;
    // for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
    //     for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //         for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
    //             cerr << "_vRGEdge[" << twoPinNetId << "][" << layId << "][" << RGEdgeId << "] = ";
    //             _rGraph.vEdge(twoPinNetId,layId,RGEdgeId)->print();
    //         }
    //     }
    // }

    
    // calculate normalized current demand
    vector< vector<double> > vNetWeight;
    double totalCurrent = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            totalCurrent += tPort->current();
        }
    }
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        vector<double> temp;
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            temp.push_back(tPort->current() / totalCurrent);
        }
        vNetWeight.push_back(temp);
    }

    // calculate the accumulated via length of each layer
    vector<double> vAccuViaLength;
    double accuViaLength = 0;
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
        vAccuViaLength.push_back(accuViaLength);
        accuViaLength += _db.vMetalLayer(layId)->thickness() + _db.vMediumLayer(layId)->thickness();
    }

    // distribute layers to RGEdges with an ILP solver
    try {
        LayerILP solver(_rGraph, vNetWeight, vAccuViaLength);
        solver.formulate();
        solver.solve();
        solver.collectResult();
    } catch (GRBException e) {
        cerr << "Error = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
}

void GlobalMgr::plotRGraph() {
    // _plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                if (rgEdge->selected()) {
                    for (size_t OASGEdgeId = 0; OASGEdgeId < rgEdge->numEdges(); ++ OASGEdgeId) {
                        OASGEdge* e = rgEdge->vEdge(OASGEdgeId);
                        _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), e->netId(), layId);
                    }
                }
            }
        }
    }
}

void GlobalMgr::buildTestNCOASG() {
    RGraph* NCOASG = new RGraph();
    NCOASG->initRGraph(_db);
    vector<int> vNewNodeId(_rGraph.numOASGNodes(), -1);
    vector<int> vNewEdgeId(_rGraph.numOASGEdges(), -1);
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            OASGNode* sNode = _rGraph.sourceOASGNode(netId, layId);
            vNewNodeId[sNode->nodeId()] = sNode->nodeId();
            for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
                OASGNode* tNode = _rGraph.targetOASGNode(netId, netTPortId, layId);
                vNewNodeId[tNode->nodeId()] = tNode->nodeId();
            }
        }
    }
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                // NCOASG->addRGEdge(rgEdge, twoPinNetId, layId, RGEdgeId);
                if (rgEdge->selected()) {
                    for (size_t OASGEdgeId = 0; OASGEdgeId < rgEdge->numEdges(); ++ OASGEdgeId) {
                        OASGEdge* e = rgEdge->vEdge(OASGEdgeId);
                        if (vNewEdgeId[e->edgeId()] == -1) {
                            // find or create a new sNode for the new edge
                            OASGNode* sNode = e->sNode();
                            OASGNode* newSNode;
                            if (vNewNodeId[sNode->nodeId()] == -1) {
                                newSNode = NCOASG->addOASGNode(sNode->netId(), sNode->x(), sNode->y(), sNode->nodeType());
                                vNewNodeId[sNode->nodeId()] = newSNode->nodeId();
                            } else {
                                newSNode = NCOASG->vOASGNode( vNewNodeId[sNode->nodeId()] );
                            }
                            // find or create a new tNode for the new edge
                            OASGNode* tNode = e->tNode();
                            OASGNode* newTNode;
                            if (vNewNodeId[tNode->nodeId()] == -1) {
                                newTNode = NCOASG->addOASGNode(tNode->netId(), tNode->x(), tNode->y(), tNode->nodeType());
                                vNewNodeId[tNode->nodeId()] = newTNode->nodeId();
                            } else {
                                newTNode = NCOASG->vOASGNode( vNewNodeId[tNode->nodeId()] );
                            }
                            size_t newEdgeId = NCOASG->addOASGEdge(e->netId(), e->layId(), newSNode, newTNode, e->viaEdge());
                            vNewEdgeId[e->edgeId()] = newEdgeId;
                        }
                    }
                }
            }
        }
    }

    // set redundant nodes and edges
    for (size_t netId = 0; netId < NCOASG->numNets(); ++ netId) {
        for (int layId = NCOASG->numLayers()-1; layId >= 0; -- layId) {
            // set redundant source nodes and edges
            OASGNode* sNode = NCOASG->sourceOASGNode(netId, layId);
            if (sNode->numOutEdges() == 0) {
                sNode->setRedundant();
                NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
            } else if (sNode->numOutEdges() == 1) {
                if (NCOASG->vOASGEdge(sNode->outEdgeId(0))->redundant()) {
                    sNode->setRedundant();
                    NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
                }
            }
            // set redundant target nodes and edges
            for (size_t tPortId = 0; tPortId < NCOASG->numTPorts(netId); ++ tPortId) {
                OASGNode* tNode = NCOASG->targetOASGNode(netId, tPortId, layId);
                if (tNode->numInEdges() == 0) {
                    tNode->setRedundant();
                    NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                } else if (tNode->numInEdges() == 1) {
                    if (NCOASG->vOASGEdge(tNode->inEdgeId(0))->redundant()) {
                        tNode->setRedundant();
                        NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                    }
                }
            }
        }
    }
    _rGraph = *NCOASG;
}

void GlobalMgr::plotNCOASG() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId);
            }
        }
    }
}

void GlobalMgr::voltageAssignment() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        VoltEigen solver(_rGraph.numNPortOASGNodes(netId));
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            OASGNode* nPortNode = _rGraph.vNPortOASGNode(netId, nPortNodeId);
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < nPortNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId))->tNode();
                double resistance;
                double l;
                if (outEdge->viaEdge()) {
                    // cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / 32;
                } else {
                    // cerr << "planeEdge ";
                    l = outEdge->length();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                }
                // cerr << "resistance = " << resistance << endl;

                if (outNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                } else {
                    solver.setInputVector(nPortNode->nPortNodeId(), outNode->port()->voltage(), resistance);
                    // cerr << "voltage = " << outNode->port()->voltage() << endl;
                }
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < nPortNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId))->sNode();
                double resistance;
                double l;
                if (inEdge->viaEdge()) {
                    // cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / 32;
                } else {
                    // cerr << "planeEdge ";
                    l = inEdge->length();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                // cerr << "resistance = " << resistance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), inNode->nPortNodeId(), resistance);
                } else {
                    solver.setInputVector(nPortNode->nPortNodeId(), inNode->port()->voltage(), resistance);
                    // cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }
        // cerr << "G = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     for (size_t colId = 0; colId < solver.numNodes(); ++ colId) {
        //         cerr << setprecision(15) << solver.G(rowId, colId);
        //         if (colId < solver.numNodes()-1) {
        //             cerr << ", ";
        //         }
        //     }
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;
        // cerr << "I = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     cerr << setprecision(15) << solver.I(rowId);
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;

        // cerr << "V = ";
        solver.solve();
        // cerr << endl;

        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(solver.V(nPortNodeId));
        }
        _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
        }
    }

    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     _rGraph.vOASGNode(nodeId) -> print();
    // }
    // vector<double> netVoltage;
    // vector< vector<double> > vVoltage(3, netVoltage);
    // vVoltage[0] = { 4.99999975251689,
    //                 4.99999948245706,
    //                 4.99999939003966,
    //                 4.50000024748309,
    //                 4.50000051754296,
    //                 4.50000060996036,
    //                 4.78574240598038,
    //                 4.86315393669003 };
    // vVoltage[1] = { 4.99999967817019,
    //                 4.99999932627796,
    //                 4.99999920660867,
    //                 4.50000028543947,
    //                 4.50000063733168,
    //                 4.50000075700098,
    //                 4.40000003639034,
    //                 4.40000003639039,
    //                 4.40000003639039,
    //                 4.70485438179277,
    //                 4.80649030108812,
    //                 4.45000000000000,
    //                 4.44999999999998,
    //                 4.45000016091490 };
    // vVoltage[2] = { 4.99999948018458,
    //                 4.99999882037443,
    //                 4.99999859900624,
    //                 4.50000026702640,
    //                 4.50000052112573,
    //                 4.50000060637678,
    //                 4.40000025278904,
    //                 4.40000065849982,
    //                 4.40000079461696 };

    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
    //         _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(vVoltage[netId][nPortNodeId]);
    //     }
    //     _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
    //     for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
    //         _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
    //     }
    // }
    // OASGNode* t1 = _rGraph.vNPortOASGNode(1, 7);
    // cerr << "t1(" << t1->x() << "," << t1->y() << ")" << endl;
    // OASGNode* t2 = _rGraph.vNPortOASGNode(1, 8);
    // cerr << "t2(" << t2->x() << "," << t2->y() << ")" << endl;

    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     cerr << "Node[" << nodeId << "].voltage = " << _rGraph.vOASGNode(nodeId)->voltage() << endl;
    // }

    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     Port* sPort = _rGraph.sPort(netId);
    //     for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
    //         GRBLinExpr netCurrent;
    //         Port* tPort = _rGraph.tPort(netId, netTPortId);
    //         size_t twoPinNetId = _rGraph.twoPinNetId(sPort->portId(), tPort->portId());
    //         for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //             for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
    //                 RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
    //             }
    //         }
    //     }
    // }
    
}

void GlobalMgr::currentDistribution() {
    vector<double> vMediumLayerThickness;
    vector<double> vMetalLayerThickness;
    vector<double> vConductivity;
    for (size_t mediumLayId = 0; mediumLayId < _db.numMediumLayers(); ++ mediumLayId) {
        vMediumLayerThickness.push_back(_db.vMediumLayer(mediumLayId)->thickness());
    }
    for (size_t metalLayId = 0; metalLayId < _db.numLayers(); ++ metalLayId) {
        vMetalLayerThickness.push_back(_db.vMetalLayer(metalLayId)->thickness());
        vConductivity.push_back(_db.vMetalLayer(metalLayId)->conductivity());
    }
    double normRatio = _rGraph.sPort(0)->current();
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        if (normRatio > _rGraph.sPort(netId)->current()) {
            normRatio = _rGraph.sPort(netId)->current();
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            if (normRatio > _rGraph.tPort(netId, tPortId)->current()) {
                normRatio = _rGraph.tPort(netId, tPortId)->current();
            }
        }
    }

    FlowLP solver(_rGraph, vMediumLayerThickness, vMetalLayerThickness, vConductivity, normRatio);
    
    // set objective
    // cerr << "setObjective..." << endl;
    solver.setObjective(_db.areaWeight(), _db.viaWeight());

    // set flow conservation constraints
    // cerr << "setConserveConstraints..." << endl;
    solver.setConserveConstraints();
    // set capacity constraints
    // TODO for Tsai and Huang:
    // for each layer, for each neighboring OASGEdges,
    
    
    //search each layer                                                                           
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId){
        cout << "LAYER :" << layId << endl << endl;
        //search each net
        for(size_t S_netId = 0; S_netId < _rGraph.numNets(); ++ S_netId){
            //search each edge
            for (size_t S_EdgeId = 0; S_EdgeId < _rGraph.numPlaneOASGEdges(S_netId, layId); ++ S_EdgeId){
                OASGEdge* e1 = _rGraph.vPlaneOASGEdge(S_netId, layId, S_EdgeId);
                
                pair<double, double> ratio;
                pair<bool, bool> right;
                double width;
                
                //compare other net edge
                for(size_t T_netId = S_netId+1; T_netId < _rGraph.numNets(); ++ T_netId){
                    //make sure to compare different net
                    for (size_t T_EdgeId = 0; T_EdgeId < _rGraph.numPlaneOASGEdges(T_netId, layId); ++ T_EdgeId){
                        OASGEdge* e2 = _rGraph.vPlaneOASGEdge(T_netId, layId, T_EdgeId);
    
                    //    BuildCapacityConstraint(e1,e2,solver);
                        if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                         make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                         make_pair(e2->sNode()->x(), e2->sNode()->y()),
                                         make_pair(e2->tNode()->x(), e2->tNode()->y()),
                                         ratio, right, width))
                            solver.addCapacityConstraints(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                        
                    }   
                } 

                // obstacle constraint
                for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
                   
                    Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
                    pair<double, double> S2, T2;
                    for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                        for(size_t vtxId = 0; vtxId < obs->vShape(shapeId)->numBPolyVtcs(); ++ vtxId){
                            // get the edge coordinates
                            S2 = make_pair(obs->vShape(shapeId)->bPolygonX(vtxId), obs->vShape(shapeId)->bPolygonY(vtxId));
                            T2 = make_pair(obs->vShape(shapeId)->bPolygonX((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()), obs->vShape(shapeId)->bPolygonY((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()));
                            double vectorX = (T2.first - S2.first)/sqrt(pow(T2.first - S2.first,2)+pow(T2.second - S2.second,2));
                            double vectorY = (T2.second - S2.second)/sqrt(pow(T2.first - S2.first,2)+pow(T2.second - S2.second,2));
                            double normalX = vectorY;
                            double normalY = -vectorX;
                            //new S2
                            S2 = make_pair((S2.first - pow(10,-5)*(-vectorX +  normalX)), (S2.second - pow(10,-5)*(-vectorY +  normalY)));
                            //new T2 
                            T2 = make_pair((T2.first - pow(10,-5)*(vectorX +  normalX)), (S2.second - pow(10,-5)*(vectorY +  normalY)));
                        
                            if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                             make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                             S2, T2, ratio, right, width))
                                solver.addCapacityConstraints(e1, right.first, ratio.first, width);

                        }
                    }
                }

                // board cnstraint
                // bottom
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(_db.boardWidth(), 0),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // left
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(0, _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // top
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, _db.boardHeight()),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // right
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(_db.boardWidth(), 0),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
            }
        }  
    }
    
    
    // use solver.addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width)
    // to add their capacity constraints
    
    /*
    // layer0
    
    solver.addCapacityConstraints(_rGraph.vOASGEdge(24), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(24), true, 1, _rGraph.vOASGEdge(30), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(30), true, 1, _rGraph.vOASGEdge(36), false, 1, 150);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(36), true, 1, 80);
    // layer1
    solver.addCapacityConstraints(_rGraph.vOASGEdge(25), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(24), true, 1, _rGraph.vOASGEdge(31), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(31), true, 1, _rGraph.vOASGEdge(43), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(43), true, 1, 80);
    //layer2
    solver.addCapacityConstraints(_rGraph.vOASGEdge(27), false, 1, 80);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(27), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(33), false, 1, 0);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(33), true, 1, _rGraph.vOASGEdge(44), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(44), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(48), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(48), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(46), false, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(46), true, 1, 80);
    // layer3
    solver.addCapacityConstraints(_rGraph.vOASGEdge(29), false, 1, 80);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(29), true, 1, _rGraph.vOASGEdge(35), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(35), true, 1, _rGraph.vOASGEdge(45), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(45), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(49), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(49), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(47), false, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(47), true, 1, 80);
    */
    // solve the MCFP formulation and collect the result
    solver.solve();
    solver.collectResult();
    solver.printResult();

    // add traces to each net
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                if (e->current() > 0) {
                    Segment* segment = edge2Segment(e);
                    _db.vNet(netId)->addSegment(segment, layId);
                }
            }
        }
    }
}

void GlobalMgr::plotCurrentPaths() {
    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //         for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
    //             OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
    //             Trace* trace = edge2Trace(e);
    //             trace->plot(netId, layId);
    //             // _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId);
    //         }
    //     }
    // }
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        Net* net = _db.vNet(netId);
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
                net->vSegment(layId, segId)->plot(netId, layId);
            }
        }
    }
}

Trace* GlobalMgr::edge2Trace(OASGEdge* edge) {
    assert (!edge->viaEdge());
    double offset = 0.5 * ( edge->widthRight() - edge->widthLeft() );
    double xOffset = offset * (edge->tNode()->y() - edge->sNode()->y()) / edge->length(); // offset * sin(theta)
    double yOffset = offset * (edge->sNode()->x() - edge->tNode()->x()) / edge->length(); // offset * -cos(theta)
    Node* sNode = new Node(edge->sNode()->x()+xOffset, edge->sNode()->y()+yOffset, _plot);
    Node* tNode = new Node(edge->tNode()->x()+xOffset, edge->tNode()->y()+yOffset, _plot);
    Trace* trace = new Trace(sNode, tNode, edge->widthRight() + edge->widthLeft(), _plot);
    return trace;
}

Segment* GlobalMgr::edge2Segment(OASGEdge* edge) {
    assert (!edge->viaEdge());
    double offset = 0.5 * ( edge->widthRight() - edge->widthLeft() );
    double xOffset = offset * (edge->tNode()->y() - edge->sNode()->y()) / edge->length(); // offset * sin(theta)
    double yOffset = offset * (edge->sNode()->x() - edge->tNode()->x()) / edge->length(); // offset * -cos(theta)
    Node* sNode = new Node(edge->sNode()->x()+xOffset, edge->sNode()->y()+yOffset, _plot);
    Node* tNode = new Node(edge->tNode()->x()+xOffset, edge->tNode()->y()+yOffset, _plot);
    Trace* trace = new Trace(sNode, tNode, edge->widthRight() + edge->widthLeft(), _plot);
    Segment* segment = new Segment(trace, edge->sNode()->voltage(), edge->tNode()->voltage(), edge->current());
    return segment;
}