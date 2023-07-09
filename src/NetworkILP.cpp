#include "NetworkILP.h"

NetworkILP::NetworkILP(RGraph& rGraph, vector< double > netCrit, double areaCost, double widthCost, double viaCost)
            : _model(_env), _rGraph(rGraph), _netCrit(netCrit), _areaCost(areaCost), _widthCost(widthCost), _viaCost(viaCost) {
    clear();
    // vector< vector<GRBVar> > _vF2DVia;  // whether to add via of Net on 2D Feasible Node, index = [netId][F2DNodeId]

    // vector< vector< vector<GRBVar> > > _vHFlow;     // horizontal flow of non-fixed-via nodes, index = [netId] [layId] [HEdgeId]
    // vector< vector< vector<GRBVar> > > _vF2DFlow;     // vertical flow of 2D feasible nodes from layId to layId+1, index = [netId] [layId] [F2DNodeId]
    // vector< vector<GRBVar> > _vSViaFlow;     // vertical flow of source-via nodes from layId+1 to layId, index = [netId] [layId]
    // vector< vector< vector<GRBVar> > > _vTViaFlow;     // vertical flow of target-via nodes from layId to layId+1, index = [netId] [layId] [T2DNodeId]
    // vector< vector< vector<GRBVar> > > _vRFlow;     // horizontal flow of fixed-via nodes, index = [netId] [layId] [REdgeId]

    // construct _vF2DVia, index = [netId][F2DNodeId]
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        vector<GRBVar> temp;
        for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
            GRBVar var = _model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "V_n" + to_string(netId) + "_i" + to_string(F2DNodeId));
            temp.push_back(var);
        }
        _vF2DVia.push_back(temp);
    }

    // construct _vHFlow, index = [netId] [layId] [HEdgeId]
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        vector< vector< GRBVar > > tempNet;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
            vector< GRBVar > tempLay;
            for (size_t HEdgeId = 0; HEdgeId < _rGraph.numHEdges(layId); ++HEdgeId) {
                GRBVar var = _model.addVar(-1.0, 1.0, 0.0, GRB_INTEGER, "H_n" + to_string(netId) + "_l" + to_string(layId) + "_i" + to_string(HEdgeId));
                tempLay.push_back(var);
            }
            tempNet.push_back(tempLay);
        }
        _vHFlow.push_back(tempNet);
    }

    // construct _vF2DFlow, index = [netId] [layId] [F2DNodeId]
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        vector< vector< GRBVar > > tempNet;
        for (size_t layId = 0; layId < _rGraph.numLayers()-1; ++layId) {
            vector< GRBVar > tempLay;
            for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
                GRBVar var = _model.addVar(-1.0, 1.0, 0.0, GRB_INTEGER, "F_n" + to_string(netId) + "_l" + to_string(layId) + "_i" + to_string(F2DNodeId));
                tempLay.push_back(var);
            }
            tempNet.push_back(tempLay);
        }
        _vF2DFlow.push_back(tempNet);
    }

    // construct _vSViaFlow, index = [netId] [layId]
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        vector< GRBVar > temp;
        for (size_t layId = 0; layId < _rGraph.numLayers()-1; ++layId) {
            GRBVar var = _model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "S_n" + to_string(netId) + "_l" + to_string(layId));
            temp.push_back(var);
        }
        _vSViaFlow.push_back(temp);
    }

    // construct _vTViaFlow, index = [netId] [layId] [T2DNodeId]
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        vector< vector< GRBVar > > tempNet;
        for (size_t layId = 0; layId < _rGraph.numLayers()-1; ++layId) {
            vector< GRBVar > tempLay;
            for (size_t T2DNodeId = 0; T2DNodeId < _rGraph.numT2DNodes(netId); ++T2DNodeId) {
                GRBVar var = _model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "T_n" + to_string(netId) + "_l" + to_string(layId) + "_i" + to_string(T2DNodeId));
                tempLay.push_back(var);
            }
            tempNet.push_back(tempLay);
        }
        _vTViaFlow.push_back(tempNet);
    }

    // construct _vSRFlow, index = [netId] [layId] [SREdgeId]
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        vector< vector< GRBVar > > tempNet;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
            vector< GRBVar > tempLay;
            for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, layId); ++SREdgeId) {
                GRBVar var = _model.addVar(-1.0, 1.0, 0.0, GRB_INTEGER, "SR_n" + to_string(netId) + "_l" + to_string(layId) + "_i" + to_string(SREdgeId));
                tempLay.push_back(var);
            }
            tempNet.push_back(tempLay);
        }
        _vSRFlow.push_back(tempNet);
    }

    // construct _vTRFlow, index = [netId] [layId] [TREdgeId]
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        vector< vector< GRBVar > > tempNet;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
            vector< GRBVar > tempLay;
            for (size_t TREdgeId = 0; TREdgeId < _rGraph.numTREdges(netId, layId); ++TREdgeId) {
                GRBVar var = _model.addVar(-1.0, 1.0, 0.0, GRB_INTEGER, "TR_n" + to_string(netId) + "_l" + to_string(layId) + "_i" + to_string(TREdgeId));
                tempLay.push_back(var);
            }
            tempNet.push_back(tempLay);
        }
        _vTRFlow.push_back(tempNet);
    }
}

void NetworkILP::setObjective() {
    // set resistance upperbound constraints
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        GRBQuadExpr netArea;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
            for (size_t HEdgeId = 0; HEdgeId < _rGraph.numHEdges(layId); ++HEdgeId) {
                TEdge* e = _rGraph.vHEdge(layId, HEdgeId);
                netArea += e->cost() * _vHFlow[netId][layId][HEdgeId] * _vHFlow[netId][layId][HEdgeId];
            }
        }
        GRBQuadExpr netWidth;
        netWidth += _vSViaFlow[netId][_rGraph.numLayerPairs()-1];   // vertical flow of the source on the last layer pair
        for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, _rGraph.numLayers()-1); ++SREdgeId) {
            netWidth += _vSRFlow[netId][_rGraph.numLayers()-1][SREdgeId];
        }
        GRBQuadExpr viaUsage;
        for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
            viaUsage += _vF2DVia[netId][F2DNodeId];
        }
        
        GRBQuadExpr netResistance = _areaCost * netArea - _widthCost * netWidth + _viaCost * viaUsage;

        _model.addConstr(_netCrit[netId] * netResistance <= _ubResistance);
    }

    // minimize the resistance upperbound
    GRBLinExpr obj;
    obj += _ubResistance;
    _model.setObjective(obj, GRB_MINIMIZE);
}

void NetworkILP::setFlowConstraints() {
    // flow conservation
    // flow conservation on SViaNode and TViaNode
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        // flow conservation on SViaNode
        GRBLinExpr s0Out;
        for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, 0); ++SREdgeId) {
            s0Out += _vSRFlow[netId][0][SREdgeId];
        }
        s0Out -= _vSViaFlow[netId][0];
        _model.addConstr(s0Out == 0);

        for (size_t layId = 1; layId < _rGraph.numLayers()-1; ++layId) {
            GRBLinExpr sLayIdOut;
            for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, layId); ++SREdgeId) {
                sLayIdOut += _vSRFlow[netId][layId][SREdgeId];
            }
            sLayIdOut += _vSViaFlow[netId][layId-1];
            sLayIdOut -= _vSViaFlow[netId][layId];
            _model.addConstr(sLayIdOut == 0);
        }

        GRBLinExpr sLastOut;
        for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, _rGraph.numLayers()-1); ++SREdgeId) {
            sLastOut += _vSRFlow[netId][_rGraph.numLayers()-1][SREdgeId];
        }
        sLastOut += _vSViaFlow[netId][_rGraph.numLayerPairs()-1];

        // flow conservation on TViaNode
        GRBLinExpr tLastInTotal;
        for (size_t T2DNodeId = 0; T2DNodeId < _rGraph.numT2DNodes(netId); ++T2DNodeId) {
            GRBLinExpr t0In;
            TNode* node0 = _rGraph.vTViaNode(netId, 0, T2DNodeId);
            for (size_t eId = 0; eId < node0->numInEdges(); ++eId) {
                TEdge* edge0 = node0->vInEdge(eId);
                if (edge0->edgeType() == TEdgeType::TRoundVia) {
                    t0In += _vTRFlow[netId][0][edge0->edgeTypeId()];
                } 
            }
            t0In -= _vTViaFlow[netId][0][T2DNodeId];
            _model.addConstr(t0In == 0);

            for (size_t layId = 1; layId < _rGraph.numLayers()-1; ++layId) {
                GRBLinExpr tLayIdIn;
                TNode* node = _rGraph.vTViaNode(netId, layId, T2DNodeId);
                for (size_t eId = 0; eId < node->numInEdges(); ++eId) {
                    TEdge* edge = node->vInEdge(eId);
                    if (edge->edgeType() == TEdgeType::TRoundVia) {
                        tLayIdIn += _vTRFlow[netId][layId][edge->edgeTypeId()];
                    } 
                }
                tLayIdIn += _vTViaFlow[netId][layId-1][T2DNodeId];
                tLayIdIn -= _vTViaFlow[netId][layId][T2DNodeId];
                _model.addConstr(tLayIdIn == 0);
            }

            GRBLinExpr tLastIn;
            TNode* nodeLast = _rGraph.vTViaNode(netId, _rGraph.numLayers()-1, T2DNodeId);
            for (size_t eId = 0; eId < nodeLast->numInEdges(); ++eId) {
                TEdge* edgeLast = nodeLast->vInEdge(eId);
                if (edgeLast->edgeType() == TEdgeType::TRoundVia) {
                    tLastIn += _vTRFlow[netId][_rGraph.numLayers()-1][edgeLast->edgeTypeId()];
                } 
            }
            tLastIn += _vTViaFlow[netId][_rGraph.numLayerPairs()-1][T2DNodeId];

            tLastInTotal += tLastIn;
        }

        // cycle constraint
        _model.addConstr(sLastOut == tLastInTotal);
    }

    // flow conservation on F2DNode
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
        for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
            TNode* node = _rGraph.vF2DNode(layId, F2DNodeId);
            for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
                GRBLinExpr outFlow;
                for (size_t oEdgeId = 0; oEdgeId < node->numOutEdges(); ++oEdgeId) {
                    TEdge* edge = node->vOutEdge(oEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        outFlow += _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else if (edge->edgeType() == TEdgeType::Feasible2D) {
                        outFlow += _vF2DFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::TRoundVia);
                        if (edge->netId() == netId) {
                            outFlow += _vTRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                GRBLinExpr inFlow;
                for (size_t iEdgeId = 0; iEdgeId < node->numInEdges(); ++iEdgeId) {
                    TEdge* edge = node->vInEdge(iEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        inFlow += _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else if (edge->edgeType() == TEdgeType::Feasible2D) {
                        inFlow += _vF2DFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::SRoundVia);
                        if (edge->netId() == netId) {
                            inFlow += _vSRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                _model.addConstr(outFlow == inFlow);
            }
        }
    }

    // flow conservation on otherNode
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
        for (size_t otherNodeId = 0; otherNodeId < _rGraph.numOtherNodes(layId); ++otherNodeId) {
            TNode* node = _rGraph.vOtherNode(layId, otherNodeId);
            for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
                GRBLinExpr outFlow;
                for (size_t oEdgeId = 0; oEdgeId < node->numOutEdges(); ++oEdgeId) {
                    TEdge* edge = node->vOutEdge(oEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        outFlow += _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::TRoundVia);
                        if (edge->netId() == netId) {
                            outFlow += _vTRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                GRBLinExpr inFlow;
                for (size_t iEdgeId = 0; iEdgeId < node->numInEdges(); ++iEdgeId) {
                    TEdge* edge = node->vInEdge(iEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        inFlow += _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::SRoundVia);
                        if (edge->netId() == netId) {
                            inFlow += _vSRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                _model.addConstr(outFlow == inFlow);
            }
        }
    }
}

void NetworkILP::setViaConstraints() {
    // F2D via constraints
    for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
        for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
            GRBLinExpr F2DViaFlow;
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++layPairId) {
                F2DViaFlow += _vF2DFlow[netId][layPairId][F2DNodeId];
            }
            _model.addConstr(F2DViaFlow >= _vF2DVia[netId][F2DNodeId]);
            _model.addConstr(F2DViaFlow <= _rGraph.numLayerPairs() * _vF2DVia[netId][F2DNodeId]);
        }
    }

    // F2D node approval constraints
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
        for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
            TNode* node = _rGraph.vF2DNode(layId, F2DNodeId);
            for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
                GRBQuadExpr outFlow;
                for (size_t oEdgeId = 0; oEdgeId < node->numOutEdges(); ++oEdgeId) {
                    TEdge* edge = node->vOutEdge(oEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        outFlow += _vHFlow[netId][layId][edge->edgeTypeId()] * _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else if (edge->edgeType() == TEdgeType::Feasible2D) {
                        outFlow += _vF2DFlow[netId][layId][edge->edgeTypeId()] * _vF2DFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::TRoundVia);
                        if (edge->netId() == netId) {
                            outFlow += _vTRFlow[netId][layId][edge->edgeTypeId()] * _vTRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                GRBQuadExpr inFlow;
                for (size_t iEdgeId = 0; iEdgeId < node->numInEdges(); ++iEdgeId) {
                    TEdge* edge = node->vInEdge(iEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        inFlow += _vHFlow[netId][layId][edge->edgeTypeId()] * _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else if (edge->edgeType() == TEdgeType::Feasible2D) {
                        inFlow += _vF2DFlow[netId][layId][edge->edgeTypeId()] * _vF2DFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::SRoundVia);
                        if (edge->netId() == netId) {
                            inFlow += _vSRFlow[netId][layId][edge->edgeTypeId()] * _vSRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                _model.addConstr(outFlow + inFlow + _vF2DVia[netId][F2DNodeId] <= (_rGraph.numNeighborsUB()+1) * _vF2DApproval[netId][layId][F2DNodeId]);
            }
        }
    }

    // Other node approval constraints
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
        for (size_t otherNodeId = 0; otherNodeId < _rGraph.numOtherNodes(layId); ++otherNodeId) {
            TNode* node = _rGraph.vOtherNode(layId, otherNodeId);
            for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
                GRBQuadExpr outFlow;
                for (size_t oEdgeId = 0; oEdgeId < node->numOutEdges(); ++oEdgeId) {
                    TEdge* edge = node->vOutEdge(oEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        outFlow += _vHFlow[netId][layId][edge->edgeTypeId()] * _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::TRoundVia);
                        if (edge->netId() == netId) {
                            outFlow += _vTRFlow[netId][layId][edge->edgeTypeId()] * _vTRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                GRBQuadExpr inFlow;
                for (size_t iEdgeId = 0; iEdgeId < node->numInEdges(); ++iEdgeId) {
                    TEdge* edge = node->vInEdge(iEdgeId);
                    if (edge->edgeType() == TEdgeType::Horizontal) {
                        inFlow += _vHFlow[netId][layId][edge->edgeTypeId()] * _vHFlow[netId][layId][edge->edgeTypeId()];
                    } else {
                        assert(edge->edgeType() == TEdgeType::SRoundVia);
                        if (edge->netId() == netId) {
                            inFlow += _vSRFlow[netId][layId][edge->edgeTypeId()] * _vSRFlow[netId][layId][edge->edgeTypeId()];
                        }
                    }
                }
                _model.addConstr(outFlow + inFlow + _vF2DVia[netId][otherNodeId] <= (_rGraph.numNeighborsUB()-1) * _vF2DApproval[netId][layId][otherNodeId]);
            }
        }
    }

    // single node single net constraint
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId) {
        // F2D node
        for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
            GRBLinExpr approval;
            for (size_t netId = 0; netId <_rGraph.numNets(); ++netId) {
                approval += _vF2DApproval[netId][layId][F2DNodeId];
            }
            _model.addConstr(approval == 1);
        }
        // other node
        for (size_t otherNodeId = 0; otherNodeId < _rGraph.numOtherNodes(layId); ++otherNodeId) {
            GRBLinExpr approval;
            for (size_t netId = 0; netId <_rGraph.numNets(); ++netId) {
                approval += _vOtherApproval[netId][layId][otherNodeId];
            }
            _model.addConstr(approval == 1);
        }
    }
}