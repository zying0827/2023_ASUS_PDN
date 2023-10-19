#include "FlowLP.h"

FlowLP::FlowLP(RGraph& rGraph, vector<double> vMediumLayerThickness, vector<double> vMetalLayerThickness, vector<double> vConductivity, double currentNorm)
    : _model(_env), _rGraph(rGraph), _vMediumLayerThickness(vMediumLayerThickness), _vMetalLayerThickness(vMetalLayerThickness), _vConductivity(vConductivity), _currentNorm(currentNorm) {
    // _env.set("LogToConsole", 0);
    // _env.set("OutputFlag", 0);
    // _env.start();
    // _model = new GRBModel(_env);
    _numCapConstrs = 0;
    _vPlaneLeftFlow = new GRBVar** [_rGraph.numNets()];
    _vPlaneRightFlow = new GRBVar** [_rGraph.numNets()];
    _vViaFlow = new GRBVar** [_rGraph.numNets()];
    _vMaxViaCost = new GRBVar* [_rGraph.numNets()];
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        _vPlaneLeftFlow[netId] = new GRBVar* [_rGraph.numLayers()];
        _vPlaneRightFlow[netId] = new GRBVar* [_rGraph.numLayers()];
        _vViaFlow[netId] = new GRBVar* [_rGraph.numLayerPairs()];
        _vMaxViaCost[netId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            _vPlaneLeftFlow[netId][layId] = new GRBVar [_rGraph.numPlaneOASGEdges(netId, layId)];
            _vPlaneRightFlow[netId][layId] = new GRBVar [_rGraph.numPlaneOASGEdges(netId, layId)];
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                _vPlaneLeftFlow[netId][layId][pEdgeId] = _model.addVar(0.0, std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, 
                                                        "Fl_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId));
                _vPlaneRightFlow[netId][layId][pEdgeId] = _model.addVar(0.0, std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, 
                                                        "Fr_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId));                                        
            }
        }
        for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
            _vViaFlow[netId][layPairId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
            for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
                if (!_rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->redundant()) {
                    _vViaFlow[netId][layPairId][vEdgeId] = _model.addVar(0.0, std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, 
                                                            "Fv_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
                }
            }     
        }
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            _vMaxViaCost[netId][vEdgeId] = _model.addVar(0.0, std::numeric_limits<double>::max(), 0.0, GRB_CONTINUOUS, 
                                                        "Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId));
        }
    }
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowLP::setObjective(double areaWeight, double viaWeight){
    GRBLinExpr obj;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // add cost for horizontal flows
        // cerr << "add cost for horizontal flows" << endl;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double cost = (areaWeight * pow(1E-3 * e->length(), 2)) / (abs(e->sNode()->voltage()-e->tNode()->voltage()) * _vMetalLayerThickness[layId] * 1E-3);
                // cerr << "cost = " << cost << endl;
                obj += cost * (_vPlaneLeftFlow[netId][layId][pEdgeId] + _vPlaneRightFlow[netId][layId][pEdgeId]);
            }
        }
        // add cost for vertical flows
        // cerr << "add cost for vertical flows" << endl;
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                if (!e->redundant()) {
                    double costNum = 1E-3 * (0.5*_vMetalLayerThickness[layPairId]+_vMediumLayerThickness[layPairId+1]+0.5*_vMetalLayerThickness[layPairId+1]);
                    double costDen = abs(e->sNode()->voltage() - e->tNode()->voltage()); // has not * via cross-sectional area
                    // cerr << "costDen=" << setprecision(15) << costDen << ", ";
                    // cerr << "sNode->voltage=" << setprecision(15) << e->sNode()->voltage() << ", ";
                    // cerr << "tNode->voltage=" << setprecision(15) << e->tNode()->voltage() << endl;
                    double cost = costNum / costDen;
                    // cerr << "cost = " << cost << endl;
                    _model.addConstr(viaWeight * cost * _vViaFlow[netId][layPairId][vEdgeId] <= _vMaxViaCost[netId][vEdgeId], 
                                    "max_via_cost_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
                }
            }
            obj += _vMaxViaCost[netId][vEdgeId];
        }
    }
    // cerr << "_model.setObjective(obj, GRB_MINIMIZE)" << endl;
    _model.setObjective(obj, GRB_MINIMIZE);
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowLP::setConserveConstraints(){
    // the input and output flows for port nodes
    cerr << "the input and output flows for port nodes" << endl;
    vector<double> vInputFlow(_rGraph.numOASGNodes(), 0.0);
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        OASGNode* sNode = _rGraph.sourceOASGNode(netId, 0);
        vInputFlow[sNode->nodeId()] = sNode->port()->current() / _currentNorm;
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, netTPortId, 0);
            vInputFlow[tNode->nodeId()] = - tNode->port()->current() / _currentNorm;
        }
    }
    // flow conservation constraints for all nodes
    cerr << "flow conservation constraints for all nodes" << endl;
    for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
        GRBLinExpr outFlow;
        OASGNode* node = _rGraph.vOASGNode(nodeId);
        if (!node->redundant()) {
            // node->print();
            for (size_t outEdgeId = 0; outEdgeId < node->numOutEdges(); ++ outEdgeId) {
                OASGEdge* edge = _rGraph.vOASGEdge(node->outEdgeId(outEdgeId)); 
                if (!edge->redundant()) {
                    if (edge->viaEdge()) {
                        outFlow += _vViaFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    } else {
                        outFlow += _vPlaneLeftFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                        outFlow += _vPlaneRightFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    }
                }
            }
            for (size_t inEdgeId = 0; inEdgeId < node->numInEdges(); ++ inEdgeId) {
                OASGEdge* edge = _rGraph.vOASGEdge(node->inEdgeId(inEdgeId)); 
                if (!edge->redundant()) {
                    if (edge->viaEdge()) {
                        outFlow -= _vViaFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    } else {
                        outFlow -= _vPlaneLeftFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                        outFlow -= _vPlaneRightFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    }
                }
            }
            _model.addConstr(outFlow == vInputFlow[nodeId], "flow_conserve_n" + to_string(nodeId));
        }
    }
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowLP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width){
    printf("E%-2d_%c * %.2f + E%-2d_%c * %.2f <= %.3f\n", e1->edgeId(), right1? 'r': 'l', ratio1, e2->edgeId(), right2? 'r': 'l', ratio2, width);
    // width unit = millimeter
    GRBLinExpr totalWidth;
    double widthWeight1 = (e1->length()) / (_vConductivity[e1->layId()] * abs(e1->sNode()->voltage()-e1->tNode()->voltage()) * _vMetalLayerThickness[e1->layId()]);
    double widthWeight2 = (e2->length()) / (_vConductivity[e2->layId()] * abs(e2->sNode()->voltage()-e2->tNode()->voltage()) * _vMetalLayerThickness[e2->layId()]);
    if (right1) {
        totalWidth += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
    } else {
        totalWidth += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
    }
    if (right2) {
        totalWidth += _vPlaneRightFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
    } else {
        totalWidth += _vPlaneLeftFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
    }
    _model.addConstr(totalWidth * 1E3 <= width, "capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    _numCapConstrs ++;
}

void FlowLP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, double width) {
    printf("E%-2d_%c * %.2f <= %.3f\n", e1->edgeId(), right1? 'r': 'l', ratio1, width);
    // width unit = millimeter
    GRBLinExpr e1Width;
    double widthWeight1 = (e1->length()) / (_vConductivity[e1->layId()] * abs(e1->sNode()->voltage()-e1->tNode()->voltage()) * _vMetalLayerThickness[e1->layId()]);
    if (right1) {
        e1Width += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
    } else {
        e1Width += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
    }
    _model.addConstr(e1Width * 1E3 <= width, "single_capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    // _numCapConstrs ++;
}

void FlowLP::relaxCapacityConstraints(vector<double> vLambda) {
    assert(vLambda.size() == _numCapConstrs);
    _model.update();
    _modelRelaxed = new GRBModel(_model);
    for (size_t capId = 0; capId < _numCapConstrs; ++ capId) {
        const GRBConstr& c = _modelRelaxed->getConstrByName("capacity_" + to_string(capId));
        // char sense = c->get ( GRB_CharAttr_Sense );
        // if ( sense != '>') {
        //     double coef = -1.0;
        //     _model.addVar (0.0 , GRB_INFINITY , 1.0 , GRB_CONTINUOUS , 1 , & c, & coef , " ArtN_ " + c-> get ( GRB_StringAttr_ConstrName ));
        // }
        // if ( sense != '<') {
        //     double coef = 1.0;
        //     _model.addVar (0.0 , GRB_INFINITY , 1.0 , GRB_CONTINUOUS , 1 ,& c , & coef , " ArtP_ " + c-> get ( GRB_StringAttr_ConstrName ));
        // }
        double coef = -1.0;
        _modelRelaxed->addVar(0.0 , GRB_INFINITY , vLambda[capId] , GRB_CONTINUOUS, 1, &c, &coef , "lambda_" + c.get ( GRB_StringAttr_ConstrName ));
    }
    // _modelRelaxed->update();
    // _modelRelaxed->write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowLP::solve() {
    _model.optimize();
}

void FlowLP::collectResult(){
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // collect results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                // cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double widthWeight = (e->length()) / (_vConductivity[e->layId()] * abs(e->sNode()->voltage()-e->tNode()->voltage()) * _vMetalLayerThickness[e->layId()]);
                double leftFlow = _vPlaneLeftFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                // cerr << "leftFlow = " << leftFlow;
                double rightFlow = _vPlaneRightFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                // cerr << " rightFlow = " << rightFlow << endl;
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentRight(rightFlow);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentLeft(leftFlow);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(leftFlow * widthWeight * 1E3);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(rightFlow * widthWeight * 1E3);
            }
        }
        // collect results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            double viaArea = _vMaxViaCost[netId][vEdgeId].get(GRB_DoubleAttr_X) * _currentNorm / _vConductivity[0];
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                // cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    double flow = _vViaFlow[netId][layPairId][vEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                    // cerr << "flow = " << flow << endl;
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentRight(flow);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentLeft(0.0);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
        }
    }
}
void FlowLP::printResult() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // print results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                cerr << "edgeId=" << e->edgeId() << ", current = " << e->current() << ", widthLeft = " << e->widthLeft();
                cerr << ", widthRight = " << e->widthRight() << endl;
            }
        }
        // collect results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                cerr << "edgeId = " << e->edgeId() << " ";
                if (!e->redundant()) {
                    cerr << "current = " << e->current() << ", viaArea = " << e->viaArea() << endl;
                } else {
                    cerr << "redundant edge" << endl;
                }
            }
        }
    }
    // cerr << _model. << endl;
    // _model.printQuality();
}

void FlowLP::solveRelaxed() {
    _modelRelaxed->optimize();
}

void FlowLP::collectRelaxedResult() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // collect results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                // cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double widthWeight = (e->length()) / (_vConductivity[e->layId()] * abs(e->sNode()->voltage()-e->tNode()->voltage()) * _vMetalLayerThickness[e->layId()]);
                // double leftFlow = _vPlaneLeftFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                double leftFlow = _modelRelaxed->getVarByName("Fl_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm;
                // cerr << "leftFlow = " << leftFlow;
                double rightFlow = _modelRelaxed->getVarByName("Fr_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm;
                // cerr << " rightFlow = " << rightFlow << endl;
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentRight(rightFlow);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentLeft(leftFlow);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(leftFlow * widthWeight * 1E3);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(rightFlow * widthWeight * 1E3);
            }
        }
        // collect results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            // double viaArea = _vMaxViaCost[netId][vEdgeId].get(GRB_DoubleAttr_X) * _currentNorm / _vConductivity[0];
            double viaArea = _modelRelaxed->getVarByName("Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm / _vConductivity[0];
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                // cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    double flow = _modelRelaxed->getVarByName("Fv_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm;
                    // cerr << "flow = " << flow << endl;
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentRight(flow);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentLeft(0.0);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
        }
    }
}

void FlowLP::printRelaxedResult() {
    double violation = 0;
    for (size_t capId = 0; capId < _numCapConstrs; ++capId) {
        violation += _modelRelaxed->getVarByName("lambda_capacity_" + to_string(capId)).get(GRB_DoubleAttr_X);
    }
    cerr << "violation = " << violation << endl;
    double planeArea = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                planeArea += e->length() * (e->widthLeft() + e->widthRight());
            }
        }
    }
    cerr << "planeArea = " << planeArea << endl;
}