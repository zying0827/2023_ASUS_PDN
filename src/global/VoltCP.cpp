#include "VoltCP.h"

VoltCP::VoltCP(DB& db, RGraph& rGraph) : _model(_env), _db(db), _rGraph(rGraph) {
    _numCapConstrs = 0;
    _vVoltage = new GRBVar* [_rGraph.numNets()];
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        _vVoltage[netId] = new GRBVar [_rGraph.numNPortOASGNodes(netId)];
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _vVoltage[netId][nPortNodeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "V_n" + to_string(netId) + "_i_" + to_string(nPortNodeId));
        }
    }
    _vPEdgeInV = new GRBVar** [_rGraph.numNets()];
    _vVEdgeInV = new GRBVar** [_rGraph.numNets()];
    _vMaxViaCost = new GRBVar* [_rGraph.numNets()];
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        _vPEdgeInV[netId] = new GRBVar* [_rGraph.numLayers()];
        _vVEdgeInV[netId] = new GRBVar* [_rGraph.numLayerPairs()];
        _vMaxViaCost[netId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            _vPEdgeInV[netId][layId] = new GRBVar [_rGraph.numPlaneOASGEdges(netId, layId)];
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                _vPEdgeInV[netId][layId][pEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "PIV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId));                                        
            }
        }
        for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
            _vVEdgeInV[netId][layPairId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
            for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
                if (!_rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->redundant()) {
                    _vVEdgeInV[netId][layPairId][vEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                            "VIV_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
                }
            }     
        }
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            _vMaxViaCost[netId][vEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId));
        }
    }
}

void VoltCP::setVoltConstraints(double threshold) {
    _model.set(GRB_IntParam_NonConvex, 2);
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* edge = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                GRBLinExpr sVolt;
                GRBLinExpr tVolt;
                if (edge->sNode()->nPort()) {
                    sVolt += _vVoltage[netId][edge->sNode()->nPortNodeId()];
                } else {
                    sVolt += edge->sNode()->voltage();
                }
                if (edge->tNode()->nPort()) {
                    tVolt += _vVoltage[netId][edge->tNode()->nPortNodeId()];
                } else {
                    tVolt += edge->tNode()->voltage();
                }
                _model.addQConstr((sVolt - tVolt) * _vPEdgeInV[netId][layId][pEdgeId] == 1);
                _model.addConstr(sVolt - tVolt >= threshold);
            }
        }
        for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
            for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
                OASGEdge* edge = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                if(!edge->redundant()) {
                    GRBLinExpr sVolt;
                    GRBLinExpr tVolt;
                    if (edge->sNode()->nPort()) {
                        sVolt += _vVoltage[netId][edge->sNode()->nPortNodeId()];
                    } else {
                        sVolt += edge->sNode()->voltage();
                    }
                    if (edge->tNode()->nPort()) {
                        tVolt += _vVoltage[netId][edge->tNode()->nPortNodeId()];
                    } else {
                        tVolt += edge->tNode()->voltage();
                    }
                    _model.addQConstr((sVolt - tVolt) * _vVEdgeInV[netId][layPairId][vEdgeId] == 1);
                    _model.addConstr(sVolt - tVolt >= threshold);
                }
            }
        }
    }
}

void VoltCP::setObjective(double areaWeight, double viaWeight){
    GRBLinExpr obj;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // add cost for horizontal flows
        // cerr << "add cost for horizontal flows" << endl;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double cost = (areaWeight * pow(1E-3 * e->length(), 2)) * e->current() / (_db.vMetalLayer(layId)->thickness() * 1E-3);
                // cerr << "cost = " << cost << endl;
                obj += cost * _vPEdgeInV[netId][layId][pEdgeId];
            }
        }
        // add cost for vertical flows
        // cerr << "add cost for vertical flows" << endl;
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                if (!e->redundant()) {
                    double l = 1E-3 * (0.5*_db.vMetalLayer(layPairId)->thickness()+_db.vMediumLayer(layPairId+1)->thickness()+0.5*_db.vMetalLayer(layPairId+1)->thickness());
                    double costNum = l * e->current();
                    double costDen = 1; // has not * via cross-sectional area
                    // cerr << "costDen=" << setprecision(15) << costDen << ", ";
                    // cerr << "sNode->voltage=" << setprecision(15) << e->sNode()->voltage() << ", ";
                    // cerr << "tNode->voltage=" << setprecision(15) << e->tNode()->voltage() << endl;
                    double cost = costNum / costDen;
                    // cerr << "cost = " << cost << endl;
                    _model.addConstr(cost * _vVEdgeInV[netId][layPairId][vEdgeId] <= _vMaxViaCost[netId][vEdgeId], 
                                    "max_via_cost_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
                }
            }
            obj += viaWeight * _vMaxViaCost[netId][vEdgeId];
        }
    }
    // cerr << "_model.setObjective(obj, GRB_MINIMIZE)" << endl;
    _model.setObjective(obj, GRB_MINIMIZE);
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void VoltCP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width){
    // width unit = millimeter
    GRBLinExpr totalWidth;
    double widthWeight1 = (e1->length()) / (_db.vMetalLayer(e1->layId())->conductivity() * _db.vMetalLayer(e1->layId())->thickness());
    double widthWeight2 = (e2->length()) / (_db.vMetalLayer(e2->layId())->conductivity() * _db.vMetalLayer(e2->layId())->thickness());
    if (right1) {
        totalWidth += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentRight() * widthWeight1 * ratio1;
    } else {
        totalWidth += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentLeft() * widthWeight1 * ratio1;
    }
    if (right2) {
        totalWidth += _vPEdgeInV[e2->netId()][e2->layId()][e2->typeEdgeId()] * e2->currentRight() * widthWeight2 * ratio2;
    } else {
        totalWidth += _vPEdgeInV[e2->netId()][e2->layId()][e2->typeEdgeId()] * e2->currentLeft() * widthWeight2 * ratio2;
    }
    _model.addConstr(totalWidth * 1E3 <= width, "capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    _numCapConstrs ++;
}

void VoltCP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, double width) {
    // width unit = millimeter
    GRBLinExpr e1Width;
    double widthWeight1 = (e1->length()) / (_db.vMetalLayer(e1->layId())->conductivity() * _db.vMetalLayer(e1->layId())->thickness());
    if (right1) {
        e1Width += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentRight() * widthWeight1 * ratio1;
    } else {
        e1Width += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentLeft() * widthWeight1 * ratio1;
    }
    _model.addConstr(e1Width * 1E3 <= width, "single_capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    // _numCapConstrs ++;
}

void VoltCP::relaxCapacityConstraints(vector<double> vLambda) {
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
}

void VoltCP::solve() {
    _model.optimize();
}

void VoltCP::collectResult(){
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // collect voltage results
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId)->setVoltage( _vVoltage[netId][nPortNodeId].get(GRB_DoubleAttr_X) );
        }
        // collect width results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                // cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double widthWeightRight = (e->length() * e->currentRight()) / (_db.vMetalLayer(e->layId())->conductivity() * _db.vMetalLayer(e->layId())->thickness());
                double widthWeightLeft = (e->length() * e->currentLeft()) / (_db.vMetalLayer(e->layId())->conductivity() * _db.vMetalLayer(e->layId())->thickness());
                double VoltInV = _vPEdgeInV[netId][layId][pEdgeId].get(GRB_DoubleAttr_X);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(VoltInV * widthWeightLeft * 1E3);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(VoltInV * widthWeightRight * 1E3);
            }
        }
        // collect area results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            double viaArea = _vMaxViaCost[netId][vEdgeId].get(GRB_DoubleAttr_X) / _db.vMetalLayer(0)->conductivity();
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                // cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
        }
    }
}
// void FlowLP::printResult() {
//     for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
//         // print results for horizontal flows
//         for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
//             for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
//                 OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
//                 cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
//                 cerr << "edgeId=" << e->edgeId() << ", current = " << e->current() << ", widthLeft = " << e->widthLeft();
//                 cerr << ", widthRight = " << e->widthRight() << endl;
//             }
//         }
//         // collect results for vertical flows
//         for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
//             for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
//                 OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
//                 cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
//                 cerr << "edgeId = " << e->edgeId() << " ";
//                 if (!e->redundant()) {
//                     cerr << "current = " << e->current() << ", viaArea = " << e->viaArea() << endl;
//                 } else {
//                     cerr << "redundant edge" << endl;
//                 }
//             }
//         }
//     }
//     // cerr << _model. << endl;
//     // _model.printQuality();
// }

void VoltCP::solveRelaxed() {
    _modelRelaxed->optimize();
}

void VoltCP::collectRelaxedResult() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // collect voltage results
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId)->setVoltage( _modelRelaxed->getVarByName("V_n" + to_string(netId) + "_i_" + to_string(nPortNodeId)).get(GRB_DoubleAttr_X) );
        }
        // collect width results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                // cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double widthWeightRight = (e->length() * e->currentRight()) / (_db.vMetalLayer(e->layId())->conductivity() * _db.vMetalLayer(e->layId())->thickness());
                double widthWeightLeft = (e->length() * e->currentLeft()) / (_db.vMetalLayer(e->layId())->conductivity() * _db.vMetalLayer(e->layId())->thickness());
                double VoltInV = _modelRelaxed->getVarByName("PIV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(VoltInV * widthWeightLeft * 1E3);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(VoltInV * widthWeightRight * 1E3);
            }
        }
        // collect area results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            double viaArea = _modelRelaxed->getVarByName("Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X) / _db.vMetalLayer(0)->conductivity();
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                // cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
        }
    }
}

void VoltCP::printRelaxedResult() {
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