#include "VoltSLP.h"

VoltSLP::VoltSLP(DB& db, RGraph& rGraph, vector< vector< double > > vOldVoltage)
 : _model(_env), _db(db), _rGraph(rGraph), _vOldVoltage(vOldVoltage) {
    _model.set(GRB_DoubleParam_FeasibilityTol, 1e-9);
    _area = 0;
    _overlap = 0;
    _numCapConstrs = 0;
    _numNetCapConstrs = 0;
    _vVoltage = new GRBVar* [_rGraph.numNets()];
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        _vVoltage[netId] = new GRBVar [_rGraph.numNPortOASGNodes(netId)];
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _vVoltage[netId][nPortNodeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "V_n" + to_string(netId) + "_i_" + to_string(nPortNodeId));
        }
    }
    // _vPEdgeInV = new GRBVar** [_rGraph.numNets()];
    // _vVEdgeInV = new GRBVar** [_rGraph.numNets()];
    _vMaxViaCost = new GRBVar* [_rGraph.numNets()];
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // _vPEdgeInV[netId] = new GRBVar* [_rGraph.numLayers()];
        // _vVEdgeInV[netId] = new GRBVar* [_rGraph.numLayerPairs()];
        _vMaxViaCost[netId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
        // for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
        //     _vPEdgeInV[netId][layId] = new GRBVar [_rGraph.numPlaneOASGEdges(netId, layId)];
        //     for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
        //         _vPEdgeInV[netId][layId][pEdgeId] = _model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
        //                                                 "PIV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId));                                        
        //     }
        // }
        // for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
        //     _vVEdgeInV[netId][layPairId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
        //     for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
        //         if (!_rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->redundant()) {
        //             _vVEdgeInV[netId][layPairId][vEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
        //                                                     "VIV_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
        //         }
        //     }     
        // }
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            _vMaxViaCost[netId][vEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId));
        }
    }

    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        vector<double> temp(_rGraph.numNPortOASGNodes(netId), 0.0);
        _vNewVoltage.push_back(temp);
    }
}

void VoltSLP::setVoltConstraints(double threshold) {
    assert(false);
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
                    _model.addConstr(sVolt - tVolt >= threshold);
                }
            }
        }
    }
}

GRBLinExpr VoltSLP::linApprox(double cost, OASGEdge* edge) {
    assert(!edge->redundant());
    GRBLinExpr lin;
    double sOldVolt, tOldVolt;
    GRBLinExpr sVolt, tVolt;
    if (edge->sNode()->nPort()) {
        sOldVolt = _vOldVoltage[edge->netId()][edge->sNode()->nPortNodeId()];
        sVolt += _vVoltage[edge->netId()][edge->sNode()->nPortNodeId()];
    } else {
        sOldVolt = edge->sNode()->voltage();
        sVolt += edge->sNode()->voltage();
    }
    if (edge->tNode()->nPort()) {
        tOldVolt = _vOldVoltage[edge->netId()][edge->tNode()->nPortNodeId()];
        tVolt += _vVoltage[edge->netId()][edge->tNode()->nPortNodeId()];
    } else {
        tOldVolt = edge->tNode()->voltage();
        tVolt += edge->tNode()->voltage();
    }
    assert(edge->current() * (sOldVolt - tOldVolt) >= 0);
    if (sOldVolt == tOldVolt) {
        lin += 0;
    } else {
        lin += (2*cost)/(sOldVolt-tOldVolt) - (cost/pow(sOldVolt-tOldVolt, 2)) * (sVolt-tVolt);
    }
    return lin;
}

void VoltSLP::setObjective(double areaWeight, double viaWeight){
    GRBLinExpr obj;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // add cost for horizontal flows
        // cerr << "add cost for horizontal flows" << endl;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                // assert(e->current() * (e->sNode()->voltage() - e->tNode()->voltage()) >= 0); // asserted in linApprox (use oldVolt)
                double cost = (areaWeight * pow(1E-3 * e->length(), 2)) * e->current() / (_db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3);
                // cerr << "cost = " << cost << endl;
                obj += linApprox(cost, e);
            }
        }
        // add cost for vertical flows
        // cerr << "add cost for vertical flows" << endl;
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                if (!e->redundant()) {
                    // assert(e->current() * (e->sNode()->voltage() - e->tNode()->voltage()) >= 0); // asserted in linApprox (use oldVolt)
                    double l = 1E-3 * (0.5*_db.vMetalLayer(layPairId)->thickness()+_db.vMediumLayer(layPairId+1)->thickness()+0.5*_db.vMetalLayer(layPairId+1)->thickness());
                    double costNum = l * e->current();
                    double costDen = _db.vMetalLayer(0)->conductivity(); // has not * via cross-sectional area
                    // cerr << "costDen=" << setprecision(15) << costDen << ", ";
                    // cerr << "sNode->voltage=" << setprecision(15) << e->sNode()->voltage() << ", ";
                    // cerr << "tNode->voltage=" << setprecision(15) << e->tNode()->voltage() << endl;
                    double cost = costNum / costDen;
                    // cerr << "cost = " << cost << endl;
                    _model.addConstr(linApprox(cost, e) <= _vMaxViaCost[netId][vEdgeId], 
                                    "max_via_cost_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
                    _model.addConstr(linApprox(cost, e) * 1E6 >= _db.VIA16D8A24()->metalArea());
                }
            }
            obj += viaWeight * _vMaxViaCost[netId][vEdgeId];
        }
    }
    // cerr << "_model.setObjective(obj, GRB_MINIMIZE)" << endl;
    _model.setObjective(obj, GRB_MINIMIZE);
    // assert(false);
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void VoltSLP::setLimitConstraint(double ratio) {
    // cerr << "setLimitConstraint..." << endl;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* edge = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double sOldVolt, tOldVolt;
                GRBLinExpr sVolt, tVolt;
                if (edge->sNode()->nPort()) {
                    sOldVolt = _vOldVoltage[edge->netId()][edge->sNode()->nPortNodeId()];
                    sVolt += _vVoltage[edge->netId()][edge->sNode()->nPortNodeId()];
                } else {
                    sOldVolt = edge->sNode()->voltage();
                    sVolt += edge->sNode()->voltage();
                }
                if (edge->tNode()->nPort()) {
                    tOldVolt = _vOldVoltage[edge->netId()][edge->tNode()->nPortNodeId()];
                    tVolt += _vVoltage[edge->netId()][edge->tNode()->nPortNodeId()];
                } else {
                    tOldVolt = edge->tNode()->voltage();
                    tVolt += edge->tNode()->voltage();
                }
                if (sOldVolt >= tOldVolt) {
                    _model.addConstr( (sVolt-tVolt) >= ratio*(sOldVolt-tOldVolt), "lbPV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId) );
                    _model.addConstr( (sVolt-tVolt) <= (2.0-ratio)*(sOldVolt-tOldVolt), "ubPV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId) );
                } else {
                    _model.addConstr( (sVolt-tVolt) <= ratio*(sOldVolt-tOldVolt), "lbPV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId) );
                    _model.addConstr( (sVolt-tVolt) >= (2.0-ratio)*(sOldVolt-tOldVolt), "ubPV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId) );
                }

            }
        }
        // add cost for vertical flows
        // cerr << "add cost for vertical flows" << endl;
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* edge = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                if (!edge->redundant()) {
                    double sOldVolt, tOldVolt;
                    GRBLinExpr sVolt, tVolt;
                    if (edge->sNode()->nPort()) {
                        sOldVolt = _vOldVoltage[edge->netId()][edge->sNode()->nPortNodeId()];
                        sVolt += _vVoltage[edge->netId()][edge->sNode()->nPortNodeId()];
                    } else {
                        sOldVolt = edge->sNode()->voltage();
                        sVolt += edge->sNode()->voltage();
                    }
                    if (edge->tNode()->nPort()) {
                        tOldVolt = _vOldVoltage[edge->netId()][edge->tNode()->nPortNodeId()];
                        tVolt += _vVoltage[edge->netId()][edge->tNode()->nPortNodeId()];
                    } else {
                        tOldVolt = edge->tNode()->voltage();
                        tVolt += edge->tNode()->voltage();
                    }
                    assert(sOldVolt >= tOldVolt);
                    _model.addConstr( (sVolt-tVolt) >= ratio*(sOldVolt-tOldVolt), "lbVV_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId) );
                    _model.addConstr( (sVolt-tVolt) <= (2.0-ratio)*(sOldVolt-tOldVolt), "ubVV_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId) );
                }
            }
        }
    }
}

void VoltSLP::addViaAreaConstraints(size_t netId, size_t vEdgeId, double area) {
    cerr << "addViaAreaConstraints: net" << netId << " vEdge" << vEdgeId << " area = " << area << endl;
    _model.addConstr(_vMaxViaCost[netId][vEdgeId] * 1E6 <= area);
}

void VoltSLP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width){
    // width unit = millimeter
    GRBLinExpr totalWidth;
    double widthWeight1 = (e1->length()) / (_db.vMetalLayer(e1->layId())->conductivity() * _db.vMetalLayer(e1->layId())->thickness());
    double widthWeight2 = (e2->length()) / (_db.vMetalLayer(e2->layId())->conductivity() * _db.vMetalLayer(e2->layId())->thickness());
    double cost1, cost2;
    if (right1) {
        cost1 = e1->currentRight() * widthWeight1 * ratio1;
        totalWidth += linApprox(cost1, e1);
        // totalWidth += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentRight() * widthWeight1 * ratio1;
    } else {
        cost1 = e1->currentLeft() * widthWeight1 * ratio1;
        totalWidth += linApprox(cost1, e1);
        // totalWidth += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentLeft() * widthWeight1 * ratio1;
    }
    if (right2) {
        cost2 = e2->currentRight() * widthWeight2 * ratio2;
        totalWidth += linApprox(cost2, e2);
        // totalWidth += _vPEdgeInV[e2->netId()][e2->layId()][e2->typeEdgeId()] * e2->currentRight() * widthWeight2 * ratio2;
    } else {
        cost2 = e2->currentLeft() * widthWeight2 * ratio2;
        totalWidth += linApprox(cost2, e2);
        // totalWidth += _vPEdgeInV[e2->netId()][e2->layId()][e2->typeEdgeId()] * e2->currentLeft() * widthWeight2 * ratio2;
    }
    _model.addConstr(totalWidth * 1E3 <= width, "capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    _numCapConstrs ++;
}

void VoltSLP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, double width) {
    // width unit = millimeter
    GRBLinExpr e1Width;
    double widthWeight1 = (e1->length()) / (_db.vMetalLayer(e1->layId())->conductivity() * _db.vMetalLayer(e1->layId())->thickness());
    double cost1;
    if (right1) {
        cost1 = e1->currentRight() * widthWeight1 * ratio1;
        e1Width += linApprox(cost1, e1);
        // e1Width += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentRight() * widthWeight1 * ratio1;
    } else {
        cost1 = e1->currentLeft() * widthWeight1 * ratio1;
        e1Width += linApprox(cost1, e1);
        // e1Width += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentLeft() * widthWeight1 * ratio1;
    }
    _model.addConstr(e1Width * 1E3 <= width, "single_capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    // _numCapConstrs ++;
}

void VoltSLP::addSameNetCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width) {
    GRBLinExpr totalWidth;
    double widthWeight1 = (e1->length()) / (_db.vMetalLayer(e1->layId())->conductivity() * _db.vMetalLayer(e1->layId())->thickness());
    double widthWeight2 = (e2->length()) / (_db.vMetalLayer(e2->layId())->conductivity() * _db.vMetalLayer(e2->layId())->thickness());
    double cost1, cost2;
    if (right1) {
        cost1 = e1->currentRight() * widthWeight1 * ratio1;
        totalWidth += linApprox(cost1, e1);
        // totalWidth += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentRight() * widthWeight1 * ratio1;
    } else {
        cost1 = e1->currentLeft() * widthWeight1 * ratio1;
        totalWidth += linApprox(cost1, e1);
        // totalWidth += _vPEdgeInV[e1->netId()][e1->layId()][e1->typeEdgeId()] * e1->currentLeft() * widthWeight1 * ratio1;
    }
    if (right2) {
        cost2 = e2->currentRight() * widthWeight2 * ratio2;
        totalWidth += linApprox(cost2, e2);
        // totalWidth += _vPEdgeInV[e2->netId()][e2->layId()][e2->typeEdgeId()] * e2->currentRight() * widthWeight2 * ratio2;
    } else {
        cost2 = e2->currentLeft() * widthWeight2 * ratio2;
        totalWidth += linApprox(cost2, e2);
        // totalWidth += _vPEdgeInV[e2->netId()][e2->layId()][e2->typeEdgeId()] * e2->currentLeft() * widthWeight2 * ratio2;
    }
    _model.addConstr(totalWidth * 1E3 <= width, "same_net_capacity_" + to_string(_numNetCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    _numNetCapConstrs ++;
}

void VoltSLP::relaxCapacityConstraints(vector<double> vLambda) {
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

void VoltSLP::relaxCapacityConstraints(vector<double> vLambda, vector<double> vNetLambda) {
    assert(vLambda.size() == _numCapConstrs);
    assert(vNetLambda.size() == _numNetCapConstrs);
    _model.update();
    _modelRelaxed = new GRBModel(_model);
    for (size_t capId = 0; capId < _numCapConstrs; ++ capId) {
        const GRBConstr& c = _modelRelaxed->getConstrByName("capacity_" + to_string(capId));
        double coef = -1.0;
        _modelRelaxed->addVar(0.0 , GRB_INFINITY , vLambda[capId] , GRB_CONTINUOUS, 1, &c, &coef , "lambda_" + c.get ( GRB_StringAttr_ConstrName ));
    }
    for (size_t netCapId = 0; netCapId < _numNetCapConstrs; ++ netCapId) {
        const GRBConstr& c = _modelRelaxed->getConstrByName("same_net_capacity_" + to_string(netCapId));
        double coef = -1.0;
        _modelRelaxed->addVar(0.0 , GRB_INFINITY , vNetLambda[netCapId] , GRB_CONTINUOUS, 1, &c, &coef , "same_net_lambda_" + c.get ( GRB_StringAttr_ConstrName ));
    }
}

void VoltSLP::solve() {
    _model.optimize();
}

void VoltSLP::collectTempVoltage() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _vNewVoltage[netId][nPortNodeId] = _vVoltage[netId][nPortNodeId].get(GRB_DoubleAttr_X) ;
        }
    }
}

void VoltSLP::collectResult() {
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
                // // double VoltInV = _vPEdgeInV[netId][layId][pEdgeId].get(GRB_DoubleAttr_X);
                // double VoltInV = 1.0/(e->sNode()->voltage() - e->tNode()->voltage());
                // _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(VoltInV * widthWeightLeft * 1E3);
                // _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(VoltInV * widthWeightRight * 1E3);
                double widthLeft, widthRight;
                if (e->sNode()->voltage() == e->tNode()->voltage()) {
                    widthLeft = 0;
                    widthRight = 0;
                } else {
                    assert(e->currentLeft() * e->currentRight() >= 0);
                    assert((e->sNode()->voltage() - e->tNode()->voltage()) * e->current() > 0);
                    widthLeft = (widthWeightLeft * 1E3) / (e->sNode()->voltage() - e->tNode()->voltage());
                    widthRight = (widthWeightRight * 1E3) / (e->sNode()->voltage() - e->tNode()->voltage());
                }
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(widthLeft);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(widthRight);
            }
        }
        // collect area results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            double viaArea = _vMaxViaCost[netId][vEdgeId].get(GRB_DoubleAttr_X);
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                // cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
        }
    }
}

void VoltSLP::solveRelaxed() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                double sVolt, tVolt;
                if (e->sNode()->nPort()) {
                    sVolt  = _vOldVoltage[netId][e->sNode()->nPortNodeId()];
                } else {
                    sVolt = e->sNode()->voltage();
                }
                if (e->tNode()->nPort()) {
                    tVolt  = _vOldVoltage[netId][e->tNode()->nPortNodeId()];
                } else {
                    tVolt = e->tNode()->voltage();
                }
                cerr << "dVolt = " << sVolt - tVolt << endl;
            }
        }
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                if (!e->redundant()) {
                    cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                    double sVolt, tVolt;
                    if (e->sNode()->nPort()) {
                        sVolt  = _vOldVoltage[netId][e->sNode()->nPortNodeId()];
                    } else {
                        sVolt = e->sNode()->voltage();
                    }
                    if (e->tNode()->nPort()) {
                        tVolt  = _vOldVoltage[netId][e->tNode()->nPortNodeId()];
                    } else {
                        tVolt = e->tNode()->voltage();
                    }
                    cerr << "dVolt = " << sVolt - tVolt << endl;
                }
            }
        }
    }
    _modelRelaxed->optimize();
}

void VoltSLP::collectRelaxedTempVoltage() {
    for(size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _vNewVoltage[netId][nPortNodeId] = _modelRelaxed->getVarByName("V_n" + to_string(netId) + "_i_" + to_string(nPortNodeId)).get(GRB_DoubleAttr_X);
        }
    }
}

void VoltSLP::collectRelaxedResult() {
    _viaArea = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // collect voltage results
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId)->setVoltage( _modelRelaxed->getVarByName("V_n" + to_string(netId) + "_i_" + to_string(nPortNodeId)).get(GRB_DoubleAttr_X) );
        }
        // collect width results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                double widthWeightRight = (e->length() * e->currentRight()) / (_db.vMetalLayer(e->layId())->conductivity() * _db.vMetalLayer(e->layId())->thickness());
                double widthWeightLeft = (e->length() * e->currentLeft()) / (_db.vMetalLayer(e->layId())->conductivity() * _db.vMetalLayer(e->layId())->thickness());
                // double VoltInV = _modelRelaxed->getVarByName("PIV_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X);
                // double VoltInV = 1.0/(e->sNode()->voltage() - e->tNode()->voltage());
                cerr << "sVolt = " << e->sNode()->voltage() << ", tVolt = " << e->tNode()->voltage() << ", ";
                double widthLeft, widthRight;
                if (e->sNode()->voltage() == e->tNode()->voltage()) {
                    widthLeft = 0;
                    widthRight = 0;
                } else {
                    assert(e->currentLeft() * e->currentRight() >= 0);
                    assert((e->sNode()->voltage() - e->tNode()->voltage()) * e->current() >= 0);
                    widthLeft = (widthWeightLeft * 1E3) / (e->sNode()->voltage() - e->tNode()->voltage());
                    widthRight = (widthWeightRight * 1E3) / (e->sNode()->voltage() - e->tNode()->voltage());
                }
                cerr << "widthLeft = " << widthLeft << ", widthRight = " << widthRight << endl;
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(widthLeft);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(widthRight);
                // assert(VoltInV * e->currentRight() >= 0);
                // assert(VoltInV * e->currentLeft() >=0);
                // _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(VoltInV * widthWeightLeft * 1E3);
                // _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(VoltInV * widthWeightRight * 1E3);
            }
        }
        // collect area results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            double viaArea = _modelRelaxed->getVarByName("Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X);
            _viaArea += viaArea * 1E6;
            assert(viaArea * 1E6 > 0);
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                // cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
        }
    }

    // record area and overlapped width
    _area = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _area += e->length() * (e->widthLeft() + e->widthRight());
            }
        }
    }
    _overlap = 0;
    for (size_t capId = 0; capId < _numCapConstrs; ++capId) {
        _overlap += _modelRelaxed->getVarByName("lambda_capacity_" + to_string(capId)).get(GRB_DoubleAttr_X);
    }
    _sameNetOverlap = 0;
    for (size_t netCapId = 0; netCapId < _numNetCapConstrs; ++ netCapId) {
        _vSameNetOverlap.push_back(_modelRelaxed->getVarByName("same_net_lambda_same_net_capacity_" + to_string(netCapId)).get(GRB_DoubleAttr_X));
        _sameNetOverlap += _vSameNetOverlap[netCapId];
    }
}

void VoltSLP::printRelaxedResult() {
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