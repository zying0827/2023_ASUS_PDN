#include "DetailedMgr.h"
#include "DetailedDB.h"
#include "Shape.h"
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <tuple>
#include <utility>
#include <vector>
#include <Eigen/IterativeLinearSolvers>

void DetailedMgr::initGridMap() {
    auto occupiedBySegments = [&] (size_t layId, size_t xId, size_t yId, size_t netId) -> bool {
        for (size_t segId = 0; segId < _db.vNet(netId)->numSegments(layId); ++ segId) {
            Trace* trace = _db.vNet(netId)->vSegment(layId, segId)->trace();
            if (trace->width() > 0) {
                if (trace->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
                if (trace->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
                if (trace->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
                if (trace->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
            }
        }
        return false;
    };
    auto occupiedByPorts = [&] (size_t xId, size_t yId, size_t netId) -> bool {
        Polygon* sBPolygon = _db.vNet(netId)->sourcePort()->boundPolygon();
        if (sBPolygon->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
        if (sBPolygon->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
        if (sBPolygon->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
        if (sBPolygon->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            Polygon* tBPolygon = _db.vNet(netId)->targetPort(tPortId)->boundPolygon();
            if (tBPolygon->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
            if (tBPolygon->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
            if (tBPolygon->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
            if (tBPolygon->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        }
        return false;
    };

    auto occupiedBySPort = [&] (size_t xId, size_t yId, size_t netId) -> bool {
        Polygon* sBPolygon = _db.vNet(netId)->sourcePort()->boundPolygon();
        if (sBPolygon->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
        if (sBPolygon->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
        if (sBPolygon->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
        if (sBPolygon->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        return false;
    };

    auto occupiedByTPort = [&] (size_t xId, size_t yId, size_t netId, size_t tPortId) -> bool {
        Polygon* tBPolygon = _db.vNet(netId)->targetPort(tPortId)->boundPolygon();
        if (netId == 2) {
            if (tBPolygon->minX() <= xId*_gridWidth && tBPolygon->maxX() >= xId*_gridWidth &&
                tBPolygon->minY() <= yId*_gridWidth && tBPolygon->maxY() >= yId*_gridWidth) return true;
            if (tBPolygon->minX() <= (xId+1)*_gridWidth && tBPolygon->maxX() >= (xId+1)*_gridWidth &&
                tBPolygon->minY() <= yId*_gridWidth && tBPolygon->maxY() >= yId*_gridWidth) return true;
            if (tBPolygon->minX() <= xId*_gridWidth && tBPolygon->maxX() >= xId*_gridWidth &&
                tBPolygon->minY() <= (yId+1)*_gridWidth && tBPolygon->maxY() >= (yId+1)*_gridWidth) return true;
            if (tBPolygon->minX() <= (xId+1)*_gridWidth && tBPolygon->maxX() >= (xId+1)*_gridWidth &&
                tBPolygon->minY() <= (yId+1)*_gridWidth && tBPolygon->maxY() >= (yId+1)*_gridWidth) return true;
        } else {
            if (tBPolygon->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
            if (tBPolygon->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
            if (tBPolygon->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
            if (tBPolygon->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        }
        return false;
    };

    auto occupiedByObstacle = [&] (size_t xId, size_t yId, size_t layId) -> bool {
        for (size_t obsId = 0; obsId < _db.numObstacles(layId); ++ obsId) {
            Shape* shape = _db.vObstacle(layId, obsId)->vShape(0);
            if (shape->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
            if (shape->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
            if (shape->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
            if (shape->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        }
        return false;
    };

    // init grids occupied by segments and ports
    for (size_t xId = 0; xId < _numXs; ++ xId) {
        for (size_t yId = 0; yId < _numYs; ++ yId) {
            // segments
            for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                Grid* grid = _vGrid[layId][xId][yId];
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    // if (occupiedByPorts(xId, yId, netId)) {
                    //     assert(!grid->hasNet(netId));
                    //     grid->addNet(netId);
                    //     grid->incCongestCur();
                    //     _vNetGrid[netId][layId].push_back(grid);
                    //     _vNetPortGrid[netId].push_back(grid);
                    // }
                    // if (occupiedBySegments(layId, xId, yId, netId) && !grid->hasNet(netId)) {
                    //     // assert(!grid->hasNet(netId));
                    //     grid->addNet(netId);
                    //     grid->incCongestCur();
                    //     _vNetGrid[netId][layId].push_back(grid);
                    // }
                    if (occupiedBySegments(layId, xId, yId, netId)) {
                        assert(!grid->hasNet(netId));
                        grid->addNet(netId);
                        grid->incCongestCur();
                        _vNetGrid[netId][layId].push_back(grid);
                    }
                }
            }
            // ports
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                if (occupiedBySPort(xId, yId, netId)) {
                    _vNetPortGrid[netId][0].push_back(make_pair(xId, yId));
                    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                        Grid* grid = _vGrid[layId][xId][yId];
                        if (! grid->hasNet(netId)) {
                            grid->addNet(netId);
                            grid->incCongestCur();
                            _vNetGrid[netId][layId].push_back(grid);
                        }
                    }
                }
                for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                    if (occupiedByTPort(xId, yId, netId, tPortId)) {
                        _vNetPortGrid[netId][tPortId+1].push_back(make_pair(xId, yId));
                        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                            Grid* grid = _vGrid[layId][xId][yId];
                            if (! grid->hasNet(netId)) {
                                grid->addNet(netId);
                                grid->incCongestCur();
                                _vNetGrid[netId][layId].push_back(grid);
                            }
                        }
                    }
                }
            }
        }
    }

    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                if (occupiedByObstacle(xId, yId, layId)) {
                    grid->incCongestCur();
                    grid->setObs();
                }
            }
        }
    }

    // init grids occupied by circular pad
    // for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
    //     Net* net = _db.vNet(netId);
    //     for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //         for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
    //             Segment* segment = net->vSegment(layId, segId);
    //             Circle* sCircle = new Circle(segment->sX(), segment->sY(), segment->width()/2, _plot);
    //             Circle* tCircle = new Circle(segment->tX(), segment->tY(), segment->width()/2, _plot);
    //             int sXId = floor(segment->sX() / _gridWidth);
    //             int sYId = floor(segment->sY() / _gridWidth);
    //             int tXId = floor(segment->tX() / _gridWidth);
    //             int tYId = floor(segment->tY() / _gridWidth);
    //             int boxWidth = ceil(segment->width() / _gridWidth);
    //             for (int xId = sXId - ceil(boxWidth); xId <= sXId + ceil(boxWidth); ++ xId) {
    //                 for (int yId = sYId - ceil(boxWidth); yId <= sYId + ceil(boxWidth); ++ yId) {
    //                     if (legal(xId, yId)) {
    //                         Grid* grid = _vGrid[layId][xId][yId];
    //                         if (!grid->hasNet(netId)) {
    //                             if (sCircle->Circle::enclose(xId*_gridWidth, yId*_gridWidth) ||
    //                                 sCircle->Circle::enclose((xId+1)*_gridWidth, yId*_gridWidth) ||
    //                                 sCircle->Circle::enclose(xId*_gridWidth, (yId+1)*_gridWidth) ||
    //                                 sCircle->Circle::enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //             for (int xId = tXId - ceil(boxWidth/2.0); xId <= tXId + ceil(boxWidth/2.0); ++ xId) {
    //                 for (int yId = tYId - ceil(boxWidth/2.0); yId <= tYId + ceil(boxWidth/2.0); ++ yId) {
    //                     if (legal(xId, yId)) {
    //                         Grid* grid = _vGrid[layId][xId][yId];
    //                         if (!grid->hasNet(netId)) {
    //                             if (tCircle->enclose(xId*_gridWidth, yId*_gridWidth) ||
    //                                 tCircle->enclose((xId+1)*_gridWidth, yId*_gridWidth) ||
    //                                 tCircle->enclose(xId*_gridWidth, (yId+1)*_gridWidth) ||
    //                                 tCircle->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }    
    // }
    
    // init grids occupied by squared pads
    // for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
    //     Net* net = _db.vNet(netId);
    //     for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //         for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
    //             Segment* segment = net->vSegment(layId, segId);
    //             int sXId = floor(segment->sX() / _gridWidth);
    //             int sYId = floor(segment->sY() / _gridWidth);
    //             int tXId = floor(segment->tX() / _gridWidth);
    //             int tYId = floor(segment->tY() / _gridWidth);
    //             int boxWidth = ceil(segment->width() / _gridWidth);
    //             for (int xId = sXId - floor(boxWidth/2.0); xId <= sXId + floor(boxWidth/2.0); ++ xId) {
    //                 if (xId >= 0 && xId < _vGrid[0].size()) {
    //                     for (int yId = sYId - floor(boxWidth/2.0); yId <= sYId + floor(boxWidth/2.0); ++ yId) {
    //                         if (yId >= 0 && yId < _vGrid[0][0].size()) {
    //                             Grid* grid = _vGrid[layId][xId][yId];
    //                             if (!grid->hasNet(netId)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //             for (int xId = tXId - floor(boxWidth/2.0); xId <= tXId + floor(boxWidth/2.0); ++ xId) {
    //                 if (xId >= 0 && xId < _vGrid[0].size()) {
    //                     for (int yId = tYId - floor(boxWidth/2.0); yId <= tYId + floor(boxWidth/2.0); ++ yId) {
    //                         if (yId >= 0 && yId < _vGrid[0][0].size()) {
    //                             Grid* grid = _vGrid[layId][xId][yId];
    //                             if (!grid->hasNet(netId)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
}

void DetailedMgr::plotGridMap() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                vector< pair<double, double> > vVtx;
                vVtx.push_back(make_pair(xId*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, (yId+1)*_gridWidth));
                vVtx.push_back(make_pair(xId*_gridWidth, (yId+1)*_gridWidth));
                Polygon* p = new Polygon(vVtx, _plot);
                if (grid->congestCur() == 0) {
                    p->plot(SVGPlotColor::white, layId);
                } else if (grid->hasObs()) {
                    p->plot(SVGPlotColor::gray, layId);
                } else {
                    for (size_t netId = 0; netId < grid->numNets(); ++ netId) {
                        p->plot(grid->vNetId(netId), layId);
                    }
                }
            }
        }
    }
    int area = 0;
    int overlapArea = 0;
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                if (grid->congestCur() >= 1) area++;
                overlapArea += grid->congestCur();
            }
        }
    }
    overlapArea -= area;
    cerr << "area = " << area << endl;
    cerr << "overlapArea = " << overlapArea << endl;
}

void DetailedMgr::plotGridMapVoltage() {
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        // cerr << "netId = " << netId << endl;
        double ubVolt = _db.vNet(netId)->sourcePort()->voltage();
        // double lbVolt = _db.vNet(netId)->targetPort(0)->voltage();
        // for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
        //     if (_db.vNet(netId)->targetPort(tPortId)->voltage() < lbVolt) {
        //         lbVolt = _db.vNet(netId)->targetPort(tPortId)->voltage();
        //     }
        // }
        double lbVolt = ubVolt * 0.9;
        _plot.setColorValueRange(lbVolt, ubVolt);
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            // cerr << " layId = " << layId << endl;
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid = _vNetGrid[netId][layId][gridId];
                // cerr << "   voltage = " << grid->voltage() << endl;
                Square* square = new Square(grid->xId()*_gridWidth, grid->yId()*_gridWidth, _gridWidth, _plot);
                square->plotValue(grid->voltage(netId), layId);
            }
        }
    }
}

void DetailedMgr::plotGridMapCurrent() {
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
        // cerr << "netId = " << netId << endl;
        // double ubCurr = _db.vNet(netId)->sourcePort()->voltage() * via_condutance;
        double ubCurr = 6;
        // double lbVolt = _db.vNet(netId)->targetPort(0)->voltage();
        // for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
        //     if (_db.vNet(netId)->targetPort(tPortId)->voltage() < lbVolt) {
        //         lbVolt = _db.vNet(netId)->targetPort(tPortId)->voltage();
        //     }
        // }
        double lbCurr = 0;
        _plot.setColorValueRange(lbCurr, ubCurr);
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            // cerr << " layId = " << layId << endl;
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid = _vNetGrid[netId][layId][gridId];
                // cerr << "   voltage = " << grid->voltage() << endl;
                Square* square = new Square(grid->xId()*_gridWidth, grid->yId()*_gridWidth, _gridWidth, _plot);
                square->plotValue(grid->current(netId), layId);
            }
        }
    }
}

void DetailedMgr::naiveAStar() {
    // cerr << "naiveAStar..." << endl;
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        // cerr << "layId = " << layId;
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            // cerr << " netId = " << netId << endl;
    // size_t layId = 0;
    // size_t netId = 0;
            Net* net = _db.vNet(netId);
            clearNet(layId, netId);
            for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
                Segment* segment = net->vSegment(layId, segId);
                if (segment->width() > 0) {
                    int sXId = floor(segment->trace()->sNode()->ctrX() / _gridWidth);
                    int sYId = floor(segment->trace()->sNode()->ctrY() / _gridWidth);
                    int tXId = floor(segment->trace()->tNode()->ctrX() / _gridWidth);
                    int tYId = floor(segment->trace()->tNode()->ctrY() / _gridWidth);
                    int sRealXId = floor(segment->sX() / _gridWidth);
                    int sRealYId = floor(segment->sY() / _gridWidth);
                    int tRealXId = floor(segment->tX() / _gridWidth);
                    int tRealYId = floor(segment->tY() / _gridWidth);
                    AStarRouter router(_vGrid[layId], make_pair(sXId, sYId), make_pair(tXId, tYId), make_pair(sRealXId, sRealYId), make_pair(tRealXId, tRealYId), 
                                       _gridWidth, segment->length(), segment->width(), 0.9, _db.numNets() * 10.0, 0.2);
                    router.route();
                    segment->setWidth(router.exactWidth() * _gridWidth);
                    segment->setLength(router.exactLength() * _gridWidth);
                    for (size_t pGridId = 0; pGridId < router.numPGrids(); ++ pGridId) {
                        Grid* grid = router.vPGrid(pGridId);
                        if (!grid->hasNet(netId)) {
                            _vNetGrid[netId][layId].push_back(grid);
                            grid->addNet(netId);
                        }
                    }
                    // _vNetGrid[netId][layId].insert(_vNetGrid[netId][layId].end(), router.path().begin(), router.path().end());
                }
            }
            // for (size_t gridId = 0; gridId < _vNetPortGrid[netId].size(); ++ gridId) {
            //     if (! _vNetPortGrid[netId][gridId]->hasNet(netId)) {
            //         _vNetGrid[netId][layId].push_back(_vNetPortGrid[netId][gridId]);
            //         _vNetPortGrid[netId][gridId]->addNet(netId);
            //     }
            // }
            for (size_t portId = 0; portId < _db.vNet(netId)->numTPorts()+1; ++ portId) {
                for (size_t gridId = 0; gridId < _vNetPortGrid[netId][portId].size(); ++ gridId) {
                    Grid* grid = _vGrid[layId][_vNetPortGrid[netId][portId][gridId].first][_vNetPortGrid[netId][portId][gridId].second];
                    if (! grid->hasNet(netId)) {
                        _vNetGrid[netId][layId].push_back(grid);
                        grid->addNet(netId);
                    }
                }
            }
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
                _vNetGrid[netId][layId][gridId]->incCongestCur();
            }
        }
    }
    // int area = 0;
    // int overlapArea = 0;
    // for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //     for (size_t xId = 0; xId < _numXs; ++ xId) {
    //         for (size_t yId = 0; yId < _numYs; ++ yId) {
    //             Grid* grid = _vGrid[layId][xId][yId];
    //             if (grid->congestCur() >= 1) area++;
    //             overlapArea += grid->congestCur();
    //         }
    //     }
    // }
    // overlapArea -= area;
    // cerr << "area = " << area << endl;
    // cerr << "overlapArea = " << overlapArea << endl;
}

void DetailedMgr::clearNet(size_t layId, size_t netId) {
    for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
        _vNetGrid[netId][layId][gridId]->removeNet(netId);
        _vNetGrid[netId][layId][gridId]->decCongestCur();
    }
    _vNetGrid[netId][layId].clear();
}

void DetailedMgr::addPortVia() {
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        Port* sPort = _db.vNet(netId)->sourcePort();
        if (sPort->viaArea() > 0) {
            int numSVias = ceil(sPort->viaArea() / _db.VIA16D8A24()->metalArea());
            assert(numSVias > 1);
            vector< pair<double, double> > centPos = kMeansClustering(_vNetPortGrid[netId][0], numSVias, 100);
            assert(centPos.size() == numSVias);
            vector<size_t> vViaId(centPos.size(), 0);
            for (size_t viaId = 0; viaId < centPos.size(); ++ viaId) {
                vViaId[viaId] = _db.addVia(centPos[viaId].first, centPos[viaId].second, netId, ViaType::Source);
            }
            sPort->setViaCluster(_db.clusterVia(vViaId));
        } else {
            cerr << "WARNNING: net" << netId << " sPort has no via!" << endl;
            ViaCluster* viaCluster = new ViaCluster();
            sPort->setViaCluster(viaCluster);
        }
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            Port* tPort = _db.vNet(netId)->targetPort(tPortId);
            if (tPort->viaArea() > 0) {
                int numTVias = ceil(tPort->viaArea() / _db.VIA16D8A24()->metalArea());
                assert(numTVias > 1);
                vector< pair<double, double> > centPosT = kMeansClustering(_vNetPortGrid[netId][tPortId+1], numTVias, 100);
                assert(centPosT.size() == numTVias);
                vector<size_t> vViaIdT(centPosT.size(), 0);
                for (size_t viaIdT = 0; viaIdT < centPosT.size(); ++ viaIdT) {
                    vViaIdT[viaIdT] = _db.addVia(centPosT[viaIdT].first, centPosT[viaIdT].second, netId, ViaType::Target);
                }
                tPort->setViaCluster(_db.clusterVia(vViaIdT));
            } else {
                cerr << "WARNNING: net" << netId << " tPort" << tPortId << " has no via!" << endl;
                ViaCluster* viaCluster = new ViaCluster();
                tPort->setViaCluster(viaCluster);
            }
        }
    }

}

vector< pair<double, double> > DetailedMgr::kMeansClustering(vector< pair<int,int> > vGrid, int numClusters, int numEpochs) {
    assert(numClusters <= vGrid.size());
    // if (numClusters == vGrid.size()) {
    //     return vGrid;
    // }
    struct Point {
        double x, y;     // coordinates
        int cluster;     // no default cluster
        double minDist;  // default infinite dist to nearest cluster
    };
    auto distance = [] (Point p1, Point p2) -> double {
        return pow((p1.x-p2.x), 2) + pow((p1.y-p2.y), 2);
    }; 

    vector<Point> points;
    for (size_t gridId = 0; gridId < vGrid.size(); ++ gridId) {
        Point p = {(vGrid[gridId].first+0.5)*_gridWidth, (vGrid[gridId].second+0.5)*_gridWidth, -1, numeric_limits<double>::max()};
        points.push_back(p);
    }

    // vector<bool> isCentroid(points.size(), false);
    vector<Point> centroids;
    // srand(time(0));  // need to set the random seed
    // for (int i = 0; i < numClusters; ++i) {
    //     int pointId = rand() % points.size();
    //     while(isCentroid[pointId]) {
    //         pointId = rand() % points.size();
    //     }
    //     centroids.push_back(points.at(pointId));
    //     isCentroid[pointId] = true;
    //     // cerr << "centroid: (" << centroids[i].x << ", " << centroids[i].y << ")" << endl;
    // }
    for (int i = 0; i < numClusters; ++i) {
        centroids.push_back(points.at(i));
    }

    // vector<int> nPoints(k,0);
    // vector<double> sumX(k,0.0);
    // vector<double> sumY(k,0.0);
    int* nPoints = new int[numClusters];
    double* sumX = new double[numClusters];
    double* sumY = new double[numClusters];
    for (size_t epoch = 0; epoch < numEpochs; ++ epoch) {
        for (size_t centId = 0; centId < centroids.size(); ++centId) {
            // quick hack to get cluster index
            // Point c = centroids[centId];
            int clusterId = centId;

            for (size_t pointId = 0; pointId < points.size(); ++ pointId) {
                // Point p = points[pointId];
                double dist = distance(centroids[centId],points[pointId]);
                if (dist < points[pointId].minDist) {
                    points[pointId].minDist = dist;
                    points[pointId].cluster = clusterId;
                    // cerr << "p.cluster = " << points[pointId].cluster << endl;
                }
                // *it = p;
            }
        }

        
        // sumX.clear();
        
        // sumY.clear();
        // // Initialise with zeroes
        // for (int j = 0; j < k; ++j) {
        //     nPoints.push_back(0);
        //     sumX.push_back(0.0);
        //     sumY.push_back(0.0);
        // }
        for (size_t centId=0; centId<centroids.size(); ++centId) {
            nPoints[centId] = 0;
            sumX[centId] = 0.0;
            sumY[centId] = 0.0;
        }
        // Iterate over points to append data to centroids
        // for (vector<Point>::iterator it = points.begin(); it != points.end(); ++it) {
            // int clusterId = it->cluster;
        for (size_t pointId = 0; pointId < points.size(); ++ pointId) {
            int clusterId = points[pointId].cluster;
            nPoints[clusterId] += 1;
            sumX[clusterId] += points[pointId].x;
            sumY[clusterId] += points[pointId].y;
            // cerr << "sumX" << clusterId << ": " << sumX[clusterId] << endl;
            // cerr << "sumY" << clusterId << ": " << sumY[clusterId] << endl;
            // cerr << "nPoints" << clusterId << ": " << nPoints[clusterId] << endl;
            points[pointId].minDist = numeric_limits<double>::max();  // reset distance
        }

        // Compute the new centroids
        // cerr << "Compute the new centroids" << endl;
        for (size_t centId = 0; centId < centroids.size(); ++ centId) {
            int clusterId = centId;
            centroids[centId].x = sumX[clusterId] / nPoints[clusterId];
            centroids[centId].y = sumY[clusterId] / nPoints[clusterId];
            // cerr << "sumX" << clusterId << ": " << sumX[clusterId] << endl;
            // cerr << "sumY" << clusterId << ": " << sumY[clusterId] << endl;
            // cerr << "nPoints" << clusterId << ": " << nPoints[clusterId] << endl;
            // cerr << "centroid: (" << centroids[centId].x << ", " << centroids[centId].y << ")" << endl;
        }
    }

    // for (size_t centId = 0; centId < centroids.size(); ++ centId) {
    //     int centX = floor(centroids[centId].x);
    //     int centY = floor(centroids[centId].y);
    //     _db.addVia(centroids[centId].x, centroids[centId].y, )
    // }

    // for (size_t pointId = 0; pointId < points.size(); ++ pointId) {
    //     Point p = points[pointId];
    //     for (size_t tPortId = 0; tPortId < k; ++ tPortId) {
    //         if (p.cluster == tPortId) {
    //             // cerr << "tPortId = " << tPortId << endl;
    //             // cerr << "   node = " << p.node->name() << endl;
    //             _vTClusteredNode[netId][tPortId].push_back(p.node);
    //         }
    //     }
    // }

    vector< pair<double, double> > centPos;
    for (size_t centId = 0; centId < centroids.size(); ++ centId) {
        centPos.push_back(make_pair(centroids[centId].x, centroids[centId].y));
    }
    return centPos;
}

void DetailedMgr::plotVia() {
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        ViaCluster* sViaCstr = _db.vNet(netId)->sourcePort()->viaCluster();    
        for (size_t viaId = 0; viaId < sViaCstr->numVias(); ++ viaId) {
            sViaCstr->vVia(viaId)->shape()->plot(netId, 0);
        }
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            ViaCluster* tViaCstr = _db.vNet(netId)->targetPort(tPortId)->viaCluster();    
            for (size_t viaId = 0; viaId < tViaCstr->numVias(); ++ viaId) {
                tViaCstr->vVia(viaId)->shape()->plot(netId, 0);
            }
        }
    }
}

void DetailedMgr::addViaGrid() {
    auto gridEnclose = [&] (Grid* grid, double x, double y) -> bool {
        double gridLX = grid->xId() * _gridWidth;
        double gridUX = (grid->xId()+1) * _gridWidth;
        double gridLY = grid->yId() * _gridWidth;
        double gridUY = (grid->yId()+1) * _gridWidth;
        // to avoid a via enclosed by multiple grids, set ">=" but "<" only
        return ((x >= gridLX) && (x < gridUX) && (y >= gridLY) && (y < gridUY));
    };
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    if (!grid->hasNet(netId)) {
                        for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
                            double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
                            double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
                            if (gridEnclose(grid, sX, sY)) {
                                _vNetGrid[netId][layId].push_back(grid);
                                grid->addNet(netId);
                            }
                        }
                        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                            for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
                                double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
                                double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
                                if (gridEnclose(grid, tX, tY)) {
                                    _vNetGrid[netId][layId].push_back(grid);
                                    grid->addNet(netId);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void DetailedMgr::buildMtx() {
    // https://i.imgur.com/rIwlXJQ.png
    // return an impedance matrix for each net
    // number of nodes: \sum_{layId=0}^{_vNetGrid[netID].size()} _vNetGrid[netID][layId].size()

    auto gridEnclose = [&] (Grid* grid, double x, double y) -> bool {
        double gridLX = grid->xId() * _gridWidth;
        double gridUX = (grid->xId()+1) * _gridWidth;
        double gridLY = grid->yId() * _gridWidth;
        double gridUY = (grid->yId()+1) * _gridWidth;
        // to avoid a via enclosed by multiple grids, set ">=" but "<" only
        return ((x >= gridLX) && (x < gridUX) && (y >= gridLY) && (y < gridUY));
    };

    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        printf("netID: %d\n", netId);
        
        size_t numNode = 0;
        map< tuple<size_t, size_t, size_t>, size_t > getID; // i = getID[layID, xId, yId] = ith node
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                getID[make_tuple(layId, _vNetGrid[netId][layId][gridId]->xId(), _vNetGrid[netId][layId][gridId]->yId())] = numNode;
                numNode++;
            }
        }
        printf("numNode: %d\n", numNode);

        // initialize matrix and vector
        Eigen::SparseMatrix<double, Eigen::RowMajor> Y(numNode, numNode);
        Eigen::VectorXd I(numNode);
        Eigen::VectorXd V(numNode);
        vector< Eigen::Triplet<double> > vTplY;
        vTplY.reserve(6 * numNode);
        for (size_t i = 0; i < numNode; ++ i) {
            I[i] = 0;
            V[i] = 0;
        }

        // // initialize
        // vector< vector<double > > mtx;
        // // numNode += (_db.vNet(netId)->numTPorts() + 1) * _db.numLayers();    // add the source/target via nodes on each layer
        // for(int i=0; i<numNode; i++) {
        //     mtx.push_back(vector<double>());
        //     for(int j=0; j<numNode; j++)
        //         mtx[i].push_back(0.0);
        // }
        assert(_vNetGrid[netId].size() == _db.numLayers());
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            int numSVias = 0;
            vector<int> numTVias(_db.vNet(netId)->numTPorts(), 0);
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                // cerr << "grid = (" << grid_i->xId() << " " << grid_i->yId() << ")" << endl;
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
            //    printf("x: %-4d, y: %-4d, lay: %-4d, ID: %-4d\n", i->xId(), i->yId(), layId, getID[make_tuple(layId, i->xId(), i->yId())]);
            
                double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
                double via_condutance_up, via_condutance_down;
                if (layId > 0) {
                    via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
                }
                if (layId < _db.numLayers() - 1) {
                    via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
                }
                double small_conductance = 1.0;
                // if (gridId == 0) {
                //     cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
                //     cerr << ", via_conductance_down = " << via_condutance_down << endl;
                // }

                // check left
                if(grid_i->xId() > 0 && _vGrid[layId][grid_i->xId()-1][grid_i->yId()]->hasNet(netId)) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())], -g2g_condutance));
                }

                // check right
                if(grid_i->xId() < _numXs-1 && _vGrid[layId][grid_i->xId()+1][grid_i->yId()]->hasNet(netId)) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())], -g2g_condutance));
                }
                
                // check down
                if(grid_i->yId() > 0 && _vGrid[layId][grid_i->xId()][grid_i->yId()-1]->hasNet(netId)) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)], -g2g_condutance));
                }
                
                // check up
                if(grid_i->yId() < _numYs-1 && _vGrid[layId][grid_i->xId()][grid_i->yId()+1]->hasNet(netId)) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)], -g2g_condutance));
                }

                // // check top layer
                // if(layId > 0 && _vGrid[layId-1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
                //     mtx[node_id][node_id] += via_condutance;
                //     mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                // }

                // // check bottom layer
                // if(layId < _db.numLayers()-1 && _vGrid[layId+1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
                //     mtx[node_id][node_id] += via_condutance;
                //     mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                // }

                for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
                    double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
                    double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
                    if (gridEnclose(grid_i, sX, sY)) {
                        numSVias ++;
                        // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
                        if (layId > 0) {
                            // mtx[node_id][node_id] += via_condutance;
                            // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                            vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                        } else {
                            // if (netId != 1) {
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                            I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * via_condutance_up;
                            // }
                            // cerr << "layer" << layId << " node" << node_id;
                            // cerr << ": sVolt = " << _db.vNet(netId)->sourcePort()->voltage();
                            // cerr << ", via_conductance_up = " << via_condutance_up;
                            // cerr << ", I" << node_id << " = " << I(node_id) << endl;
                        }
                        if (layId < _db.numLayers()-1) {
                            // mtx[node_id][node_id] += via_condutance;
                            // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                            vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
                        }
                    }
                }
                for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                    for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
                        double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
                        double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
                        if (gridEnclose(grid_i, tX, tY)) {
                            numTVias[tPortId] ++;
                            if (layId > 0) {
                                // mtx[node_id][node_id] += via_condutance;
                                // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                                vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                            } else {
                                double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
                                //  * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias()
                                // if (netId != 1) {
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_up + 1.0/loadConductance)));
                                // } else {
                                //     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_down + 1.0/loadConductance)));
                                // }
                                // cerr << "layer" << layId << " node" << node_id;
                                // cerr << ": tPort" << tPortId ;
                                // cerr << ", total conductance = " << 1.0/(1.0/via_condutance_up + 1.0/loadConductance) << endl;
                                // << ": via_conductance = " << via_condutance_up << ", loadConductance = " << loadConductance;
                            }
                            if (layId < _db.numLayers()-1) {
                                // mtx[node_id][node_id] += via_condutance;
                                // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                                vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
                            } 
                        }
                    }
                }
            }
            // Port* sPort = _db.vNet(netId)->sourcePort();
            // int realNumSVias = sPort->viaCluster()->numVias();
            // cerr << "layer" << layId << ": numSVias = " << numSVias << ", real numSVias = " << realNumSVias << endl;
            // for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            //     Port* tPort = _db.vNet(netId)->targetPort(tPortId);
            //     int realNumTVias = tPort->viaCluster()->numVias();
            //     cerr << "layer" << layId << " tPort" << tPortId << ": numTVias = " << numTVias[tPortId] << ", real numTVias = " << realNumTVias << endl;
            // }
        }

        // if (netId == 1) {
        //     cerr << "net1, I = " << endl;
        //     for (size_t i = 0; i < I.size(); ++ i) {
        //         if (I[i] > 0) {
        //             cerr << "I[" << i << "] = " << I[i] << endl;
        //         }
        //     }
        // }

        Y.setFromTriplets(vTplY.begin(), vTplY.end());
        // Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IdentityPreconditioner> solver;
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
        // solver.setMaxIterations(1000000);
        // solver.setTolerance(1e-14);
        solver.compute(Y);
        // V = solver.solveWithGuess(I, V);
        V = solver.solve(I);
        assert(solver.info() == Eigen::Success);

        // set voltage of each grid
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
                grid_i->setVoltage(netId, V[node_id]);
                assert(grid_i->voltage(netId) <= _db.vNet(netId)->sourcePort()->voltage());
            }
        }

        // set current of each grid
        assert(_vNetGrid[netId].size() == _db.numLayers());
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                size_t xId = grid_i->xId();
                size_t yId = grid_i->yId();
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
                double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
                double via_condutance_up, via_condutance_down;
                if (layId > 0) {
                    via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
                }
                if (layId < _db.numLayers() - 1) {
                    via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
                }
                if (gridId == 0) {
                    // cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
                    // cerr << ", via_conductance_down = " << via_condutance_down << endl;
                }
                double small_conductance = 1.0;
                double current = 0;
                size_t nbrId;
                if (legal(xId+1, yId)) {
                    if (_vGrid[layId][xId+1][yId]->hasNet(netId)) {
                        current += abs(grid_i->voltage(netId) - _vGrid[layId][xId+1][yId]->voltage(netId)) * g2g_condutance;
                    }
                }
                if (legal(xId-1, yId)) {
                    if (_vGrid[layId][xId-1][yId]->hasNet(netId)) {
                        current += abs(grid_i->voltage(netId) - _vGrid[layId][xId-1][yId]->voltage(netId)) * g2g_condutance;
                    }
                }
                if (legal(xId, yId+1)) {
                    if (_vGrid[layId][xId][yId+1]->hasNet(netId)) {
                        current += abs(grid_i->voltage(netId) - _vGrid[layId][xId][yId+1]->voltage(netId)) * g2g_condutance;
                    }
                }
                if (legal(xId, yId-1)) {
                    if (_vGrid[layId][xId][yId-1]->hasNet(netId)) {
                        current += abs(grid_i->voltage(netId) - _vGrid[layId][xId][yId-1]->voltage(netId)) * g2g_condutance;
                    }
                }
                // via current
                for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
                    double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
                    double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
                    if (gridEnclose(grid_i, sX, sY)) {
                        // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
                        if (layId > 0) {
                            current += abs(grid_i->voltage(netId) - _vGrid[layId-1][xId][yId]->voltage(netId)) * via_condutance_down;
                        } else {
                            current += abs(grid_i->voltage(netId) - _db.vNet(netId)->sourcePort()->voltage()) * via_condutance_up;
                        }
                        if (layId < _db.numLayers()-1) {
                            current += abs(grid_i->voltage(netId) - _vGrid[layId+1][xId][yId]->voltage(netId)) * via_condutance_up;
                        }
                    }
                }
                for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                    // double tPortCurr = 0;
                    double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage()* _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
                    for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
                        double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
                        double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
                        if (gridEnclose(grid_i, tX, tY)) {
                            if (layId > 0) {
                                current += abs(grid_i->voltage(netId) - _vGrid[layId-1][xId][yId]->voltage(netId)) * via_condutance_down;
                            } else {
                                current += abs(grid_i->voltage(netId)) /(1.0/via_condutance_up + 1.0/loadConductance);
                                _vTPortCurr[netId][tPortId] += abs(grid_i->voltage(netId)) /(1.0/via_condutance_up + 1.0/loadConductance);
                                cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage(netId);
                                cerr << ", current = " << abs(grid_i->voltage(netId)) /(1.0/via_condutance_up + 1.0/loadConductance) << endl;
                            }
                            if (layId < _db.numLayers()-1) {
                                current += abs(grid_i->voltage(netId) - _vGrid[layId+1][xId][yId]->voltage(netId)) * via_condutance_up;
                            } 
                        }
                    }
                    // _vTPortCurr[netId][tPortId] += tPortCurr;
                    // _vTPortCurr[netId].push_back(tPortCurr);
                    // _vTPortVolt[netId].push_back(tPortCurr / loadConductance);
                    // cerr << "tPortCurr = " << tPortCurr << ", tPortVolt = " << _vTPortVolt[netId][tPortId] << endl;
                }
                grid_i->setCurrent(netId, current * 0.5);
                // cerr << "gridCurrent = " << current * 0.5 << endl;
            }
        }
        // for(int i=0; i<20; i++) {
        //     for(int j=0; j<20; j++)
        //         printf("%4.1f ", mtx[i][j]);
        //     printf("\n");
        // }
    }

    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            double loadResistance = _db.vNet(netId)->targetPort(tPortId)->voltage() / _db.vNet(netId)->targetPort(tPortId)->current();
            cerr << "net" << netId << " tPort" << tPortId << ": current = " << _vTPortCurr[netId][tPortId];
            cerr << ", voltage = " << _vTPortCurr[netId][tPortId] * loadResistance << endl;
        }
    }
}

double DetailedMgr::getResistance(Grid* g1, Grid* g2) {
    return 0.0;
}

void DetailedMgr::check() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
                assert(_vNetGrid[netId][layId][gridId]->hasNet(netId));
                for (size_t gridId1 = gridId+1; gridId1 < _vNetGrid[netId][layId].size(); ++ gridId1) {
                    assert(_vNetGrid[netId][layId][gridId] != _vNetGrid[netId][layId][gridId1]);
                }
            }
        }
    }
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId <_numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    if (grid->hasNet(netId)) {
                        bool inVNetGrid = false;
                        for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
                            if (_vNetGrid[netId][layId][gridId] == grid) {
                                inVNetGrid = true;
                            }
                        }
                        assert(inVNetGrid);
                    }
                }
            }
        }
    }
}

void DetailedMgr::writeColorMap(const char* path, bool isVoltage) {

/*
    for(int i=0; i<_vGrid.size(); i++) {
        for(int j=0; j<_vGrid[i].size(); j++) {
            for(int k=0; k<_vGrid[i][j].size(); k++) {
                _vGrid[i][j][k]->setCurrent((double)rand()/RAND_MAX);
                _vGrid[i][j][k]->setVoltage((double)rand()/RAND_MAX);
                _vGrid[i][j][k]->setVoltage(k);
            }
        }
    }
*/
    FILE *fp = fopen(path, "w");

    fprintf(fp, "%d\n", _db.numNets());
    fprintf(fp, "%d\n", _db.numLayers());
    fprintf(fp, "%d\n", _numXs);
    fprintf(fp, "%d\n", _numYs);
/*
    fprintf(fp, "%d\n\n", _db.numVias());

    for(int i=0; i<_db.numVias(); i++)
        fprintf(fp, "%f %f %f\n", (double)_db.vVia(i)->shape()->ctrX() / _gridWidth, (double)_db.vVia(i)->shape()->ctrY() / _gridWidth, (double)_db.vVia(i)->shape()->radius() / _gridWidth);
*/
    fprintf(fp, "\n");

    for(int layId=0; layId<_db.numLayers(); layId++) {

        for(int netId=0; netId<_db.numNets(); netId++) {
            double **mtx = new double *[_numXs];
            for(int xId=0; xId<_numXs; xId++)
                mtx[xId] = new double [_numYs];
            /*    
            for(int lay=0; lay<_db.numLayers(); lay++) {
                mtx[lay] = new double *[_numXs];
                for(int x=0; x<_numXs; x++) {
                    mtx[lay][x] = new double [_numYs];
                }
            }
            */

            for(int gridId=0; gridId<_vNetGrid[netId][layId].size(); gridId++) {
                int x = _vNetGrid[netId][layId][gridId]->xId(), y = _vNetGrid[netId][layId][gridId]->yId();
                mtx[x][y] = isVoltage? _vGrid[layId][x][y]->voltage(netId): _vGrid[layId][x][y]->current(netId);
            }

            // fprintf(fp, "lay: %d, net: %d\n", layId, netId);
            for(int xId=0; xId<_numXs; xId++) {
                for(int yId=0; yId<_numYs; yId++)
                    fprintf(fp, "%.4f ", mtx[xId][yId]);
                fprintf(fp, "\n");
            }
            fprintf(fp, "\n");

            for(int xId=0; xId<_numXs; xId++)
                delete [] mtx[xId];
            delete [] mtx;
            /*
            for(int lay=0; lay<_db.numNets(); lay++) {
                for(int x=0; x<_vGrid[lay].size(); x++) {
                    delete [] mtx[lay][x];
                }
                delete [] mtx[lay];
            }
            delete [] mtx;
            */
        }
    }
    /*
    for(int net=0; net<_db.numNets(); net++) {
        double ***mtx = new double **[_db.numLayers()];
        for(int lay=0; lay<_db.numLayers(); lay++) {
            mtx[lay] = new double *[_numXs];
            for(int x=0; x<_numXs; x++) {
                mtx[lay][x] = new double [_numYs];
            }
        }

        for(int lay=0; lay<_vGrid.size(); lay++) {
            for(int x=0; x<_vGrid[lay].size(); x++) {
                for(int y=0; y<_vGrid[lay][x].size(); y++) {
                    fprintf(fp, "%.4f ", mtx[lay][x][y]);
                }
                fprintf(fp, "\n");
            }
            fprintf(fp, "\n");
        }

        for(int lay=0; lay<_db.numNets(); lay++) {
            for(int x=0; x<_vGrid[lay].size(); x++) {
                delete [] mtx[lay][x];
            }
            delete [] mtx[lay];
        }
        delete [] mtx;
    }
    */
/*
    for(int net=0; net<_db.numNets(); net++) {
        for(int lay=0; lay<_vGrid.size(); lay++) {
            for(int x=0; x<_vGrid[lay].size(); x++) {
                for(int y=0; y<_vGrid[lay][x].size(); y++) {
                    fprintf(fp, "%.4f ", isVoltage? _vGrid[lay][x][y]->voltage(net): _vGrid[lay][x][y]->current(net));
                }
                fprintf(fp, "\n");
            }
            fprintf(fp, "\n");
        }
    }
*/    
    printf("--- finish write color map ---\n");
    fclose(fp);
}

void DetailedMgr::writeColorMap_v2(const char* path, bool isVoltage) {

    vector<double> valList;
    valList.clear();
    for(int layId=0; layId<_db.numLayers(); layId++) {
        for(int netId=0; netId<_db.numNets(); netId++) {
            for(int gridId=0; gridId<_vNetGrid[netId][layId].size(); gridId++) {
                int x = _vNetGrid[netId][layId][gridId]->xId(), y = _vNetGrid[netId][layId][gridId]->yId();
                valList.push_back(isVoltage? _vGrid[layId][x][y]->voltage(netId): _vGrid[layId][x][y]->current(netId));
            }
        }
    }
    sort(valList.begin(), valList.end());

    

    FILE *fp = fopen(path, "w");

    fprintf(fp, "%d\n", _db.numNets());
    fprintf(fp, "%d\n", _db.numLayers());
    fprintf(fp, "%d\n", _numXs);
    fprintf(fp, "%d\n", _numYs);
    
    sort(valList.begin(), valList.end());
    fprintf(fp, "%16.10f %16.10f\n", valList[(int)(0.01*valList.size())], valList[(int)(0.99*valList.size())]);

    fprintf(fp, "%d\n\n", _db.numVias());
    for(int i=0; i<_db.numVias(); i++)
        fprintf(fp, "%13.6f %13.6f %13.6f\n", (double)_db.vVia(i)->shape()->ctrX() / _gridWidth, (double)_db.vVia(i)->shape()->ctrY() / _gridWidth, (double)_db.vVia(i)->drillRadius() / _gridWidth);
    
    fprintf(fp, "\n");

    for(int layId=0; layId<_db.numLayers(); layId++) {

        for(int netId=0; netId<_db.numNets(); netId++) {
            
            // fprintf(fp, "lay: %d, net: %d\n", layId, netId);
            fprintf(fp, "%d\n", _vNetGrid[netId][layId].size());
            for(int gridId=0; gridId<_vNetGrid[netId][layId].size(); gridId++) {
                int x = _vNetGrid[netId][layId][gridId]->xId(), y = _vNetGrid[netId][layId][gridId]->yId();
                fprintf(fp, "%4d %4d %18.12f\n", x, y, isVoltage? _vGrid[layId][x][y]->voltage(netId): _vGrid[layId][x][y]->current(netId));
            }
            fprintf(fp, "\n");
        }
    }

    printf("--- finish write color map ---\n");
    fclose(fp);
}