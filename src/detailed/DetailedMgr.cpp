#include "DetailedMgr.h"

void DetailedMgr::initGridMap() {
    auto occupy = [&] (size_t layId, size_t xId, size_t yId, size_t netId) -> bool {
        for (size_t segId = 0; segId < _db.vNet(netId)->numSegments(layId); ++ segId) {
            Trace* trace = _db.vNet(netId)->vSegment(layId, segId)->trace();
            if (trace->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
            if (trace->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
            if (trace->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
            if (trace->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        }
        return false;
    };

    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    if (occupy(layId, xId, yId, netId)) {
                        grid->addNet(netId);
                        grid->incCongestCur();
                        _vNetGrid[netId][layId].push_back(grid);
                    }
                }
            }
        }
    }

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
    
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        Net* net = _db.vNet(netId);
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
                Segment* segment = net->vSegment(layId, segId);
                int sXId = floor(segment->sX() / _gridWidth);
                int sYId = floor(segment->sY() / _gridWidth);
                int tXId = floor(segment->tX() / _gridWidth);
                int tYId = floor(segment->tY() / _gridWidth);
                int boxWidth = ceil(segment->width() / _gridWidth);
                for (int xId = sXId - floor(boxWidth/2.0); xId <= sXId + floor(boxWidth/2.0); ++ xId) {
                    if (xId >= 0 && xId < _vGrid[0].size()) {
                        for (int yId = sYId - floor(boxWidth/2.0); yId <= sYId + floor(boxWidth/2.0); ++ yId) {
                            if (yId >= 0 && yId < _vGrid[0][0].size()) {
                                Grid* grid = _vGrid[layId][xId][yId];
                                if (!grid->hasNet(netId)) {
                                    grid->addNet(netId);
                                    grid->incCongestCur();
                                    _vNetGrid[netId][layId].push_back(grid);
                                }
                            }
                        }
                    }
                }
                for (int xId = tXId - floor(boxWidth/2.0); xId <= tXId + floor(boxWidth/2.0); ++ xId) {
                    if (xId >= 0 && xId < _vGrid[0].size()) {
                        for (int yId = tYId - floor(boxWidth/2.0); yId <= tYId + floor(boxWidth/2.0); ++ yId) {
                            if (yId >= 0 && yId < _vGrid[0][0].size()) {
                                Grid* grid = _vGrid[layId][xId][yId];
                                if (!grid->hasNet(netId)) {
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
    }
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
                int sXId = floor(segment->sX() / _gridWidth);
                int sYId = floor(segment->sY() / _gridWidth);
                int tXId = floor(segment->tX() / _gridWidth);
                int tYId = floor(segment->tY() / _gridWidth);
                AStarRouter router(_vGrid[layId], make_pair(sXId, sYId), make_pair(tXId, tYId), _gridWidth, segment->length(), segment->width(), 0.9, _db.numNets() * 10.0, 0.2);
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