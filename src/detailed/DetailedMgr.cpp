#include "DetailedMgr.h"
#include "DetailedDB.h"
#include "Shape.h"
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <tuple>
#include <utility>
#include <vector>

void DetailedMgr::initGridMap() {
    auto occupiedBySegments = [&] (size_t layId, size_t xId, size_t yId, size_t netId) -> bool {
        for (size_t segId = 0; segId < _db.vNet(netId)->numSegments(layId); ++ segId) {
            Trace* trace = _db.vNet(netId)->vSegment(layId, segId)->trace();
            if (trace->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
            if (trace->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
            if (trace->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
            if (trace->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        }
        return false;
    };
    auto occupiedByPorts = [&] (size_t layId, size_t xId, size_t yId, size_t netId) -> bool {
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

    // init grids occupied by segments and ports
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    if (occupiedBySegments(layId, xId, yId, netId) || occupiedByPorts(layId, xId, yId, netId)) {
                        grid->addNet(netId);
                        grid->incCongestCur();
                        _vNetGrid[netId][layId].push_back(grid);
                    }
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

void DetailedMgr::buildMtx() {
    // https://i.imgur.com/rIwlXJQ.png
    // return an impedance matrix for each net
    // number of nodes: \sum_{layId=0}^{_vNetGrid[netID].size()} _vNetGrid[netID][layId].size()
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

        // initialize
        vector< vector<double > > mtx;
        for(int i=0; i<numNode; i++) {
            mtx.push_back(vector<double>());
            for(int j=0; j<numNode; j++)
                mtx[i].push_back(0.0);
        }

        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
            //    printf("x: %-4d, y: %-4d, lay: %-4d, ID: %-4d\n", i->xId(), i->yId(), layId, getID[make_tuple(layId, i->xId(), i->yId())]);
            
                double g2g_condutance = 1.0;
                double via_condutance = 2.0;

                // check left
                if(grid_i->xId() > 0 && _vGrid[layId][grid_i->xId()-1][grid_i->yId()]->hasNet(netId)) {
                    mtx[node_id][node_id] += g2g_condutance;
                    mtx[node_id][getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())]] -= g2g_condutance;
                }

                // check right
                if(grid_i->xId() < _numXs-1 && _vGrid[layId][grid_i->xId()+1][grid_i->yId()]->hasNet(netId)) {
                    mtx[node_id][node_id] += g2g_condutance;
                    mtx[node_id][getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())]] -= g2g_condutance;
                }
                
                // check down
                if(grid_i->yId() > 0 && _vGrid[layId][grid_i->xId()][grid_i->yId()-1]->hasNet(netId)) {
                    mtx[node_id][node_id] += g2g_condutance;
                    mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)]] -= g2g_condutance;
                }
                
                // check up
                if(grid_i->yId() < _numYs-1 && _vGrid[layId][grid_i->xId()][grid_i->yId()+1]->hasNet(netId)) {
                    mtx[node_id][node_id] += g2g_condutance;
                    mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)]] -= g2g_condutance;
                }

                // check top layer
                if(layId > 0 && _vGrid[layId-1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
                    mtx[node_id][node_id] += via_condutance;
                    mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                }

                // check bottom layer
                if(layId < _db.numLayers()-1 && _vGrid[layId+1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
                    mtx[node_id][node_id] += via_condutance;
                    mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                }
            }
        }

        for(int i=0; i<20; i++) {
            for(int j=0; j<20; j++)
                printf("%4.1f ", mtx[i][j]);
            printf("\n");
        }
    }
}

void DetailedMgr::writeColorMap(const char* path, bool isVoltage) {


    for(int i=0; i<_vGrid.size(); i++) {
        for(int j=0; j<_vGrid[i].size(); j++) {
            for(int k=0; k<_vGrid[i][j].size(); k++) {
                _vGrid[i][j][k]->setCurrent((double)rand()/RAND_MAX);
                _vGrid[i][j][k]->setVoltage((double)rand()/RAND_MAX);
                _vGrid[i][j][k]->setVoltage(k);
            }
        }
    }

    FILE *fp = fopen(path, "w");

    fprintf(fp, "%d\n", _db.numLayers());
    fprintf(fp, "%d\n", _numXs);
    fprintf(fp, "%d\n", _numYs);
    fprintf(fp, "%d\n\n", _db.numVias());

    for(int i=0; i<_db.numVias(); i++)
        fprintf(fp, "%f %f %f\n", (double)_db.vVia(i)->shape()->ctrX() / _gridWidth, (double)_db.vVia(i)->shape()->ctrY() / _gridWidth, (double)_db.vVia(i)->shape()->radius() / _gridWidth);

    fprintf(fp, "\n");

    for(int i=0; i<_vGrid.size(); i++) {
        for(int j=0; j<_vGrid[i].size(); j++) {
            for(int k=0; k<_vGrid[i][j].size(); k++) {
                fprintf(fp, "%.4f ", isVoltage? _vGrid[i][j][k]->voltage(): _vGrid[i][j][k]->current());
            }
            fprintf(fp, "\n");
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
}