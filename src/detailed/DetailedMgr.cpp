#include "DetailedMgr.h"

void DetailedMgr::initGridMap() {
    auto occupy = [&] (size_t layId, size_t xId, size_t yId, size_t netId) -> bool {
        for (size_t traceId = 0; traceId < _db.vNet(netId)->numTraces(layId); ++ traceId) {
            Trace* trace = _db.vNet(netId)->vTrace(layId, traceId);
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
}