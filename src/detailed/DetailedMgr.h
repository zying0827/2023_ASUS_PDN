#ifndef DETAILED_MGR_H
#define DETAILED_MGR_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "DetailedDB.h"
using namespace std;

class DetailedMgr {
    public:
        DetailedMgr(DB& db, SVGPlot& plot, double gridWidth) : _db(db), _plot(plot), _gridWidth(gridWidth) {
            _numXs = _db.boardWidth() / _gridWidth;
            _numYs = _db.boardHeight() / _gridWidth;
            for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                vector< vector<Grid*> > vLayGrid;
                for (size_t xId = 0; xId < _numXs; ++ xId) {
                    vector<Grid*> vXGrid;
                    for (size_t yId = 0; yId < _numYs; ++ yId) {
                        Grid* grid = new Grid;
                        vXGrid.push_back(grid);
                    }
                    vLayGrid.push_back(vXGrid);
                }
                _vGrid.push_back(vLayGrid);
            }
        }
        ~DetailedMgr() {}

        void initGridMap();
        void plotGridMap();
    private:
        DB& _db;
        SVGPlot& _plot;
        double _gridWidth;
        vector< vector< vector< Grid* > > > _vGrid;     // index = [layId] [xId] [yId]; from left xId = 0, from bottom yId = 0
        size_t _numXs;
        size_t _numYs;
};

#endif