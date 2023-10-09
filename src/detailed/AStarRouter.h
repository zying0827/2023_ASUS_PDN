#ifndef ASTAR_ROUTER_H
#define ASTAR_ROUTER_H

#include "../base/Include.h"
#include "DetailedDB.h"
using namespace std;

enum Direction {
    Up, Down, Right, Left
};

class AStarRouter {
    public:
        AStarRouter(vector< vector< Grid* > > vGrid, pair<int, int> sPos, pair<int, int> tPos, double lbWidth, double widthRatio)
        : _vGrid(vGrid), _sPos(sPos), _tPos(tPos), _lbWidth(lbWidth), _widthRatio(widthRatio) {
            for (size_t xId = 0; xId < numXId(); ++ xId) {
                vector<GNode*> temp;
                for (size_t yId = 0; yId < numYId(); ++ yId) {
                    GNode* node = new GNode(xId, yId);
                    temp.push_back(node);
                }
                _vGNode.push_back(temp);
            }
        }
        ~AStarRouter() {}

        bool route();
        void backTrace();
        double marginCongestCost(int xId, int yId, Direction dir);
        double estDistCost(int xId, int yId) {
            return abs(_tPos.first - xId) + abs(_tPos.second - yId);
        }

        size_t numXId() const { return _vGrid.size(); }
        size_t numYId() const { return _vGrid[0].size(); }
    private:
        // input
        vector< vector< Grid* > > _vGrid;   // index = [xId] [yId]
        pair<int, int> _sPos;     // (source xId, source yId)
        pair<int, int> _tPos;     // (target xId, target yId)
        double _lbWidth;    // the width lower bound calculated from the shortest wirelength
        double _widthRatio;     // the probability of straight routing (without detour on a step)
        // output
        vector< Grid* > _path;
        size_t _exactWidth;
        // process
        vector< vector< GNode* > > _vGNode;     // index = [xId] [yId]
};

#endif