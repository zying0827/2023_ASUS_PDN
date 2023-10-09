#include "AStarRouter.h"

bool AStarRouter::route() {
    // define the priority queue
    auto cmp = [](GNode* a, GNode* b) { return a->cost() > b->cost(); };
    set<GNode*, decltype(cmp) > Q(cmp);

    // initialize the source node
    GNode* sNode = _vGNode[_sPos.first][_sPos.second];
    sNode->setParent(sNode);
    sNode->setCost(0.0);
    sNode->setCurDist(0.0);
    sNode->setEstDist(0.0);
    sNode->setCongest(0.0);
    Q.insert(sNode);
    sNode->setStatus(GNodeStatus::InQueue);

    // define lambda functions
    auto step = [&](GNode* orgNode, int xId, int yId, Direction dir) -> bool {
        GNode* stepNode = _vGNode[xId][yId];
        double newCurDist, newEstDist, newCongest, newCost;
        if (xId == _tPos.first && yId == _tPos.second) {
            stepNode->setParent(orgNode);
            backTrace();
            return true;
        } else if (stepNode->status() != GNodeStatus::InPath) {
            newCurDist = orgNode->curDist()+1;
            newEstDist = estDistCost(xId, yId);
            newCongest = marginCongestCost(xId, yId, dir) + orgNode->congest();
            newCost = newCurDist + newEstDist + newCongest;
            
            if (stepNode->status() == GNodeStatus::Init) {
                stepNode->setCurDist(newCurDist);
                stepNode->setEstDist(newEstDist);
                stepNode->setCongest(newCongest);
                stepNode->setCost(newCost);
                stepNode->setParent(orgNode);
                stepNode->setStatus(GNodeStatus::InQueue);
                Q.insert(stepNode);
            } else {
                assert(stepNode->status() == GNodeStatus::InQueue);
                if (stepNode->cost() > newCost) {
                    Q.erase(Q.find(stepNode));
                    stepNode->setCurDist(newCurDist);
                    stepNode->setEstDist(newEstDist);
                    stepNode->setCongest(newCongest);
                    stepNode->setCost(newCost);
                    stepNode->setParent(orgNode);
                    Q.insert(stepNode);
                }
            }
            return false;
        }
        return false;
    };

    // start searching
    while (!Q.empty()) {
        GNode* node = *Q.begin();
        Q.erase(Q.begin());
        node->setStatus(GNodeStatus::InPath);
        if (node->yId() < numYId()-1) {
            if (step(node, node->xId(), node->yId()+1, Direction::Up)) return true;
        }
        if (node->yId() > 0) {
            if (step(node, node->xId(), node->yId()-1, Direction::Down)) return true;
        }
        if (node->xId() < numXId()-1) {
            if (step(node, node->xId()+1, node->yId(), Direction::Right)) return true;
        }
        if (node->xId() > 0) {
            if (step(node, node->xId()-1, node->yId(), Direction::Left)) return true;
        }
    }
    return false;
}

void AStarRouter::backTrace() {}

double AStarRouter::marginCongestCost(int xId, int yId, Direction dir) {
    auto prob = [&](int w) -> double{
        return pow(1-_widthRatio, w-ceil(_lbWidth)) * _widthRatio;
    };
    double congestion = 0.0;
    double threshold = 1E-3;
    for (int w = ceil(_lbWidth); prob(w) > threshold; ++ w) {
        double wCongestion = 0.0;
        if (dir == Direction::Up) {
            for (int x = xId-w/2; x <= xId+w/2; ++ x) {
                wCongestion += _vGrid[x][yId+w/2]->congestion();
            }
        } else if (dir == Direction::Down) {
            for (int x = xId-w/2; x <= xId+w/2; ++ x) {
                wCongestion += _vGrid[x][yId-w/2]->congestion();
            }
        } else if (dir == Direction::Right) {
            for (int y = yId-w/2; y <= yId+w/2; ++ y) {
                wCongestion += _vGrid[xId+w/2][y]->congestion();
            }
        } else {
            assert (dir == Direction::Left);
            for (int y = yId-w/2; y <= yId+w/2; ++ y) {
                wCongestion += _vGrid[xId-w/2][y]->congestion();
            }
        }
        wCongestion *= prob(w);
        congestion += wCongestion;
    }
    return congestion;
}