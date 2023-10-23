#include "GlobalMgr.h"
#include "LayerILP.h"
#include "VoltEigen.h"
#include "FlowLP.h"
#include "VoltCP.h"
#include "VoltSLP.h"
#include "AddCapacity.h"
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>
// public functions
using namespace std;

void GlobalMgr::plotDB() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t viaId = 0; viaId < _db.numVias(); ++ viaId) {
            Via* via = _db.vVia(viaId);
            via->shape()->plot(via->netId(), layId);
            // _plot.drawCircle(via->shape()->ctrX(), via->shape()->ctrY(), via->shape()->radius(), via->netId());
        }
        for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
            Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
            for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                obs->vShape(shapeId)->plot(SVGPlotColor::gray, layId);
            }
        }
    }
    
}


void GlobalMgr::buildTestOASG() {
    cerr << "buildTestOASG..." << endl;
    // layer0
    cerr << "layer0..." << endl;
    OASGNode* lay0_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay0_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);
    _rGraph.addOASGEdge(0, 0, _rGraph.sourceOASGNode(0,0), _rGraph.targetOASGNode(0,0,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.sourceOASGNode(1,0), _rGraph.targetOASGNode(1,0,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.sourceOASGNode(1,0), _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.targetOASGNode(1,0,0), lay0_detNode1, false);
    _rGraph.addOASGEdge(1, 0, lay0_detNode1, _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.targetOASGNode(1,0,0), lay0_detNode2, false);
    _rGraph.addOASGEdge(1, 0, lay0_detNode2, _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.sourceOASGNode(2,0), _rGraph.targetOASGNode(2,0,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.sourceOASGNode(2,0), _rGraph.targetOASGNode(2,1,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.targetOASGNode(2,0,0), _rGraph.targetOASGNode(2,1,0), false);

    // layer1
    cerr << "layer1..." << endl;
    OASGNode* lay1_obsNode1 = _rGraph.addOASGNode(2, 32, 56, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode2 = _rGraph.addOASGNode(2, 48, 56, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode3 = _rGraph.addOASGNode(2, 48, 60, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode4 = _rGraph.addOASGNode(2, 32, 60, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);

    _rGraph.addOASGEdge(0, 1, _rGraph.sourceOASGNode(0,1), _rGraph.targetOASGNode(0,0,1), false);

    _rGraph.addOASGEdge(1, 1, _rGraph.sourceOASGNode(1,1), _rGraph.targetOASGNode(1,0,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.sourceOASGNode(1,1), _rGraph.targetOASGNode(1,1,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.targetOASGNode(1,0,1), lay1_detNode1, false);
    _rGraph.addOASGEdge(1, 1, lay1_detNode1, _rGraph.targetOASGNode(1,1,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.targetOASGNode(1,0,1), lay1_detNode2, false);
    _rGraph.addOASGEdge(1, 1, lay1_detNode2, _rGraph.targetOASGNode(1,1,1), false);

    _rGraph.addOASGEdge(2, 1, _rGraph.sourceOASGNode(2,1), _rGraph.targetOASGNode(2,0,1), false);
    _rGraph.addOASGEdge(2, 1, _rGraph.sourceOASGNode(2,1), lay1_obsNode1, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode1, lay1_obsNode4, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode4, _rGraph.targetOASGNode(2,1,1), false);
    _rGraph.addOASGEdge(2, 1, _rGraph.targetOASGNode(2,0,1), lay1_obsNode1, false);
    _rGraph.addOASGEdge(2, 1, _rGraph.targetOASGNode(2,0,1), lay1_obsNode2, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode2, lay1_obsNode3, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode3, _rGraph.targetOASGNode(2,1,1), false);

    // layer2
    cerr << "layer2..." << endl;
    OASGNode* lay2_net0_obsNode1 = _rGraph.addOASGNode(0, 8, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode2 = _rGraph.addOASGNode(0, 32, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode3 = _rGraph.addOASGNode(0, 32, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode4 = _rGraph.addOASGNode(0, 8, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode1 = _rGraph.addOASGNode(1, 8, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode2 = _rGraph.addOASGNode(1, 32, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode3 = _rGraph.addOASGNode(1, 32, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode4 = _rGraph.addOASGNode(1, 8, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);

    _rGraph.addOASGEdge(0, 2, _rGraph.sourceOASGNode(0,2), lay2_net0_obsNode1, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode1, lay2_net0_obsNode4, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode4, _rGraph.targetOASGNode(0,0,2), false);
    _rGraph.addOASGEdge(0, 2, _rGraph.sourceOASGNode(0,2), lay2_net0_obsNode2, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode2, lay2_net0_obsNode3, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode3, _rGraph.targetOASGNode(0,0,2), false);
    
    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), lay2_net1_obsNode1, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode1, lay2_net1_obsNode4, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode4, _rGraph.targetOASGNode(1,0,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), lay2_net1_obsNode2, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode2, lay2_net1_obsNode3, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode3, _rGraph.targetOASGNode(1,0,2), false);

    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), _rGraph.targetOASGNode(1,1,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.targetOASGNode(1,0,2), lay2_detNode1, false);
    _rGraph.addOASGEdge(1, 2, lay2_detNode1, _rGraph.targetOASGNode(1,1,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.targetOASGNode(1,0,2), lay2_detNode2, false);
    _rGraph.addOASGEdge(1, 2, lay2_detNode2, _rGraph.targetOASGNode(1,1,2), false);

    _rGraph.addOASGEdge(2, 2, _rGraph.sourceOASGNode(2,2), _rGraph.targetOASGNode(2,0,2), false);
    _rGraph.addOASGEdge(2, 2, _rGraph.sourceOASGNode(2,2), _rGraph.targetOASGNode(2,1,2), false);
    _rGraph.addOASGEdge(2, 2, _rGraph.targetOASGNode(2,0,2), _rGraph.targetOASGNode(2,1,2), false);

    // layer3
    cerr << "layer3..." << endl;
    OASGNode* lay3_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay3_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);
    _rGraph.addOASGEdge(0, 3, _rGraph.sourceOASGNode(0,3), _rGraph.targetOASGNode(0,0,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.sourceOASGNode(1,3), _rGraph.targetOASGNode(1,0,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.sourceOASGNode(1,3), _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.targetOASGNode(1,0,3), lay3_detNode1, false);
    _rGraph.addOASGEdge(1, 3, lay3_detNode1, _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.targetOASGNode(1,0,3), lay3_detNode2, false);
    _rGraph.addOASGEdge(1, 3, lay3_detNode2, _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.sourceOASGNode(2,3), _rGraph.targetOASGNode(2,0,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.sourceOASGNode(2,3), _rGraph.targetOASGNode(2,1,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.targetOASGNode(2,0,3), _rGraph.targetOASGNode(2,1,3), false);
}

// Given three collinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool GlobalMgr::onSegment(OASGNode* p, OASGNode* q, OASGNode* r){
    if (q->x() <= max(p->x(), r->x()) && q->x() >= min(p->x(), r->x()) && 
        q->y() <= max(p->y(), r->y()) && q->y() >= min(p->y(), r->y())) 
       return true; 
  
    return false; 
}

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are collinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int GlobalMgr::orientation(OASGNode* p, OASGNode* q, OASGNode* r){
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
    // for details of below formula. 
    int val = (q->y() - p->y()) * (r->x() - q->x()) - 
              (q->x() - p->x()) * (r->y() - q->y()); 
  
    if (val == 0) return 0;  // collinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
}
// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool GlobalMgr::doIntersect(OASGNode* p1, OASGNode* q1, OASGNode* p2, OASGNode* q2){
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are collinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
}

bool GlobalMgr::isSegmentIntersectingWithObstacles(OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle){
    int numObs = obstacle.size();
    for(int i = 0; i< numObs;++i){
        int numVertices = obstacle[0].size();
        for (int j = 0; j< numVertices-1;++j){
            if(doIntersect(a, b, obstacle[i][j], obstacle[i][j+1])){
                return true;
            }
        }
        if(doIntersect(a, b, obstacle[i][0], obstacle[i][numVertices-1])) return true;
    }
    return false;
}

void GlobalMgr::connectWithObstacle(int netId, int layerId, OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle){
    
    // 紀錄這兩個點跟哪兩個
    
    int numObs = obstacle.size();
    
    bool obs1IsTested = false;

    
    bool inTouchWithThisObs = false;
    for(int i = 0; i< numObs;++i){
        inTouchWithThisObs = false;
        OASGNode * obs1A;
        OASGNode * obs1B;
        OASGNode * obs2A;
        OASGNode * obs2B;

        int numVertices = obstacle[i].size();
        
        for (int j = 0; j< numVertices; j++){
            
            if(j == (numVertices-1)){
                if(!obs1IsTested && doIntersect(a, b, obstacle[i][0], obstacle[i][numVertices-1])){
                    inTouchWithThisObs = true;
                    obs1A = obstacle[i][0];
                    obs1B = obstacle[i][numVertices-1];
                    obs1IsTested = true;
                }
                else if(obs1IsTested && doIntersect(a, b, obstacle[i][0], obstacle[i][numVertices-1])){
                    inTouchWithThisObs = true;
                    obs2A = obstacle[i][0];
                    obs2B = obstacle[i][numVertices-1];
                }
            }
            else{
                if(!obs1IsTested && doIntersect(a, b, obstacle[i][j], obstacle[i][j+1])){
                    inTouchWithThisObs = true;
                    obs1A = obstacle[i][j];
                    obs1B = obstacle[i][j+1];
                    obs1IsTested = true;
                }
                else if(obs1IsTested && doIntersect(a, b, obstacle[i][j], obstacle[i][j+1])){
                    inTouchWithThisObs = true;
                    obs2A = obstacle[i][j];
                    obs2B = obstacle[i][j+1];
                }
            }
        }
        
        if(inTouchWithThisObs ==true){
            for(int nodeId = 0; nodeId < (obstacle[i].size()-1); ++nodeId){
                _rGraph.addOASGEdge(netId, layerId, obstacle[i][nodeId], obstacle[i][nodeId+1], false);
            }
            _rGraph.addOASGEdge(netId, layerId, obstacle[i][0], obstacle[i][(obstacle[i].size()-1)], false);
            double scanX = a->x();
            double scanY = a->y(); 
            double dis1Aa = sqrt(pow(obs1A->x() - scanX, 2) + pow(obs1A->y() - scanY, 2));
            double dis1Ba = sqrt(pow(obs1B->x() - scanX, 2) + pow(obs1B->y() - scanY, 2));
            double dis2Aa = sqrt(pow(obs2A->x() - scanX, 2) + pow(obs2A->y() - scanY, 2));
            double dis2Ba = sqrt(pow(obs2B->x() - scanX, 2) + pow(obs2B->y() - scanY, 2));
            double minDis = std::min({dis1Aa , dis2Aa, dis1Ba, dis2Ba}); 
            if (minDis == dis1Aa || minDis == dis1Ba){
                _rGraph.addOASGEdge(netId, layerId, a, obs1A, false);
                _rGraph.addOASGEdge(netId, layerId, a, obs1B, false);
                _rGraph.addOASGEdge(netId, layerId, b, obs2A, false);
                _rGraph.addOASGEdge(netId, layerId, b, obs2B, false);
            }
            else{
                _rGraph.addOASGEdge(netId, layerId, b, obs1A, false);
                _rGraph.addOASGEdge(netId, layerId, b, obs1B, false);
                _rGraph.addOASGEdge(netId, layerId, a, obs2A, false);
                _rGraph.addOASGEdge(netId, layerId, a, obs2B, false);
            }
        }
    }
}

bool GlobalMgr::checkWithVias(int netId, int layerId, OASGNode* a, OASGNode* b, vector<vector<vector<OASGNode*>>> viaOASGNodes){


    bool edgeTouchVia = false;
    for(int i = 0; i < viaOASGNodes.size();++i){
        if(netId == i) continue;
        int numVias = viaOASGNodes[i].size();
        if(isSegmentIntersectingWithObstacles(a,b,viaOASGNodes[i])){

            connectWithObstacle(netId, layerId, a,b,viaOASGNodes[i]);
            edgeTouchVia = true;
        }
    }    
    return edgeTouchVia;
}


//Bug 1: Now obstacle are creating OASG nodes with fixed number(4 for rectangle)
//Therefore, there will be bug when encountering polygon with number of vertices greater than 4.
//Bug 2: Now there's only one obstacle, when we have more than 1, we will create redundant obstacle loop for other obstacles that is not connected.

void GlobalMgr::buildOASG() {
    // TODO for Lo:
    // for each layer, for each net, use addOASGNode() and addOASGEdge() to construct a crossing OASG
    // in the later stage, all possible paths from the source to target ports and from target ports to lower-voltage target ports will be searched by DFS
    // so the OASGEdges should point from the source to the target ports or from higher-voltage to lower-voltage target ports all along
    cout << "########################################\n";
    cout << "Build OASG Start \n";
    cout << "########################################\n";
    
    //Create viaOASGNodes
    //3 dim, 1dim =  netId, 2dim = Source vias and then targets' vias, 3dim = 4points
    vector<vector<vector<OASGNode*> > > viaOASGNodes;
    viaOASGNodes.resize(_rGraph.numNets());
    for(int i = 0; i < _rGraph.numNets(); ++i){

        int viaNodeId = 0;
        int numSourceVias = _db.vNet(i)->sourceViaCstr()->numVias();
        vector<vector<OASGNode*>> tempViaOASGNodes;
        tempViaOASGNodes.resize((1+_db.vNet(i)->numTPorts()), vector<OASGNode*>(4, nullptr));

        // Step1: Create Source Via        
        double minX = 1000000000;
        double minY = 1000000000;
        double maxX = -1;
        double maxY = -1;
        double tempMinX, tempMinY, tempMaxX, tempMaxY;

        for (int j = 0; j < numSourceVias; ++j ){
            tempMinX = _db.vNet(i)->sourceViaCstr()->vVia(j)->shape()->minX();
            tempMinY = _db.vNet(i)->sourceViaCstr()->vVia(j)->shape()->minY();
            tempMaxX = _db.vNet(i)->sourceViaCstr()->vVia(j)->shape()->maxX();
            tempMaxY = _db.vNet(i)->sourceViaCstr()->vVia(j)->shape()->maxY();
            if(tempMinX<minX){
                minX = tempMinX;
            }
            if(tempMinY<minY){
                minY = tempMinY;
            }
            if(tempMaxX>maxX){
                maxX = tempMaxX;
            }
            if(tempMaxY>maxY){
                maxY = tempMaxY;
            }
        }

        tempViaOASGNodes[viaNodeId][0] = _rGraph.addOASGNode(i, minX, minY, OASGNodeType::MIDDLE);
        tempViaOASGNodes[viaNodeId][1] = _rGraph.addOASGNode(i, maxX, minY, OASGNodeType::MIDDLE);
        tempViaOASGNodes[viaNodeId][2] = _rGraph.addOASGNode(i, maxX, maxY, OASGNodeType::MIDDLE);
        tempViaOASGNodes[viaNodeId][3] = _rGraph.addOASGNode(i, minX, maxY, OASGNodeType::MIDDLE);
        ++viaNodeId ;

        //Secondly, check with the target viaclusters
        for(int tId = 0; tId<_db.vNet(i)->numTPorts();++tId){
            minX = 1000000000;
            minY = 1000000000;
            maxX = -1;
            maxY = -1;
            for(int viaId=0; viaId<_db.vNet(i)->vTargetViaCstr(tId)->numVias();++viaId ){
                tempMinX = _db.vNet(i)->vTargetViaCstr(tId)->vVia(viaId)->shape()->minX();
                tempMinY = _db.vNet(i)->vTargetViaCstr(tId)->vVia(viaId)->shape()->minY();
                tempMaxX = _db.vNet(i)->vTargetViaCstr(tId)->vVia(viaId)->shape()->maxX();
                tempMaxY = _db.vNet(i)->vTargetViaCstr(tId)->vVia(viaId)->shape()->maxY();
                if(tempMinX<minX){
                    minX = tempMinX;
                }
                if(tempMinY<minY){
                    minY = tempMinY;
                }
                if(tempMaxX>maxX){
                    maxX = tempMaxX;
                }
                if(tempMaxY>maxY){
                    maxY = tempMaxY;
                }
            }


            tempViaOASGNodes[viaNodeId][0] = _rGraph.addOASGNode(i, minX, minY, OASGNodeType::MIDDLE);
            tempViaOASGNodes[viaNodeId][1] = _rGraph.addOASGNode(i, maxX, minY, OASGNodeType::MIDDLE);
            tempViaOASGNodes[viaNodeId][2] = _rGraph.addOASGNode(i, maxX, maxY, OASGNodeType::MIDDLE);
            tempViaOASGNodes[viaNodeId][3] = _rGraph.addOASGNode(i, minX, maxY, OASGNodeType::MIDDLE);
            ++viaNodeId;
        }

        viaOASGNodes[i] = tempViaOASGNodes;
    }    


    for (size_t layerId = 0; layerId < _rGraph.numLayers(); ++ layerId){
        //Step 0: 先把每層的Middle 的OASG Node建完(因為每條Net的OASG Node都不一樣，所以直接包在裡面)
        //Obs Node的順序是1左下、2右下、3右上、4左上
        for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId){
            vector<vector<OASGNode*>> obsNodes;

            bool thisLayerHaveObs = false;
            bool thisNetTouchObsThisLayer = false;

            if(_db.numObstacles(layerId)>0){

                thisLayerHaveObs = true;

                obsNodes.resize(_db.numObstacles(layerId), vector<OASGNode*>(4, nullptr));
                for (int obsId = 0; obsId < _db.numObstacles(layerId); ++obsId){
                    double tempX, tempY;
                    for(int j = 0; j < 4; ++j){
                        if(j == 0){
                            tempX =  _db.vObstacle(layerId, obsId)->vShape(0)->minX();
                            tempY =  _db.vObstacle(layerId, obsId)->vShape(0)->minY();
                        } 
                        else if(j == 1){
                            tempX =  _db.vObstacle(layerId, obsId)->vShape(0)->maxX();
                            tempY =  _db.vObstacle(layerId, obsId)->vShape(0)->minY();
                        } 
                        else if(j == 2){
                            tempX =  _db.vObstacle(layerId, obsId)->vShape(0)->maxX();
                            tempY =  _db.vObstacle(layerId, obsId)->vShape(0)->maxY();
                        }                         else{
                            tempX =  _db.vObstacle(layerId, obsId)->vShape(0)->minX();
                            tempY =  _db.vObstacle(layerId, obsId)->vShape(0)->maxY();
                        } 
                        obsNodes[obsId][j] = _rGraph.addOASGNode(netId, tempX, tempY, OASGNodeType::MIDDLE);
                    }
                }
            }

            // int numScanNode = 1 + _db.vNet(netId)->numTPorts() + (4 * _db.numObstacles(layerId));
            int numScanNode = 1 + _db.vNet(netId)->numTPorts(); 
            vector<OASGNode*> traverseNodes;
            traverseNodes.push_back(_rGraph.sourceOASGNode(netId,layerId));
            for (int netTPortId = 0; netTPortId < _db.vNet(netId)->numTPorts(); netTPortId++){
                traverseNodes.push_back(_rGraph.targetOASGNode(netId,netTPortId,layerId));
            }

            //OASG Source
            for (size_t currentScanNodeId = 0; currentScanNodeId < numScanNode; ++ currentScanNodeId){
                // Current Scan Node Id: Source is 0; 1 ~ is target ports
                if (currentScanNodeId == 0){
                    double scanX = _rGraph.sourceOASGNode(netId,layerId)->x();
                    double scanY = _rGraph.sourceOASGNode(netId,layerId)->y();

                    for (int i = 1;i < numScanNode; ++i){
                        double curX = traverseNodes[i]-> x();
                        double curY = traverseNodes[i]-> y();

                            if (isSegmentIntersectingWithObstacles(_rGraph.sourceOASGNode(netId,layerId), traverseNodes[i], obsNodes)){
                                //先把Obs 的四邊都加上ObsEdge
                                for(int obsId = 0; obsId < _db.numObstacles(layerId); ++obsId){
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][0], obsNodes[obsId][1], false);
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][1], obsNodes[obsId][2], false);
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][2], obsNodes[obsId][3], false);
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][3], obsNodes[obsId][0], false);
                                }
                                connectWithObstacle(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), traverseNodes[i], obsNodes);
                                thisNetTouchObsThisLayer = true;
                            }
                            else {
                                if(!checkWithVias(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), traverseNodes[i],viaOASGNodes)){
                                    _rGraph.addOASGEdge(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), traverseNodes[i], false);
                                }
                            }
                    }

                }
                
                //開始處理Target
                else if(currentScanNodeId > 0 && currentScanNodeId <= _db.vNet(netId)->numTPorts()){
                    // cout << "Start building for targets" << endl ;
                    double scanX = traverseNodes[currentScanNodeId]->x();
                    double scanY = traverseNodes[currentScanNodeId]->y();

                    for (int i = 1;i < numScanNode-1; i++ ){
                        if(i <= currentScanNodeId) continue;

                        double curX = traverseNodes[i]-> x();
                        double curY = traverseNodes[i]-> y();
                        if(curX >= scanX && curY >= scanY){
                            if (isSegmentIntersectingWithObstacles(traverseNodes[i], traverseNodes[i+1], obsNodes)){
                                //先把Obs 的四邊都加上ObsEdge
                                for(int obsId = 0; obsId < _db.numObstacles(layerId); ++obsId){
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][0], obsNodes[obsId][1], false);
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][1], obsNodes[obsId][2], false);
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][2], obsNodes[obsId][3], false);
                                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][3], obsNodes[obsId][0], false);
                                }
                                connectWithObstacle(netId, layerId, traverseNodes[i], traverseNodes[i+1], obsNodes);
                                thisNetTouchObsThisLayer = true;
                            }
                            else {
                                if(!checkWithVias(netId, layerId, traverseNodes[i], traverseNodes[i+1],viaOASGNodes)){
                                    _rGraph.addOASGEdge(netId, layerId, traverseNodes[i], traverseNodes[i+1], false);
                                }
                            }
                        }
                        if(i == 1){
                            curX = traverseNodes[numScanNode-1]-> x();
                            curY = traverseNodes[numScanNode-1]-> y();
                            if(curX >= scanX && curY >= scanY){
                                if (isSegmentIntersectingWithObstacles(traverseNodes[1], traverseNodes[numScanNode-1], obsNodes)){
                                    //先把Obs 的四邊都加上ObsEdge
                                    for(int obsId = 0; obsId < _db.numObstacles(layerId); ++obsId){
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][0], obsNodes[obsId][1], false);
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][1], obsNodes[obsId][2], false);
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][2], obsNodes[obsId][3], false);
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][3], obsNodes[obsId][0], false);
                                    }
                                    connectWithObstacle(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], obsNodes);
                                    thisNetTouchObsThisLayer = true;
                                }
                                else {
                                    if(!checkWithVias(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1],viaOASGNodes)){
                                        _rGraph.addOASGEdge(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], false);
                                    }
                                }
                            }
                        }
                    }
                    for (int i = 1;i < numScanNode-1; i++ ){
                        if(i == currentScanNodeId) continue;
                        double curX = traverseNodes[i]-> x();
                        double curY = traverseNodes[i]-> y();
                        //後面是要判斷他們不是同一個點

                        if(curX >= scanX && curY <= scanY){
                            if (isSegmentIntersectingWithObstacles(traverseNodes[i], traverseNodes[i+1], obsNodes)){
                                connectWithObstacle(netId, layerId, traverseNodes[i], traverseNodes[i+1], obsNodes);
                                thisNetTouchObsThisLayer = true;
                            }
                            else {
                                if(!checkWithVias(netId, layerId,  traverseNodes[i], traverseNodes[i+1],viaOASGNodes)){
                                    _rGraph.addOASGEdge(netId, layerId,  traverseNodes[i], traverseNodes[i+1], false);
                                }
                            }
                        }
                        if(i == 1){
                            curX = traverseNodes[numScanNode-1]-> x();
                            curY = traverseNodes[numScanNode-1]-> y();
                            if(curX >= scanX && curY >= scanY){
                                if (isSegmentIntersectingWithObstacles(traverseNodes[1], traverseNodes[numScanNode-1], obsNodes)){
                                    //先把Obs 的四邊都加上ObsEdge
                                    for(int obsId = 0; obsId < _db.numObstacles(layerId); ++obsId){
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][0], obsNodes[obsId][1], false);
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][1], obsNodes[obsId][2], false);
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][2], obsNodes[obsId][3], false);
                                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][3], obsNodes[obsId][0], false);
                                    }
                                    connectWithObstacle(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], obsNodes);
                                    thisNetTouchObsThisLayer = true;
                                }
                                else {
                                    if(!checkWithVias(netId, layerId,  traverseNodes[1], traverseNodes[numScanNode-1],viaOASGNodes)){
                                        _rGraph.addOASGEdge(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], false);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

    }

    //Step 4: 判斷有沒有Net是和Obsticle撞到的，有撞到的把中間那段移動到Obsticle的旁邊
    cout << "########################################\n";
    cout << "Finishing Building OASG \n";
    cout << "########################################\n";
}

void GlobalMgr::plotOASG() {
    //_plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId);
            }
        }
    }
}

void GlobalMgr::layerDistribution() {
    // construct the routing graph
    _rGraph.constructRGraph();
    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     _rGraph.vOASGNode(nodeId) -> print();
    // }
    // cerr << "constructRGraph DONE" << endl;
    // for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
    //     for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //         for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
    //             cerr << "_vRGEdge[" << twoPinNetId << "][" << layId << "][" << RGEdgeId << "] = ";
    //             _rGraph.vEdge(twoPinNetId,layId,RGEdgeId)->print();
    //         }
    //     }
    // }

    
    // calculate normalized current demand
    vector< vector<double> > vNetWeight;
    double totalCurrent = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            totalCurrent += tPort->current();
        }
    }
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        vector<double> temp;
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            temp.push_back(tPort->current() / totalCurrent);
        }
        vNetWeight.push_back(temp);
    }

    // calculate the accumulated via length of each layer
    vector<double> vAccuViaLength;
    double accuViaLength = 0;
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
        vAccuViaLength.push_back(accuViaLength);
        accuViaLength += _db.vMetalLayer(layId)->thickness() + _db.vMediumLayer(layId)->thickness();
    }

    // distribute layers to RGEdges with an ILP solver
    try {
        LayerILP solver(_rGraph, vNetWeight, vAccuViaLength);
        solver.formulate();
        solver.solve();
        solver.collectResult();
    } catch (GRBException e) {
        cerr << "Error = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
}

void GlobalMgr::plotRGraph() {
    // _plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                if (rgEdge->selected()) {
                    for (size_t OASGEdgeId = 0; OASGEdgeId < rgEdge->numEdges(); ++ OASGEdgeId) {
                        OASGEdge* e = rgEdge->vEdge(OASGEdgeId);
                        _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), e->netId(), layId);
                    }
                }
            }
        }
    }
}

void GlobalMgr::buildTestNCOASG() {
    RGraph* NCOASG = new RGraph();
    NCOASG->initRGraph(_db);
    vector<int> vNewNodeId(_rGraph.numOASGNodes(), -1);
    vector<int> vNewEdgeId(_rGraph.numOASGEdges(), -1);
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            OASGNode* sNode = _rGraph.sourceOASGNode(netId, layId);
            vNewNodeId[sNode->nodeId()] = sNode->nodeId();
            for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
                OASGNode* tNode = _rGraph.targetOASGNode(netId, netTPortId, layId);
                vNewNodeId[tNode->nodeId()] = tNode->nodeId();
            }
        }
    }
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                // NCOASG->addRGEdge(rgEdge, twoPinNetId, layId, RGEdgeId);
                if (rgEdge->selected()) {
                    for (size_t OASGEdgeId = 0; OASGEdgeId < rgEdge->numEdges(); ++ OASGEdgeId) {
                        OASGEdge* e = rgEdge->vEdge(OASGEdgeId);
                        if (vNewEdgeId[e->edgeId()] == -1) {
                            // find or create a new sNode for the new edge
                            OASGNode* sNode = e->sNode();
                            OASGNode* newSNode;
                            if (vNewNodeId[sNode->nodeId()] == -1) {
                                newSNode = NCOASG->addOASGNode(sNode->netId(), sNode->x(), sNode->y(), sNode->nodeType());
                                vNewNodeId[sNode->nodeId()] = newSNode->nodeId();
                            } else {
                                newSNode = NCOASG->vOASGNode( vNewNodeId[sNode->nodeId()] );
                            }
                            // find or create a new tNode for the new edge
                            OASGNode* tNode = e->tNode();
                            OASGNode* newTNode;
                            if (vNewNodeId[tNode->nodeId()] == -1) {
                                newTNode = NCOASG->addOASGNode(tNode->netId(), tNode->x(), tNode->y(), tNode->nodeType());
                                vNewNodeId[tNode->nodeId()] = newTNode->nodeId();
                            } else {
                                newTNode = NCOASG->vOASGNode( vNewNodeId[tNode->nodeId()] );
                            }
                            size_t newEdgeId = NCOASG->addOASGEdge(e->netId(), e->layId(), newSNode, newTNode, e->viaEdge());
                            vNewEdgeId[e->edgeId()] = newEdgeId;
                        }
                    }
                }
            }
        }
    }

    // set redundant nodes and edges
    for (size_t netId = 0; netId < NCOASG->numNets(); ++ netId) {
        for (int layId = NCOASG->numLayers()-1; layId >= 0; -- layId) {
            // set redundant source nodes and edges
            OASGNode* sNode = NCOASG->sourceOASGNode(netId, layId);
            if (sNode->numOutEdges() == 0) {
                sNode->setRedundant();
                NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
            } else if (sNode->numOutEdges() == 1) {
                if (NCOASG->vOASGEdge(sNode->outEdgeId(0))->redundant()) {
                    sNode->setRedundant();
                    NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
                }
            }
            // set redundant target nodes and edges
            for (size_t tPortId = 0; tPortId < NCOASG->numTPorts(netId); ++ tPortId) {
                OASGNode* tNode = NCOASG->targetOASGNode(netId, tPortId, layId);
                if (tNode->numInEdges() == 0) {
                    tNode->setRedundant();
                    NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                } else if (tNode->numInEdges() == 1) {
                    if (NCOASG->vOASGEdge(tNode->inEdgeId(0))->redundant()) {
                        tNode->setRedundant();
                        NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                    }
                }
            }
        }
    }
    _rGraph = *NCOASG;
}

void GlobalMgr::plotNCOASG() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId);
            }
        }
    }
}

void GlobalMgr::voltCurrOpt() {
    // store capacity constraints
    struct CapConstr {
        OASGEdge* e1;
        bool right1;
        double ratio1;
        OASGEdge* e2;
        bool right2;
        double ratio2;
        double width;
    };
    struct SingleCapConstr {
        OASGEdge* e1;
        bool right1;
        double ratio1;
        double width;
    };
    vector<CapConstr> vCapConstr;
    vector<SingleCapConstr> vSglCapConstr;
    auto addCapConstr = [&] (OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width) {
        CapConstr capConstr = {e1, right1, ratio1, e2, right2, ratio2, width};
        vCapConstr.push_back(capConstr);
    };
    auto addSglCapConstr = [&] (OASGEdge* e1, bool right1, double ratio1, double width) {
        SingleCapConstr sglCapConstr = {e1, right1, ratio1, width};
        vSglCapConstr.push_back(sglCapConstr);
    };
    // set capacity constraints
    // TODO for Tsai and Huang:
    // for each layer, for each neighboring OASGEdges,
    
    
    //search each layer                                                                           
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId){
        cout << "LAYER :" << layId << endl << endl;
        //search each net
        for(size_t S_netId = 0; S_netId < _rGraph.numNets(); ++ S_netId){
            //search each edge
            for (size_t S_EdgeId = 0; S_EdgeId < _rGraph.numPlaneOASGEdges(S_netId, layId); ++ S_EdgeId){
                OASGEdge* e1 = _rGraph.vPlaneOASGEdge(S_netId, layId, S_EdgeId);
                
                pair<double, double> ratio;
                pair<bool, bool> right;
                double width;
                
                //compare other net edge
                for(size_t T_netId = S_netId+1; T_netId < _rGraph.numNets(); ++ T_netId){
                    //make sure to compare different net
                    for (size_t T_EdgeId = 0; T_EdgeId < _rGraph.numPlaneOASGEdges(T_netId, layId); ++ T_EdgeId){
                        OASGEdge* e2 = _rGraph.vPlaneOASGEdge(T_netId, layId, T_EdgeId);
    
                    //    BuildCapacityConstraint(e1,e2,solver);
                        if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                         make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                         make_pair(e2->sNode()->x(), e2->sNode()->y()),
                                         make_pair(e2->tNode()->x(), e2->tNode()->y()),
                                         ratio, right, width))
                            addCapConstr(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                        
                    }   
                } 

                // obstacle constraint
                for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
                   
                    Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
                    pair<double, double> S2, T2;
                    for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                        for(size_t vtxId = 0; vtxId < obs->vShape(shapeId)->numBPolyVtcs(); ++ vtxId){
                            // get the edge coordinates
                            S2 = make_pair(obs->vShape(shapeId)->bPolygonX(vtxId), obs->vShape(shapeId)->bPolygonY(vtxId));
                            T2 = make_pair(obs->vShape(shapeId)->bPolygonX((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()), obs->vShape(shapeId)->bPolygonY((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()));
                        
                            if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                             make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                             S2, T2, ratio, right, width))
                                addSglCapConstr(e1, right.first, ratio.first, width);

                        }
                    }
                }

                // board cnstraint
                // bottom
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(_db.boardWidth(), 0),
                                 ratio, right, width))
                    addSglCapConstr(e1, right.first, ratio.first, width);
                // left
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(0, _db.boardHeight()),
                                 ratio, right, width))
                    addSglCapConstr(e1, right.first, ratio.first, width);
                // top
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, _db.boardHeight()),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    addSglCapConstr(e1, right.first, ratio.first, width);
                // right
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(_db.boardWidth(), 0),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    addSglCapConstr(e1, right.first, ratio.first, width);
            }
        }  
    }

    voltageAssignment();

    vector<double> vMediumLayerThickness;
    vector<double> vMetalLayerThickness;
    vector<double> vConductivity;
    for (size_t mediumLayId = 0; mediumLayId < _db.numMediumLayers(); ++ mediumLayId) {
        vMediumLayerThickness.push_back(_db.vMediumLayer(mediumLayId)->thickness());
    }
    for (size_t metalLayId = 0; metalLayId < _db.numLayers(); ++ metalLayId) {
        vMetalLayerThickness.push_back(_db.vMetalLayer(metalLayId)->thickness());
        vConductivity.push_back(_db.vMetalLayer(metalLayId)->conductivity());
    }
    double normRatio = _rGraph.sPort(0)->current();
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        if (normRatio > _rGraph.sPort(netId)->current()) {
            normRatio = _rGraph.sPort(netId)->current();
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            if (normRatio > _rGraph.tPort(netId, tPortId)->current()) {
                normRatio = _rGraph.tPort(netId, tPortId)->current();
            }
        }
    }

    FlowLP* currentSolver;
    // VoltCP* voltageSolver;
    VoltSLP* voltageSolver;
    vector<double> vLambda(vCapConstr.size(), 2.0);
    vector<double> vLastOverlap(vCapConstr.size(), 0.0);
    vector<double> vDiffLastOverlap;
    double PRatio = 10.0;
    double DRatio = 1.0;
    size_t numIVIter = 3;
    size_t numIIter = 6;
    size_t numVIter = 10;

    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
                vLambda[capId] = 2;
            }
        // current optimization
        currentSolver = new FlowLP(_rGraph, vMediumLayerThickness, vMetalLayerThickness, vConductivity, normRatio);
        currentSolver->setObjective(_db.areaWeight(), _db.viaWeight());
        currentSolver->setConserveConstraints();
        for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
            CapConstr cap = vCapConstr[capId];
            currentSolver->addCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
        }
        for (size_t sglCapId = 0; sglCapId < vSglCapConstr.size(); ++ sglCapId) {
            SingleCapConstr sglCap = vSglCapConstr[sglCapId];
            currentSolver->addCapacityConstraints(sglCap.e1, sglCap.right1, sglCap.ratio1, sglCap.width);
        }
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            currentSolver->relaxCapacityConstraints(vLambda);
            currentSolver->solveRelaxed();
            currentSolver->collectRelaxedResult();
            _vArea.push_back(currentSolver->area());
            _vOverlap.push_back(currentSolver->overlap());
            cerr << "iIter = " << iIter << endl;
            currentSolver->printRelaxedResult();
            // lagrange multiplier scheduling
            for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
                // schedule1: exp(exp(.))
                // vLambda[capId] *= vLambda[capId];

                // schedule2: exp(.)
                vLambda[capId] *= 2;

                // schedule3: P control
                // vLambda[capId] += PRatio * currentSolver->vOverlap(capId);

                // schedule4: PD control
                // double curOverlap = currentSolver->vOverlap(capId);
                // double diffOverlap = 0;
                // if (iIter > 0) {
                //     diffOverlap = vLastOverlap[capId] - curOverlap;
                // }
                // vLambda[capId] += PRatio * curOverlap + DRatio * diffOverlap;
                // vLastOverlap[capId] = curOverlap;
            }
        }

        // voltage optimization
        vector< vector< double > > vOldVoltage;
        for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
            vector<double> temp;
            for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
                temp.push_back(_rGraph.vNPortOASGNode(netId, nPortNodeId)->voltage());
                // vOldVoltage[netId][nPortNodeId] = _rGraph.vNPortOASGNode(netId, nPortNodeId)->voltage();
            }
            vOldVoltage.push_back(temp);
        }
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            voltageSolver = new VoltSLP(_db, _rGraph, vOldVoltage);
            voltageSolver->setObjective(_db.areaWeight(), _db.viaWeight());
            voltageSolver->setVoltConstraints(1E-10);
            voltageSolver->setLimitConstraint(0.9);
            for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
                CapConstr cap = vCapConstr[capId];
                voltageSolver->addCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
            }
            for (size_t sglCapId = 0; sglCapId < vSglCapConstr.size(); ++ sglCapId) {
                SingleCapConstr sglCap = vSglCapConstr[sglCapId];
                voltageSolver->addCapacityConstraints(sglCap.e1, sglCap.right1, sglCap.ratio1, sglCap.width);
            }
            voltageSolver->relaxCapacityConstraints(vLambda);
            voltageSolver->solveRelaxed();
            voltageSolver->collectRelaxedResult();
            _vArea.push_back(voltageSolver->area());
            _vOverlap.push_back(voltageSolver->overlap());
            cerr << "vIter = " << vIter << endl;
            voltageSolver->printRelaxedResult();
            voltageSolver->collectRelaxedTempVoltage();
            vOldVoltage = voltageSolver->vNewVoltage();
        }
        // voltageSolver->collectRelaxedResult();
        // // cerr << "vIter = " << vIter << endl;
        // voltageSolver->printRelaxedResult();
        

        // voltage optimization
        // voltageSolver = new VoltCP(_db, _rGraph);
        // voltageSolver->setObjective(_db.areaWeight(), _db.viaWeight());
        // voltageSolver->setVoltConstraints(1E-10);
        // for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
        //     CapConstr cap = vCapConstr[capId];
        //     voltageSolver->addCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
        // }
        // for (size_t sglCapId = 0; sglCapId < vSglCapConstr.size(); ++ sglCapId) {
        //     SingleCapConstr sglCap = vSglCapConstr[sglCapId];
        //     voltageSolver->addCapacityConstraints(sglCap.e1, sglCap.right1, sglCap.ratio1, sglCap.width);
        // }
        // for (size_t vIter = 0; vIter < 1; ++ vIter) {
        //     voltageSolver->relaxCapacityConstraints(vLambda);
        //     voltageSolver->solveRelaxed();
        //     voltageSolver->collectRelaxedResult();
        //     cerr << "vIter = " << vIter << endl;
        //     voltageSolver->printRelaxedResult();
        //     // for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
        //     //     vLambda[capId] *= vLambda[capId];
        //     // }
        // }

    }

    // add traces to each net
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                if (e->current() > 0) {
                    Segment* segment = edge2Segment(e);
                    _db.vNet(netId)->addSegment(segment, layId);
                }
            }
        }
    }

    // add vias of each net

    // print the recorded area and overlapped width
    cerr << "////////////////" << endl;
    cerr << "//    area    //" << endl;
    cerr << "////////////////" << endl;
    size_t i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << _vArea[i] << " -> ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << _vArea[i] << " -> ";
            i++;
        }
        cerr << endl;
    }
    cerr << "////////////////////////////" << endl;
    cerr << "//    overlapped width    //" << endl;
    cerr << "////////////////////////////" << endl;
    i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << _vOverlap[i] << " -> ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << _vOverlap[i] << " -> ";
            i++;
        }
        cerr << endl;
    }

}

void GlobalMgr::voltageAssignment() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        VoltEigen solver(_rGraph.numNPortOASGNodes(netId));
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            OASGNode* nPortNode = _rGraph.vNPortOASGNode(netId, nPortNodeId);
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < nPortNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId))->tNode();
                double resistance;
                double l;
                if (outEdge->viaEdge()) {
                    // cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / 32;
                } else {
                    // cerr << "planeEdge ";
                    l = outEdge->length();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                }
                // cerr << "resistance = " << resistance << endl;

                if (outNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                } else {
                    solver.setInputVector(nPortNode->nPortNodeId(), outNode->port()->voltage(), resistance);
                    // cerr << "voltage = " << outNode->port()->voltage() << endl;
                }
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < nPortNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId))->sNode();
                double resistance;
                double l;
                if (inEdge->viaEdge()) {
                    // cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / 32;
                } else {
                    // cerr << "planeEdge ";
                    l = inEdge->length();
                    resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                // cerr << "resistance = " << resistance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), inNode->nPortNodeId(), resistance);
                } else {
                    solver.setInputVector(nPortNode->nPortNodeId(), inNode->port()->voltage(), resistance);
                    // cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }
        // cerr << "G = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     for (size_t colId = 0; colId < solver.numNodes(); ++ colId) {
        //         cerr << setprecision(15) << solver.G(rowId, colId);
        //         if (colId < solver.numNodes()-1) {
        //             cerr << ", ";
        //         }
        //     }
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;
        // cerr << "I = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     cerr << setprecision(15) << solver.I(rowId);
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;

        // cerr << "V = ";
        solver.solve();
        // cerr << endl;

        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(solver.V(nPortNodeId));
        }
        _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
        }
    }

    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     _rGraph.vOASGNode(nodeId) -> print();
    // }
    // vector<double> netVoltage;
    // vector< vector<double> > vVoltage(3, netVoltage);
    // vVoltage[0] = { 4.99999975251689,
    //                 4.99999948245706,
    //                 4.99999939003966,
    //                 4.50000024748309,
    //                 4.50000051754296,
    //                 4.50000060996036,
    //                 4.78574240598038,
    //                 4.86315393669003 };
    // vVoltage[1] = { 4.99999967817019,
    //                 4.99999932627796,
    //                 4.99999920660867,
    //                 4.50000028543947,
    //                 4.50000063733168,
    //                 4.50000075700098,
    //                 4.40000003639034,
    //                 4.40000003639039,
    //                 4.40000003639039,
    //                 4.70485438179277,
    //                 4.80649030108812,
    //                 4.45000000000000,
    //                 4.44999999999998,
    //                 4.45000016091490 };
    // vVoltage[2] = { 4.99999948018458,
    //                 4.99999882037443,
    //                 4.99999859900624,
    //                 4.50000026702640,
    //                 4.50000052112573,
    //                 4.50000060637678,
    //                 4.40000025278904,
    //                 4.40000065849982,
    //                 4.40000079461696 };

    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
    //         _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(vVoltage[netId][nPortNodeId]);
    //     }
    //     _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
    //     for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
    //         _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
    //     }
    // }
    // OASGNode* t1 = _rGraph.vNPortOASGNode(1, 7);
    // cerr << "t1(" << t1->x() << "," << t1->y() << ")" << endl;
    // OASGNode* t2 = _rGraph.vNPortOASGNode(1, 8);
    // cerr << "t2(" << t2->x() << "," << t2->y() << ")" << endl;

    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     cerr << "Node[" << nodeId << "].voltage = " << _rGraph.vOASGNode(nodeId)->voltage() << endl;
    // }

    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     Port* sPort = _rGraph.sPort(netId);
    //     for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
    //         GRBLinExpr netCurrent;
    //         Port* tPort = _rGraph.tPort(netId, netTPortId);
    //         size_t twoPinNetId = _rGraph.twoPinNetId(sPort->portId(), tPort->portId());
    //         for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //             for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
    //                 RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
    //             }
    //         }
    //     }
    // }
    
}

void GlobalMgr::currentDistribution() {
    vector<double> vMediumLayerThickness;
    vector<double> vMetalLayerThickness;
    vector<double> vConductivity;
    for (size_t mediumLayId = 0; mediumLayId < _db.numMediumLayers(); ++ mediumLayId) {
        vMediumLayerThickness.push_back(_db.vMediumLayer(mediumLayId)->thickness());
    }
    for (size_t metalLayId = 0; metalLayId < _db.numLayers(); ++ metalLayId) {
        vMetalLayerThickness.push_back(_db.vMetalLayer(metalLayId)->thickness());
        vConductivity.push_back(_db.vMetalLayer(metalLayId)->conductivity());
    }
    double normRatio = _rGraph.sPort(0)->current();
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        if (normRatio > _rGraph.sPort(netId)->current()) {
            normRatio = _rGraph.sPort(netId)->current();
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            if (normRatio > _rGraph.tPort(netId, tPortId)->current()) {
                normRatio = _rGraph.tPort(netId, tPortId)->current();
            }
        }
    }

    FlowLP solver(_rGraph, vMediumLayerThickness, vMetalLayerThickness, vConductivity, normRatio);
    
    // set objective
    // cerr << "setObjective..." << endl;
    solver.setObjective(_db.areaWeight(), _db.viaWeight());

    // set flow conservation constraints
    // cerr << "setConserveConstraints..." << endl;
    solver.setConserveConstraints();
    // set capacity constraints
    // TODO for Tsai and Huang:
    // for each layer, for each neighboring OASGEdges,
    
    
    //search each layer                                                                           
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId){
        cout << "LAYER :" << layId << endl << endl;
        //search each net
        for(size_t S_netId = 0; S_netId < _rGraph.numNets(); ++ S_netId){
            //search each edge
            for (size_t S_EdgeId = 0; S_EdgeId < _rGraph.numPlaneOASGEdges(S_netId, layId); ++ S_EdgeId){
                OASGEdge* e1 = _rGraph.vPlaneOASGEdge(S_netId, layId, S_EdgeId);
                
                pair<double, double> ratio;
                pair<bool, bool> right;
                double width;
                
                //compare other net edge
                for(size_t T_netId = S_netId+1; T_netId < _rGraph.numNets(); ++ T_netId){
                    //make sure to compare different net
                    for (size_t T_EdgeId = 0; T_EdgeId < _rGraph.numPlaneOASGEdges(T_netId, layId); ++ T_EdgeId){
                        OASGEdge* e2 = _rGraph.vPlaneOASGEdge(T_netId, layId, T_EdgeId);
    
                    //    BuildCapacityConstraint(e1,e2,solver);
                        if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                         make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                         make_pair(e2->sNode()->x(), e2->sNode()->y()),
                                         make_pair(e2->tNode()->x(), e2->tNode()->y()),
                                         ratio, right, width))
                            solver.addCapacityConstraints(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                        
                    }   
                } 

                // obstacle constraint
                for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
                   
                    Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
                    pair<double, double> S2, T2;
                    for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                        for(size_t vtxId = 0; vtxId < obs->vShape(shapeId)->numBPolyVtcs(); ++ vtxId){
                            // get the edge coordinates
                            S2 = make_pair(obs->vShape(shapeId)->bPolygonX(vtxId), obs->vShape(shapeId)->bPolygonY(vtxId));
                            T2 = make_pair(obs->vShape(shapeId)->bPolygonX((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()), obs->vShape(shapeId)->bPolygonY((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()));
                        
                            if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                             make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                             S2, T2, ratio, right, width))
                                solver.addCapacityConstraints(e1, right.first, ratio.first, width);

                        }
                    }
                }

                // board cnstraint
                // bottom
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(_db.boardWidth(), 0),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // left
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(0, _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // top
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, _db.boardHeight()),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // right
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(_db.boardWidth(), 0),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
            }
        }  
    }
    
    // use solver.addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width)
    // to add their capacity constraints
    
    /*
    // layer0
    
    solver.addCapacityConstraints(_rGraph.vOASGEdge(24), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(24), true, 1, _rGraph.vOASGEdge(30), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(30), true, 1, _rGraph.vOASGEdge(36), false, 1, 150);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(36), true, 1, 80);
    // layer1
    solver.addCapacityConstraints(_rGraph.vOASGEdge(25), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(24), true, 1, _rGraph.vOASGEdge(31), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(31), true, 1, _rGraph.vOASGEdge(43), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(43), true, 1, 80);
    //layer2
    solver.addCapacityConstraints(_rGraph.vOASGEdge(27), false, 1, 80);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(27), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(33), false, 1, 0);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(33), true, 1, _rGraph.vOASGEdge(44), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(44), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(48), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(48), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(46), false, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(46), true, 1, 80);
    // layer3
    solver.addCapacityConstraints(_rGraph.vOASGEdge(29), false, 1, 80);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(29), true, 1, _rGraph.vOASGEdge(35), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(35), true, 1, _rGraph.vOASGEdge(45), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(45), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(49), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(49), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(47), false, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(47), true, 1, 80);
    */
    // solve the MCFP formulation and collect the result
    // solver.solve();
    // solver.collectResult();
    // solver.printResult();
    vector<double> vLambda(solver.numCapConstrs(), 2.0);
    for (size_t iter = 0; iter < 5; ++iter) {
        solver.relaxCapacityConstraints(vLambda);
        solver.solveRelaxed();
        solver.collectRelaxedResult();
        cerr << "iter = " << iter << endl;
        solver.printRelaxedResult();
        for (size_t capId = 0; capId < solver.numCapConstrs(); ++ capId) {
            vLambda[capId] *= vLambda[capId];
        }
    }
    solver.relaxCapacityConstraints(vLambda);
    solver.solveRelaxed();
    solver.collectRelaxedResult();
    solver.printRelaxedResult();

    // add traces to each net
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                if (e->current() > 0) {
                    Segment* segment = edge2Segment(e);
                    _db.vNet(netId)->addSegment(segment, layId);
                }
            }
        }
    }
}

void GlobalMgr::plotCurrentPaths() {
    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //         for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
    //             OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
    //             Trace* trace = edge2Trace(e);
    //             trace->plot(netId, layId);
    //             // _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId);
    //         }
    //     }
    // }
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        Net* net = _db.vNet(netId);
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
                net->vSegment(layId, segId)->plot(netId, layId);
            }
        }
    }
}

Trace* GlobalMgr::edge2Trace(OASGEdge* edge) {
    assert (!edge->viaEdge());
    double offset = 0.5 * ( edge->widthRight() - edge->widthLeft() );
    double xOffset = offset * (edge->tNode()->y() - edge->sNode()->y()) / edge->length(); // offset * sin(theta)
    double yOffset = offset * (edge->sNode()->x() - edge->tNode()->x()) / edge->length(); // offset * -cos(theta)
    Node* sNode = new Node(edge->sNode()->x()+xOffset, edge->sNode()->y()+yOffset, _plot);
    Node* tNode = new Node(edge->tNode()->x()+xOffset, edge->tNode()->y()+yOffset, _plot);
    Trace* trace = new Trace(sNode, tNode, edge->widthRight() + edge->widthLeft(), _plot);
    return trace;
}

Segment* GlobalMgr::edge2Segment(OASGEdge* edge) {
    assert (!edge->viaEdge());
    double offset = 0.5 * ( edge->widthRight() - edge->widthLeft() );
    double xOffset = offset * (edge->tNode()->y() - edge->sNode()->y()) / edge->length(); // offset * sin(theta)
    double yOffset = offset * (edge->sNode()->x() - edge->tNode()->x()) / edge->length(); // offset * -cos(theta)
    Node* sNode = new Node(edge->sNode()->x()+xOffset, edge->sNode()->y()+yOffset, _plot);
    Node* tNode = new Node(edge->tNode()->x()+xOffset, edge->tNode()->y()+yOffset, _plot);
    Trace* trace = new Trace(sNode, tNode, edge->widthRight() + edge->widthLeft(), _plot);
    Segment* segment = new Segment(trace, edge->sNode()->voltage(), edge->tNode()->voltage(), edge->current());
    return segment;
}