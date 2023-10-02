#include "GlobalMgr.h"
#include "LayerILP.h"
#include "FlowLP.h"

#define PI 3.1415926

// public functions

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
    OASGNode* lay0_detNode1 = _rGraph.addOASGNode(400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay0_detNode2 = _rGraph.addOASGNode(400, 520, OASGNodeType::MIDDLE);
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
    OASGNode* lay1_obsNode1 = _rGraph.addOASGNode(320, 560, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode2 = _rGraph.addOASGNode(480, 560, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode3 = _rGraph.addOASGNode(480, 600, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode4 = _rGraph.addOASGNode(320, 600, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode1 = _rGraph.addOASGNode(400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode2 = _rGraph.addOASGNode(400, 520, OASGNodeType::MIDDLE);

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
    OASGNode* lay2_net0_obsNode1 = _rGraph.addOASGNode(80, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode2 = _rGraph.addOASGNode(320, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode3 = _rGraph.addOASGNode(320, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode4 = _rGraph.addOASGNode(80, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode1 = _rGraph.addOASGNode(80, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode2 = _rGraph.addOASGNode(320, 240, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode3 = _rGraph.addOASGNode(320, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode4 = _rGraph.addOASGNode(80, 320, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode1 = _rGraph.addOASGNode(400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode2 = _rGraph.addOASGNode(400, 520, OASGNodeType::MIDDLE);

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
    OASGNode* lay3_detNode1 = _rGraph.addOASGNode(400, 400, OASGNodeType::MIDDLE);
    OASGNode* lay3_detNode2 = _rGraph.addOASGNode(400, 520, OASGNodeType::MIDDLE);
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

void GlobalMgr::buildOASG() {
    // TODO for Luo:
    // for each layer, for each net, use addOASGNode() and addOASGEdge() to construct a crossing OASG
    // in the later stage, all possible paths from the source to target ports and from target ports to lower-voltage target ports will be searched by DFS
    // so the OASGEdges should point from the source to the target ports or from higher-voltage to lower-voltage target ports all along
}

void GlobalMgr::plotOASG() {
    // _plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
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

void GlobalMgr::currentDistribution() {
    FlowLP solver(_rGraph);
    
    // set objective

    // set flow conservation constraints

    // set capacity constraints
    // TODO for Tsai and Huang:
    // for each layer, for each neighboring OASGEdges,

    //search each layer                                                                           
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId){
        //search each net
        for(size_t S_netId = 0; S_netId < _rGraph.numNets(); ++ S_netId){
            //search each edge
            for (size_t S_EdgeId = 0; S_EdgeId < _rGraph.numPlaneOASGEdges(S_netId, layId); ++ S_EdgeId){
                //compare other net edge
                for(size_t T_netId = 0; T_netId < _rGraph.numNets(); ++ T_netId){
                    OASGEdge* e1 = _rGraph.vPlaneOASGEdge(S_netId, layId, S_EdgeId);
                    bool right1 = false; //default left side
                    double ratio1 = 0; //default zero ratio

                    double S_rpoint = max(e1->sNode()->x(),e1->tNode()->x());//right point of segment (x-coordinate)
                    double S_lpoint = min(e1->sNode()->x(),e1->tNode()->x());//left point of segment  (x-coordinate)
                    double S_upoint = max(e1->sNode()->y(),e1->tNode()->y());//upper point of segment (y-coordinate)
                    double S_dpoint = min(e1->sNode()->y(),e1->tNode()->y());//lower(down) point of segment (y-coordinate)

                    //make sure to compare different net
                    if(S_netId != T_netId){
                        for (size_t T_EdgeId = 0; T_EdgeId < _rGraph.numPlaneOASGEdges(T_netId, layId); ++ T_EdgeId){
                            OASGEdge* e2 = _rGraph.vPlaneOASGEdge(T_netId, layId, T_EdgeId);
                            bool right2 = false; //default left side
                            double ratio2 = 0; //default zero ratio
                            double width = 0; //default no width

                            double T_rpoint = max(e2->sNode()->x(),e2->tNode()->x());//right point of segment (x-coordinate)
                            double T_lpoint = min(e2->sNode()->x(),e2->tNode()->x());//left point of segment  (x-coordinate)
                            double T_upoint = max(e2->sNode()->y(),e2->tNode()->y());//upper point of segment (y-coordinate)
                            double T_dpoint = min(e2->sNode()->y(),e2->tNode()->y());//lower(down) point of segment (y-coordinate)

                            //type1 source (horizon)
                            if(e1->sNode()->y() == e1->tNode()->y()){
                                //check if the target is in the searching area
                                //target point is in searching area
                                if(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint || S_lpoint <= T_lpoint && T_lpoint <= S_rpoint){
                                    //type1 target
                                    if(e2->sNode()->y() == e2->tNode()->y()){
                                        //S -> left , T -> right
                                        if(e1->sNode()->y() < e2->sNode()->y()){
                                            width = e2->sNode()->y() - e1->sNode()->y();
                                            right1 = false;
                                            right2 = true;
                                            ratio1 = 1;
                                            ratio2 = 1;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                        //S -> rigth , T -> left
                                        else{
                                            width = e1->sNode()->y() - e2->sNode()->y();
                                            right1 = true;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = 1;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                    }
                                    //type2 target
                                    else if(e2->sNode()->x() == e2->tNode()->x()){
                                        //S -> left , T -> none
                                        if(e1->sNode()->y() < T_dpoint){
                                            width = T_dpoint - e1->sNode()->y();
                                            right1 = false;
                                            right2 = true;
                                            ratio1 = 1;
                                            ratio2 = 0;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                        //S -> rigth , T -> none
                                        else{
                                            width = e1->sNode()->y() - T_upoint;
                                            right1 = true;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = 0;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                    }
                                    //type3 target
                                    else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                                        double slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double theta = atan(slope);
                                        /*double degrees = theta * 180.0 / PI;*/

                                        //both target points are in searching area
                                        if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                                            //S -> left , T -> right
                                            if(e1->sNode()->y() < T_dpoint){
                                                width = T_dpoint - e1->sNode()->y();
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(theta - PI/2);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> left
                                            else{
                                                width = e1->sNode()->y() - T_upoint;
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(theta - PI/2);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target left point is in searching area and target right point isn't
                                        else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                                            //S -> left , T -> right
                                            if(e1->sNode()->y() < T_dpoint){
                                                width = T_dpoint - e1->sNode()->y();
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(theta - PI/2);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> left
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(theta - PI/2);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target right point is in searching area but target left point isn't
                                        else{
                                            //S -> left , T -> rigth
                                            if(e1->sNode()->y() < T_upoint){
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(theta - PI/2);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> left
                                            else{
                                                width = e1->sNode()->y() - T_upoint;
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(theta - PI/2);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                    //type4 target
                                    else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                                        double slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double theta = atan(slope);
                                        //double degrees = theta * 180.0 / PI;
                                        
                                        //both target points are in searching area
                                        if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                                            //S -> left , T -> left
                                            if(e1->sNode()->y() < T_dpoint){
                                                width = T_dpoint - e1->sNode()->y();
                                                right1 = false;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(PI/2 - theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> right
                                            else{
                                                width = e1->sNode()->y() - T_upoint;
                                                right1 = true;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI/2 - theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target left point is in searching area but target right point isn't
                                        else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                                            //S -> left , T -> left
                                            if(e1->sNode()->y() < T_upoint){
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                                                right1 = false;
                                                right2 = false;
                                                ratio1 = cos(PI/2 - theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> right
                                            else{
                                                width = T_upoint - e1->sNode()->y();
                                                right1 = true;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI/2 - theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target right point is in searching area but target left point isn't
                                        else{
                                            //S -> left , T -> left
                                            if(e1->sNode()->y() < T_dpoint){
                                                width = T_dpoint - e1->sNode()->y();
                                                right1 = false;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(theta - PI/2);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> right
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                                                right1 = true;
                                                right2 = true;
                                                ratio1 = cos(theta - PI/2);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                }    
                            }
                            //type2 source (vertical)
                            else if(e1->sNode()->x() == e1->tNode()->x()){
                                //check if the target is in the searching area
                                //target point is in searching area
                                if(S_dpoint <= T_dpoint && T_dpoint <= S_upoint || S_dpoint <= T_upoint && T_upoint <= S_upoint){
                                    //type1 target
                                    if(e2->sNode()->y() == e2->tNode()->y()){
                                        //S -> left , T -> none
                                        if(e1->sNode()->x() > T_rpoint){
                                            width = e1->sNode()->x() - T_rpoint;
                                            right1 = false;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = 0;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                        //S -> rigth , T -> none
                                        else{
                                            width = T_lpoint - e1->sNode()->x();
                                            right1 = true;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = 0;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                    }
                                    //type2 target
                                    else if(e2->sNode()->x() == e2->tNode()->x()){
                                        //S -> left , T -> right
                                        if(e1->sNode()->x() > T_lpoint){
                                            width = e1->sNode()->x() - T_lpoint;
                                            right1 = false;
                                            right2 = true;
                                            ratio1 = 1;
                                            ratio2 = 1;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                        //S -> rigth , T -> left
                                        else{
                                            width = T_lpoint - e1->sNode()-x();
                                            right1 = true;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = 1;
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                        }
                                    }
                                    //type3 target
                                    else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                                        double slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double theta = atan(slope);
                                        /*double degrees = theta * 180.0 / PI;*/

                                        //both target points are in searching area
                                        if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                                            //S -> left , T -> right
                                            if(e1->sNode()->x() > T_rpoint){
                                                width =  e1->sNode()->x() - T_rpoint;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> left
                                            else{
                                                width = T_lpoint - e1->sNode()->x();
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target down point is in searching area but target upper point isn't
                                        else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                                            //S -> left , T -> right
                                            if(e1->sNode()->x() > T_lpoint){
                                                width =  shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(PI - theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> left
                                            else{
                                                width = T_lpoint - e1->sNode()->x();
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target right point is in searching area but target left point isn't
                                        else{
                                            //S -> left , T -> rigth
                                            if(e1->sNode()->x() > T_rpoint){
                                                width = e1->sNode()->x() - T_rpoint;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> left
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(PI - theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                    //type4 target
                                    else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                                        double slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double theta = atan(slope);
                                        //double degrees = theta * 180.0 / PI;
                                        
                                        //both target points are in searching area
                                        if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                                            //S -> right , T -> left
                                            if(e1->sNode()->x() < T_lpoint){
                                                width = T_lpoint - e1->sNode()->x();
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> left , T -> right
                                            else{
                                                width = e1->sNode()->x() - T_rpoint;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target down point is in searching area but target upper point isn't
                                        else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                                            //S -> left , T -> right
                                            if(e1->sNode()->x() > T_rpoint){
                                                width = e1->sNode()->x() - T_rpoint;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> right
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_upoint,e2->length());;
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target upper point is in searching area but target down point isn't
                                        else{
                                            //S -> left , T -> right
                                            if(e1->sNode()->x() > T_lpoint){
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //S -> rigth , T -> left
                                            else{
                                                width = T_lpoint - e1->sNode()->x();
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                }    
                            }
                            //type3 source (slope > 0)
                            else if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x())){
                                double S_slope = -1/(e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x()); //not slope of edge but its width's direction
                                double S_theta = atan(S_slope);
                                //check if target is in searching area
                                double rs = (e2->sNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->sNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//T_sNode
                                double rt = (e2->tNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->tNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//T_tNode
                                //target point is in searching area
                                if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
                                    //type1 target
                                    if(e2->sNode()->y() == e2->tNode()->y()){
                                        double r1 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target left point
                                        double r2 = (T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target right point
                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                           double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                           double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                                           //if r1 is closer to e1, S -> right, T -> left
                                           if(r1_distance < r2_distance){
                                            width =  r1_distance;
                                            right1 = true;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = cos(S_theta - PI/2);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }
                                           //else if r2 is closer to e1, S -> left, T -> right
                                           else{
                                            width =  r2_distance;
                                            right1 = false;
                                            right2 = true;
                                            ratio1 = 1;
                                            ratio2 = cos(S_theta - PI/2);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }   
                                        }
                                        //target left point is in searching area but target right point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            //if T_upoint > S_upoint, S -> left, T -> right
                                            if(T_upoint > S_upoint){
                                                width =  shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(S_theta - PI/2);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> right, T -> left
                                            else{
                                                width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(S_theta - PI/2);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target right point is in searching area but target left point isn't
                                        else{
                                            //if T_dpoint > S_dpoint, S -> left, T -> right
                                            if(T_dpoint > S_dpoint){
                                                width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(S_theta - PI/2);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> right, T -> left
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(S_theta - PI/2);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                    //type2 target
                                    else if(e2->sNode()->x() == e2->tNode()->x()){
                                        double r1 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target upper point
                                        double r2 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target down point
                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                           double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                                           double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                           //if r1 is closer to e1, S -> right, T -> left
                                           if(r1_distance < r2_distance){
                                            width =  r1_distance;
                                            right1 = true;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = cos(PI - S_theta);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }
                                           //else if r2 is closer to e1, S -> left, T -> right
                                           else{
                                            width =  r2_distance;
                                            right1 = false;
                                            right2 = true;
                                            ratio1 = 1;
                                            ratio2 = cos(PI - S_theta);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }   
                                        }
                                        //target upper point is in searching area but target down point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            //if T_lpoint < S_lpoint, S -> left, T -> right
                                            if(T_lpoint < S_lpoint){
                                                width =  shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(PI - S_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> right, T -> left
                                            else{
                                                width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - S_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target down point is in searching area but target upper point isn't
                                        else{
                                            //if T_lpoint < S_rpoint, S -> left, T -> right
                                            if(T_lpoint < S_rpoint){
                                                width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - S_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> right, T -> left
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(PI - S_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                    //type3 target
                                    else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                                        double T_slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double T_theta = atan(T_slope);
                                        double s1 = -1/S_slope; // slope of e1
                                        double s2 = -1/T_slope; // slope of e2

                                        double r1 = (T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target upper right point
                                        double r2 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target down left point

                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                            //compare the slope of two edges
                                            if(s2 > s1){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                //if r1 is closer to e1, S -> right, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> left, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                            else if(s1 == s2){
                                                double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1)//x-intercept of source edge
                                                double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2)//x-intercept of target edge
                                                //x1 < x2, S -> right, T -> left
                                                if(x1 < x2){
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //x1 > x2, S -> left, T -> right
                                                else{
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                //if r1 is closer to e1, S -> left, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> right, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                        //target upper right point is in searching area but target down left point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            if(s2 > s1){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());//distance of source down left point to target edge
                                                //if r1 is smaller, S -> right, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is smaller, S -> left, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = cos(T_theta - S_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                            else if(s1 == s2){
                                                double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1)//x-intercept of source edge
                                                double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2)//x-intercept of target edge
                                                //x1 < x2, S -> right, T -> left
                                                if(x1 < x2){
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //x1 > x2, S -> left, T -> right
                                                else{
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());//distance of source down left point to target edge
                                                //if r1 is smaller, S -> left, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is smaller, S -> right, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = cos(S_theta - T_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                        //target down left point is in searching area but target upper right point isn't
                                        else{
                                            if(s2 > s1){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_rpoint,e2->length());//distance of source upper right point to target edge
                                                //if r1 is smaller, S -> left, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is smaller, S -> right, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = cos(T_theta - S_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }  
                                            }
                                            else if(s1 == s2){
                                                double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1)//x-intercept of source edge
                                                double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2)//x-intercept of target edge
                                                //x1 < x2, S -> right, T -> left
                                                if(x1 < x2){
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length()) ;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //x1 > x2, S -> left, T -> right
                                                else{
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length()) ;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_rpoint,e2->length());//distance of source upper right point to target edge
                                                //if r1 is smaller, S -> right, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is smaller, S -> left, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = cos(S_theta - T_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                    }
                                    //type4 target
                                    else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                                        double T_slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double T_theta = atan(T_slope);
                                        double s1 = -1/S_slope; // slope of e1
                                        double s2 = -1/T_slope; // slope of e2

                                        double r1 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target upper left point
                                        double r2 = (T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target down right point

                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                            //compare the slope of two edges
                                            if(s2 > S_slope){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                                                //if r1 is closer to e1, S -> right, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> left, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                            else if(s2 == S_slope){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                                                //if r1 is closer to e1, S -> right, T -> none
                                                if(r1_distance < r2_distance){
                                                    width = r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 0;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> left, T -> none
                                                else{
                                                    width = r2_distance;
                                                    right1 = false;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 0;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                                                //if r1 is closer to e1, S -> right, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(PI - S_theta + T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> left, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(PI - S_theta + T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                        //target upper left point is in searching area but target down right point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            //only s2 < s_slope would happen
                                            double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                                            double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());//distance of source down left point to target edge
                                            //if r1 is smaller, S -> right, T -> left
                                            if(r1_distance < r2_distance){
                                                width =  r1_distance;
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - S_theta + T_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else if r2 is smaller, S -> left, T -> right
                                            else{
                                                width =  r2_distance;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(PI - S_theta + T_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }   
                                        }
                                        //target down right point is in searching area but target upper left point isn't
                                        else{
                                            //only s2 < s_slope would happen
                                            double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                                            double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_rpoint,e2->length());//distance of source upper right point to target edge
                                            //if r1 is smaller, S -> left, T -> right
                                            if(r1_distance < r2_distance){
                                                width =  r1_distance;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - S_theta + T_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else if r2 is smaller, S -> right, T -> left
                                            else{
                                                width =  r2_distance;
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(PI - S_theta + T_theta);;
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }  
                                        }
                                    }
                                }
                            }
                            //type4 source (slope < 0)
                            else/*if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()))*/{
                                double S_slope = -1/(e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x()); //not slope of edge but its width's direction
                                double S_theta = atan(S_slope);
                                //check if target is in searching area
                                double rs = (e2->sNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->sNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//T_sNode
                                double rt = (e2->tNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->tNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//T_tNode
                                //target point is in searching area
                                if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
                                    //type1 target
                                    if(e2->sNode()->y() == e2->tNode()->y()){
                                        double r1 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target left point
                                        double r2 = (T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target right point
                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                           double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());//distance of target left point to source edge
                                           double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//distance of target right point to source edge
                                           //if r1 is closer to e1, S -> right, T -> right
                                           if(r1_distance < r2_distance){
                                            width =  r1_distance;
                                            right1 = true;
                                            right2 = true;
                                            ratio1 = 1;
                                            ratio2 = cos(PI/2 - S_theta);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }
                                           //else if r2 is closer to e1, S -> left, T -> left
                                           else{
                                            width =  r2_distance;
                                            right1 = false;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = cos(PI/2 - S_theta);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }   
                                        }
                                        //target left point is in searching area but target right point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            //if T_dpoint > S_dpoint, S -> right, T -> right
                                            if(T_dpoint > S_dpoint){
                                                width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                right1 = true;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI/2 - S_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> right, T -> left
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());
                                                right1 = false;
                                                right2 = false;
                                                ratio1 = cos(PI/2 - S_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target right point is in searching area but target left point isn't
                                        else{
                                            //if T_upoint > S_upoint, S -> right, T -> right
                                            if(T_upoint > S_upoint){
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_upoint,e2->length());
                                                right1 = true;
                                                right2 = true;
                                                ratio1 = cos(PI/2 - S_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> left, T -> left
                                            else{
                                                width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                                                right1 = false;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(PI/2 - S_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                    //type2 target
                                    else if(e2->sNode()->x() == e2->tNode()->x()){
                                        double r1 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target upper point
                                        double r2 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target down point
                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                           double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper point to source edge
                                           double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());//distance of target down point to source edge
                                           //if r1 is closer to e1, S -> left, T -> right
                                           if(r1_distance < r2_distance){
                                            width =  r1_distance;
                                            right1 = false;
                                            right2 = true;
                                            ratio1 = 1;
                                            ratio2 = cos(S_theta);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }
                                           //else if r2 is closer to e1, S -> right, T -> left
                                           else{
                                            width =  r2_distance;
                                            right1 = true;
                                            right2 = false;
                                            ratio1 = 1;
                                            ratio2 = cos(S_theta);
                                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                           }   
                                        }
                                        //target upper point is in searching area but target down point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            //if T_rpoint < S_rpoint, S -> left, T -> right
                                            if(T_rpoint < S_rpoint){
                                                width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(S_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> right, T -> left
                                            else{
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(S_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                        //target down point is in searching area but target upper point isn't
                                        else{
                                            //if T_lpoint < S_lpoint, S -> left, T -> right
                                            if(T_lpoint < S_lpoint){
                                                width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_upoint,e2->length());
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(S_theta);;
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else, S -> right, T -> left
                                            else{
                                                width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(S_theta);;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                        }
                                    }
                                    //type3 target
                                    else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                                        double T_slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double T_theta = atan(T_slope);
                                        double s1 = -1/S_slope; // slope of e1
                                        double s2 = -1/T_slope; // slope of e2

                                        double r1 = (T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target upper right point
                                        double r2 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target down left point

                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                            //compare the slope of two edges
                                            if(s2 > S_slope){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                //if r1 is closer to e1, S -> left, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(PI - T_theta + S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> right, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(PI - T_theta + S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                            else if(s2 == S_slope){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                //if r1 is closer to e1, S -> left, T -> none
                                                if(r1_distance < r2_distance){
                                                    width = r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = 0;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> right, T -> left
                                                else{
                                                    width = r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 0;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                                                //if r1 is closer to e1, S -> left, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> right, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                        //target upper right point is in searching area but target down left point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            //only s2 > S_slope would happen
                                            double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                                            double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());//distance of source down right point to target edge
                                            //if r1 is smaller, S -> left, T -> right
                                            if(r1_distance < r2_distance){
                                                width =  r1_distance;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - T_theta + S_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else if r2 is smaller, S -> right, T -> left
                                            else{
                                                width = r2_distance;
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = cos(PI - T_theta + S_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }   
                                        }
                                        //target down left point is in searching area but target upper right point isn't
                                        else{
                                            //only s2 > S_slope would happen
                                            double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                                            double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_lpoint,e2->length());//distance of source upper left point to target edge
                                            //if r1 is smaller, S -> right, T -> left
                                            if(r1_distance < r2_distance){
                                                width =  r1_distance;
                                                right1 = true;
                                                right2 = false;
                                                ratio1 = 1;
                                                ratio2 = cos(PI - T_theta + S_theta);
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }
                                            //else if r2 is smaller, S -> left, T -> right
                                            else{
                                                width =  r2_distance;
                                                right1 = false;
                                                right2 = true;
                                                ratio1 = cos(PI - T_theta + S_theta);
                                                ratio2 = 1;
                                                solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                            }  
                                        }
                                    }
                                    //type4 target
                                    else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                                        double T_slope = -1/(e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x()); //not slope of edge but its width's direction
                                        double T_theta = atan(T_slope);
                                        double s1 = -1/S_slope; // slope of e1
                                        double s2 = -1/T_slope; // slope of e2

                                        double r1 = (T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target upper left point
                                        double r2 = (T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y()) / pow(e1->length(),2);//Target down right point

                                        //both target points are in searching area
                                        if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                                            //compare the slope of two edges
                                            if(s2 > s1){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                                                //if r1 is closer to e1, S -> right, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> left, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                            else if(s2 == s1){
                                                double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1)//x-intercept of source edge
                                                double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2)//x-intercept of target edge
                                                //x1 < x2, S -> right, T -> left
                                                if(x1 < x2){
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else, S -> left, T -> right
                                                else{
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                                                double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                                                //if r1 is closer to e1, S -> left, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> right, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                        //target upper left point is in searching area but target down right point isn't
                                        else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                                            if(s2 > s1){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());//distance of source down right point to target edge
                                                //if r1 is smaller, S -> right, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is smaller, S -> left, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = cos(T_theta - S_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                            else if(s2 == s1){
                                                double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1)//x-intercept of source edge
                                                double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2)//x-intercept of target edge
                                                //x1 < x2, S -> right, T -> left
                                                if(x1 < x2){
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else, S -> left, T -> right
                                                else{
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());//distance of source down right point to target edge
                                                //if r1 is closer to e1, S -> left, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> right, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = cos(S_theta - T_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                        //target down right point is in searching area but target upper left point isn't
                                        else{
                                            if(s2 > s1){
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_lpoint,e2->length());//distance of source upper left point to target edge
                                                //if r1 is smaller, S -> left, T -> right
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = cos(T_theta - S_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is smaller, S -> right, T -> left
                                                else{
                                                    width =  r2_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = cos(T_theta - S_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }  
                                            }
                                            else if(s2 == s1){
                                                double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1)//x-intercept of source edge
                                                double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2)//x-intercept of target edge
                                                //x1 < x2, S -> right, T -> left
                                                if(x1 < x2){
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else, S -> left, T -> right
                                                else{
                                                    width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = 1;
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                            }
                                            else{
                                                double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                                                double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_lpoint,e2->length());//distance of source upper left point to target edge
                                                //if r1 is closer to e1, S -> right, T -> left
                                                if(r1_distance < r2_distance){
                                                    width =  r1_distance;
                                                    right1 = true;
                                                    right2 = false;
                                                    ratio1 = 1;
                                                    ratio2 = cos(S_theta - T_theta);
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }
                                                //else if r2 is closer to e1, S -> left, T -> right
                                                else{
                                                    width =  r2_distance;
                                                    right1 = false;
                                                    right2 = true;
                                                    ratio1 = cos(S_theta - T_theta);
                                                    ratio2 = 1;
                                                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                                                }   
                                            }
                                        }
                                    }
                                }
                            }
                        }   
                    }
                } 
            }
        }  
    }

    // use solver.addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width)
    // to add their capacity constraints

    // solve the MCFP formulation and collect the result

}

//function that find the sortest path between one point and one segment (point's foot must lay on the segment)
double shortest_distance(double segment_x1, double segment_y1, double segment_x2, double segment_y2, double point_x, double point_y, double length_of_segment){
    double r = (pointx - segment_x1) * (segment_x2 - segment_x1) + (point_y - segment_y1) * (segment_y2 - segment_y1) / pow(length_of_segment,2);
    double footx = segment_x1 + r * (segment_x2 - segment_x1);
    double footy = segment_y1 + r * (segment_y2 - segment_y1);

    return sqrt(pow(footx - point_x,2) + pow(footy - point_y,2));
}


