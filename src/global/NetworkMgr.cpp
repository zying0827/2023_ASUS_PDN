#include "NetworkMgr.h"

// void NetworkMgr::genRGraph() {
//     cerr << "genRGraph..." << endl;
//     // cerr << "numNets = " << _db.numNets() << ", numLayers = " << _db.numLayers() << endl;
//     // generate nodes for feasible tiles
//     for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//         for (size_t rowId = 0; rowId < _db.numRows(); ++rowId) {
//             for (size_t colId = 0; colId < _db.numCols(); ++colId) {
//                 Tile* tile = _db.vTile(layId, rowId, colId);
//                 if (!tile->hasObstacle() && !tile->hasVia()) {
//                     TNode* node = new TNode(tile, NULL);
//                     tile->setNodeId(_rGraph.numNodes());
//                     node->setNodeId(_rGraph.numNodes());
//                     _rGraph.addNode(node);
//                 }
//             }
//         }
//     }

//     // generate source/target via nodes/viaEdges/rViaEdges
//     for (size_t netId = 0; netId < _db.numNets(); ++netId) {
//         // cerr << " _db.numViaClusters(" << netId << ") = " << _db.numViaClusters(netId) << endl;
//         for (size_t viaClusterId = 0; viaClusterId < _db.numViaClusters(netId); ++viaClusterId) {
//             ViaCluster* viaCluster = _db.vViaCluster(netId, viaClusterId);
//             // viaCluster->print();
//             // construct source/target via nodes
//             for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//                 TNode* node = new TNode(NULL, viaCluster);
//                 viaCluster->setNodeId(_rGraph.numNodes());
//                 node->setNodeId(_rGraph.numNodes());
//                 _rGraph.addNode(node);
//                 if (viaCluster->viaType() == ViaType::Source) {
//                     node->setNodeType(TNodeType::SVia);
//                     node->setNodeTypeId(0);
//                     _rGraph.addSViaNode(node, netId);
//                 } else {
//                     assert (viaCluster->viaType() == ViaType::Target);
//                     node->setNodeType(TNodeType::TVia);
//                     node->setNodeTypeId(viaClusterId-1);     // assume for each net vViaCluster = {sVia, tVia, ... , tVia}
//                     _rGraph.addTViaNode(node, netId, layId);
//                 }
//             }
//             // construct source/target via edges
//             for (size_t layPairId = 0; layPairId < _db.numLayers()-1; ++layPairId) {
//                 TEdge* edge = new TEdge;
//                 _rGraph.addEdge(edge);
//                 if (viaCluster->viaType() == ViaType::Source) {
//                     edge->setEdgeType(TEdgeType::SourceVia);
//                     edge->setEdgeTypeId(0);
//                     _rGraph.addSViaEdge(edge, netId);
//                     edge->setNetId(netId);
//                     TNode* nodeUp = _rGraph.vSViaNode(netId, layPairId);
//                     TNode* nodeDown = _rGraph.vSViaNode(netId, layPairId+1);
//                     nodeUp->addInEdge(edge);
//                     nodeDown->addOutEdge(edge);
//                     edge->setSNodeId(nodeDown->nodeId());
//                     edge->setTNodeId(nodeUp->nodeId());
//                 } else {
//                     assert (viaCluster->viaType() == ViaType::Target);
//                     edge->setEdgeType(TEdgeType::TargetVia);
//                     edge->setEdgeTypeId(viaClusterId-1);
//                     _rGraph.addTViaEdge(edge, netId, layPairId);
//                     edge->setNetId(netId);
//                     TNode* nodeUp = _rGraph.vTViaNode(netId, layPairId, viaClusterId-1);
//                     TNode* nodeDown = _rGraph.vTViaNode(netId, layPairId+1, viaClusterId-1);
//                     nodeUp->addOutEdge(edge);
//                     nodeDown->addInEdge(edge);
//                     edge->setSNodeId(nodeUp->nodeId());
//                     edge->setTNodeId(nodeDown->nodeId());
//                 }
//             }
            
//             // construct source/target round via edges
//             // cerr << "construct source/target round via edges" << endl;
//             vector< pair<size_t, size_t> > vRViaPos;
//             for (size_t viaId = 0; viaId < viaCluster->numVias(); ++viaId) {
//                 // cerr << "viaId = " << viaId << endl;
//                 Via* via = viaCluster->vVia(viaId);
//                 // cerr << "viaPos = (" << via->rowId() << ", " << via->colId() << ")" << endl;
//                 if (via->rowId() > 0) {
//                     bool outOfCluster = true;
//                     for (size_t otherViaId = 0; otherViaId < viaCluster->numVias(); ++otherViaId) {
//                         Via* otherVia = viaCluster->vVia(otherViaId);
//                         if (via->rowId()-1 == otherVia->rowId() && via->colId() == otherVia->colId()) {
//                             outOfCluster = false;
//                         }
//                     }
//                     if (outOfCluster) {
//                         vRViaPos.push_back(make_pair(via->rowId()-1, via->colId()));
//                     }
//                 }
//                 if (via->rowId() < _db.numRows()-1) {
//                     bool outOfCluster = true;
//                     for (size_t otherViaId = 0; otherViaId < viaCluster->numVias(); ++otherViaId) {
//                         Via* otherVia = viaCluster->vVia(otherViaId);
//                         if (via->rowId()+1 == otherVia->rowId() && via->colId() == otherVia->colId()) {
//                             outOfCluster = false;
//                         }
//                     }
//                     if (outOfCluster) {
//                         vRViaPos.push_back(make_pair(via->rowId()+1, via->colId()));
//                     }
//                 }
//                 if (via->colId() > 0) {
//                     bool outOfCluster = true;
//                     for (size_t otherViaId = 0; otherViaId < viaCluster->numVias(); ++otherViaId) {
//                         Via* otherVia = viaCluster->vVia(otherViaId);
//                         if (via->rowId() == otherVia->rowId() && via->colId()-1 == otherVia->colId()) {
//                             outOfCluster = false;
//                         }
//                     }
//                     if (outOfCluster) {
//                         vRViaPos.push_back(make_pair(via->rowId(), via->colId()-1));
//                     }
//                 }
//                 if (via->colId() < _db.numCols()-1) {
//                     bool outOfCluster = true;
//                     for (size_t otherViaId = 0; otherViaId < viaCluster->numVias(); ++otherViaId) {
//                         Via* otherVia = viaCluster->vVia(otherViaId);
//                         if (via->rowId() == otherVia->rowId() && via->colId()+1 == otherVia->colId()) {
//                             outOfCluster = false;
//                         }
//                     }
//                     if (outOfCluster) {
//                         vRViaPos.push_back(make_pair(via->rowId(), via->colId()+1));
//                     }
//                 }
//             }
//             for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//                 for (size_t rViaPosId = 0; rViaPosId < vRViaPos.size(); ++rViaPosId) {
//                     // cerr << "rViaPosId = " << rViaPosId << endl;
//                     // cerr << "   rowId = " << vRViaPos[rViaPosId].first << ", colId = " << vRViaPos[rViaPosId].second << endl;
//                     Tile* rTile = _db.vTile(layId, vRViaPos[rViaPosId].first, vRViaPos[rViaPosId].second);
//                     if (!rTile->hasObstacle()) {
//                         TNode* rNode = _rGraph.vNode(rTile->nodeId());
//                         TEdge* edge = new TEdge;
//                         _rGraph.addEdge(edge);
//                         if (viaCluster->viaType() == ViaType::Source) {
//                             edge->setEdgeType(TEdgeType::SRoundVia);
//                             edge->setEdgeTypeId(_rGraph.numSREdges(netId, layId));
//                             _rGraph.addSREdge(edge, netId, layId);
//                             TNode* viaNode = _rGraph.vSViaNode(netId, layId);
//                             viaNode->addOutEdge(edge);
//                             rNode->addInEdge(edge);
//                             edge->setSNodeId(viaNode->nodeId());
//                             edge->setTNodeId(rNode->nodeId());
//                             edge->setNetId(netId);
//                         } else {
//                             assert (viaCluster->viaType() == ViaType::Target);
//                             edge->setEdgeType(TEdgeType::TRoundVia);
//                             edge->setEdgeTypeId(_rGraph.numTREdges(netId, layId));
//                             _rGraph.addTREdge(edge, netId, layId);
//                             TNode* viaNode = _rGraph.vTViaNode(netId, layId, viaClusterId-1);
//                             viaNode->addInEdge(edge);
//                             rNode->addOutEdge(edge);
//                             edge->setSNodeId(rNode->nodeId());
//                             edge->setTNodeId(viaNode->nodeId());
//                             edge->setNetId(netId);
//                         }
//                     }
//                 }
//             }

//             // add via edges
//             // if (viaCluster->viaType() == ViaType::Source) {
//             //     for (size_t layPairId = 0; layPairId < _db.numLayers()-1; ++layPairId) {
//             //         TEdge* edge = new TEdge;
//             //         _rGraph.addEdge(edge);
//             //         if (viaCluster->viaType() == ViaType::Source) {
//             //             edge->setEdgeType(TEdgeType::SRoundVia);
//             //             edge->setEdgeTypeId(0);
//             //             _rGraph.addSRViaEdge(edge, netId);
//             //             edge->setNetId(netId);
//             //             TNode* nodeUp = _rGraph.vSViaNode(netId, layPairId);
//             //             TNode* nodeDown = _rGraph.vSViaNode(netId, layPairId+1);
//             //             nodeUp->addInEdge(edge);
//             //             nodeDown->addOutEdge(edge);
//             //             // cannot set nodeId in edges
//             //         } 
//             //     }
//             // }
            
//         }
//     }

//     // merge via cluster nodes // not done
//     // for (size_t viaClusterId = 0; viaClusterId < _db.numViaClusters(); ++viaClusterId) {
//     //     ViaCluster* viaCluster = _db.vViaCluster(viaClusterId);
//     //     for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//     //         TNode node(NULL, viaCluster);
//     //         _rGraph.addNode(node);
//     //     }
//     // }

//     // classify source via nodes // not done
//     // for (size_t viaClusterId = 0; viaClusterId < _db.numViaClusters(); ++viaClusterId) {
//     //     ViaCluster* viaCluster = _db.vViaCluster(viaClusterId);
//     //     if (viaCluster->viaType() == ViaType::Source) {
//     //         for (size_t viaId = 0; viaId < viaCluster->numVias(); ++viaId) {
//     //             Via* via = viaCluster->vVia(viaId);
//     //             for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//     //                 TNode* node = _rGraph.vNode( _db.vTile(layId, via->rowId(), via->colId())->nodeId() );
//     //                 node->setNodeType(TNodeType::SVia);
//     //                 node->setNodeTypeId(0);
//     //                 _rGraph.addSViaNode(node, viaCluster->netId());
//     //             }
//     //         }
//     //     } else {
//     //         assert (viaCluster->viaType() == ViaType::Target);
//     //         for (size_t viaId = 0; viaId < viaCluster->numVias(); ++viaId) {
//     //             Via* via = viaCluster->vVia(viaId);
//     //             for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//     //                 TNode* node = _rGraph.vNode( _db.vTile(layId, via->rowId(), via->colId())->nodeId() );
//     //                 node->setNodeType(TNodeType::TVia);
//     //                 node->setNodeTypeId(_rGraph.numT2DNodes(viaCluster->netId()));
//     //                 _rGraph.addTViaNode(node, viaCluster->netId(), layId);
//     //             }
//     //         }
//     //     }
//     // }

//     // classify F2DNode
//     size_t F2DNodeId = 0;
//     for (size_t rowId = 0; rowId < _db.numRows(); ++rowId) {
//         for (size_t colId = 0; colId < _db.numCols(); ++colId) {
//             bool F2D = true;
//             for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//                 Tile* tile = _db.vTile(layId, rowId, colId);
//                 if (tile->hasObstacle() || tile->hasVia()) {
//                     F2D = false;
//                 }
//             }
//             if (F2D) {
//                 for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//                     Tile* tile = _db.vTile(layId, rowId, colId);
//                     TNode* node = _rGraph.vNode( _db.vTile(layId, rowId, colId)->nodeId() );
//                     node->setNodeType(TNodeType::F2D);
//                     node->setNodeTypeId(F2DNodeId);
//                     _rGraph.addF2DNode(node, layId);
//                 }
//                 F2DNodeId ++;
//             }
//         }
//     }

//     // classify otherNode
//     for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//         size_t otherNodeId = 0;
//         for (size_t rowId = 0; rowId < _db.numRows(); ++rowId) {
//             for (size_t colId = 0; colId < _db.numCols(); ++colId) {
//                 Tile* tile = _db.vTile(layId, rowId, colId);
//                 if (!tile->hasObstacle() && !tile->hasVia()) {
//                     TNode* node = _rGraph.vNode( _db.vTile(layId, rowId, colId)->nodeId() );
//                     if (node->nodeType() == TNodeType::Other) {
//                         node->setNodeTypeId(otherNodeId);
//                         _rGraph.addOtherNode(node, layId);
//                         otherNodeId ++;
//                     }
//                 }
//             }
//         }
//     }

//     // classify HEdge
//     for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//         for (size_t rowId = 0; rowId < _db.numRows(); ++rowId) {
//             for (size_t colId = 0; colId < _db.numCols(); ++colId) {
//                 Tile* tile = _db.vTile(layId, rowId, colId);
//                 if (!tile->hasObstacle() && !tile->hasVia()) {
//                     TNode* node = _rGraph.vNode(tile->nodeId());
//                     if (rowId < _db.numRows()-1) {
//                         Tile* tileDown = _db.vTile(layId, rowId+1, colId);
//                         if (!tileDown->hasObstacle() && !tileDown->hasVia()) {
//                             TNode* nodeDown = _rGraph.vNode(tileDown->nodeId());
//                             TEdge* edge = new TEdge;
//                             node->addOutEdge(edge);
//                             nodeDown->addInEdge(edge);
//                             edge->setSNodeId(node->nodeId());
//                             edge->setTNodeId(nodeDown->nodeId());
//                             _rGraph.addEdge(edge);
//                             edge->setEdgeType(TEdgeType::Horizontal);
//                             edge->setEdgeTypeId(_rGraph.numHEdges(layId));
//                             _rGraph.addHEdge(edge, layId);
//                         }
//                     }
//                     if (colId < _db.numCols()-1) {
//                         Tile* tileRight = _db.vTile(layId, rowId, colId+1);
//                         if (!tileRight->hasObstacle() && !tileRight->hasVia()) {
//                             TNode* nodeRight = _rGraph.vNode(tileRight->nodeId());
//                             TEdge* edge = new TEdge;
//                             node->addOutEdge(edge);
//                             nodeRight->addInEdge(edge);
//                             edge->setSNodeId(node->nodeId());
//                             edge->setTNodeId(nodeRight->nodeId());
//                             _rGraph.addEdge(edge);
//                             edge->setEdgeType(TEdgeType::Horizontal);
//                             edge->setEdgeTypeId(_rGraph.numHEdges(layId));
//                             _rGraph.addHEdge(edge, layId);
//                         }
//                     }
//                 }  
//             }
//         }
//     }

//     // classify F2DEdge
//     for (size_t F2DId = 0; F2DId < _rGraph.numF2DNodes(); ++F2DId) {
//         for (size_t layPairId = 0; layPairId < _db.numLayers()-1; ++layPairId) {
//             TNode* nodeUp = _rGraph.vF2DNode(layPairId, F2DId);
//             TNode* nodeDown = _rGraph.vF2DNode(layPairId+1, F2DId);
//             TEdge* edge = new TEdge;
//             edge->setEdgeType(TEdgeType::Feasible2D);
//             edge->setEdgeTypeId(F2DId);
//             _rGraph.addF2DEdge(edge, layPairId);
//             nodeUp->addOutEdge(edge);
//             nodeDown->addInEdge(edge);
//             edge->setSNodeId(nodeUp->nodeId());
//             edge->setTNodeId(nodeDown->nodeId());
//             _rGraph.addEdge(edge);
//         }
//     }
    
// }

// void NetworkMgr::distrNet() {
//     vector<double> netCrit{1.0, 1.0, 1.0};
//     double areaCost = 0.2;
//     double widthCost = 100.0;
//     double viaCost = 0;
//     try {
//         NetworkILP solver(_rGraph, netCrit, areaCost, widthCost, viaCost);
//         solver.formulate();
//         solver.solve();
//         solver.collectResult();
//     } catch (GRBException e) {
//         cerr << "Error = " << e.getErrorCode() << endl;
//         cerr << e.getMessage() << endl;
//     }
    
// }

// void NetworkMgr::drawRGraph(bool distrDone = false) {
//     _plot.startPlot(800, 800 * _db.numLayers());
//     int gridWidth = 30;
//     int layDist = 800;
//     int r = 10;
//     // size_t layId = 1;
//     if (distrDone) {
//         for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//             // draw sr/tr edges
//             for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
//                 for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, layId); ++SREdgeId) {
//                     TEdge* edge = _rGraph.vSREdge(netId, layId, SREdgeId);
//                     TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                     TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                     if (edge->hasNet()) {
//                         _plot.drawLine(gridWidth*(sNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sNode->viaCluster()->centerColId()+1),
//                                     gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), edge->netId());
//                     } else {
//                         _plot.drawLine(gridWidth*(sNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sNode->viaCluster()->centerColId()+1),
//                                     gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), 10);
//                     }
                    
//                 }
//                 for (size_t TREdgeId = 0; TREdgeId < _rGraph.numTREdges(netId, layId); ++TREdgeId) {
//                     TEdge* edge = _rGraph.vTREdge(netId, layId, TREdgeId);
//                     TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                     TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                     if (edge->hasNet()) {
//                         _plot.drawLine(gridWidth*(tNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tNode->viaCluster()->centerColId()+1),
//                                     gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1), edge->netId());
//                     } else {
//                         _plot.drawLine(gridWidth*(tNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tNode->viaCluster()->centerColId()+1),
//                                     gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1), 10);
//                     }
                    
//                 }
//             }
            
//             // draw horizontal edges
//             for (size_t HEdgeId = 0; HEdgeId < _rGraph.numHEdges(layId); ++HEdgeId) {
//                 TEdge* edge = _rGraph.vHEdge(layId, HEdgeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                 if (edge->hasNet()) {
//                     _plot.drawLine(gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1),
//                                 gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), edge->netId());
//                 } else {
//                     _plot.drawLine(gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1),
//                             gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), 10);
//                 }
                
//             }

//             // draw nodes
//             for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
//                 TNode* F2DNode = _rGraph.vF2DNode(layId, F2DNodeId);
//                 _plot.drawCircle(gridWidth*(F2DNode->tile()->rowId()+1) + layDist*layId, gridWidth*(F2DNode->tile()->colId()+1),r,3);
//             }
//             for (size_t netId = 0; netId < _db.numNets(); ++netId) {
//                 TNode* sViaNode = _rGraph.vSViaNode(netId, layId);
//                 _plot.drawCircle(gridWidth*(sViaNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sViaNode->viaCluster()->centerColId()+1),r,netId);
//                 for (size_t T2DNodeId = 0; T2DNodeId < _rGraph.numT2DNodes(netId); ++T2DNodeId) {
//                     TNode* tViaNode = _rGraph.vTViaNode(netId, layId, T2DNodeId);
//                     _plot.drawCircle(gridWidth*(tViaNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tViaNode->viaCluster()->centerColId()+1),r,netId);
//                 }
//             }
//             for (size_t otherNodeId = 0; otherNodeId < _rGraph.numOtherNodes(layId); ++otherNodeId) {
//                 TNode* otherNode = _rGraph.vOtherNode(layId, otherNodeId);
//                 _plot.drawCircle(gridWidth*(otherNode->tile()->rowId()+1) + layDist*layId, gridWidth*(otherNode->tile()->colId()+1),r,4);
//             }

//             // draw vertical edges
//             for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); layPairId++) {
//                 for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
//                     TEdge* edge = _rGraph.vF2DEdge(layPairId, F2DNodeId);
//                     TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                     TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                     // cerr << "   " << sNode->tile()->rowId()+0.5 << " " << sNode->tile()->colId()+0.5 << endl;
//                     if (edge->hasNet()) {
//                         _plot.drawSquare(gridWidth*(sNode->tile()->rowId()+1)-0.5*r + layDist*layPairId, gridWidth*(sNode->tile()->colId()+1)-0.5*r,r,edge->netId());
//                         _plot.drawSquare(gridWidth*(tNode->tile()->rowId()+1)-0.5*r + layDist*(layPairId+1), gridWidth*(tNode->tile()->colId()+1)-0.5*r,r,edge->netId());
//                     } else {
//                         _plot.drawSquare(gridWidth*(sNode->tile()->rowId()+1)-0.5*r + layDist*layPairId, gridWidth*(sNode->tile()->colId()+1)-0.5*r,r,10);
//                         _plot.drawSquare(gridWidth*(tNode->tile()->rowId()+1)-0.5*r + layDist*(layPairId+1), gridWidth*(tNode->tile()->colId()+1)-0.5*r,r,10);
//                     }
                    
//                 }
//             }
            
//         }
//     } else {
//         for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//             // draw sr/tr edges
//             for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
//                 for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, layId); ++SREdgeId) {
//                     TEdge* edge = _rGraph.vSREdge(netId, layId, SREdgeId);
//                     TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                     TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                     _plot.drawLine(gridWidth*(sNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sNode->viaCluster()->centerColId()+1),
//                                 gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), edge->edgeType());
//                 }
//                 for (size_t TREdgeId = 0; TREdgeId < _rGraph.numTREdges(netId, layId); ++TREdgeId) {
//                     TEdge* edge = _rGraph.vTREdge(netId, layId, TREdgeId);
//                     TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                     TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                     _plot.drawLine(gridWidth*(tNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tNode->viaCluster()->centerColId()+1),
//                                 gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1), edge->edgeType());
//                 }
//             }
            
//             // draw horizontal edges
//             for (size_t HEdgeId = 0; HEdgeId < _rGraph.numHEdges(layId); ++HEdgeId) {
//                 TEdge* edge = _rGraph.vHEdge(layId, HEdgeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                 _plot.drawLine(gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1),
//                             gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), edge->edgeType());
//             }

//             // draw nodes
//             for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
//                 TNode* F2DNode = _rGraph.vF2DNode(layId, F2DNodeId);
//                 _plot.drawCircle(gridWidth*(F2DNode->tile()->rowId()+1) + layDist*layId, gridWidth*(F2DNode->tile()->colId()+1),r,3);
//             }
//             for (size_t netId = 0; netId < _db.numNets(); ++netId) {
//                 TNode* sViaNode = _rGraph.vSViaNode(netId, layId);
//                 _plot.drawCircle(gridWidth*(sViaNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sViaNode->viaCluster()->centerColId()+1),r,netId);
//                 for (size_t T2DNodeId = 0; T2DNodeId < _rGraph.numT2DNodes(netId); ++T2DNodeId) {
//                     TNode* tViaNode = _rGraph.vTViaNode(netId, layId, T2DNodeId);
//                     _plot.drawCircle(gridWidth*(tViaNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tViaNode->viaCluster()->centerColId()+1),r,netId);
//                 }
//             }
//             for (size_t otherNodeId = 0; otherNodeId < _rGraph.numOtherNodes(layId); ++otherNodeId) {
//                 TNode* otherNode = _rGraph.vOtherNode(layId, otherNodeId);
//                 _plot.drawCircle(gridWidth*(otherNode->tile()->rowId()+1) + layDist*layId, gridWidth*(otherNode->tile()->colId()+1),r,4);
//             }

//             // draw vertical edges
//             for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); layPairId++) {
//                 for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
//                     TEdge* edge = _rGraph.vF2DEdge(layPairId, F2DNodeId);
//                     TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                     // cerr << "   " << sNode->tile()->rowId()+0.5 << " " << sNode->tile()->colId()+0.5 << endl;
//                     _plot.drawSquare(gridWidth*(sNode->tile()->rowId()+1)-0.5*r + layDist*layPairId, gridWidth*(sNode->tile()->colId()+1)-0.5*r,r,edge->edgeType());
//                 }
//             }
            
//         }
//     }
    
//     _plot.endPlot();
// }

// void NetworkMgr::drawResult() {
//     _plot.startPlot(800, 800 * _db.numLayers());
//     int gridWidth = 30;
//     int layDist = 800;
//     int r = 10;

//     for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//         // draw sr/tr edges
//         for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
//             for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, layId); ++SREdgeId) {
//                 TEdge* edge = _rGraph.vSREdge(netId, layId, SREdgeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                 if (edge->hasNet()) {
//                     _plot.drawLine(gridWidth*(sNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sNode->viaCluster()->centerColId()+1),
//                                 gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), edge->netId());
//                 } else {
//                     _plot.drawLine(gridWidth*(sNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sNode->viaCluster()->centerColId()+1),
//                                 gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), 10);
//                 }
                
//             }
//             for (size_t TREdgeId = 0; TREdgeId < _rGraph.numTREdges(netId, layId); ++TREdgeId) {
//                 TEdge* edge = _rGraph.vTREdge(netId, layId, TREdgeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                 if (edge->hasNet()) {
//                     _plot.drawLine(gridWidth*(tNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tNode->viaCluster()->centerColId()+1),
//                                 gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1), edge->netId());
//                 } else {
//                     _plot.drawLine(gridWidth*(tNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tNode->viaCluster()->centerColId()+1),
//                                 gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1), 10);
//                 }
                
//             }
//         }
        
//         // draw horizontal edges
//         for (size_t HEdgeId = 0; HEdgeId < _rGraph.numHEdges(layId); ++HEdgeId) {
//             TEdge* edge = _rGraph.vHEdge(layId, HEdgeId);
//             TNode* sNode = _rGraph.vNode(edge->sNodeId());
//             TNode* tNode = _rGraph.vNode(edge->tNodeId());
//             if (edge->hasNet()) {
//                 _plot.drawLine(gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1),
//                             gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), edge->netId());
//             } else {
//                 _plot.drawLine(gridWidth*(sNode->tile()->rowId()+1) + layDist*layId, gridWidth*(sNode->tile()->colId()+1),
//                         gridWidth*(tNode->tile()->rowId()+1) + layDist*layId, gridWidth*(tNode->tile()->colId()+1), 10);
//             }
            
//         }

//         // draw nodes
//         for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
//             TNode* F2DNode = _rGraph.vF2DNode(layId, F2DNodeId);
//             _plot.drawCircle(gridWidth*(F2DNode->tile()->rowId()+1) + layDist*layId, gridWidth*(F2DNode->tile()->colId()+1),r,3);
//         }
//         for (size_t netId = 0; netId < _db.numNets(); ++netId) {
//             TNode* sViaNode = _rGraph.vSViaNode(netId, layId);
//             _plot.drawCircle(gridWidth*(sViaNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(sViaNode->viaCluster()->centerColId()+1),r,netId);
//             for (size_t T2DNodeId = 0; T2DNodeId < _rGraph.numT2DNodes(netId); ++T2DNodeId) {
//                 TNode* tViaNode = _rGraph.vTViaNode(netId, layId, T2DNodeId);
//                 _plot.drawCircle(gridWidth*(tViaNode->viaCluster()->centerRowId()+1) + layDist*layId, gridWidth*(tViaNode->viaCluster()->centerColId()+1),r,netId);
//             }
//         }
//         for (size_t otherNodeId = 0; otherNodeId < _rGraph.numOtherNodes(layId); ++otherNodeId) {
//             TNode* otherNode = _rGraph.vOtherNode(layId, otherNodeId);
//             _plot.drawCircle(gridWidth*(otherNode->tile()->rowId()+1) + layDist*layId, gridWidth*(otherNode->tile()->colId()+1),r,4);
//         }

//         // draw vertical edges
//         for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); layPairId++) {
//             for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
//                 TEdge* edge = _rGraph.vF2DEdge(layPairId, F2DNodeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                 // cerr << "   " << sNode->tile()->rowId()+0.5 << " " << sNode->tile()->colId()+0.5 << endl;
//                 if (edge->hasNet()) {
//                     _plot.drawSquare(gridWidth*(sNode->tile()->rowId()+1)-0.5*r + layDist*layPairId, gridWidth*(sNode->tile()->colId()+1)-0.5*r,r,edge->netId());
//                     _plot.drawSquare(gridWidth*(tNode->tile()->rowId()+1)-0.5*r + layDist*(layPairId+1), gridWidth*(tNode->tile()->colId()+1)-0.5*r,r,edge->netId());
//                 } else {
//                     _plot.drawSquare(gridWidth*(sNode->tile()->rowId()+1)-0.5*r + layDist*layPairId, gridWidth*(sNode->tile()->colId()+1)-0.5*r,r,10);
//                     _plot.drawSquare(gridWidth*(tNode->tile()->rowId()+1)-0.5*r + layDist*(layPairId+1), gridWidth*(tNode->tile()->colId()+1)-0.5*r,r,10);
//                 }
                
//             }
//         }
        
//     }

//     _plot.endPlot();
// }

// void NetworkMgr::drawDB() {
//     for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//         // draw sr/tr edges
//         for (size_t netId = 0; netId < _rGraph.numNets(); ++netId) {
//             for (size_t SREdgeId = 0; SREdgeId < _rGraph.numSREdges(netId, layId); ++SREdgeId) {
//                 TEdge* edge = _rGraph.vSREdge(netId, layId, SREdgeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                 if (edge->hasNet()) {
//                     tNode->tile()->setNet(edge->netId());
//                 }
//             }
//             for (size_t TREdgeId = 0; TREdgeId < _rGraph.numTREdges(netId, layId); ++TREdgeId) {
//                 TEdge* edge = _rGraph.vTREdge(netId, layId, TREdgeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
//                 if (edge->hasNet()) {
//                     sNode->tile()->setNet(edge->netId());
//                 }
//             }
//         }
        
//         // draw horizontal edges
//         for (size_t HEdgeId = 0; HEdgeId < _rGraph.numHEdges(layId); ++HEdgeId) {
//             TEdge* edge = _rGraph.vHEdge(layId, HEdgeId);
//             TNode* sNode = _rGraph.vNode(edge->sNodeId());
//             TNode* tNode = _rGraph.vNode(edge->tNodeId());
//             if (edge->hasNet()) {
//                 sNode->tile()->setNet(edge->netId());
//                 tNode->tile()->setNet(edge->netId());
//             }
//         }

//         // draw vertical edges
//         for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); layPairId++) {
//             for (size_t F2DNodeId = 0; F2DNodeId < _rGraph.numF2DNodes(); ++F2DNodeId) {
//                 TEdge* edge = _rGraph.vF2DEdge(layPairId, F2DNodeId);
//                 TNode* sNode = _rGraph.vNode(edge->sNodeId());
//                 TNode* tNode = _rGraph.vNode(edge->tNodeId());
                
//                 // cerr << "   " << sNode->tile()->rowId()+0.5 << " " << sNode->tile()->colId()+0.5 << endl;
//                 if (edge->hasNet()) {
//                     _db.addVia(sNode->tile()->rowId(), sNode->tile()->colId(), edge->netId(), ViaType::F2DVia);
//                 }
                
//             }
//         }
        
//     }

//     // draw
//     _plot.startPlot(800, 800 * _db.numLayers());
//     int gridWidth = 30;
//     int layDist = 800;
//     int r = 10;
//     for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
//         for (size_t rowId = 0; rowId < _db.numRows(); ++rowId) {
//             for (size_t colId = 0; colId < _db.numCols(); ++colId) {
//                 int colorId = 12;
//                 if (_db.vTile(layId, rowId, colId)->hasObstacle()) {
//                     colorId = 10;
//                 } else if (_db.vTile(layId, rowId, colId)->hasNet()) {
//                     colorId = _db.vTile(layId, rowId, colId)->netId();
//                 }
//                 _plot.drawSquare(gridWidth*(rowId+1) + layDist*layId, gridWidth*(colId+1), gridWidth, colorId);
//             }
//         }
//         for (size_t viaId = 0; viaId < _db.numVias(); ++viaId) {
//             size_t rowId = _db.vVia(viaId)->rowId();
//             size_t colId = _db.vVia(viaId)->colId();
//             size_t colorId = _db.vVia(viaId)->netId();
//             if (_db.vVia(viaId)->viaType() == ViaType::Source || _db.vVia(viaId)->viaType() == ViaType::Target) {
//                 _plot.drawSquare(gridWidth*(rowId+1) + layDist*layId, gridWidth*(colId+1), gridWidth, colorId);
//             }
//             _plot.drawCircle(gridWidth*(rowId+1.5) + layDist*layId, gridWidth*(colId+1.5), r, colorId);
//         }
//     }
//     _plot.endPlot();
// }