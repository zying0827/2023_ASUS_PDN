#include "Parser.h"
using namespace std;

void Parser::testInitialize(double boardWidth, double boardHeight, double gridWidth) {
    // double gridWidth = 40.0;
    // _db.setBoundary(15*gridWidth, 19*gridWidth);
    _db.setBoundary(boardWidth, boardHeight);

    _db.addMediumLayer("medium64", 1.524000e-02, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("BOTTOM", 3.556000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium62", 4.927600e-02, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("VCC1", 3.048000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium52", 1.016000e-01, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("VCC", 3.048000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium42", 4.927600e-02, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("TOP", 3.556000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium40", 1.524000e-02, 4.500000e+00, 3.500000e-02);

    _db.initNet(3);

    ViaCluster* viaCstr;
    // _vNet[0]
    _db.addCircleVia(2*gridWidth, 2*gridWidth, 0, ViaType::Source);
    _db.addCircleVia(3*gridWidth, 2*gridWidth, 0, ViaType::Source);
    _db.addCircleVia(2*gridWidth, 3*gridWidth, 0, ViaType::Source);
    _db.addCircleVia(3*gridWidth, 3*gridWidth, 0, ViaType::Source);
    viaCstr = _db.clusterVia({0,1,2,3});
    _db.addPort(5.0, 1.0, viaCstr);
    _db.addCircleVia(5*gridWidth, 14*gridWidth, 0, ViaType::Target);
    _db.addCircleVia(6*gridWidth, 14*gridWidth, 0, ViaType::Target);
    _db.addCircleVia(5*gridWidth, 15*gridWidth, 0, ViaType::Target);
    _db.addCircleVia(6*gridWidth, 15*gridWidth, 0, ViaType::Target);
    viaCstr = _db.clusterVia({4,5,6,7});
    _db.addPort(4.5, 1.0, viaCstr);
    // _vNet[1]
    _db.addCircleVia(6*gridWidth, 2*gridWidth, 1, ViaType::Source);
    _db.addCircleVia(7*gridWidth, 2*gridWidth, 1, ViaType::Source);
    _db.addCircleVia(6*gridWidth, 3*gridWidth, 1, ViaType::Source);
    _db.addCircleVia(7*gridWidth, 3*gridWidth, 1, ViaType::Source);
    viaCstr = _db.clusterVia({8,9,10,11});
    _db.addPort(5.0, 1.0, viaCstr);
    _db.addCircleVia(7*gridWidth, 12*gridWidth, 1, ViaType::Target);
    _db.addCircleVia(8*gridWidth, 12*gridWidth, 1, ViaType::Target);
    viaCstr = _db.clusterVia({12,13});
    _db.addPort(4.5, 1.0, viaCstr);
    _db.addCircleVia(12*gridWidth, 12*gridWidth, 1, ViaType::Target);
    _db.addCircleVia(13*gridWidth, 12*gridWidth, 1, ViaType::Target);
    viaCstr = _db.clusterVia({14,15});
    _db.addPort(4.4, 1.0, viaCstr);
    // _vNet[2]
    _db.addCircleVia(10*gridWidth, 2*gridWidth, 2, ViaType::Source);
    _db.addCircleVia(11*gridWidth, 2*gridWidth, 2, ViaType::Source);
    _db.addCircleVia(10*gridWidth, 3*gridWidth, 2, ViaType::Source);
    _db.addCircleVia(11*gridWidth, 3*gridWidth, 2, ViaType::Source);
    viaCstr = _db.clusterVia({16,17,18,19});
    _db.addPort(5.0, 1.0, viaCstr);
    _db.addCircleVia(10*gridWidth, 11*gridWidth, 2, ViaType::Target);
    _db.addCircleVia(10*gridWidth, 12*gridWidth, 2, ViaType::Target);
    viaCstr = _db.clusterVia({20,21});
    _db.addPort(4.5, 1.0, viaCstr);
    _db.addCircleVia(9*gridWidth, 17*gridWidth, 2, ViaType::Target);
    _db.addCircleVia(10*gridWidth, 17*gridWidth, 2, ViaType::Target);
    _db.addCircleVia(11*gridWidth, 17*gridWidth, 2, ViaType::Target);
    viaCstr = _db.clusterVia({22,23,24});
    _db.addPort(4.4, 1.0, viaCstr);

    // Obstacle
    _db.addRectObstacle(1, 8*gridWidth, 12*gridWidth, 14*gridWidth, 15*gridWidth);
    _db.addRectObstacle(2, 2*gridWidth, 8*gridWidth, 6*gridWidth, 8*gridWidth);

    // area & via weight
    _db.setFlowWeight(0.5, 0.5);
}