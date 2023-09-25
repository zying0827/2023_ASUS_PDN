#include "Parser.h"
using namespace std;

void Parser::testInitialize() {
    double gridWidth = 40.0;
    _db.setBoundary(15*gridWidth, 19*gridWidth);

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


    // db.addVia(16,2,0,ViaType::Source);
    // // cerr << db.vVia(0)->rowId() << " " << db.vVia(0)->colId() << " " << db.vVia(0)->netId() << endl;
    // db.addVia(16,3,0,ViaType::Source);
    // db.addVia(17,2,0,ViaType::Source);
    // db.addVia(17,3,0,ViaType::Source);
    // db.clusterVia(0,{0,1,2,3});
    // // cerr << db.vViaCluster(0,0)->vVia(0)->rowId() << " " << db.vViaCluster(0,0)->vVia(0)->colId() << " " << db.vViaCluster(0,0)->vVia(0)->netId() << endl;
    // db.vViaCluster(0,0) -> print();
    // db.addVia(11,4,0,ViaType::Target);
    // db.addVia(11,5,0,ViaType::Target);
    // db.clusterVia(0,{4,5});
    // db.addVia(4,5,0,ViaType::Target);
    // db.addVia(4,6,0,ViaType::Target);
    // db.addVia(5,5,0,ViaType::Target);
    // db.addVia(5,6,0,ViaType::Target);
    // db.clusterVia(0,{6,7,8,9});

    // db.addVia(16,6,1,ViaType::Source);
    // db.addVia(16,7,1,ViaType::Source);
    // db.addVia(17,6,1,ViaType::Source);
    // db.addVia(17,7,1,ViaType::Source);
    // db.clusterVia(1,{10,11,12,13});
    // db.addVia(7,7,1,ViaType::Target);
    // db.addVia(7,8,1,ViaType::Target);
    // db.clusterVia(1,{14,15});
    // db.addVia(7,12,1,ViaType::Target);
    // db.addVia(7,13,1,ViaType::Target);
    // db.clusterVia(1,{16,17});

    // db.addVia(16,10,2,ViaType::Source);
    // db.addVia(16,11,2,ViaType::Source);
    // db.addVia(17,10,2,ViaType::Source);
    // db.addVia(17,11,2,ViaType::Source);
    // db.clusterVia(2,{18,19,20,21});
    // db.addVia(7,10,2,ViaType::Target);
    // db.addVia(8,10,2,ViaType::Target);
    // db.clusterVia(2,{22,23});
    // db.addVia(2,7,2,ViaType::Target);
    // db.addVia(2,8,2,ViaType::Target);
    // db.addVia(2,9,2,ViaType::Target);
    // db.addVia(2,10,2,ViaType::Target);
    // db.addVia(2,11,2,ViaType::Target);
    // db.addVia(2,12,2,ViaType::Target);
    // db.addVia(2,13,2,ViaType::Target);
    // db.clusterVia(2,{24,25,26,27,28,29,30});

    // db.addObstacle(2,4,8);
    // db.addObstacle(2,4,9);
    // db.addObstacle(2,4,10);
    // db.addObstacle(2,4,11);
    // db.addObstacle(2,4,12);
    // db.addObstacle(2,4,13);
    // for (size_t rowId = 0; rowId < 13; ++rowId) {
    //     for (size_t colId = 0; colId < 16; ++colId) {
    //         db.addObstacle(3, rowId, colId);
    //     }
    // }
}