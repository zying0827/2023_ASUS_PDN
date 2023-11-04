#include <cstdio>
#include <gurobi_c++.h>
#include "base/Include.h"
#include "base/DB.h"
#include "base/Parser.h"
#include "global/GlobalMgr.h"
#include "base/SVGPlot.h"
#include "detailed/DetailedMgr.h"
#include "global/PreMgr.h"

using namespace std;

int main(int argc, char* argv[]){
    ifstream finST, fin, finOb;
    ofstream fout;
    finST.open(argv[1], ifstream::in);
    if (finST.is_open()) {
        cout << "input file (st components) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file (st components)" << endl;
    }
    fin.open(argv[2], ifstream::in);
    if (fin.is_open()) {
        cout << "input file is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }
    finOb.open(argv[3], ifstream::in);
    if (finOb.is_open()) {
        cout << "input file (obstacle) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }
    fout.open(argv[4], ofstream::out);
    if (fout.is_open()) {
        cout << "output file is opened successfully" << endl;
    } else {
        cerr << "Error opening output file" << endl;
    }
    // ofstream fout1;
    // fout1.open(argv[2], ofstream::out);
    // if (fout1.is_open()) {
    //     cout << "output file is opened successfully" << endl;
    // } else {
    //     cerr << "Error opening output file" << endl;
    // }

    // double gridWidth = 4;
    // double boardWidth = 15*gridWidth;
    // double boardHeight = 19*gridWidth;
    // size_t numLayers = 4;
    // double gridWidth = 8;
    // double boardWidth = 50*gridWidth;
    // double boardHeight = 15*gridWidth;
    // size_t numLayers = 12;
    double gridWidth = 1;
    double boardWidth = 75*gridWidth;
    double boardHeight = 40*gridWidth;
    size_t numLayers = 4;
    double offsetX = 40;
    double offsetY = 40;

    // SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 6.0);
    SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 10.0);
    DB db(plot);
    db.setBoundary(boardWidth, boardHeight);
    db.setFlowWeight(0.5, 0.5);
    Parser parser(finST, fin, finOb, db, offsetX, offsetY, plot);
    parser.parse();
    // NetworkMgr mgr(db, plot);
    PreMgr preMgr(db, plot);
    preMgr.nodeClustering();
    preMgr.assignPortPolygon();
    preMgr.plotBoundBox();
    
    // // replace this line with a real parser function
    // parser.testInitialize(boardWidth, boardHeight, gridWidth);

    // db.print();
    
    GlobalMgr globalMgr(db, plot);
    

    // // replace this line with a real OASG building function
    // globalMgr.buildTestOASG();
    // globalMgr.buildOASG();
    globalMgr.buildOASGXObs();
    // globalMgr.plotOASG();
    // globalMgr.layerDistribution();
    // // //globalMgr.plotRGraph();
    // globalMgr.buildTestNCOASG();
    // // globalMgr.plotNCOASG();
    // // globalMgr.voltageAssignment();
    globalMgr.genCapConstrs();
    try {
        // globalMgr.voltageDemandAssignment();
        // globalMgr.voltageAssignment();
        // globalMgr.currentDistribution();
        globalMgr.voltCurrOpt();
        // globalMgr.checkFeasible();
        // globalMgr.checkVoltDemandFeasible();
    } catch (GRBException e) {
        cerr << "Error = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
    globalMgr.plotCurrentPaths();

    // DetailedMgr detailedMgr(db, plot, 0.5);
    // detailedMgr.initGridMap();
    // // detailedMgr.plotGridMap();
    // detailedMgr.naiveAStar();
    // detailedMgr.plotGridMap();
    // detailedMgr.addViaGrid();

    // printf("\n==================== print ===================\n");
    // detailedMgr.print();

    // printf("\n==================== buildMtx ===================\n");
    // detailedMgr.buildMtx();
    // // detailedMgr.plotGridMapVoltage();
    // detailedMgr.plotGridMapCurrent();

    globalMgr.plotDB();


    // mgr.genRGraph();
    // // mgr.drawRGraph();
    // mgr.distrNet();

    // // draw routing graph
    // // mgr.drawRGraph(true);
    // mgr.drawDB();
    // fout.close();
    return 0;
}