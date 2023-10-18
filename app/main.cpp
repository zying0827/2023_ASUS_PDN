#include <gurobi_c++.h>
#include "base/Include.h"
#include "base/DB.h"
#include "base/Parser.h"
#include "global/GlobalMgr.h"
#include "base/SVGPlot.h"
#include "detailed/DetailedMgr.h"

using namespace std;

int main(int argc, char* argv[]){
    ifstream fin;
    ofstream fout;
    fin.open(argv[1], ifstream::in);
    if (fin.is_open()) {
        cout << "input file is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }
    fout.open(argv[2], ofstream::out);
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

    double gridWidth = 40;
    double boardWidth = 15*gridWidth;
    double boardHeight = 19*gridWidth;
    size_t numLayers = 4;
    // double gridWidth = 8;
    // double boardWidth = 50*gridWidth;
    // double boardHeight = 15*gridWidth;
    // size_t numLayers = 12;

    SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 1.0);
    // SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 5.0);
    DB db(plot);
    Parser parser(fin, db, plot);
    // parser.parse();
    // NetworkMgr mgr(db, plot);
    
    // replace this line with a real parser function
    parser.testInitialize(boardWidth, boardHeight, gridWidth);

    // db.print();
    
    GlobalMgr globalMgr(db, plot);
    

    // replace this line with a real OASG building function
    globalMgr.buildTestOASG();
    // globalMgr.plotOASG();
    globalMgr.layerDistribution();
    // globalMgr.plotRGraph();
    globalMgr.buildTestNCOASG();
    // globalMgr.plotNCOASG();
    globalMgr.voltageAssignment();
    try {
        globalMgr.currentDistribution();
    } catch (GRBException e) {
        cerr << "Error = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
    // globalMgr.plotCurrentPaths();

    DetailedMgr detailedMgr(db, plot, 10);
    detailedMgr.initGridMap();
    // detailedMgr.plotGridMap();
    detailedMgr.naiveAStar();
    detailedMgr.plotGridMap();

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