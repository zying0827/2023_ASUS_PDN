#include "Include.h"
#include "DB.h"
#include "Parser.h"
#include "global/GlobalMgr.h"
#include "SVGPlot.h"

using namespace std;

int main(int argc, char* argv[]){
    ofstream fout;
    fout.open(argv[1], ofstream::out);
    if (fout.is_open()) {
        cout << "output file is opened successfully" << endl;
    } else {
        cerr << "Error opening output file" << endl;
    }

    SVGPlot plot(fout);
    // DB db(3, 4, 20, 16);
    DB db;
    Parser parser(db);
    // NetworkMgr mgr(db, plot);
    // test initialize
    parser.testInitialize();
    // db.print();
    GlobalMgr globalMgr(db, plot);
    globalMgr.buildTestOASG();
    // globalMgr.plotOASG();
    globalMgr.layerDistribution();
    globalMgr.plotRGraph();

    


    // mgr.genRGraph();
    // // mgr.drawRGraph();
    // mgr.distrNet();

    // // draw routing graph
    // // mgr.drawRGraph(true);
    // mgr.drawDB();
    // fout.close();
    return 0;
}