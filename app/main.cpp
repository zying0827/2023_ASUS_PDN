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

    //羅：這裡用來讀參數
    //################################
    // 定义一个map来存储参数
    std::map<std::string, int> parameters;
    int numVIter, numIIter, numIVIter; 

    // 打开参数文件
    string root_dir = "/home/b08901047/2023_ASUS_PDN";
    string parameter_dir = "/exp/input/parameters.txt";
    string inputFile_dir = root_dir + parameter_dir;
    std::ifstream input_file(inputFile_dir);

    if (input_file.is_open()) {
        std::string line;

        // 逐行读取文件
        while (std::getline(input_file, line)) {
            size_t delimiter_pos = line.find("=");
            if (delimiter_pos != std::string::npos) {
                // 提取键和值
                std::string key = line.substr(0, delimiter_pos);
                std::string value_str = line.substr(delimiter_pos + 1);
                // 去除前导和尾随空格
                key = key.substr(key.find_first_not_of(" "), key.find_last_not_of(" ") + 1);
                value_str = value_str.substr(value_str.find_first_not_of(" "), value_str.find_last_not_of(" ") + 1);
                
                // 将值转换为整数
                int value = std::stoi(value_str);

                // 存储到map中
                parameters[key] = value;
            }
        }

        input_file.close();

        // 现在你可以访问这些参数
        if (parameters.find("numIVIter") != parameters.end()) {
            numIVIter = parameters["numIVIter"];
            std::cout << "numIVIter: " << numIVIter << std::endl;
        }

        if (parameters.find("numIIter") != parameters.end()) {
            numIIter = parameters["numIIter"];
            std::cout << "numIIter: " << numIIter << std::endl;
        }

        if (parameters.find("numVIter") != parameters.end()) {
            numVIter = parameters["numVIter"];
            std::cout << "numVIter: " << numVIter << std::endl;
        }
    } else {
        std::cerr << "无法打开参数文件 'parameters.txt'" << std::endl;
    }

    //#################################
    //




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
    globalMgr.numIIter = numIIter;
    globalMgr.numVIter = numVIter;
    globalMgr.numIVIter = numIVIter;

    

    // // replace this line with a real OASG building function
    // globalMgr.buildTestOASG();

    globalMgr.buildOASG();
    // globalMgr.buildOASGXObs();
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
    
    // DetailedMgr detailedMgr(db, plot, 2 * db.VIA16D8A24()->drillRadius());
    // detailedMgr.initGridMap();
    // detailedMgr.check();
    // // detailedMgr.plotGridMap();
    // detailedMgr.naiveAStar();
    // detailedMgr.check();
    // // detailedMgr.plotGridMap();
    // detailedMgr.addPortVia();
    // detailedMgr.check();
    // // // detailedMgr.plotVia();
    // detailedMgr.addViaGrid();
    // detailedMgr.check();

    // // printf("\n==================== print ===================\n");
    // // detailedMgr.print();

    // // printf("\n==================== buildMtx ===================\n");
    // detailedMgr.buildMtx();
    // detailedMgr.plotGridMapVoltage();
    // // detailedMgr.plotGridMapCurrent();

    globalMgr.plotDB();


    // // mgr.genRGraph();
    // // // mgr.drawRGraph();
    // // mgr.distrNet();

    // // // draw routing graph
    // // // mgr.drawRGraph(true);
    // // mgr.drawDB();
    // // fout.close();
    return 0;
}