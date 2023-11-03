#ifndef GLOBAL_MGR
#define GLOBAL_MGR

#include "../base/Include.h"
#include "../base/SVGPlot.h"
#include "../base/DB.h"
#include "RGraph.h"

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

class GlobalMgr {
    public:
        GlobalMgr(DB& db, SVGPlot& plot): _db(db), _plot(plot) {
            cerr << "numNets = " << _db.numNets() << endl;
            _rGraph.initRGraph(db);
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector<double> temp(_rGraph.numViaOASGEdges(netId), -1);
                _vUBViaArea.push_back(temp);
            }
        }
        ~GlobalMgr() {}

        void plotDB();
        void buildTestOASG();
        void buildOASG();
        void buildOASGXObs();

        bool isSegmentIntersectingWithObstacles(OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle);
        bool onSegment(OASGNode* p, OASGNode* q, OASGNode* r);
        int orientation(OASGNode* p, OASGNode* q, OASGNode* r);
        bool doIntersect(OASGNode* p1, OASGNode* q1, OASGNode* p2, OASGNode* q2);
        void connectWithObstacle(int netId, int layerId,OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle);
        bool checkWithVias(int netId, int layerId, OASGNode* a, OASGNode* b, vector<vector<vector<OASGNode*>>> viaOASGNodes);

        void plotOASG();
        void plotRGraph();
        void layerDistribution();
        void buildTestNCOASG();
        void plotNCOASG();
        void genCapConstrs();
        void voltCurrOpt();
        void voltageAssignment(bool currentBased);
        void voltageDemandAssignment();
        void currentDistribution();
        void plotCurrentPaths();
        void checkFeasible(bool currentBased);
        void checkVoltDemandFeasible();
    private:
        Trace* edge2Trace(OASGEdge* edge);
        Segment* edge2Segment(OASGEdge* edge);
        // void constructRGraph();
        // void DFS(OASGNode* node, vector< vector<OASGEdge*> >& paths);
        // vector< vector<OASGEdge*> > DFS(OASGNode* node);
        DB& _db;
        SVGPlot& _plot;
        RGraph _rGraph;
        vector<double> _vArea;  // record the plane area of each iteration in voltCurrOpt
        vector<double> _vViaArea;
        vector<double> _vOverlap;   // record the overlapped width of each iteration in voltCurrOpt
        vector<CapConstr> _vCapConstr;
        vector<SingleCapConstr> _vSglCapConstr;
        vector< vector< double > > _vUBViaArea;     // the upper bound of a via area, index = [netId] [vEdgeId]
};

#endif