#ifndef GLOBAL_MGR
#define GLOBAL_MGR

#include "../base/Include.h"
#include "../base/SVGPlot.h"
#include "../base/DB.h"
#include "RGraph.h"

class GlobalMgr {
    public:
        GlobalMgr(DB& db, SVGPlot& plot): _db(db), _plot(plot) {
            cerr << "numNets = " << _db.numNets() << endl;
            _rGraph.initRGraph(db);
        }
        ~GlobalMgr() {}

        void plotDB();
        void buildTestOASG();
        void buildOASG();

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
        void voltageAssignment();
        void currentDistribution();
        void plotCurrentPaths();
    private:
        Trace* edge2Trace(OASGEdge* edge);
        // void constructRGraph();
        // void DFS(OASGNode* node, vector< vector<OASGEdge*> >& paths);
        // vector< vector<OASGEdge*> > DFS(OASGNode* node);
        DB& _db;
        SVGPlot& _plot;
        RGraph _rGraph;
};

#endif