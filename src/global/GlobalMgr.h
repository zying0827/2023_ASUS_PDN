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
        void plotOASG();
        void plotRGraph();
        void layerDistribution();
        void currentDistribution();
    private:
        // void constructRGraph();
        // void DFS(OASGNode* node, vector< vector<OASGEdge*> >& paths);
        // vector< vector<OASGEdge*> > DFS(OASGNode* node);
        DB& _db;
        SVGPlot& _plot;
        RGraph _rGraph;
};

#endif