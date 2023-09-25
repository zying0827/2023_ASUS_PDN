#ifndef GLOBAL_MGR
#define GLOBAL_MGR

#include "../Include.h"
#include "../SVGPlot.h"
#include "../DB.h"
#include "RGraph.h"

class GlobalMgr {
    public:
        GlobalMgr(DB& db, SVGPlot& plot): _db(db), _plot(plot) {
            cerr << "numNets = " << _db.numNets() << endl;
            _rGraph.initRGraph(db);
        }
        ~GlobalMgr() {}

        void buildTestOASG();
        void plotOASG();
        void plotRGraph();
        void layerDistribution();
    private:
        // void constructRGraph();
        // void DFS(OASGNode* node, vector< vector<OASGEdge*> >& paths);
        // vector< vector<OASGEdge*> > DFS(OASGNode* node);
        DB& _db;
        SVGPlot& _plot;
        RGraph _rGraph;
};

#endif