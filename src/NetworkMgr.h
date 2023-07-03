#ifndef NETWORKMGR_H
#define NETWORKMGR_H

#include "Include.h"
#include "DB.h"
#include "TNode.h"
#include "TEdge.h"
#include "NetworkILP.h"

class NetworkMgr {
    public:

    private:
        DB& _db;
        vector<TNode> _vFixedViaNode;   // nodes occupied by fixed via clusters
        vector<TNode> _vAllClearNode;   // nodes with all 

};

#endif
