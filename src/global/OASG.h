#ifndef OASG_H
#define OASG_H

#include "../Include.h"
using namespace std;

class OASGNode {
    public:
        OASGNode() {}
        ~OASGNode() {}

        double x() const { return _x; }
        double y() const { return _y; }
    private:
        double _x;
        double _y;
};

class OASGEdge {
    public:
        OASGEdge() {}
        ~OASGEdge() {}

        OASGNode* sNode() { return _sNode; }
        OASGNode* tNode() { return _tNode; }

        double length() const { return _length; }
        bool cross(OASGEdge* e) {
            // reference: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
            
            auto onSegment = [] (OASGNode* p, OASGNode* q, OASGNode* r) -> bool {
                if (q->x() <= max(p->x(), r->x()) && q->x() >= min(p->x(), r->x()) &&
                    q->y() <= max(p->y(), r->y()) && q->y() >= min(p->y(), r->y()))
                return true;
            
                return false;
            };
            auto orientation = [] (OASGNode* p, OASGNode* q, OASGNode* r) -> int {
                int val = (q->y() - p->y()) * (r->x() - q->x()) -
                        (q->x() - p->x()) * (r->y() - q->y());
            
                if (val == 0) return 0;  // collinear
            
                return (val > 0)? 1: 2; // clock or counterclock wise
            };

            OASGNode* p1 = _sNode;
            OASGNode* q1 = _tNode;
            OASGNode* p2 = e->sNode();
            OASGNode* q2 = e->tNode();
            int o1 = orientation(p1, q1, p2);
            int o2 = orientation(p1, q1, q2);
            int o3 = orientation(p2, q2, p1);
            int o4 = orientation(p2, q2, q1);
        
            // General case
            if (o1 != o2 && o3 != o4)
                return true;
        
            // Special Cases
            // p1, q1 and p2 are collinear and p2 lies on segment p1q1
            if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        
            // p1, q1 and q2 are collinear and q2 lies on segment p1q1
            if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        
            // p2, q2 and p1 are collinear and p1 lies on segment p2q2
            if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        
            // p2, q2 and q1 are collinear and q1 lies on segment p2q2
            if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        
            return false; // Doesn't fall in any of the above cases
        }

        void setNode(OASGNode* sNode, OASGNode* tNode) {
            _sNode = sNode;
            _tNode = tNode;
            _length = sqrt( pow(_sNode->x() - _tNode->x(), 2) + pow(_sNode->y() - _tNode->y(), 2) );
        }
    private:
        double _length;
        OASGNode* _sNode;
        OASGNode* _tNode;
};

class OASG {

};

#endif