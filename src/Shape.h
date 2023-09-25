#ifndef SHAPE_H
#define SHAPE_H

#include "Include.h"
using namespace std;

class Shape {
    public:
        Shape() {}
        ~Shape() {}
        // double ctrX() const { return _center.first; }
        // double ctrY() const { return _center.second; }
        virtual double ctrX() { double ctrX; return ctrX; }
        virtual double ctrY() { double ctrY; return ctrY; }
        virtual void print() {}
    protected:
        // pair<double, double> _center;
};

class Polygon : public Shape {
    public:
        Polygon(vector< pair<double, double> > vVtx) : _vVtx(vVtx) {}
        ~Polygon(){}
        size_t numVtcs() const { return _vVtx.size(); }
        double vtxX(size_t vtxIdx) const { return _vVtx[vtxIdx].first;}
        double vtxY(size_t vtxIdx) const { return _vVtx[vtxIdx].second;}
        double ctrX() {
            double sumX = 0;
            for (size_t vtxId = 0; vtxId < _vVtx.size(); ++ vtxId) {
                sumX += _vVtx[vtxId].first;
            }
            return sumX / _vVtx.size();
        }
        double ctrY() {
            double sumY = 0;
            for (size_t vtxId = 0; vtxId < _vVtx.size(); ++ vtxId) {
                sumY += _vVtx[vtxId].second;
            }
            return sumY / _vVtx.size();
        }
        void print() {
            cerr << "Polygon {vVtx= ";
            for (size_t vtxId = 0; vtxId < _vVtx.size(); ++ vtxId) {
                cerr << "(" << _vVtx[vtxId].first << " " << _vVtx[vtxId].second << "), ";
            }
            cerr << "}" << endl;
        }
    private:
        vector< pair<double, double> > _vVtx;
};

class Circle : public Shape {
    public:
        Circle(double ctrX, double ctrY, double radius) : _radius(radius) {
            _ctr = make_pair(ctrX, ctrY);
        }
        ~Circle() {}
        double ctrX() { return _ctr.first; }
        double ctrY() { return _ctr.second; }
        void print() {
            cerr << "Circle {center=(" << _ctr.first << " " << _ctr.second << "), radius=" << _radius << "}" << endl;
        }
    private:
        pair<double, double> _ctr;
        double _radius;
};

class Node : public Shape {
    public:
        Node(double ctrX, double ctrY) {
            _ctr = make_pair(ctrX, ctrY);
        }
        ~Node() {}
        double ctrX() { return _ctr.first; }
        double ctrY() { return _ctr.second; }
        void print() {
            cerr << "Node {center=(" << _ctr.first << " " << _ctr.second << ")}" << endl;
        }
    private:
        pair<double, double> _ctr;
};

class Trace : public Shape {
    public:
        Trace(Node* sNode, Node* tNode, double width) : _sNode(sNode), _tNode(tNode), _width(width) {}
        ~Trace() {}
        Node* sNode() { return _sNode; }
        Node* tNode() { return _tNode; }
        double width() const { return _width; }
        void print() {
            cerr << "Trace {" << endl;
            cerr << "sNode=";
            _sNode->print();
            cerr << ", tNode=";
            _tNode->print();
            cerr << ", width=" << _width << endl;
            cerr << "}" << endl;
        }
    private:
        Node* _sNode;
        Node* _tNode;
        double _width;
};

#endif