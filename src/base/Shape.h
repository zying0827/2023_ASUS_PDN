#ifndef SHAPE_H
#define SHAPE_H

#include "Include.h"
#include "SVGPlot.h"
using namespace std;

class Shape {
    public:
        Shape(SVGPlot& plot) : _plot(plot) {}
        ~Shape() {}
        // double ctrX() const { return _center.first; }
        // double ctrY() const { return _center.second; }
        virtual double ctrX() { double ctrX; return ctrX; }
        virtual double ctrY() { double ctrY; return ctrY; }
        virtual void print() {}
        virtual void plot(size_t colorId, size_t layId) {}
    protected:
        SVGPlot& _plot;
        // pair<double, double> _center;
};

class Polygon : public Shape {
    public:
        Polygon(vector< pair<double, double> > vVtx, SVGPlot& plot) : _vVtx(vVtx), Shape(plot) {}
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
        void plot(size_t colorId, size_t layId) {
            _plot.drawPolygon(_vVtx, colorId, layId);
        }

    private:
        vector< pair<double, double> > _vVtx;
};

class Circle : public Shape {
    public:
        Circle(double ctrX, double ctrY, double radius, SVGPlot& plot) : _radius(radius), Shape(plot) {
            _ctr = make_pair(ctrX, ctrY);
        }
        ~Circle() {}
        double ctrX() { return _ctr.first; }
        double ctrY() { return _ctr.second; }
        double radius() const { return _radius; }
        void print() {
            cerr << "Circle {center=(" << _ctr.first << " " << _ctr.second << "), radius=" << _radius << "}" << endl;
        }
        void plot(size_t colorId, size_t layId) {
            _plot.drawCircle(_ctr.first, _ctr.second, _radius, colorId, layId);
        }

    private:
        pair<double, double> _ctr;
        double _radius;
};

class Node : public Shape {
    public:
        Node(double ctrX, double ctrY, SVGPlot& plot) : Shape(plot) {
            _ctr = make_pair(ctrX, ctrY);
        }
        ~Node() {}
        double ctrX() { return _ctr.first; }
        double ctrY() { return _ctr.second; }
        void print() {
            cerr << "Node {center=(" << _ctr.first << " " << _ctr.second << ")}" << endl;
        }
        void plot(size_t colorId, size_t layId) {
            _plot.drawCircle(_ctr.first, _ctr.second, 2, colorId, layId);
        } 
    private:
        pair<double, double> _ctr;
};

class Trace : public Shape {
    public:
        Trace(Node* sNode, Node* tNode, double width, SVGPlot& plot) : _sNode(sNode), _tNode(tNode), _width(width), Shape(plot) {}
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

        void plot(size_t colorId, size_t layId) {
            _plot.drawLine(_sNode->ctrX(), _sNode->ctrY(), _tNode->ctrX(), _tNode->ctrY(), colorId, layId);
        }

    private:
        Node* _sNode;
        Node* _tNode;
        double _width;
};

#endif