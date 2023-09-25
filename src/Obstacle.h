#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Include.h"
#include "Shape.h"
using  namespace std;

class Obstacle {
    public:
        Obstacle(vector<Shape*> vShape) : _vShape(vShape) {}
        ~Obstacle() {}
        void print() {
            cerr << "Obstacle {vShape=" << endl;
            for (size_t shapeId = 0; shapeId < _vShape.size(); ++ shapeId) {
                _vShape[shapeId]->print();
            }
            cerr << "}" << endl;
        }
    private:
        vector<Shape*> _vShape;
};

#endif