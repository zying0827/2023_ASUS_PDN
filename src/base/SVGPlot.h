#ifndef SVGPLOT_H
#define SVGPLOT_H

#include "Include.h"
using namespace std;

enum SVGPlotColor {
    lightsalmon,gold,greenyellow,lightblue,mediumpurple,red,orange,green,blue,purple,gray,black,white
};

class SVGPlot {
    public:
        SVGPlot(ofstream& fout, double boardWidth, double boardHeight, double gridWidth, size_t numLayers)
        : _fout(fout), _boardWidth(boardWidth), _boardHeight(boardHeight), _gridWidth(gridWidth) {
            _vColor = {"lightsalmon","gold","greenyellow","lightblue","mediumpurple","red", "orange", "green", "blue", "purple", "gray", "black", "white"};
            startPlot(_boardWidth*numLayers, _boardHeight);
        }
        ~SVGPlot() {}

        void startPlot(double canvasW, double canvasH);
        void endPlot();
        void drawSquare(double leftX, double topY, double width, size_t colorId, size_t layId);
        void drawRect(double leftX, double bottY, double width, double height, size_t colorId, size_t layId);
        void drawCircle(double centerX, double centerY, double r, size_t colorId, size_t layId);
        void drawLine(double x1, double y1, double x2, double y2, size_t colorId, size_t layId, double width = 5);
        void drawPolygon(vector< pair<double, double> > vVtx, size_t colorId, size_t layId);

        double gridWidth() const { return _gridWidth; }

    private:
        ofstream& _fout;
        vector<string> _vColor;
        double _gridWidth;
        double _boardWidth;
        double _boardHeight;
};

#endif