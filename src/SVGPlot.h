#ifndef SVGPLOT_H
#define SVGPLOT_H

#include "Include.h"
using namespace std;

class SVGPlot {
    public:
        SVGPlot(ofstream& fout): _fout(fout) {
            _vColor = {"lightsalmon","gold","greenyellow","lightblue","mediumpurple","red", "orange", "green", "blue", "purple", "gray", "black", "white"};
        }
        ~SVGPlot() {}

        void startPlot(int canvasW, int canvasH);
        void endPlot();
        void drawSquare(int leftX, int topY, int width, int colorId);
        void drawCircle(int centerX, int centerY, int r, int colorId);
        void drawLine(int x1, int y1, int x2, int y2, int colorId);

    private:
        ofstream& _fout;
        vector<string> _vColor;
};

#endif