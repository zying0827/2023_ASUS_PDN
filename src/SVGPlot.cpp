#include "SVGPlot.h"

void SVGPlot::startPlot(int canvasW, int canvasH) {
    _fout << "<html>" << endl;
    _fout << "<body>" << endl;
    _fout << "<svg width=\"" << canvasW <<"\" height=\"" << canvasH << "\">" << endl;
}

void SVGPlot::endPlot() {
    _fout << "</svg>" << endl;
    _fout << "</body>" << endl;
    _fout << "</html>" << endl;
}

void SVGPlot::drawSquare(int leftX, int topY, int width, int colorId) {
    _fout << "  <rect x=\"" << topY << "\" y=\"" << leftX << "\" width=\"" << width << "\" height=\"" << width;
    _fout << "\" style=\"fill:" << _vColor[colorId] << ";stroke:gray;stroke-width:1\" />\n";
}

void SVGPlot::drawCircle(int centerX, int centerY, int r, int colorId) {
    _fout << "  <circle cx=\"" << centerY << "\" cy=\"" << centerX << "\" r=\"" << r;
    _fout << "\" fill=\"" << _vColor[colorId] << "\" stroke=\"gray\" stroke-width=\"1\" />\n";
}

void SVGPlot::drawLine(int x1, int y1, int x2, int y2, int colorId) {
    _fout << "  <line x1=\"" << x1 << "\" y1=\"" << 760-y1 << "\" x2=\"" << x2 << "\" y2=\"" << 760-y2;
    _fout << "\" style=\"stroke:" << _vColor[colorId] << ";stroke-width:5\" />\n";
}