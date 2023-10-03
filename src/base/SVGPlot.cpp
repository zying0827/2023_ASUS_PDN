#include "SVGPlot.h"

void SVGPlot::startPlot(double canvasW, double canvasH) {
    _fout << "<html>" << endl;
    _fout << "<body>" << endl;
    _fout << "<svg width=\"" << canvasW <<"\" height=\"" << canvasH << "\">" << endl;
    drawRect(0, 0, canvasW, canvasH, SVGPlotColor::black, 0);
}

void SVGPlot::endPlot() {
    _fout << "</svg>" << endl;
    _fout << "</body>" << endl;
    _fout << "</html>" << endl;
}

void SVGPlot::drawSquare(double leftX, double bottY, double width, size_t colorId, size_t layId) {
    _fout << "  <rect x=\"" << leftX + _boardWidth*layId << "\" y=\"" << bottY << "\" width=\"" << width << "\" height=\"" << width;
    _fout << "\" style=\"fill:" << _vColor[colorId] << ";stroke:gray;stroke-width:1\" />\n";
}

void SVGPlot::drawRect(double leftX, double bottY, double width, double height, size_t colorId, size_t layId) {
    _fout << "  <rect x=\"" << leftX + _boardWidth*layId << "\" y=\"" << bottY << "\" width=\"" << width << "\" height=\"" << height;
    _fout << "\" style=\"fill:" << _vColor[colorId] << ";stroke:gray;stroke-width:1\" />\n";
}

void SVGPlot::drawCircle(double centerX, double centerY, double r, size_t colorId, size_t layId) {
    _fout << "  <circle cx=\"" << centerX + _boardWidth*layId << "\" cy=\"" << _boardHeight - centerY << "\" r=\"" << r;
    _fout << "\" fill=\"" << _vColor[colorId] << "\" stroke=\"gray\" stroke-width=\"1\" />\n";
}

void SVGPlot::drawLine(double x1, double y1, double x2, double y2, size_t colorId, size_t layId, double width) {
    _fout << "  <line x1=\"" << x1 + _boardWidth*layId << "\" y1=\"" << _boardHeight-y1 << "\" x2=\"" << x2 + _boardWidth*layId << "\" y2=\"" << _boardHeight-y2;
    _fout << "\" style=\"stroke:" << _vColor[colorId] << ";stroke-width:" << width << "\" />\n";
}

void SVGPlot::drawPolygon(vector< pair<double, double> > vVtx, size_t colorId, size_t layId) {
    _fout << "  <polygon points=\"";
    for (size_t vtxId = 0; vtxId < vVtx.size(); ++ vtxId) {
        _fout << vVtx[vtxId].first + _boardWidth*layId << "," << _boardHeight - vVtx[vtxId].second;
        if (vtxId < vVtx.size()-1) _fout << " ";
    }
    _fout << "\" style=\"fill:" << _vColor[colorId] << ";stroke:gray;stroke-width:1\" />" << endl;
}