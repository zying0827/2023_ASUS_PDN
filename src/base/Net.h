#ifndef NET_H
#define NET_H

#include "Include.h"
#include "Via.h"
#include "Shape.h"
using namespace std;

class Port {
    public:
        Port(size_t portId, double voltage, double current, ViaCluster* viaCstr)
        : _portId(portId), _voltage(voltage), _current(current), _viaCluster(viaCstr) {}
        ~Port() {}
        size_t portId() const { return _portId; }
        double voltage() const { return _voltage; }
        double current() const { return _current; }
        ViaCluster* viaCluster() { return _viaCluster; }
        void print() {
            cerr << "Port {portId=" << _portId << ", voltage=" << _voltage << ", current=" << _current << endl;
            cerr << ", viaCluster=";
            _viaCluster->print();
            cerr << "}" << endl;
        }
    private:
        size_t _portId;
        double _voltage;
        double _current;
        ViaCluster* _viaCluster;
};

// class TwoPinNet {
//     public:
//         TwoPinNet() {}
//         ~TwoPinNet() {}
//         size_t TwoPinNetId() const { return _TwoPinNetId; }
//     private:
//         size_t _TwoPinNetId;
//         ViaCluster* _sourceViaCstr;
//         ViaCluster* _targetViaCstr;
//         vector<Shape*> _vShape;
// };

class Net {
    public:
        Net(size_t numLayers) {
            vector<Trace*> tempTrace;
            vector<Shape*> tempShape;
            for (size_t layId = 0; layId < numLayers; ++ layId) {
                _vTrace.push_back(tempTrace);
                _vShape.push_back(tempShape);
            }
        }
        ~Net() {}
        ViaCluster* sourceViaCstr()                   { return _sourcePort->viaCluster(); }
        ViaCluster* vTargetViaCstr(size_t netTPortId) { return _vTargetPort[netTPortId]->viaCluster(); }
        ViaCluster* vAddedViaCstr(size_t aViaCstrIdx) { return _vAddedViaCstr[aViaCstrIdx]; }
        Port*       sourcePort()                      { return _sourcePort; }
        Port*       targetPort(size_t netTPortId)     { return _vTargetPort[netTPortId]; }
        Trace*      vTrace(size_t layId, size_t traceId) { return _vTrace[layId][traceId]; }
        size_t      numTPorts() const                 { return _vTargetPort.size(); }
        size_t      numTraces(size_t layId) const     { return _vTrace[layId].size(); }

        void addSPort(Port* port)                 { _sourcePort = port; }
        void addTPort(Port* port)                 { _vTargetPort.push_back(port); }
        void addAddedViaCstr(ViaCluster* viaCstr) { _vAddedViaCstr.push_back(viaCstr); }
        void addTrace(Trace* trace, size_t layId) { _vTrace[layId].push_back(trace); }

        void print() {
            cerr << "Net {" << endl;
            cerr << "sourcePort=";
            _sourcePort->print();
            cerr << "vTargetPort=" << endl;
            for (size_t tPortId = 0; tPortId < _vTargetPort.size(); ++ tPortId) {
                _vTargetPort[tPortId]->print();
            }
            cerr << "vAddedViaCstr=" << endl;
            for (size_t addedViaId = 0; addedViaId < _vAddedViaCstr.size(); ++ addedViaId) {
                _vAddedViaCstr[addedViaId] -> print();
            }
            cerr << "vShape=" << endl;
            for (size_t layId = 0; layId < _vShape.size(); ++ layId) {
                cerr << "layId=" << layId << "{" << endl;
                for (size_t shapeId = 0; shapeId < _vShape[layId].size(); ++ shapeId) {
                    _vShape[layId][shapeId]->print();
                }
                cerr << "}" << endl;
            }
            cerr << "}" << endl;

        }

    private:
        Port*                    _sourcePort;
        vector<Port*>            _vTargetPort;
        // ViaCluster* _sourceViaCstr;
        // vector<ViaCluster*> _vTargetViaCstr;
        vector<ViaCluster*>      _vAddedViaCstr;
        vector< vector<Shape*> > _vShape;   // index = [layId] [shapeId]
        vector< vector<Trace*> > _vTrace;   // index = [layId] [traceId], assigned in current distribution
};


#endif