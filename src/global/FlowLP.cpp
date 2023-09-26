#include "FlowLP.h"

FlowLP::FlowLP(RGraph& rGraph) : _model(_env), _rGraph(rGraph) {}

void FlowLP::setObjective(){}
void FlowLP::setConserveConstraints(){}
void FlowLP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width){}
void FlowLP::solve(){}
void FlowLP::collectResult(){}
void FlowLP::printResult(){}