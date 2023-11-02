#ifndef ADDCAPACITY_H
#define ADDCAPACITY_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "RGraph.h"
#include "FlowLP.h"

#define PI 3.1415926

double shortest_distance(pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>&);
bool isRight(pair<double, double>, pair<double, double>, pair<double, double>);
bool addConstraint(pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>, pair<double, double>&, pair<bool, bool>&, double&, bool isSameNet = false);

#endif