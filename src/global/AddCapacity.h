#ifndef ADDCAPACITY_H
#define ADDCAPACITY_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "RGraph.h"
#include "FlowLP.h"

#define PI 3.1415926

void BuildCapicityConstraint(OASGEdge* e1, OASGEdge* e2, FlowLP &solver);

double shortest_distance(double segment_x1, double segment_y1, double segment_x2, double segment_y2, double point_x, double point_y, double length_of_segment);


#endif