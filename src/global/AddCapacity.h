#ifndef ADDCAPACITY_H
#define ADDCAPACITY_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "RGraph.h"
#include "FlowLP.h"

#define PI 3.1415926

double shortest_distance(double segment_x1, double segment_y1, double segment_x2, double segment_y2, double point_x, double point_y, double length_of_segment);

void BuildCapicityConstraint(OASGEdge* e1, OASGEdge* e2, FlowLP &solver);

void AsSourceCapacityConstraint(OASGEdge* e1, double x1, double y1, double x2, double y2, FlowLP &solver);
bool AsTargetCapacityConstraint(OASGEdge* e1, double x1, double y1, double x2, double y2, FlowLP &solver);

void AddObstacleConstraint(OASGEdge* e1, Obstacle* obs, FlowLP &solver);

void AddRectangularBoardConstraint(OASGEdge* e1, double Board_width, double Board_height ,FlowLP &solver);

#endif