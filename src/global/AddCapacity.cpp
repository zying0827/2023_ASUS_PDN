#include "AddCapacity.h"
#include "LayerILP.h"
#include "VoltEigen.h"
#include "FlowLP.h"


//function that find the sortest path between one point and one segment (point's foot must lay on the segment)
double shortest_distance(double segment_x1, double segment_y1, double segment_x2, double segment_y2, double point_x, double point_y, double length_of_segment, double& footx, double& footy){

    double r = ((point_x - segment_x1) * (segment_x2 - segment_x1) + (point_y - segment_y1) * (segment_y2 - segment_y1)) / pow(length_of_segment,2);
    // printf("r: %.3f\n", r);
    //projection point is on the segment
    if(r >= 0 && r <= 1){
        footx = segment_x1 + r * (segment_x2 - segment_x1);
        footy = segment_y1 + r * (segment_y2 - segment_y1);

        // printf("foot: (%.3f, %.3f\n", footx, footy);
        return sqrt(pow(footx - point_x,2) + pow(footy - point_y,2));
    }
    else{
        double dist1 = sqrt(pow(segment_x1 - point_x, 2) + pow(segment_y1 - point_y, 2));
        double dist2 = sqrt(pow(segment_x2 - point_x, 2) + pow(segment_y2 - point_y, 2));
        footx = -1;
        footy = -1;
        return fmin(dist1, dist2);
    } 
}

void BuildCapacityConstraint(OASGEdge* e1, OASGEdge* e2, FlowLP &solver){

    double v1x = e1->tNode()->x()-e1->sNode()->x();
    double v1y = e1->tNode()->y()-e1->sNode()->y();
    double v2x = e2->tNode()->x()-e2->sNode()->x();
    double v2y = e2->tNode()->y()-e2->sNode()->y();
    
    double fx = -1, fy = -1; // foot
    double min_dist = 999999;
    double dist;
    double cos = fabs((v1x*v2x + v1y*v2y)/(sqrt(v1x*v1x+v1y*v1y)*sqrt(v2x*v2x+v2y*v2y)));
    double ratio1 = -1, ratio2 = -1;
    bool right1 = 0, right2 = 0;
    
    bool detected = 0;
    
    // d(S1, T)
    dist = shortest_distance(e2->sNode()->x(), e2->sNode()->y(), e2->tNode()->x(), e2->tNode()->y(), e1->sNode()->x(), e1->sNode()->y(), sqrt((e2->sNode()->x()-e2->tNode()->x())*(e2->sNode()->x()-e2->tNode()->x()) + (e2->sNode()->y()-e2->tNode()->y())*(e2->sNode()->y()-e2->tNode()->y())), fx, fy);
    if(fx != -1) {
        detected = 1;
        min_dist = dist;
        ratio1 = cos, ratio2 = 1;

        // T is horizontal
        if(e1->sNode()->x() == fx) {
            if(e1->sNode()->y() < fy) {
                right2 = 1;
                if(v1x*v1y >= 0) // S slope >= 0
                    right1 = 0;
                else
                    right1 = 1;
            }
            else {
                right2 = 0;
                if(v1x*v1y >= 0) // S slope > 0
                    right1 = 1;
                else
                    right1 = 0;
            }
        }
        // S1 is left to foot (edge T)
        else if(e1->sNode()->x() < fx) {
            right2 = 0;

            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T

                // special case : T_slope > 0 and 0 > S_slope > -1/T_slope 
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 0;
                // special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if (e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 0;
                else
                    right1 = 1;
            }
            else
                right1 = 1;
        }
        else {
            right2 = 1;
            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                //special case : T_slope > 0 and 0 > S_slope > -1/T_slope
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 1;
                //special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if(e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 1;
                else
                    right1 = 0;
            }
            else
                right1 = 0;
        }
    }
    // d(S2, T)
    dist = shortest_distance(e2->sNode()->x(), e2->sNode()->y(), e2->tNode()->x(), e2->tNode()->y(), e1->tNode()->x(), e1->tNode()->y(), sqrt((e2->sNode()->x()-e2->tNode()->x())*(e2->sNode()->x()-e2->tNode()->x()) + (e2->sNode()->y()-e2->tNode()->y())*(e2->sNode()->y()-e2->tNode()->y())), fx, fy);
    if(fx != -1 && dist < min_dist) {
        detected = 1;
        min_dist = dist;
        ratio1 = cos, ratio2 = 1;

        // T is horizontal
        if(e1->tNode()->x() == fx) {
            if(e1->tNode()->y() < fy) {
                right2 = 1;
                if(v1x*v1y >= 0) // S slope >= 0
                    right1 = 0;
                else
                    right1 = 1;
            }
            else {
                right2 = 0;
                if(v1x*v1y >= 0) // S slope >= 0
                    right1 = 1;
                else
                    right1 = 0;
            }
        }
        // S2 is left to foot (edge T)
        else if(e1->tNode()->x() < fx) {
            right2 = 0;
            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                // special case : T_slope > 0 and 0 > S_slope > -1/T_slope 
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 0;
                // special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if (e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 0;
                else
                    right1 = 1;
            }
            else
                right1 = 1;
        }
        else {
            right2 = 1;
            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                //special case : T_slope > 0 and 0 > S_slope > -1/T_slope
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 1;
                //special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if(e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 1;
                else
                    right1 = 0;
            }
            else
                right1 = 0;
        }
    }
    // d(S, T1)
    dist = shortest_distance(e1->sNode()->x(), e1->sNode()->y(), e1->tNode()->x(), e1->tNode()->y(), e2->sNode()->x(), e2->sNode()->y(), sqrt((e1->sNode()->x()-e1->tNode()->x())*(e1->sNode()->x()-e1->tNode()->x()) + (e1->sNode()->y()-e1->tNode()->y())*(e1->sNode()->y()-e1->tNode()->y())), fx, fy);
    if(fx != -1 && dist < min_dist) {
        detected = 1;
        min_dist = dist; 
        ratio1 = 1, ratio2 = cos;

        // S is horizontal
        if(e2->sNode()->x() == fx) {
            if(e2->sNode()->y() < fy) {
                right1 = 1;
                if(v2x*v2y >= 0) // T slope >= 0
                    right2 = 0;
                else
                    right2 = 1;
            }
            else {
                right1 = 0;
                if(v2x*v2y >= 0) // T slope >= 0
                    right2 = 1;
                else
                    right2 = 0;
            }
        }
        // T2 is left to foot (edge S)
        else if(e2->sNode()->x() < fx) {
            right1 = 0;
            if(v2x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                // special case : S_slope > 0 and 0 > T_slope > -1/S_slope 
                if(e1_slope > 0 && (0 > e2_slope && e2_slope > -1/e1_slope))
                    right2 = 0;
                // special case : S_slope < 0 and -1/S_slope > T_slope >= 0
                else if (e1_slope < 0 && (-1/e1_slope > e2_slope && e2_slope >= 0))
                    right2 = 0;
                else
                    right2 = 1;
            }
            else
                right2 = 1;
        }
        else {
            right1 = 1;
            if(v2x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                //special case : S_slope > 0 and 0 > T_slope > -1/S_slope
                if(e1_slope > 0 && (0 > e2_slope && e2_slope > -1/e1_slope))
                    right2 = 1;
                //special case : S_slope < 0 and -1/S_slope > T_slope >= 0
                else if(e1_slope < 0 && (-1/e1_slope > e2_slope && e2_slope >= 0))
                    right2 = 1;
                else
                    right2 = 0;
            }
            else
                right2 = 0;
        }
    }
    // d(S, T2)
    dist = shortest_distance(e1->sNode()->x(), e1->sNode()->y(), e1->tNode()->x(), e1->tNode()->y(), e2->tNode()->x(), e2->tNode()->y(), sqrt((e1->sNode()->x()-e1->tNode()->x())*(e1->sNode()->x()-e1->tNode()->x()) + (e1->sNode()->y()-e1->tNode()->y())*(e1->sNode()->y()-e1->tNode()->y())), fx, fy);
    if(fx != -1 && dist < min_dist) {
        detected = 1;
        min_dist = dist;
        ratio1 = 1, ratio2 = cos;
        
        // S is horizontal
        if(e2->tNode()->x() == fx) {
            if(e2->tNode()->y() < fy) {
                right1 = 1;
                if(v2x*v2y >= 0) // T slope > 0
                    right2 = 0;
                else
                    right2 = 1;
            }
            else {
                right1 = 0;
                if(v2x*v2y >= 0) // T slope > 0
                    right2 = 1;
                else
                    right2 = 0;
            }
        }
        // T2 is left to foot (edge S)
        else if(e2->tNode()->x() < fx) {
            right1 = 0;
            if(v2x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                // special case : S_slope > 0 and 0 > T_slope > -1/S_slope 
                if(e1_slope > 0 && (0 > e2_slope && e2_slope > -1/e1_slope))
                    right2 = 0;
                // special case : S_slope < 0 and -1/S_slope > T_slope >= 0
                else if (e1_slope < 0 && (-1/e1_slope > e2_slope && e2_slope >= 0))
                    right2 = 0;
                else
                    right2 = 1;
            }
            else 
                right2 = 1;
        }
        else {
            right1 = 1;
            if(v2x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                //special case : S_slope > 0 and 0 > T_slope > -1/S_slope
                if(e1_slope > 0 && (0 > e2_slope && e2_slope > -1/e1_slope))
                    right2 = 1;
                //special case : S_slope < 0 and -1/S_slope > T_slope >= 0
                else if(e1_slope < 0 && (-1/e1_slope > e2_slope && e2_slope >= 0))
                    right2 = 1;
                else
                    right2 = 0;
            }
            else
                right2 = 0;
        }
    }
    //add constraint
    if(detected == 1){
        double width = min_dist;
        cout << "add constraint => " << "width :" << width << " | ratio1 : " << ratio1 << " | ratio2 : "<< ratio2 << endl;
        cout << "right1 : " << right1 << " right2 : " << right2 << endl;
        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
    }
}

void BuildObstacleCapacityConstraint(OASGEdge* e1, double x1, double y1, double x2, double y2, FlowLP &solver){
    double v1x = e1->tNode()->x()-e1->sNode()->x();
    double v1y = e1->tNode()->y()-e1->sNode()->y();
    double v2x = x2 - x1;
    double v2y = y2 - y1;
    
    double fx = -1, fy = -1; // foot
    double min_dist = 999999;
    double dist;
    double cos = fabs((v1x*v2x + v1y*v2y)/(sqrt(v1x*v1x+v1y*v1y)*sqrt(v2x*v2x+v2y*v2y)));
    double ratio1 = -1;
    bool right1 = 0;

    bool detected = 0;
    
    // d(S1, T)
    dist = shortest_distance(x1, y1, x2, y2, e1->sNode()->x(), e1->sNode()->y(), sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)), fx, fy);
    if(fx != -1) {
        detected = 1;
        min_dist = dist;
        ratio1 = cos;

        // T is horizontal
        if(e1->sNode()->x() == fx) {
            if(e1->sNode()->y() < fy) {
                if(v1x*v1y >= 0) // S slope >= 0
                    right1 = 0;
                else
                    right1 = 1;
            }
            else {
                if(v1x*v1y >= 0) // S slope > 0
                    right1 = 1;
                else
                    right1 = 0;
            }
        }
        // S1 is left to foot (edge T)
        else if(e1->sNode()->x() < fx) {
            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                // special case : T_slope > 0 and 0 > S_slope > -1/T_slope 
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 0;
                // special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if (e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 0;
                else
                    right1 = 1;
            }
            else
                right1 = 1;
        }
        else {
            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                //special case : T_slope > 0 and 0 > S_slope > -1/T_slope
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 1;
                //special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if(e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 1;
                else
                    right1 = 0;
            }
            else
                right1 = 0;
        }
    }
    // d(S2, T)
    dist = shortest_distance(x1, y1, x2, y2, e1->tNode()->x(), e1->tNode()->y(), sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)), fx, fy);
    if(fx != -1 && dist < min_dist) {
        detected = 1;
        min_dist = dist;
        ratio1 = cos;

        // T is horizontal
        if(e1->tNode()->x() == fx) {
            if(e1->tNode()->y() < fy) {
                if(v1x*v1y >= 0) // S slope >= 0
                    right1 = 0;
                else
                    right1 = 1;
            }
            else {
                if(v1x*v1y >= 0) // S slope >= 0
                    right1 = 1;
                else
                    right1 = 0;
            }
        }
        // S2 is left to foot (edge T)
        else if(e1->tNode()->x() < fx) {
            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                // special case : T_slope > 0 and 0 > S_slope > -1/T_slope 
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 0;
                // special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if (e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 0;
                else
                    right1 = 1;
            }
            else
                right1 = 1;
        }
        else {
            if(v1x != 0){
                double e1_slope = (v1y)/(v1x);//S
                double e2_slope = (v2y)/(v2x);//T
                //special case : T_slope > 0 and 0 > S_slope > -1/T_slope
                if(e2_slope > 0 && (0 > e1_slope && e1_slope > -1/e2_slope))
                    right1 = 1;
                //special case : T_slope < 0 and -1/T_slope > S_slope >= 0
                else if(e2_slope < 0 && (-1/e2_slope > e1_slope && e1_slope >= 0))
                    right1 = 1;
                else
                    right1 = 0;
            }
            else
                right1 = 0;
        }
    }
    // d(S, T1)
    dist = shortest_distance(e1->sNode()->x(), e1->sNode()->y(), e1->tNode()->x(), e1->tNode()->y(), x1, y1, sqrt((e1->sNode()->x()-e1->tNode()->x())*(e1->sNode()->x()-e1->tNode()->x()) + (e1->sNode()->y()-e1->tNode()->y())*(e1->sNode()->y()-e1->tNode()->y())), fx, fy);
    if(fx != -1 && dist < min_dist) {
        detected = 1;
        min_dist = dist; 
        ratio1 = 1;
        // S is horizontal
        if(x1 == fx) {
            if(y1 < fy) 
                right1 = 1;
            else 
                right1 = 0;
        }
        // T2 is left to foot (edge S)
        else if(x1 < fx) 
            right1 = 0;
        else 
            right1 = 1;
    }
    // d(S, T2)
    dist = shortest_distance(e1->sNode()->x(), e1->sNode()->y(), e1->tNode()->x(), e1->tNode()->y(), x2, y2, sqrt((e1->sNode()->x()-e1->tNode()->x())*(e1->sNode()->x()-e1->tNode()->x()) + (e1->sNode()->y()-e1->tNode()->y())*(e1->sNode()->y()-e1->tNode()->y())), fx, fy);
    if(fx != -1 && dist < min_dist) {
        detected = 1;
        min_dist = dist;
        ratio1 = 1;
        
        // S is horizontal
        if(x2 == fx) {
            if(y2 < fy) 
                right1 = 1;
            else 
                right1 = 0;
        }
        // T2 is left to foot (edge S)
        else if(x2 < fx) 
            right1 = 0;
        else
            right1 = 1;
    }
    //add constraint
    if(detected == 1){
        double width = min_dist;
        cout << "add obstacle constraint => " << "width :" << width << " | ratio1 : " << ratio1 << endl;
        cout << "right1 : " << right1 << endl;
        solver.addCapacityConstraints(e1,right1,ratio1,width);
    }
}

void AddObstacleConstraint(OASGEdge* e1, Obstacle* obs, FlowLP &solver){

    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;

    for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
        
        //bottom edge
        x1 = obs->vShape(shapeId)-> minX();
        y1 = obs->vShape(shapeId)-> minY();
        x2 = obs->vShape(shapeId)-> maxX();
        y2 = obs->vShape(shapeId)-> minY();
        //cout << "bottom edge" << endl;
        //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);
        

        //left edge
        x1 = obs->vShape(shapeId)-> minX();
        y1 = obs->vShape(shapeId)-> minY();
        x2 = obs->vShape(shapeId)-> minX();
        y2 = obs->vShape(shapeId)-> maxY();
        //cout << "left edge" << endl;
        //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);

        //top edge
        x1 = obs->vShape(shapeId)-> minX();
        y1 = obs->vShape(shapeId)-> maxY();
        x2 = obs->vShape(shapeId)-> maxX();
        y2 = obs->vShape(shapeId)-> maxY();
        //cout << "top edge" << endl;
        //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);
    
        //right edge
        x1 = obs->vShape(shapeId)-> maxX();
        y1 = obs->vShape(shapeId)-> minY();
        x2 = obs->vShape(shapeId)-> maxX();
        y2 = obs->vShape(shapeId)-> maxY();
        //cout << "right edge" << endl;
        //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);
    }
}

void AddRectangularBoardConstraint(OASGEdge* e1, double Board_width, double Board_height ,FlowLP &solver){

    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    //bottom edge
    x1 = 0;
    y1 = 0;
    x2 = Board_width;
    y2 = 0;
    //cout << "bottom edge" << endl;
    //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
    //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
    BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);

    //left edge
    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = Board_height;
    //cout << "left edge" << endl;
    //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
    //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
    BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);

    //top edge
    x1 = 0;
    y1 = Board_height;
    x2 = Board_width;
    y2 = Board_height;
    //cout << "top edge" << endl;
    //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
    //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
    BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);

    //right edge
    x1 = Board_width;
    y1 = 0;
    x2 = Board_width;
    y2 = Board_height;
    //cout << "right edge" << endl;
    //cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
    //cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
    BuildObstacleCapacityConstraint(e1,x1,y1,x2,y2,solver);

}