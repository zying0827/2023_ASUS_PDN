#include "AddCapacity.h"
#include "LayerILP.h"
#include "VoltEigen.h"
#include "FlowLP.h"
#include <cstdio>


//function that find the sortest path between one point and one segment (point's foot must lay on the segment)
double shortest_distance(double segment_x1, double segment_y1, double segment_x2, double segment_y2, double point_x, double point_y, double length_of_segment){

    double r = ((point_x - segment_x1) * (segment_x2 - segment_x1) + (point_y - segment_y1) * (segment_y2 - segment_y1)) / pow(length_of_segment,2);
    // printf("r: %.3f\n", r);
    //projection point is on the segment
    if(r >= 0 && r <= 1){
        double footx = segment_x1 + r * (segment_x2 - segment_x1);
        double footy = segment_y1 + r * (segment_y2 - segment_y1);

        // printf("foot: (%.3f, %.3f\n", footx, footy);
        return sqrt(pow(footx - point_x,2) + pow(footy - point_y,2));
    }
    else{
        double dist1 = sqrt(pow(segment_x1 - point_x, 2) + pow(segment_y1 - point_y, 2));
        double dist2 = sqrt(pow(segment_x2 - point_x, 2) + pow(segment_y2 - point_y, 2));
        return fmin(dist1, dist2);
    }
    
}
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

void BuildCapicityConstraint(OASGEdge* e1, OASGEdge* e2, FlowLP &solver){
    bool right1 = false; //default left side
    double ratio1 = 0; //default zero ratio

    double S_rpoint = fmax(e1->sNode()->x(),e1->tNode()->x());//right point of segment (x-coordinate)
    double S_lpoint = fmin(e1->sNode()->x(),e1->tNode()->x());//left point of segment  (x-coordinate)
    double S_upoint = fmax(e1->sNode()->y(),e1->tNode()->y());//upper point of segment (y-coordinate)
    double S_dpoint = fmin(e1->sNode()->y(),e1->tNode()->y());//lower(down) point of segment (y-coordinate)

    bool right2 = false; //default left side
    double ratio2 = 0; //default zero ratio
    double width = 0; //default no width

    double T_rpoint = fmax(e2->sNode()->x(),e2->tNode()->x());//right point of segment (x-coordinate)
    double T_lpoint = fmin(e2->sNode()->x(),e2->tNode()->x());//left point of segment  (x-coordinate)
    double T_upoint = fmax(e2->sNode()->y(),e2->tNode()->y());//upper point of segment (y-coordinate)
    double T_dpoint = fmin(e2->sNode()->y(),e2->tNode()->y());//lower(down) point of segment (y-coordinate)

    //type1 source (horizon)
    if(e1->sNode()->y() == e1->tNode()->y()){
        cout << "Source type is: 1" << endl;
        //check if the target is in the searching area
        //target point is in searching area
        if( (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint) || (S_lpoint <= T_lpoint && T_lpoint <= S_rpoint)){
            //type1 target
            if(e2->sNode()->y() == e2->tNode()->y()){
                //S -> left , T -> right
                if(e1->sNode()->y() < e2->sNode()->y()){
                    width = e2->sNode()->y() - e1->sNode()->y();
                    right1 = false;
                    right2 = true;
                    ratio1 = 1;
                    ratio2 = 1;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
                //S -> rigth , T -> left
                else{
                    width = e1->sNode()->y() - e2->sNode()->y();
                    right1 = true;
                    right2 = false;
                    ratio1 = 1;
                    ratio2 = 1;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
            }
            //type2 target
            else if(e2->sNode()->x() == e2->tNode()->x()){
                //S -> left , T -> none
                if(e1->sNode()->y() < T_dpoint){
                    width = T_dpoint - e1->sNode()->y();
                    right1 = false;
                    right2 = true;
                    ratio1 = 1;
                    ratio2 = 0;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
                //S -> rigth , T -> none
                else{
                    width = e1->sNode()->y() - T_upoint;
                    right1 = true;
                    right2 = false;
                    ratio1 = 1;
                    ratio2 = 0;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
            }
            //type3 target
            else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                double slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                /*double degrees = theta * 180.0 / PI;*/

                //both target points are in searching area
                if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = e1->sNode()->y() - T_upoint;
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target left point is in searching area and target right point isn't
                else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(theta - PI/2);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> rigth
                    if(e1->sNode()->y() < T_upoint){
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(theta - PI/2);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = e1->sNode()->y() - T_upoint;
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
            //type4 target
            else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                double slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                //double degrees = theta * 180.0 / PI;
                
                //both target points are in searching area
                if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> left
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI/2 - theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = e1->sNode()->y() - T_upoint;
                        right1 = true;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI/2 - theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target left point is in searching area but target right point isn't
                else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> left
                    if(e1->sNode()->y() < T_upoint){
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                        right1 = false;
                        right2 = false;
                        ratio1 = cos(PI/2 - theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = T_upoint - e1->sNode()->y();
                        right1 = true;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI/2 - theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> left
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                        right1 = true;
                        right2 = true;
                        ratio1 = cos(theta - PI/2);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
        cout << "width is :" << width << endl;  
        }   
    }
    //type2 source (vertical)
    else if(e1->sNode()->x() == e1->tNode()->x()){
        cout << "Source type is: 2" << endl;
        //check if the target is in the searching area
        //target point is in searching area
        if(S_dpoint <= T_dpoint && T_dpoint <= S_upoint || S_dpoint <= T_upoint && T_upoint <= S_upoint){
            //type1 target
            if(e2->sNode()->y() == e2->tNode()->y()){
                cout << "target type is: 1" << endl;
                //S -> left , T -> none
                if(e1->sNode()->x() > T_rpoint){
                    width = e1->sNode()->x() - T_rpoint;
                    right1 = false;
                    right2 = false;
                    ratio1 = 1;
                    ratio2 = 0;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
                //S -> rigth , T -> none
                else{
                    width = T_lpoint - e1->sNode()->x();
                    right1 = true;
                    right2 = false;
                    ratio1 = 1;
                    ratio2 = 0;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
            }
            //type2 target
            else if(e2->sNode()->x() == e2->tNode()->x()){
                cout << "target type is: 2" << endl;
                //S -> left , T -> right
                if(e1->sNode()->x() > T_lpoint){
                    width = e1->sNode()->x() - T_lpoint;
                    right1 = false;
                    right2 = true;
                    ratio1 = 1;
                    ratio2 = 1;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
                //S -> rigth , T -> left
                else{
                    width = T_lpoint - e1->sNode()->x();
                    right1 = true;
                    right2 = false;
                    ratio1 = 1;
                    ratio2 = 1;
                    printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                    solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                }
            }
            //type3 target
            else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                cout << "target type is: 3" << endl;
                double slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                /*double degrees = theta * 180.0 / PI;*/

                //both target points are in searching area
                if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_rpoint){
                        width =  e1->sNode()->x() - T_rpoint;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI - theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI - theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_lpoint){
                        width =  shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(PI - theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI - theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> rigth
                    if(e1->sNode()->x() > T_rpoint){
                        width = e1->sNode()->x() - T_rpoint;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI - theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(PI - theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
            //type4 target
            else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                cout << "target type is: 4" << endl;
                double slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                //double degrees = theta * 180.0 / PI;
                
                //both target points are in searching area
                if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> right , T -> left 
                    if(e1->sNode()->x() < T_lpoint){
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> left , T -> right
                    else{
                        width = e1->sNode()->x() - T_rpoint;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_rpoint){
                        width = e1->sNode()->x() - T_rpoint;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_upoint,e2->length());;
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target upper point is in searching area but target down point isn't
                else{
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_lpoint){
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
        cout << "width is :" << width << endl; 
        }    
    }
    //type3 source (slope > 0)
    else if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x())){
        cout << "Source type is: 3" << endl;
        double S_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
        double S_theta = atan(S_slope);
        if(S_theta < 0) S_theta += PI;
        //check if target is in searching area
        double rs = ((e2->sNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->sNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_sNode
        double rt = ((e2->tNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->tNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_tNode
    //    cout<< rs<< " "<<rt<< "\n";
        //target point is in searching area
        if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
            //type1 target
            if(e2->sNode()->y() == e2->tNode()->y()){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target right point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                    //if r1 is closer to e1, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(S_theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is closer to e1, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(S_theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }   
                }
                //target left point is in searching area but target right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_upoint > S_upoint, S -> left, T -> right
                    if(T_upoint > S_upoint){
                        width =  shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(S_theta - PI/2);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(S_theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //if T_dpoint > S_dpoint, S -> left, T -> right
                    if(T_dpoint > S_dpoint){
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(S_theta - PI/2);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(S_theta - PI/2);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
            //type2 target
            else if(e2->sNode()->x() == e2->tNode()->x()){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                    //if r1 is closer to e1, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is closer to e1, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }   
                }
                //target upper point is in searching area but target down point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_lpoint < S_lpoint, S -> left, T -> right
                    if(T_lpoint < S_lpoint){
                        width =  shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(PI - S_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else{
                    //if T_lpoint < S_rpoint, S -> left, T -> right
                    if(T_lpoint < S_rpoint){
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_upoint,e2->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(PI - S_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
            //type3 target
            else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                double T_slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper right point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down left point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                    else if(s1 == s2){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
                //target upper right point is in searching area but target down left point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());//distance of source down left point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = cos(T_theta - S_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                    else if(s1 == s2){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());//distance of source down left point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = cos(S_theta - T_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
                //target down left point is in searching area but target upper right point isn't
                else{
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_rpoint,e2->length());//distance of source upper right point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = cos(T_theta - S_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }  
                    }
                    else if(s1 == s2){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length()) ;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length()) ;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_rpoint,e2->length());//distance of source upper right point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = cos(S_theta - T_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
            }
            //type4 target
            else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                double T_slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down right point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                    else if(s2 == S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> none
                        if(r1_distance < r2_distance){
                            width = r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 0;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> none
                        else{
                            width = r2_distance;
                            right1 = false;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 0;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(PI - S_theta + T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(PI - S_theta + T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
                //target upper left point is in searching area but target down right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //only s2 < s_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                    double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_dpoint,e2->length());//distance of source down left point to target edge
                    //if r1 is smaller, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI - S_theta + T_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is smaller, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(PI - S_theta + T_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }   
                }
                //target down right point is in searching area but target upper left point isn't
                else{
                    //only s2 < s_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                    double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_rpoint,e2->length());//distance of source upper right point to target edge
                    //if r1 is smaller, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI - S_theta + T_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is smaller, S -> right, T -> left
                    else{
                        width =  r2_distance;
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(PI - S_theta + T_theta);;
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }  
                }
            }
        cout << "width is :" << width << endl; 
        }
    }
    //type4 source (slope < 0)
    else/*if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()))*/{
        cout << "Source type is: 4" << endl;
        double S_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
        double S_theta = atan(S_slope);
        if(S_theta < 0) S_theta += PI;
        //check if target is in searching area
        double rs = ((e2->sNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->sNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_sNode
        double rt = ((e2->tNode()->x() - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (e2->tNode()->y() - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_tNode
        //target point is in searching area
        if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
            //type1 target
            if(e2->sNode()->y() == e2->tNode()->y()){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target right point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());//distance of target left point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//distance of target right point to source edge
                    //if r1 is closer to e1, S -> right, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI/2 - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is closer to e1, S -> left, T -> left
                    else{
                        width =  r2_distance;
                        right1 = false;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI/2 - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }   
                }
                //target left point is in searching area but target right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_dpoint > S_dpoint, S -> right, T -> right
                    if(T_dpoint > S_dpoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = true;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI/2 - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());
                        right1 = false;
                        right2 = false;
                        ratio1 = cos(PI/2 - S_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //if T_upoint > S_upoint, S -> right, T -> right
                    if(T_upoint > S_upoint){
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_upoint,e2->length());
                        right1 = true;
                        right2 = true;
                        ratio1 = cos(PI/2 - S_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> left, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        right1 = false;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI/2 - S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
            //type2 target
            else if(e2->sNode()->x() == e2->tNode()->x()){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());//distance of target down point to source edge
                    //if r1 is closer to e1, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is closer to e1, S -> right, T -> left
                    else{
                        width =  r2_distance;
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }   
                }
                //target upper point is in searching area but target down point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_rpoint < S_rpoint, S -> left, T -> right
                    if(T_rpoint < S_rpoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(S_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else{
                    //if T_lpoint < S_lpoint, S -> left, T -> right
                    if(T_lpoint < S_lpoint){
                        width = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_lpoint,S_upoint,e2->length());
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(S_theta);;
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(S_theta);;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                }
            }
            //type3 target
            else if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x())){
                double T_slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper right point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down left point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(PI - T_theta + S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(PI - T_theta + S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                    else if(s2 == S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> none
                        if(r1_distance < r2_distance){
                            width = r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = 0;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width = r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 0;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
                //target upper right point is in searching area but target down left point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //only s2 > S_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                    double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());//distance of source down right point to target edge
                    //if r1 is smaller, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        right2 = true;
                        ratio1 = 1;
                        ratio2 = cos(PI - T_theta + S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is smaller, S -> right, T -> left
                    else{
                        width = r2_distance;
                        right1 = true;
                        right2 = false;
                        ratio1 = cos(PI - T_theta + S_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }   
                }
                //target down left point is in searching area but target upper right point isn't
                else{
                    //only s2 > S_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                    double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_lpoint,e2->length());//distance of source upper left point to target edge
                    //if r1 is smaller, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        right2 = false;
                        ratio1 = 1;
                        ratio2 = cos(PI - T_theta + S_theta);
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }
                    //else if r2 is smaller, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        right2 = true;
                        ratio1 = cos(PI - T_theta + S_theta);
                        ratio2 = 1;
                        printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                        solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                    }  
                }
            }
            //type4 target
            else/*if((e2->sNode()->y() > e2->tNode()->y() && e2->sNode()->x() < e2->tNode()->x()) || (e2->sNode()->y() < e2->tNode()->y() && e2->sNode()->x() > e2->tNode()->x()))*/{
                double T_slope = -1/((e2->tNode()->y() - e2->sNode()->y()) / (e2->tNode()->x() - e2->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down right point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                    else if(s2 == s1){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
                //target upper left point is in searching area but target down right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());//distance of source down right point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = cos(T_theta - S_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                    else if(s2 == s1){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_rpoint,S_dpoint,e2->length());//distance of source down right point to target edge
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = cos(S_theta - T_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
                //target down right point is in searching area but target upper left point isn't
                else{
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_lpoint,e2->length());//distance of source upper left point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = cos(T_theta - S_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = cos(T_theta - S_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }  
                    }
                    else if(s2 == s1){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = e2->sNode()->x() - (e2->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = false;
                            right2 = true;
                            ratio1 = 1;
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                        double r2_distance = shortest_distance(e2->sNode()->x(),e2->sNode()->y(),e2->tNode()->x(),e2->tNode()->y(),S_upoint,S_lpoint,e2->length());//distance of source upper left point to target edge
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            right2 = false;
                            ratio1 = 1;
                            ratio2 = cos(S_theta - T_theta);
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            right2 = true;
                            ratio1 = cos(S_theta - T_theta);
                            ratio2 = 1;
                            printf("add constaint: S_%c * %.2f + T_%c * %.2f <= %.2f\n", (right1)? 'r': 'l', ratio1, (right2)? 'r': 'l', ratio2, width);
                            solver.addCapacityConstraints(e1,right1,ratio1,e2,right2,ratio2,width);
                        }   
                    }
                }
            }
        cout << "width is :" << width << endl; 
        }
    }
}

void AsSourceCapacityConstraint(OASGEdge* e1, double x1, double y1, double x2, double y2, FlowLP &solver){
   
    bool right1 = false; //default left side
    double ratio1 = 0; //default zero ratio

    double S_rpoint = fmax(e1->sNode()->x(),e1->tNode()->x());//right point of segment (x-coordinate)
    double S_lpoint = fmin(e1->sNode()->x(),e1->tNode()->x());//left point of segment  (x-coordinate)
    double S_upoint = fmax(e1->sNode()->y(),e1->tNode()->y());//upper point of segment (y-coordinate)
    double S_dpoint = fmin(e1->sNode()->y(),e1->tNode()->y());//lower(down) point of segment (y-coordinate)

    double width = 0; //default no width

    double edge_length = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
    double T_rpoint = fmax(x1,x2);//right point of segment (x-coordinate)
    double T_lpoint = fmin(x1,x2);//left point of segment  (x-coordinate)
    double T_upoint = fmax(y1,y2);//upper point of segment (y-coordinate)
    double T_dpoint = fmin(y1,y2);//lower(down) point of segment (y-coordinate)

    //type1 source (horizon)
    if(e1->sNode()->y() == e1->tNode()->y()){
        cout << "Source type is: 1" << endl;
        //check if the target is in the searching area
        //target point is in searching area
        if( (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint) || (S_lpoint <= T_lpoint && T_lpoint <= S_rpoint)){
            //type1 target
            if(y1 == y2){
                //S -> left , T -> right
                if(e1->sNode()->y() < y1){
                    width = y1 - e1->sNode()->y();
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
                //S -> rigth , T -> left
                else{
                    width = e1->sNode()->y() - y1;
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
            }
            //type2 target
            else if(x1 == x2){
                //S -> left , T -> none
                if(e1->sNode()->y() < T_dpoint){
                    width = T_dpoint - e1->sNode()->y();
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
                //S -> rigth , T -> none
                else{
                    width = e1->sNode()->y() - T_upoint;
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
            }
            //type3 target
            else if((y1 > y2 && x1 > x2) || (y1 < y2 && x1 < x2)){
                double slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                /*double degrees = theta * 180.0 / PI;*/

                //both target points are in searching area
                if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = e1->sNode()->y() - T_upoint;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target left point is in searching area and target right point isn't
                else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_rpoint,S_upoint,edge_length);
                        right1 = true;
                        ratio1 = cos(theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> rigth
                    if(e1->sNode()->y() < T_upoint){
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);
                        right1 = false;
                        ratio1 = cos(theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = e1->sNode()->y() - T_upoint;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
            //type4 target
            else/*if((y1 > y2 && x1 < x2) || (y1 < y2 && x1 > x2))*/{
                double slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                //double degrees = theta * 180.0 / PI;
                
                //both target points are in searching area
                if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> left
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = e1->sNode()->y() - T_upoint;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target left point is in searching area but target right point isn't
                else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> left
                    if(e1->sNode()->y() < T_upoint){
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);
                        right1 = false;
                        ratio1 = cos(PI/2 - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = T_upoint - e1->sNode()->y();
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> left
                    if(e1->sNode()->y() < T_dpoint){
                        width = T_dpoint - e1->sNode()->y();
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);
                        right1 = true;
                        
                        ratio1 = cos(theta - PI/2);

                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
        cout << "width is :" << width << endl;  
        }   
    }
    //type2 source (vertical)
    else if(e1->sNode()->x() == e1->tNode()->x()){
        cout << "Source type is: 2" << endl;
        //check if the target is in the searching area
        //target point is in searching area
        if(S_dpoint <= T_dpoint && T_dpoint <= S_upoint || S_dpoint <= T_upoint && T_upoint <= S_upoint){
            //type1 target
            if(y1 == y2){
                cout << "target type is: 1" << endl;
                //S -> left , T -> none
                if(e1->sNode()->x() > T_rpoint){
                    width = e1->sNode()->x() - T_rpoint;
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
                //S -> rigth , T -> none
                else{
                    width = T_lpoint - e1->sNode()->x();
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
            }
            //type2 target
            else if(x1 == x2){
                cout << "target type is: 2" << endl;
                //S -> left , T -> right
                if(e1->sNode()->x() > T_lpoint){
                    width = e1->sNode()->x() - T_lpoint;
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
                //S -> rigth , T -> left
                else{
                    width = T_lpoint - e1->sNode()->x();
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                }
            }
            //type3 target
            else if((y1 > y2 && x1 > x2) || (y1 < y2 && x1 < x2)){
                cout << "target type is: 3" << endl;
                double slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                /*double degrees = theta * 180.0 / PI;*/

                //both target points are in searching area
                if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_rpoint){
                        width =  e1->sNode()->x() - T_rpoint;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_lpoint){
                        width =  shortest_distance(x1,y1,x2,y2,S_rpoint,S_upoint,edge_length);
                        right1 = false;
                        ratio1 = cos(PI - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> rigth
                    if(e1->sNode()->x() > T_rpoint){
                        width = e1->sNode()->x() - T_rpoint;
                        right1 = false;    
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);
                        right1 = true;
                        ratio1 = cos(PI - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
            //type4 target
            else/*if((y1 > y2 && x1 < x2) || (y1 < y2 && x1 > x2))*/{
                cout << "target type is: 4" << endl;
                double slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                //double degrees = theta * 180.0 / PI;
                
                //both target points are in searching area
                if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> right , T -> left 
                    if(e1->sNode()->x() < T_lpoint){
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> left , T -> right
                    else{
                        width = e1->sNode()->x() - T_rpoint;
                        right1 = false; 
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_rpoint){
                        width = e1->sNode()->x() - T_rpoint;
                        right1 = false; 
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> right
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_upoint,edge_length);;
                        right1 = true;
                        ratio1 = cos(theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target upper point is in searching area but target down point isn't
                else{
                    //S -> left , T -> right
                    if(e1->sNode()->x() > T_lpoint){
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);
                        right1 = false;
                        ratio1 = cos(theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - e1->sNode()->x();
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
        cout << "width is :" << width << endl; 
        }    
    }
    //type3 source (slope > 0)
    else if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x())){
        cout << "Source type is: 3" << endl;
        double S_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
        double S_theta = atan(S_slope);
        if(S_theta < 0) S_theta += PI;
        //check if target is in searching area
        double rs = ((x1 - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (y1 - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_sNode
        double rt = ((x2 - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (y2 - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_tNode
        //target point is in searching area
        if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
            //type1 target
            if(y1 == y2){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target right point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                    //if r1 is closer to e1, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is closer to e1, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }   
                }
                //target left point is in searching area but target right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_upoint > S_upoint, S -> left, T -> right
                    if(T_upoint > S_upoint){
                        width =  shortest_distance(x1,y1,x2,y2,S_rpoint,S_upoint,edge_length);
                        right1 = false;
                        ratio1 = cos(S_theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //if T_dpoint > S_dpoint, S -> left, T -> right
                    if(T_dpoint > S_dpoint){
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);
                        right1 = true;
                        ratio1 = cos(S_theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
            //type2 target
            else if(x1 == x2){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                    //if r1 is closer to e1, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is closer to e1, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }   
                }
                //target upper point is in searching area but target down point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_lpoint < S_lpoint, S -> left, T -> right
                    if(T_lpoint < S_lpoint){
                        width =  shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);
                        right1 = false;
                        ratio1 = cos(PI - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else{
                    //if T_lpoint < S_rpoint, S -> left, T -> right
                    if(T_lpoint < S_rpoint){
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_rpoint,S_upoint,edge_length);
                        right1 = true;
                        ratio1 = cos(PI - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
            //type3 target
            else if((y1 > y2 && x1 > x2) || (y1 < y2 && x1 < x2)){
                double T_slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper right point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down left point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                    else if(s1 == s2){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = x1 - (y1/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
                //target upper right point is in searching area but target down left point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);//distance of source down left point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                    else if(s1 == s2){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = x1 - (y1/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length()) ;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);//distance of source down left point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
                //target down left point is in searching area but target upper right point isn't
                else{
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_upoint,S_rpoint,edge_length);//distance of source upper right point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }  
                    }
                    else if(s1 == s2){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = x1 - (y1/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length()) ;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length()) ;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_upoint,S_rpoint,edge_length);//distance of source upper right point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
            }
            //type4 target
            else/*if((y1 > y2 && x1 < x2) || (y1 < y2 && x1 > x2))*/{
                double T_slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down right point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                    else if(s2 == S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> none
                        if(r1_distance < r2_distance){
                            width = r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> none
                        else{
                            width = r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
                //target upper left point is in searching area but target down right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //only s2 < s_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                    double r2_distance = shortest_distance(x1,y1,x2,y2,S_lpoint,S_dpoint,edge_length);//distance of source down left point to target edge
                    //if r1 is smaller, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is smaller, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        ratio1 = cos(PI - S_theta + T_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }   
                }
                //target down right point is in searching area but target upper left point isn't
                else{
                    //only s2 < s_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                    double r2_distance = shortest_distance(x1,y1,x2,y2,S_upoint,S_rpoint,edge_length);//distance of source upper right point to target edge
                    //if r1 is smaller, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is smaller, S -> right, T -> left
                    else{
                        width =  r2_distance;
                        right1 = true;
                        ratio1 = cos(PI - S_theta + T_theta);;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }  
                }
            }
        cout << "width is :" << width << endl; 
        }
    }
    //type4 source (slope < 0)
    else/*if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()))*/{
        cout << "Source type is: 4" << endl;
        double S_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
        double S_theta = atan(S_slope);
        if(S_theta < 0) S_theta += PI;
        //check if target is in searching area
        double rs = ((x1 - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (y1 - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_sNode
        double rt = ((x2 - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (y2 - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//T_tNode
        //target point is in searching area
        if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
            //type1 target
            if(y1 == y2){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target right point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());//distance of target left point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//distance of target right point to source edge
                    //if r1 is closer to e1, S -> right, T -> right
                    if(r1_distance < r2_distance){
                    width =  r1_distance;
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is closer to e1, S -> left, T -> left
                    else{
                    width =  r2_distance;
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }   
                }
                //target left point is in searching area but target right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_dpoint > S_dpoint, S -> right, T -> right
                    if(T_dpoint > S_dpoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_rpoint,S_dpoint,edge_length);
                        right1 = false;
                        ratio1 = cos(PI/2 - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //if T_upoint > S_upoint, S -> right, T -> right
                    if(T_upoint > S_upoint){
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_upoint,edge_length);
                        right1 = true;
                        ratio1 = cos(PI/2 - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> left, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
            //type2 target
            else if(x1 == x2){
                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());//distance of target down point to source edge
                    //if r1 is closer to e1, S -> left, T -> right
                    if(r1_distance < r2_distance){
                    width =  r1_distance;
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is closer to e1, S -> right, T -> left
                    else{
                    width =  r2_distance;
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }   
                }
                //target upper point is in searching area but target down point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_rpoint < S_rpoint, S -> left, T -> right
                    if(T_rpoint < S_rpoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,S_rpoint,S_dpoint,edge_length);
                        right1 = true;
                        ratio1 = cos(S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
                //target down point is in searching area but target upper point isn't
                else{
                    //if T_lpoint < S_lpoint, S -> left, T -> right
                    if(T_lpoint < S_lpoint){
                        width = shortest_distance(x1,y1,x2,y2,S_lpoint,S_upoint,edge_length);
                        right1 = false;
                        ratio1 = cos(S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                }
            }
            //type3 target
            else if((y1 > y2 && x1 > x2) || (y1 < y2 && x1 < x2)){
                double T_slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper right point
                double r2 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down left point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                    else if(s2 == S_slope){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> none
                        if(r1_distance < r2_distance){
                            width = r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width = r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_dpoint,e1->length());
                        //if r1 is closer to e1, S -> left, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
                //target upper right point is in searching area but target down left point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //only s2 > S_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_upoint,e1->length());//distance of target upper right point to source edge
                    double r2_distance = shortest_distance(x1,y1,x2,y2,S_rpoint,S_dpoint,edge_length);//distance of source down right point to target edge
                    //if r1 is smaller, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is smaller, S -> right, T -> left
                    else{
                        width = r2_distance;
                        right1 = true;
                        ratio1 = cos(PI - T_theta + S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }   
                }
                //target down left point is in searching area but target upper right point isn't
                else{
                    //only s2 > S_slope would happen
                    double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_lpoint,e1->length());//distance of target down left point to source edge
                    double r2_distance = shortest_distance(x1,y1,x2,y2,S_upoint,S_lpoint,edge_length);//distance of source upper left point to target edge
                    //if r1 is smaller, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }
                    //else if r2 is smaller, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = false;
                        ratio1 = cos(PI - T_theta + S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                    }  
                }
            }
            //type4 target
            else/*if((y1 > y2 && x1 < x2) || (y1 < y2 && x1 > x2))*/{
                double T_slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e2

                double r1 = ((T_lpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_upoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target upper left point
                double r2 = ((T_rpoint - e1->sNode()->x()) * (e1->tNode()->x() - e1->sNode()->x()) + (T_dpoint - e1->sNode()->y()) * (e1->tNode()->y() - e1->sNode()->y())) / pow(e1->length(),2);//Target down right point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                    else if(s2 == s1){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = x1 - (y1/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//target upper left point 
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());//target down right point
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
                //target upper left point is in searching area but target down right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_rpoint,S_dpoint,edge_length);//distance of source down right point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                    else if(s2 == s1){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = x1 - (y1/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_lpoint,T_upoint,e1->length());//distance of target upper left point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_rpoint,S_dpoint,edge_length);//distance of source down right point to target edge
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
                //target down right point is in searching area but target upper left point isn't
                else{
                    if(s2 > s1){
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_upoint,S_lpoint,edge_length);//distance of source upper left point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }  
                    }
                    else if(s2 == s1){
                        double x1 = e1->sNode()->x() - (e1->sNode()->y()/s1);//x-intercept of source edge
                        double x2 = x1 - (y1/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_rpoint,T_dpoint,e1->length());
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),T_dpoint,T_rpoint,e1->length());//distance of target down right point to source edge
                        double r2_distance = shortest_distance(x1,y1,x2,y2,S_upoint,S_lpoint,edge_length);//distance of source upper left point to target edge
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                        }   
                    }
                }
            }
        cout << "width is :" << width << endl; 
        }
    }
}

bool AsTargetCapacityConstraint(OASGEdge* e1, double x1, double y1, double x2, double y2, FlowLP &solver){

    double S_rpoint = fmax(x1,x2);//right point of segment (x-coordinate)
    double S_lpoint = fmin(x1,x2);//left point of segment  (x-coordinate)
    double S_upoint = fmax(y1,y2);//upper point of segment (y-coordinate)
    double S_dpoint = fmin(y1,y2);//lower(down) point of segment (y-coordinate)

    double edge_length = sqrt(pow(x2-x1,2)+pow(y2-y1,2));

    bool right1 = false; //default left side
    double ratio1 = 0; //default zero ratio
    double width = 0; //default no width

    double T_rpoint = fmax(e1->sNode()->x(),e1->tNode()->x());//right point of segment (x-coordinate)
    double T_lpoint = fmin(e1->sNode()->x(),e1->tNode()->x());//left point of segment  (x-coordinate)
    double T_upoint = fmax(e1->sNode()->y(),e1->tNode()->y());//upper point of segment (y-coordinate)
    double T_dpoint = fmin(e1->sNode()->y(),e1->tNode()->y());//lower(down) point of segment (y-coordinate)

    //type1 source (horizon)
    if(y1 == y2){
        cout << "Source type is: 1" << endl;
        //check if the target is in the searching area
        //target point is in searching area
        if( (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint) || (S_lpoint <= T_lpoint && T_lpoint <= S_rpoint)){
            //type1 target
            if(e1->sNode()->y() == e1->tNode()->y()){
                //S -> left , T -> right
                if(y1 < e1->sNode()->y()){
                    width = e1->sNode()->y() - y1;
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
                //S -> rigth , T -> left
                else{
                    width = y1 - e1->sNode()->y();
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
            }
            //type2 target
            else if(e1->sNode()->x() == e1->tNode()->x()){
                //S -> left , T -> none
                if(y1 < T_dpoint){
                    width = T_dpoint - y1;
                    right1 = true;
                    ratio1 = 0;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
                //S -> rigth , T -> none
                else{
                    width = y1 - T_upoint;
                    right1 = false;
                    ratio1 = 0;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
            }
            //type3 target
            else if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x())){
                double slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                /*double degrees = theta * 180.0 / PI;*/

                //both target points are in searching area
                if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> right
                    if(y1 < T_dpoint){
                        width = T_dpoint - y1;
                        right1 = true;
                        ratio1 = cos(theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> left
                    else{
                        width = y1 - T_upoint;
                        right1 = false;
                        ratio1 = cos(theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target left point is in searching area and target right point isn't
                else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> right
                    if(y1 < T_dpoint){
                        width = T_dpoint - y1;
                        right1 = true;
                        ratio1 = cos(theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_upoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> rigth
                    if(y1 < T_upoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> left
                    else{
                        width = y1 - T_upoint;
                        right1 = false;
                        ratio1 = cos(theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
            //type4 target
            else/*if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()))*/{
                double slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                //double degrees = theta * 180.0 / PI;
                
                //both target points are in searching area
                if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && (S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> left
                    if(y1 < T_dpoint){
                        width = T_dpoint - y1;
                        right1 = false;
                        ratio1 = cos(PI/2 - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> right
                    else{
                        width = y1 - T_upoint;
                        right1 = true;
                        ratio1 = cos(PI/2 - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target left point is in searching area but target right point isn't
                else if((S_lpoint <= T_lpoint && T_lpoint <= S_rpoint) && !(S_lpoint <= T_rpoint && T_rpoint <= S_rpoint)){
                    //S -> left , T -> left
                    if(y1 < T_upoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> right
                    else{
                        width = T_upoint - y1;
                        right1 = true;
                        ratio1 = cos(PI/2 - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> left
                    if(y1 < T_dpoint){
                        width = T_dpoint - y1;
                        right1 = false;
                        ratio1 = cos(theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> right
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
        cout << "width is :" << width << endl;  
        }   
    }
    //type2 source (vertical)
    else if(x1 == x2){
        cout << "Source type is: 2" << endl;
        //check if the target is in the searching area
        //target point is in searching area
        if(S_dpoint <= T_dpoint && T_dpoint <= S_upoint || S_dpoint <= T_upoint && T_upoint <= S_upoint){
            //type1 target
            if(e1->sNode()->y() == e1->tNode()->y()){
                cout << "target type is: 1" << endl;
                //S -> left , T -> none
                if(x1 > T_rpoint){
                    width = x1 - T_rpoint;
                    right1 = false;
                    ratio1 = 0;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
                //S -> rigth , T -> none
                else{
                    width = T_lpoint - x1;
                    right1 = false;
                    ratio1 = 0;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
            }
            //type2 target
            else if(e1->sNode()->x() == e1->tNode()->x()){
                cout << "target type is: 2" << endl;
                //S -> left , T -> right
                if(x1 > T_lpoint){
                    width = x1 - T_lpoint;
                    right1 = true;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
                //S -> rigth , T -> left
                else{
                    width = T_lpoint - x1;
                    right1 = false;
                    ratio1 = 1;
                    solver.addCapacityConstraints(e1,right1,ratio1,width);
                    return true;
                }
            }
            //type3 target
            else if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x())){
                cout << "target type is: 3" << endl;
                double slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                /*double degrees = theta * 180.0 / PI;*/
                //both target points are in searching area
                if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(x1 > T_rpoint){
                        width =  x1 - T_rpoint;
                        right1 = true;
                        ratio1 = cos(PI - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - x1;
                        right1 = false;
                        ratio1 = cos(PI - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target down point is in searching area but target upper point isn't
                else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(x1 > T_lpoint){
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_upoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - x1;
                        right1 = false;
                        ratio1 = cos(PI - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //S -> left , T -> rigth
                    if(x1 > T_rpoint){
                        width = x1 - T_rpoint;
                        right1 = true;
                        ratio1 = cos(PI - theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
            //type4 target
            else/*if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()))*/{
                cout << "target type is: 4" << endl;
                double slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double theta = atan(slope);
                if(theta < 0) theta += PI; 
                //double degrees = theta * 180.0 / PI;
                
                //both target points are in searching area
                if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && (S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> right , T -> left 
                    if(x1 < T_lpoint){
                        width = T_lpoint - x1;
                        right1 = false;
                        ratio1 = cos(theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> left , T -> right
                    else{
                        width = x1 - T_rpoint;
                        right1 = true;
                        ratio1 = cos(theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target down point is in searching area but target upper point isn't
                else if((S_dpoint <= T_dpoint && T_dpoint <= S_upoint) && !(S_dpoint <= T_upoint && T_upoint <= S_upoint)){
                    //S -> left , T -> right
                    if(x1 > T_rpoint){
                        width = x1 - T_rpoint;
                        right1 = true;
                        ratio1 = cos(theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> right
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_upoint,e1->length());;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target upper point is in searching area but target down point isn't
                else{
                    //S -> left , T -> right
                    if(x1 > T_lpoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //S -> rigth , T -> left
                    else{
                        width = T_lpoint - x1;
                        right1 = false;
                        ratio1 = cos(theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
        cout << "width is :" << width << endl; 
        }    
    }
    //type3 source (slope > 0)
    else if((y1 > y2 && x1 > x2) || (y1 < y2 && x1 < x2)){
        cout << "Source type is: 3" << endl;
        double S_slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
        double S_theta = atan(S_slope);
        if(S_theta < 0) S_theta += PI;
        //check if target is in searching area
        double rs = ((e1->sNode()->x() - x1) * (x2 - x1) + (e1->sNode()->y() - y1) * (y2 - y1)) / pow(edge_length,2);//T_sNode
        double rt = ((e1->tNode()->x() - x1) * (x2 - x1) + (e1->tNode()->y() - y1) * (y2 - y1)) / pow(edge_length,2);//T_tNode
        //target point is in searching area
        if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
            //type1 target
            if(e1->sNode()->y() == e1->tNode()->y()){
                double r1 = ((T_lpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target left point
                double r2 = ((T_rpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target right point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                    double r2_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);
                    //if r1 is closer to e1, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        ratio1 = cos(S_theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is closer to e1, S -> left, T -> right
                    else{
                        width =  r2_distance;

                        right1 = true;
                        ratio1 = cos(S_theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }   
                }
                //target left point is in searching area but target right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_upoint > S_upoint, S -> left, T -> right
                    if(T_upoint > S_upoint){
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_upoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> right, T -> left
                    else{
                        width =  shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        right1 = false;
                        ratio1 = cos(S_theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //if T_dpoint > S_dpoint, S -> left, T -> right
                    if(T_dpoint > S_dpoint){
                        width =  shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);
                        right1 = true;
                        ratio1 = cos(S_theta - PI/2);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
            //type2 target
            else if(e1->sNode()->x() == e1->tNode()->x()){
                double r1 = ((T_lpoint - x1) * (x2 - x1) + (T_upoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target upper point
                double r2 = ((T_lpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target down point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);
                    double r2_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                    //if r1 is closer to e1, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        ratio1 = cos(PI - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is closer to e1, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = true;
                        ratio1 = cos(PI - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }   
                }
                //target upper point is in searching area but target down point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_lpoint < S_lpoint, S -> left, T -> right
                    if(T_lpoint < S_lpoint){
                        width =  shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> right, T -> left
                    else{
                        width =  shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);
                        right1 = false;
                        ratio1 = cos(PI - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target down point is in searching area but target upper point isn't
                else{
                    //if T_lpoint < S_rpoint, S -> left, T -> right
                    if(T_lpoint < S_rpoint){
                        width =  shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        right1 = true;
                        ratio1 = cos(PI - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_upoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
            //type3 target
            else if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x())){
                double T_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e1

                double r1 = ((T_rpoint - x1) * (x2 - x1) + (T_upoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target upper right point
                double r2 = ((T_lpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target down left point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > s1){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                    else if(s1 == s2){
                        double x1 = x1 - (y1/s1);//x-intercept of source edge
                        double x2 = e1->sNode()->x() - (e1->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length) ;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length) ;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
                //target upper right point is in searching area but target down left point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    if(s2 > s1){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);//distance of target upper right point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());//distance of source down left point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                    else if(s1 == s2){
                        double x1 = x1 - (y1/s1);//x-intercept of source edge
                        double x2 = e1->sNode()->x() - (e1->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length) ;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length) ;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);//distance of target upper right point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());//distance of source down left point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
                //target down left point is in searching area but target upper right point isn't
                else{
                    if(s2 > s1){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_dpoint,T_lpoint,edge_length);//distance of target down left point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_upoint,S_rpoint,e1->length());//distance of source upper right point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }  
                    }
                    else if(s1 == s2){
                        double x1 = x1 - (y1/s1);//x-intercept of source edge
                        double x2 = e1->sNode()->x() - (e1->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length) ;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //x1 > x2, S -> left, T -> right
                        else{
                            width = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length) ;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_dpoint,T_lpoint,edge_length);//distance of target down left point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_upoint,S_rpoint,e1->length());//distance of source upper right point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
            }
            //type4 target
            else/*if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()))*/{
                double T_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e1

                double r1 = ((T_lpoint - x1) * (x2 - x1) + (T_upoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target upper left point
                double r2 = ((T_rpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target down right point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > S_slope){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//target upper left point 
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);//target down right point
                        //if r1 is closer to e1, S -> right, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> left, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                    else if(s2 == S_slope){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//target upper left point 
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);//target down right point
                        //if r1 is closer to e1, S -> right, T -> none
                        if(r1_distance < r2_distance){
                            width = r1_distance;
                            right1 = false;
                            ratio1 = 0;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> left, T -> none
                        else{
                            width = r2_distance;
                            right1 = false;
                            ratio1 = 0;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//target upper left point 
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);//target down right point
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(PI - S_theta + T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(PI - S_theta + T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
                //target upper left point is in searching area but target down right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //only s2 < s_slope would happen
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//distance of target upper left point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_dpoint,e1->length());//distance of source down left point to target edge
                    //if r1 is smaller, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        ratio1 = cos(PI - S_theta + T_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is smaller, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }   
                }
                //target down right point is in searching area but target upper left point isn't
                else{
                    //only s2 < s_slope would happen
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_dpoint,T_rpoint,edge_length);//distance of target down right point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_upoint,S_rpoint,e1->length());//distance of source upper right point to target edge
                    //if r1 is smaller, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = cos(PI - S_theta + T_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is smaller, S -> right, T -> left
                    else{
                        width =  r2_distance;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }  
                }
            }
        cout << "width is :" << width << endl; 
        }
    }
    //type4 source (slope < 0)
    else/*if((y1 > y2 && x1 < x2) || (y1 < y2 && x1 > x2))*/{
        cout << "Source type is: 4" << endl;
        double S_slope = -1/((y2 - y1) / (x2 - x1)); //not slope of edge but its width's direction
        double S_theta = atan(S_slope);
        if(S_theta < 0) S_theta += PI;
        //check if target is in searching area
        double rs = ((e1->sNode()->x() - x1) * (x2 - x1) + (e1->sNode()->y() - y1) * (y2 - y1)) / pow(edge_length,2);//T_sNode
        double rt = ((e1->tNode()->x() - x1) * (x2 - x1) + (e1->tNode()->y() - y1) * (y2 - y1)) / pow(edge_length,2);//T_tNode
        //target point is in searching area
        if((0 <= rs && rs <= 1) || (0 <= rt && rt <= 1)){
            //type1 target
            if(e1->sNode()->y() == e1->tNode()->y()){
                double r1 = ((T_lpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target left point
                double r2 = ((T_rpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target right point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);//distance of target left point to source edge
                    double r2_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);//distance of target right point to source edge
                    //if r1 is closer to e1, S -> right, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = cos(PI/2 - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is closer to e1, S -> left, T -> left
                    else{
                        width =  r2_distance;
                        right1 = false;
                        ratio1 = cos(PI/2 - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }   
                }
                //target left point is in searching area but target right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_dpoint > S_dpoint, S -> right, T -> right
                    if(T_dpoint > S_dpoint){
                        width = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        right1 = true;
                        ratio1 = cos(PI/2 - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_dpoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target right point is in searching area but target left point isn't
                else{
                    //if T_upoint > S_upoint, S -> right, T -> right
                    if(T_upoint > S_upoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_upoint,e1->length());
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> left, T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);
                        right1 = false;
                        ratio1 = cos(PI/2 - S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
            //type2 target
            else if(e1->sNode()->x() == e1->tNode()->x()){
                double r1 = ((T_lpoint - x1) * (x2 - x1) + (T_upoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target upper point
                double r2 = ((T_lpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target down point
                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//distance of target upper point to source edge
                    double r2_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);//distance of target down point to source edge
                    //if r1 is closer to e1, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = cos(S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is closer to e1, S -> right, T -> left
                    else{
                        width =  r2_distance;
                        right1 = false;
                        ratio1 = cos(S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }   
                }
                //target upper point is in searching area but target down point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //if T_rpoint < S_rpoint, S -> left, T -> right
                    if(T_rpoint < S_rpoint){
                        width = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);
                        right1 = true;
                        ratio1 = cos(S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_dpoint,e1->length());
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
                //target down point is in searching area but target upper point isn't
                else{
                    //if T_lpoint < S_lpoint, S -> left, T -> right
                    if(T_lpoint < S_lpoint){
                        width = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_lpoint,S_upoint,e1->length());
                        right1 = true;;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else, S -> right, T -> left
                    else{
                        width = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        right1 = false;
                        ratio1 = cos(S_theta);;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                }
            }
            //type3 target
            else if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x())){
                double T_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e1

                double r1 = ((T_rpoint - x1) * (x2 - x1) + (T_upoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target upper right point
                double r2 = ((T_lpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target down left point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > S_slope){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(PI - T_theta + S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(PI - T_theta + S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                    else if(s2 == S_slope){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        //if r1 is closer to e1, S -> left, T -> none
                        if(r1_distance < r2_distance){
                            width = r1_distance;
                            right1 = true;
                            ratio1 = 0;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width = r2_distance;
                            right1 = false;
                            ratio1 = 0;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_dpoint,edge_length);
                        //if r1 is closer to e1, S -> left, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> right, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
                //target upper right point is in searching area but target down left point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    //only s2 > S_slope would happen
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_upoint,edge_length);//distance of target upper right point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_dpoint,e1->length());//distance of source down right point to target edge
                    //if r1 is smaller, S -> left, T -> right
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = true;
                        ratio1 = cos(PI - T_theta + S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is smaller, S -> right, T -> left
                    else{
                        width = r2_distance;
                        right1 = false;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }   
                }
                //target down left point is in searching area but target upper right point isn't
                else{
                    //only s2 > S_slope would happen
                    double r1_distance = shortest_distance(x1,y1,x2,y2,T_dpoint,T_lpoint,edge_length);//distance of target down left point to source edge
                    double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_upoint,S_lpoint,e1->length());//distance of source upper left point to target edge
                    //if r1 is smaller, S -> right, T -> left
                    if(r1_distance < r2_distance){
                        width =  r1_distance;
                        right1 = false;
                        ratio1 = cos(PI - T_theta + S_theta);
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }
                    //else if r2 is smaller, S -> left, T -> right
                    else{
                        width =  r2_distance;
                        right1 = true;
                        ratio1 = 1;
                        solver.addCapacityConstraints(e1,right1,ratio1,width);
                        return true;
                    }  
                }
            }
            //type4 target
            else/*if((e1->sNode()->y() > e1->tNode()->y() && e1->sNode()->x() < e1->tNode()->x()) || (e1->sNode()->y() < e1->tNode()->y() && e1->sNode()->x() > e1->tNode()->x()))*/{
                double T_slope = -1/((e1->tNode()->y() - e1->sNode()->y()) / (e1->tNode()->x() - e1->sNode()->x())); //not slope of edge but its width's direction
                double T_theta = atan(T_slope);
                if(T_theta < 0) T_theta += PI;
                double s1 = -1/S_slope; // slope of e1
                double s2 = -1/T_slope; // slope of e1

                double r1 = ((T_lpoint - x1) * (x2 - x1) + (T_upoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target upper left point
                double r2 = ((T_rpoint - x1) * (x2 - x1) + (T_dpoint - y1) * (y2 - y1)) / pow(edge_length,2);//Target down right point

                //both target points are in searching area
                if((0 <= r1 && r1 <= 1) && (0 <= r2 && r2 <= 1)){
                    //compare the slope of two edges
                    if(s2 > s1){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//target upper left point 
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);//target down right point
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                    else if(s2 == s1){
                        double x1 = x1 - (y1/s1);//x-intercept of source edge
                        double x2 = e1->sNode()->x() - (e1->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//target upper left point 
                        double r2_distance = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);//target down right point
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
                //target upper left point is in searching area but target down right point isn't
                else if((0 <= r1 && r1 <= 1) && !(0 <= r2 && r2 <= 1)){
                    if(s2 > s1){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//distance of target upper left point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_dpoint,e1->length());//distance of source down right point to target edge
                        //if r1 is smaller, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is smaller, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                    else if(s2 == s1){
                        double x1 = x1 - (y1/s1);//x-intercept of source edge
                        double x2 = e1->sNode()->x() - (e1->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_lpoint,T_upoint,edge_length);//distance of target upper left point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_rpoint,S_dpoint,e1->length());//distance of source down right point to target edge
                        //if r1 is closer to e1, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
                //target down right point is in searching area but target upper left point isn't
                else{
                    if(s2 > s1){
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_dpoint,T_rpoint,edge_length);//distance of target down right point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_upoint,S_lpoint,e1->length());//distance of source upper left point to target edge
                        //if r1 is smaller, S -> left, T -> right
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = true;
                            ratio1 = cos(T_theta - S_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is smaller, S -> right, T -> left
                        else{
                            width =  r2_distance;
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }  
                    }
                    else if(s2 == s1){
                        double x1 = x1 - (y1/s1);//x-intercept of source edge
                        double x2 = e1->sNode()->x() - (e1->sNode()->y()/s2);//x-intercept of target edge
                        //x1 < x2, S -> right, T -> left
                        if(x1 < x2){
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);
                            right1 = false;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else, S -> left, T -> right
                        else{
                            width = shortest_distance(x1,y1,x2,y2,T_rpoint,T_dpoint,edge_length);
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                    }
                    else{
                        double r1_distance = shortest_distance(x1,y1,x2,y2,T_dpoint,T_rpoint,edge_length);//distance of target down right point to source edge
                        double r2_distance = shortest_distance(e1->sNode()->x(),e1->sNode()->y(),e1->tNode()->x(),e1->tNode()->y(),S_upoint,S_lpoint,e1->length());//distance of source upper left point to target edge
                        //if r1 is closer to e1, S -> right, T -> left
                        if(r1_distance < r2_distance){
                            width =  r1_distance;
                            right1 = false;
                            ratio1 = cos(S_theta - T_theta);
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }
                        //else if r2 is closer to e1, S -> left, T -> right
                        else{
                            width =  r2_distance;
                            right1 = true;
                            ratio1 = 1;
                            solver.addCapacityConstraints(e1,right1,ratio1,width);
                            return true;
                        }   
                    }
                }
            }
        cout << "width is :" << width << endl; 
        }
    }
    return false;
}

void AddObstacleConstraint(OASGEdge* e1, Obstacle* obs, FlowLP &solver){

    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    bool already_add = false;

    for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
        /*
        //bottom edge
        x1 = obs->vShape(shapeId)-> minX();
        y1 = obs->vShape(shapeId)-> minY();
        x2 = obs->vShape(shapeId)-> maxX();
        y2 = obs->vShape(shapeId)-> minY();
        cout << "bottom edge" << endl;
        cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
        if(!already_add){
            AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
        }
    
        */
        //left edge
        already_add = false;
        x1 = obs->vShape(shapeId)-> minX();
        y1 = obs->vShape(shapeId)-> minY();
        x2 = obs->vShape(shapeId)-> maxX();
        y2 = obs->vShape(shapeId)-> maxY();
        cout << "left edge" << endl;
        cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
        if(!already_add){
            AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
        }

        /*
        //top edge
        already_add = false;
        x1 = obs->vShape(shapeId)-> minX();
        y1 = obs->vShape(shapeId)-> maxY();
        x2 = obs->vShape(shapeId)-> maxX();
        y2 = obs->vShape(shapeId)-> maxY();
        cout << "top edge" << endl;
        cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
        if(!already_add){
            AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
        }
        */
        //right edge
        already_add = false;
        x1 = obs->vShape(shapeId)-> maxX();
        y1 = obs->vShape(shapeId)-> minY();
        x2 = obs->vShape(shapeId)-> maxX();
        y2 = obs->vShape(shapeId)-> maxY();
        cout << "right edge" << endl;
        cout << "(x1 , y1) = " << "(" << x1 << " , " << y1 << ")" << endl;
        cout << "(x2 , y2) = " << "(" << x2 << " , " << y2 << ")" << endl;
        already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
        if(!already_add){
            AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
        }
        
    }
}

void AddRectangularBoardConstraint(OASGEdge* e1, double Board_width, double Board_height ,FlowLP &solver){

    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    bool already_add = false;
    //bottom edge
    x1 = 0;
    y1 = 0;
    x2 = Board_width;
    y2 = 0;
    already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
    if(!already_add){
        AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
    }

    //left edge
    already_add = false;
    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = Board_height;
    already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
    if(!already_add){
        AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
    }

    //top edge
    already_add = false;
    x1 = 0;
    y1 = Board_height;
    x2 = Board_width;
    y2 = Board_height;
    already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
    if(!already_add){
        AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
    }

    //right edge
    already_add = false;
    x1 = Board_width;
    y1 = 0;
    x2 = Board_width;
    y2 = Board_height;
    already_add = AsTargetCapacityConstraint(e1,x1,y1,x2,y2,solver);
    if(!already_add){
        AsSourceCapacityConstraint(e1,x1,y1,x2,y2,solver);
    }
}