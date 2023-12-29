#include "AddCapacity.h"
#include "LayerILP.h"
#include "VoltEigen.h"
#include "FlowLP.h"
#include <utility>

double shortest_distance(pair<double, double> A, pair<double, double> B, pair<double, double> P, pair<double, double> &F) {
    // https://i.imgur.com/Oa972xQ.png
    double len_square = (A.first-B.first)*(A.first-B.first) + (A.second-B.second)*(A.second-B.second);
    double r = ((P.first-A.first)*(B.first-A.first) + (P.second-A.second)*(B.second-A.second)) / len_square;

    if(r >= 0 && r <= 1) {
        F.first = A.first + r*(B.first-A.first);
        F.second = A.second + r*(B.second-A.second);

        return sqrt(pow(F.first- P.first,2) + pow(F.second - P.second,2));
    }
    else {
        F = make_pair(-1, -1);
        return -1;
    }
}

bool isRight(pair<double, double> S, pair<double, double> T, pair<double, double> P) {
    // return (P is on the right of segment ST)? 1: 0
    // https://i.imgur.com/g3XAnZo.jpg


    double v1x = T.first-S.first, v1y = T.second-S.second; // v1 = ST vector
    double v2x = P.first-S.first, v2y = P.second-S.second; // v2 = SP vector

    // z-coordinate of v1 x v2 = v1xv2y - v1yv2x, return true if z>=0
    return (v1x*v2y - v1y*v2x) <= 0;
}

bool addConstraint(pair<double, double> S1, pair<double, double> T1, pair<double, double> S2, pair<double, double> T2, pair<double, double> &ratio, pair<bool, bool> &right, double &width) {
    // input 4 points, return true if they are constrained
    // pass ratio1, ratio2, right1, right2, width by reference
    double v1x = T1.first - S1.first, v1y = T1.second - S1.second;
    double v2x = T2.first - S2.first, v2y = T2.second - S2.second;
    double cos = fabs((v1x*v2x + v1y*v2y)/(sqrt(v1x*v1x+v1y*v1y)*sqrt(v2x*v2x+v2y*v2y)));

    double dist, min_dist = 999999;
    pair<double, double> P, F; // foot
    ratio = make_pair(-1, -1);

    //middle point for S1T1
    pair<double,double> M1 = make_pair ( (S1.first+T1.first)/2 ,(S1.second+T1.second)/2 );
    //middle point for S2T2
    pair<double,double> M2 = make_pair ( (S2.first+T2.first)/2 ,(S2.second+T2.second)/2 );


    // project S1 to e2
    P = S1;
    dist = shortest_distance(S2, T2, P, F);
    if(F.first >= 0) {
        min_dist = dist;
        ratio = make_pair(cos, 1);
        right = make_pair(isRight(S1, T1, F), isRight(S2, T2, P));
    }
    // project T1 to e2
    P = T1;
    dist = shortest_distance(S2, T2, P, F);
    if(F.first >= 0 && dist < min_dist) {
        min_dist = dist;
        ratio = make_pair(cos, 1);
        right = make_pair(isRight(S1, T1, F), isRight(S2, T2, P));
    }
    // project M1 to e2
    P = M1;
    dist = shortest_distance(S2, T2, P, F);
    if(F.first >= 0 && dist < min_dist) {
        min_dist = dist;
        ratio = make_pair(cos, 1);
        right = make_pair(isRight(S1, T1, F), isRight(S2, T2, P));
    }
    // project S2 to e1
    P = S2;
    dist = shortest_distance(S1, T1, P, F);
    if(F.first >= 0 && dist < min_dist) {
        min_dist = dist;
        ratio = make_pair(1, cos);
        right = make_pair(isRight(S1, T1, P), isRight(S2, T2, F));
    }
    // project T2 to e1
    P = T2;
    dist = shortest_distance(S1, T1, P, F);
    if(F.first >= 0 && dist < min_dist) {
        min_dist = dist;
        ratio = make_pair(1, cos);
        right = make_pair(isRight(S1, T1, P), isRight(S2, T2, F));
    }
    // project M2 to e1
    P = M2;
    dist = shortest_distance(S1, T1, P, F);
    if(F.first >= 0 && dist < min_dist) {
        min_dist = dist;
        ratio = make_pair(1, cos);
        right = make_pair(isRight(S1, T1, P), isRight(S2, T2, F));
    }
    
    width = min_dist;
    return ratio.first >= 0;
}