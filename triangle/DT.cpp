#include <iostream>
#include <utility>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
using namespace std;

struct Triangle {
    pair<double, double> A, B, C;
};

void counterClock(Triangle&);

Triangle make_triangle(pair<double, double> A, pair<double, double> B, pair<double, double> C) {
    Triangle tri;
    tri.A = A;
    tri.B = B;
    tri.C = C;
    counterClock(tri);
    return tri;
}

void counterClock(Triangle& tri) {
    if((tri.B.first-tri.A.first) * (tri.C.second-tri.A.second) - (tri.C.first-tri.A.first) * (tri.B.second-tri.A.second) < 0)
        swap(tri.B, tri.C);
    // (bx - ax)*(cy - ay)-(cx - ax)*(by - ay) > 0
}

vector<Triangle> triangulate(const vector<pair<double, double> >& pointList, int n) {

    double s = 0;
    for(int i=0; i<pointList.size(); i++)
        s += pointList[i].first * pointList[(i+1)%pointList.size()].second
            -pointList[i].second * pointList[(i+1)%pointList.size()].first;

    double polyArea = fabs(s/2);
    
    double avgArea = polyArea / n;

    // printf("%d %f %f\n", n, polyArea, avgArea);


    FILE *fp = fopen("DT.poly", "w");
    fprintf(fp, "%3d 2 1 0\n", pointList.size());
    for(int i=0; i<pointList.size(); i++)
        fprintf(fp, "%3d %15.10f %15.10f\n", i+1, pointList[i].first, pointList[i].second);
    
	fprintf(fp, "%d 0\n", pointList.size());
    for(int i=0; i<pointList.size(); i++)
        fprintf(fp, "%3d %3d %3d\n", i+1, (i+2 > pointList.size())? 1: i+2, i+1);
    
    fprintf(fp, "%3d\n", 0);
    fclose(fp);

    
    // system("ls -al");

    // system("./triangle DT.poly");
    char str[1000];
    sprintf(str, "./triangle -a%f DT.poly", avgArea);
    system(str);
	

    int t1, t2, t3, t4, t5; // temp

    vector<Triangle> triList;
    fp = fopen("DT.1.node", "r");
    int numNode;
    fscanf(fp, "%d %d %d %d", &numNode, &t2, &t3, &t4);
    vector<pair<double, double> > nodeList;
    for(int i=0; i<numNode; i++) {
        double x, y;
        fscanf(fp, "%d %lf %lf %d %d", &t1, &x, &y, &t4, &t5);
        nodeList.push_back({x, y});
    }


    fp = fopen("DT.1.ele", "r");
    int numTri;
    fscanf(fp, "%d %d %d\n", &numTri, &t2, &t3);
    for(int i=0; i<numTri; i++) {
        fscanf(fp, "%d %d %d %d", &t1, &t2, &t3, &t4);
        triList.push_back(make_triangle(nodeList[t2-1], nodeList[t3-1], nodeList[t4-1]));
    }
    fclose(fp);

    return triList;
}

double area(Triangle tri) {
    /*
        Ax  Bx  Cx  Ax
          x   x   x      *   1/2
        Ay  By  Cy  Ay
    */
    return 0.5* (tri.A.first*tri.B.second - tri.B.first*tri.A.second + tri.B.first*tri.C.second - tri.C.first*tri.B.second + tri.C.first*tri.A.second - tri.A.first*tri.C.second);
}

int main() {
    vector<pair<double, double> > pointList;
    int n;
    FILE *fp = fopen("DT.in", "r");
    fscanf(fp, "%d", &n);
    for(int i=0; i<n; i++) {
        double x, y;
        fscanf(fp, "%lf %lf", &x, &y);
        pointList.push_back(make_pair(x, y));
    }
    fclose(fp);
    
    
    int nTri = 10;
    cin>> nTri;
    vector<Triangle> triList = triangulate(pointList, nTri);




    // print polygon
    printf("polygon\n%d\n", pointList.size());
    for(int i=0; i<pointList.size(); i++)
        printf("%15.10f %15.10f\n", pointList[i].first, pointList[i].second);
    printf("\n");
	
    // print triangle
    printf("triangle\n%d\n", triList.size());
    for(int i=0; i<triList.size(); i++)
        printf("%15.10f %15.10f\n%15.10f %15.10f\n%15.10f %15.10f\n", 
                                            triList[i].A.first, triList[i].A.second,
                                            triList[i].B.first, triList[i].B.second,
                                            triList[i].C.first, triList[i].C.second);
    printf("\n");


    for(int i=0; i<triList.size(); i++)
        printf("%f\n", area(triList[i]));
    
    return 0;
}