#define MAX_PTR_TABLE 50000

#include <iostream>
#include <string>
#include <cstdlib>
#include <climits>
#include <ctime>
#include <map>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#ifndef POINT_H
    #define POINT_H
    #include <project3/point.h>
#endif

class rrtTree
{
private:
    struct node
    {
        int idx;
        point rand;
        point location;
        int idx_parent;
    }*root;

    int count;
    point x_init, x_goal;

    cv::Mat map_original;
    double map_origin_x, map_origin_y;
    double res;
    node *ptrTable[MAX_PTR_TABLE];

    void visualizeTree();
    void visualizeTree(std::vector<point> path);
    inline void flushVertex();
    void addVertex(point x_new, point x_rand, int idx_near);
    int nearestNeighbor(point x_rand);
    bool isObstacle(point p);
    bool isObstacle(int row, int col);
    bool isCollision(point x1, point x2);
    point randomState(double x_max, double x_min, double y_max, double y_min);
    point newState(int idx_near, point x_rand, double MaxStep);
    void adjustPoint(std::map<int, node*>& near_map);
    double feasibleReagion();

public:
    cv::Mat map;

    rrtTree();
    rrtTree(point x_init, point x_goal,cv::Mat original_map, cv::Mat map, double map_origin_x, double map_origin_y, double res);
    ~rrtTree();

    int generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep);
    int generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep);

    std::vector<point> backtracking();
};
