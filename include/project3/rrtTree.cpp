// #define DEBUG_VISUALIZE_TREE
#define DEBUG_IMAGE_FILE

#define ETA 2.0
#define GAMMA_MULTIPLIER 1
#define GAMMA 14        
#define SAMPLE_RATE 1

#include "rrtTree.h"

using namespace std;

#ifdef DEBUG_IMAGE_FILE
    static int numberOfDeaths = 0;
    static int imageNumber = 1;
#endif


double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map_original, cv::Mat map, double map_origin_x, double map_origin_y, double res) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map_original.clone();
    this->map = map.clone();
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = 0;
    root->location = x_init;
    root->rand = x_init;
}

rrtTree::~rrtTree(){
    flushVertex();
}

#ifdef DEBUG_VISUALIZE_TREE
    void rrtTree::visualizeTree(){
        int thickness = 1;
        int lineType = 8;
        int idx_parent;
        double Res = 2;
        double radius = 6;
        cv::Point x1, x2;

        cv::Mat map_c;
        cv::Mat imgResult;
        cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
        cv::resize(map_c, imgResult, cv::Size(), Res, Res);

        cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
        cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

        for(int i = 1; i < this->count; i++) {
            idx_parent = this->ptrTable[i]->idx_parent;
            x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }

        cv::namedWindow("Mapping");
        cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
        cv::imshow("Mapping", imgResult(imgROI));
        cv::waitKey(0);

    }

    void rrtTree::visualizeTree(std::vector<point> path){
        int thickness = 1;
        int lineType = 8;
        int idx_parent;
        double Res = 2;
        double radius = 6;
        cv::Point x1, x2;

        cv::Mat map_c;
        cv::Mat imgResult;
        cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
        cv::resize(map_c, imgResult, cv::Size(), Res, Res);

        cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
        cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

        for(int i = 1; i < this->count; i++) {
            idx_parent = this->ptrTable[i]->idx_parent;
            x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }

        thickness = 3;
        for(int i = 1; i < path.size(); i++) {
            x1 = cv::Point((int)(Res*(path[i-1].y/res + map_origin_y)), (int)(Res*(path[i-1].x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }

        cv::namedWindow("Mapping");
        cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
        cv::imshow("Mapping", imgResult(imgROI));
        cv::waitKey(0);

    }
#endif

void rrtTree::addVertex(point x_new, point x_rand, int idx_near) {
    ptrTable[count] = new node;
    ptrTable[count]->idx = count;
    ptrTable[count]->location = x_new;
    ptrTable[count]->rand = x_rand;
    ptrTable[count]->idx_parent = idx_near;
    ++count;
}

inline void rrtTree::flushVertex() {
    for (int i = 1; i < count; ++i)
        delete ptrTable[i];
    count = 1;
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    point p;
    double x = fRand(x_min, x_max);
    double y = fRand(y_min, y_max);

    p.x = x;
    p.y = y;
    p.th = 0;

    return p;
}

point rrtTree::newState(int idx_near, point x_rand, double MaxStep) {
    point p_new;
    point p = ptrTable[idx_near]->location;
    double delta_x = x_rand.x - p.x;
    double delta_y = x_rand.y - p.y;
    double distance = x_rand.distanceWith(p);
    p_new.x = p.x + delta_x * MaxStep / distance;
    p_new.y = p.y + delta_y * MaxStep / distance;
    p_new.th = 0;

    return p_new;
}

int rrtTree::nearestNeighbor(point x_rand) {
    double distanceMin = INT_MAX;
    int minIdx = -1; 
    for (int i = 0; i < count; i++) {
        point p = ptrTable[i]->location;
        double randDistance = x_rand.distanceWith(p);
        if (randDistance < distanceMin) {
            distanceMin = randDistance;
            minIdx = i;
        }
    }

    return minIdx;
}

bool rrtTree::isObstacle(point p) {
    int i = (map_origin_x + 0.5) + p.x / res;
    int j = (map_origin_y + 0.5) + p.y / res;
    if (map.at<uchar>(i, j) != 0) {
        return false;
    }
    return true;
}

bool rrtTree::isObstacle(int row, int col) {
    if (map.at<uchar>(row, col) != 0) {
        return false;
    }
    return true;
}

bool rrtTree::isCollision(point x1, point x2) {
    /* i, x stands for project x axis
     * j, y stands for project y axis
     */
    double x1_i_d = (map_origin_x + 0.5) + x1.x / res;
    double x1_j_d = (map_origin_y + 0.5) + x1.y / res;
    double x2_i_d = (map_origin_x + 0.5) + x2.x / res;
    double x2_j_d = (map_origin_y + 0.5) + x2.y / res;

    double abs_delta_x = abs(x2_i_d - x1_i_d);
    double abs_delta_y = abs(x2_j_d - x1_j_d);


    if (abs_delta_x > abs_delta_y) {
        if (x1_i_d > x2_i_d) {
            swap(x1_i_d, x2_i_d);
            swap(x1_j_d, x2_j_d);
        }

        double grad = (x2_j_d - x1_j_d) / (x2_i_d - x1_i_d);

        for (int i = floor(x1_i_d); i <= floor(x2_i_d); ++i) {
            int j_small = floor(x1_j_d + grad * ((double)i - x1_i_d));
            int j_large = j_small + 1;
            if (j_large > 600 || j_small < 200) {
                continue;
            }
            if (map.at<uchar>(i, j_small) == 0 || map.at<uchar>(i, j_large) == 0) {
                return true;
            }
        }
    }
    else {
        if (x1_j_d > x2_j_d) {
            swap(x1_i_d, x2_i_d);
            swap(x1_j_d, x2_j_d);
        }

        double grad = (x2_i_d - x1_i_d) / (x2_j_d - x1_j_d);

        for (int j = floor(x1_j_d); j <= floor(x2_j_d); ++j) {
            int i_small = floor(x1_i_d + grad * ((double)j - x1_j_d));
            int i_large = i_small + 1;
            if (i_large > 600 || i_small < 200) {
                continue;
            }
            if (map.at<uchar>(i_small, j) == 0 || map.at<uchar>(i_large, j) == 0) {
                return true;
            }
        }
    }
    return false;
}

std::vector<point> rrtTree::backtracking() {
    std::vector<point> result;
    int idx = count - 1;
    node* pt = ptrTable[idx];
    
    while (idx > 0) {
        result.push_back(pt->location);
        for (int i = 0; i < SAMPLE_RATE & idx > 0; ++i) {
            idx = pt->idx_parent;
            pt = ptrTable[idx];
        }
    }
    result.push_back(ptrTable[0]->location);

    return result;
}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    flushVertex();
    for (int i = 0; i < K; ++i) {
        point x_rand = randomState(x_max, x_min, y_max, y_min);
        int idx_near = nearestNeighbor(x_rand);
        point x_new = newState(idx_near, x_rand, MaxStep);
        if (i > 20 && isCollision(ptrTable[idx_near]->location, x_new)) {
            continue;
        }
        addVertex(x_new, x_rand, idx_near);
        double dis = x_new.distanceWith(x_goal);
        if (dis < MaxStep) {
            return count;
        }

    }
    flushVertex();
    return -1;
}

double rrtTree::feasibleReagion() {
    int count = 0;
    int rows = map.rows;
    int cols = map.cols;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (isObstacle(r, c)) {
                ++count;
            }
        }
    }
    double feasibleReagionRatio = (double)count / (rows * cols);
    // return feasibleReagionRatio * 
}

void rrtTree::adjustPoint(std::map<int, node*>& near_map) {
    std::map<int, node*> ancesotors;
    while (near_map.size() > 0) {
        vector <node*> existList; // should reset the parents
        std::map<int, node*>::iterator it = near_map.begin();
        int idx_outParent = -1;
        do {
            node* value = it->second;
            near_map.erase(it);
            ancesotors[value->idx] = value;

            it = near_map.find(value->idx_parent);  
            if (it == near_map.end()) { //if node's parent is not in the near_map  
                std::map<int, node*>::iterator it2 = ancesotors.find(value->idx_parent); 
                if (it2 != ancesotors.end()) { // if node's parent is in the circle but not in near_map 
                    existList.push_back(value); 
                    std::map<int, node*>::iterator it3 = ancesotors.find(it2->second->idx_parent);
                    if (it3 == ancesotors.end()) {
                        idx_outParent = it2->second->idx;
                    } else {
                        idx_outParent = it2->second->idx_parent;
                    }
                    break;
                } else { // if node's parent is not in the circle
                    idx_outParent = value->idx;
                    break;
                }
            } else { // if node's parent is in the near_map
                existList.push_back(value); 
            }
        } while (true);
        for (int i = 0; i < existList.size(); i++) {
            if(!isCollision(existList[i]->location, ptrTable[idx_outParent]->location)) {
                existList[i]->idx_parent = idx_outParent;    
            }
        }
        existList.clear();
    }
}

int rrtTree::generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    flushVertex();

    // const double minGamma = sqrt(3)*

    for (int i = 0; i < K; ++i) {
        point x_rand = randomState(x_max, x_min, y_max, y_min);
        int idx_near = nearestNeighbor(x_rand);
        point x_new = newState(idx_near, x_rand, MaxStep);
        double nearBallRadius = min(ETA, GAMMA * sqrt(log(double(count)) / double(count)));
        if (!isCollision(ptrTable[idx_near]->location, x_new)) {
            // make a nearest ball around new point
            addVertex(x_new, x_rand, idx_near);    
            std::map<int, node*> X_near;
            for (int j = 0; j < count; ++j) {
                if (x_new.distanceWith(ptrTable[j]->location) < nearBallRadius) {
                    X_near[j] = ptrTable[j];
                }
            }
            adjustPoint(X_near);
        }
        else {
            continue;
        }
        double dis = x_new.distanceWith(x_goal);
        if (dis < MaxStep) {
            //visualizeTree();
            std::cout << "GenerateRRTst successed with " << i << " iterations" << std::endl;

#ifdef DEBUG_IMAGE_FILE
            numberOfDeaths = 0;
#endif

            return count;
        }
    }
    std::cout << "GenerateRRTst failed with " << count << " points" << std::endl;

#ifdef DEBUG_IMAGE_FILE
    ++numberOfDeaths;
    if (numberOfDeaths >= 5) {
        std::stringstream ss;
        ss << "shit" << imageNumber << ".jpg";
        ++imageNumber;
        cv::imwrite(ss.str().c_str(), map);
        if (imageNumber >= 50) {
            exit(0);
        }
    }
#endif

    flushVertex();
    return -1;
}
