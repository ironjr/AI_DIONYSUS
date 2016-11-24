// Debug Option
// #define DEBUG_TEXT
// #define DEBUG_POINT_CLOUD
// #define DEBUG_BALL

#define VISUALIZE_PATH false

#define CONST_K 50000
#define MAX_STEP 0.25
#define MARGIN 10
#define LOOK_AHEAD_DISTANCE 0.5
#define GOAL_CONFIRM_DISTANCE 0.5

// Dynamic mapping
#define OBSTACLE_DETERMINENT_DISTANCE 3
#define CAMERA_SAMPLING_RATE 3 // # of pixels skipped between each pixel observed
#define COLLISION_CHECK_RATE 3 // # of iterations passed before next collision check
#define HORIZON 274 // HORIZON <= "Ground" <= 480
#define CHECK_Z_ABOVE_HORIZON 45
#define GROUND_HEIGHT 0.309900 // Kinect y axis
#define NEW_OBSTACLE_MARGIN 10 // map update margin
#define OBSTACLE_HEIGHT 0.35
#define SLEEP_PLANNING 3
#define WIGGLING_THRESHOLD 50

// State definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <project3/purePursuit.h>
#include <project3/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/cloud_viewer.h>
#include <algorithm>


// Debug image
static int imageNumberMain = 1;

// Map spec
cv::Mat map;
cv::Mat dynamic_map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

// Waypoints
std::vector<point> waypoints;

// Path
int goal_visited = 0;
point goalPoint;
rrtTree* pathTree;
std::vector<point> path_RRT;
ros::ServiceClient gazebo_spawn;

// Robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

// Point cloud data from kinect
pcl::PointCloud<pcl::PointXYZ> point_cloud;
#ifdef DEBUG_POINT_CLOUD
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud_ptr (&point_cloud);
    pcl::visualization::CloudViewer cloud_viewer("CloudViewer");
#endif

// FSM state
int state;

// Function definition
bool isCollision();
void dynamic_mapping();
void set_waypoints();
void generate_path_RRT(point& from);
void generate_path_RRT(point& from, point& to);
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
inline void setcmdvel(double v, double w);
point transformFrameKinect2World(pcl::PointXYZ kinectPoint, point pointofRobot);
inline double getActualZ(pcl::PointXYZ kinectPoint);
cv::Mat addMargin(cv::Mat map, int margin);

#ifdef DEBUG_TEXT
    void printDebug(std::string str) {     
        std::cout<<str<<std::endl;
    }
#endif

#ifdef DEBUG_BALL
    void spawnBall(point p);

    std::string  ballLaunch = std::string("<robot name=\"simple_ball\">") +
        std::string("<link name=\"ball\">") +
        std::string("<inertial>") +
        std::string("<mass value=\"1.0\" />") +
        std::string("<origin xyz=\"0 0 0\" />") +
        std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
        std::string("</inertial>") +
        std::string("<visual>") +
        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
        std::string("<geometry>") +
        std::string("<sphere radius=\"0.09\"/>") +
        std::string("</geometry>") +
        std::string("</visual>") +
        std::string("<collision>") +
        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
        std::string("<geometry>") +
        std::string("<sphere radius=\"0\"/>") +
        std::string("</geometry>") +
        std::string("</collision>") +
        std::string("</link>") +
        std::string("<gazebo reference=\"ball\">") +
        std::string("<mu1>10</mu1>") +
        std::string("<mu2>10</mu2>") +
        std::string("<material>Gazebo/Blue</material>") +
        std::string("<turnGravityOff>true</turnGravity Off>") +
        std::string("</gazebo>") +
        std::string("</robot>");
    int ballIdx = 0;
#endif

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",1,callback_state);
    ros::Subscriber gazebo_kinect_sub = n.subscribe("/camera/depth/points",1,callback_points);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);

    gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    printf("Initialize topics\n");

    // Load Map
    char* user = getlogin();
    map = cv::imread((std::string("/home/")+
      std::string(user)+
      std::string("/catkin_ws/src/project3/src/ground_truth_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;
    res = 0.05;

    dynamic_map = addMargin(map, MARGIN);
    printf("Load map\n");

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT(waypoints[0], waypoints[1]);
    printf("Generate RRT!\n");

    // FSM
    state = INIT;
    bool running = true;
    purePursuit pure_pursuit;
    int look_ahead_idx;
    ros::Rate control_rate(10);

    while(running){
        switch (state) {
            case INIT: {
                look_ahead_idx = 0;

                // Visualize path
#ifdef DEBUG_BALL
                if (VISUALIZE_PATH) {
                    for(int i = 0; i < path_RRT.size(); i++){
                        spawnBall(path_RRT[i]);
                    }
                    printf("Spawn path\n");
                } else {
                    printf("Skipped spawning path\n");
                }
#else
                printf("Skipped spawning path\n");
#endif

                // Initialize robot position
                geometry_msgs::Pose model_pose;
                model_pose.position.x = waypoints[0].x;
                model_pose.position.y = waypoints[0].y;
                model_pose.position.z = 0.3;
                model_pose.orientation.x = 0.0;
                model_pose.orientation.y = 0.0;
                model_pose.orientation.z = 0.0;
                model_pose.orientation.w = 0.0;

                geometry_msgs::Twist model_twist;
                model_twist.linear.x = 0.0;
                model_twist.linear.y = 0.0;
                model_twist.linear.z = 0.0;
                model_twist.angular.x = 0.0;
                model_twist.angular.y = 0.0;
                model_twist.angular.z = 0.0;

                gazebo_msgs::ModelState modelstate;
                modelstate.model_name = "RosAria";
                modelstate.reference_frame = "world";
                modelstate.pose = model_pose;
                modelstate.twist = model_twist;

                gazebo_msgs::SetModelState setmodelstate;
                setmodelstate.request.model_state = modelstate;

                gazebo_set.call(setmodelstate);
                setcmdvel(0, 0);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                ros::Rate(0.33).sleep();
                printf("Initialize ROBOT\n");

                state = RUNNING;
            } break;

            case RUNNING: {
                int iteration = 0; // # of while loop iteration
                purePursuit controller;
                std::vector<point>::iterator it = path_RRT.begin();
                int cc = 0;
                while(ros::ok()) {
                    ++iteration;
                    point nextPoint = *it;

                    // Collision check
                    if (iteration % COLLISION_CHECK_RATE == 0) {
                        // setcmdvel(0, 0);
                        // cmd_vel_pub.publish(cmd_vel);
                        // ros::spinOnce();
                        // ros::Rate(1).sleep();

                        if (isCollision()) {
                            state = PATH_PLANNING;
                            break;
                        }
                    }
                    // End check
                    if (robot_pose.distanceWith(nextPoint) < LOOK_AHEAD_DISTANCE) {
                        ++it;
                        if (it == path_RRT.end()) {
                            printf("Visited goal # %d\n", goal_visited);
                            ++goal_visited;
                            if (goal_visited == waypoints.size()) {
                                state = FINISH;
                                break;
                            }
                            state = PATH_PLANNING;
                            goalPoint = waypoints[goal_visited];                         
                            break;
                        }

#ifdef DEBUG_BALL
                        spawnBall((*it));
#endif
                        continue;
                    }   

#ifdef DEBUG_POINT_CLOUD
                    cloud_viewer.showCloud(point_cloud_ptr);
#endif

                    control result = controller.get_control(robot_pose, nextPoint);
                    setcmdvel(result.v, result.w);
                    cmd_vel_pub.publish(cmd_vel);

                    ros::spinOnce();
                    control_rate.sleep();
                }
            } break;

            case PATH_PLANNING: {
                printf("Path planning begin\n");
                setcmdvel(0, 0);
                cmd_vel_pub.publish(cmd_vel);

                std::stringstream ss;
                ss << "wow" << imageNumberMain << ".jpg";
                ++imageNumberMain;
                cv::imwrite(ss.str().c_str(), dynamic_map);

                ros::spinOnce();
                ros::Duration(SLEEP_PLANNING).sleep();

                dynamic_mapping();
                printf("Path planning over\n");
                state = RUNNING;
            } break;

            case FINISH: {
                setcmdvel(0,0);
                cmd_vel_pub.publish(cmd_vel);
                running = false;

                ros::spinOnce();
                control_rate.sleep();
            } break;

            default: {
            } break;
        }
    }

    return 0;
}

void set_waypoints() {
    point waypoint_candid[4];
    waypoint_candid[0].x = 5.0;
    waypoint_candid[0].y = -8.0;
    waypoint_candid[1].x = -6.0;
    waypoint_candid[1].y = -7.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 8.0;
    waypoint_candid[3].x = 3.0;
    waypoint_candid[3].y = 7.0;

    int order[] = {3,1,2,3};
    int order_size = 4;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

// Generate path without changing goal
void generate_path_RRT(point& from) {
    int count;
    path_RRT.clear();
    pathTree = new rrtTree(from, goalPoint, map, dynamic_map, map_origin_x, map_origin_y, res);
    dynamic_map = pathTree->map.clone();
    while (true) {
        count = pathTree->generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, CONST_K, MAX_STEP);
        std::cout << "Path planning at (" << robot_pose.x << ", " << robot_pose.y << ")" << std::endl;
        GridMapPoint gp = GridMapPoint(robot_pose, res, map_origin_x, map_origin_y);
        std::cout << "Current position map value : " << (int)map.at<uchar>(gp.i,gp.j) << std::endl;
        if (count != -1) {
            std::cout << "Path planning successful" << std::endl;
            break;
        }
        std::cout << "Path planning failed" << std::endl;

    }
    std::vector<point> path;
    path = pathTree->backtracking();
    std::reverse(path.begin(), path.end());
    path_RRT.insert(path_RRT.end(), path.begin(), path.end());
    delete pathTree;
}

// Generate path with changing goal
void generate_path_RRT(point& from, point& to) {
    goalPoint = to;
    generate_path_RRT(from);
}

void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"RosAria") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void callback_points(sensor_msgs::PointCloud2ConstPtr msgs){
    pcl::fromROSMsg(*msgs,point_cloud);
}

bool isCollision() {
    bool isObstacle = false;
    bool isNewObstacle = false;
    std::vector<point> obstacle;
    point poseOfRobot = robot_pose;

    // detection method 1: check straight line above ground
    GridMapPoint gp = GridMapPoint(poseOfRobot, res, map_origin_x, map_origin_y);
    if (dynamic_map.at<uchar>(gp.i,gp.j) == 0) {
        return false;
    }

    for (int i = 0; i < point_cloud.width; i += CAMERA_SAMPLING_RATE) {
        pcl::PointXYZ checkpoint = point_cloud.at(i, HORIZON - CHECK_Z_ABOVE_HORIZON);
        // check NaN value: NaN float f has property of "f != f == true" (IEEE)
        if (!(checkpoint.z != checkpoint.z) && getActualZ(checkpoint) > OBSTACLE_HEIGHT) {
            pcl::PointXYZ checkpointLeft = point_cloud.at(i - 1, HORIZON - CHECK_Z_ABOVE_HORIZON);
            pcl::PointXYZ checkpointRight = point_cloud.at(i + 1, HORIZON - CHECK_Z_ABOVE_HORIZON);
            if ((!(checkpointLeft.z != checkpointLeft.z) && getActualZ(checkpointLeft) > OBSTACLE_HEIGHT) ||
                (!(checkpointRight.z != checkpointRight.z) && getActualZ(checkpointRight) > OBSTACLE_HEIGHT)) {
                if (checkpoint.z < OBSTACLE_DETERMINENT_DISTANCE) {
                    obstacle.push_back(transformFrameKinect2World(checkpoint, poseOfRobot));
                    isObstacle = true;
                }
            }
        }
    } 
    if (obstacle.size() > WIGGLING_THRESHOLD) {
        obstacle.clear();
        return false;
    }

    int newObstacle = 0;
    if (isObstacle) {
        std::vector<point>::iterator it;
        GridMapPoint samplePoint;
        for (it = obstacle.begin(); it != obstacle.end(); ++it) {
            samplePoint = GridMapPoint(*it, res, map_origin_x, map_origin_y);
            if (samplePoint.i >= 600 || samplePoint.j >= 600 || samplePoint.i < 200 || samplePoint.j < 200) {
                continue;
            }
            if (dynamic_map.at<uchar>(samplePoint.i, samplePoint.j) > 0) {
                ++newObstacle;
                isNewObstacle = true;
                dynamic_map.at<uchar>(samplePoint.i, samplePoint.j) = 0;
                // look at robot_pose to determine where to draw margin
                //TODO 
                for (int j = samplePoint.i - NEW_OBSTACLE_MARGIN; j < samplePoint.i + NEW_OBSTACLE_MARGIN; j++) {
                    for (int k = samplePoint.j - NEW_OBSTACLE_MARGIN; k < samplePoint.j + NEW_OBSTACLE_MARGIN; k++) {
                        dynamic_map.at<uchar>(j,k) = 0;
                    }    
                }
            }   
        }
    }
    obstacle.clear();
    std::cout << "# of obstacle point : " << newObstacle << std::endl;
    return isNewObstacle;
}

void dynamic_mapping() {
    generate_path_RRT(robot_pose);
}

inline void setcmdvel(double v, double w){
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}

#ifdef DEBUG_BALL
    void spawnBall(point p) {
        gazebo_msgs::SpawnModel model;
        model.request.model_xml = ballLaunch;

        std::ostringstream ball_name;
        ball_name << "Ball " << ballIdx;
        model.request.model_name = ball_name.str();
        model.request.reference_frame = "world";
        model.request.initial_pose.position.x = p.x;
        model.request.initial_pose.position.y = p.y;
        model.request.initial_pose.position.z = 0.7;
        model.request.initial_pose.orientation.w = 0.0;
        model.request.initial_pose.orientation.x = 0.0;
        model.request.initial_pose.orientation.y = 0.0;
        model.request.initial_pose.orientation.z = 0.0;

        gazebo_spawn.call(model);
        ballIdx++;

        ros::spinOnce();
    }
#endif

point transformFrameKinect2World(pcl::PointXYZ kinectPoint, point poseOfRobot) {
    point worldPoint;
    worldPoint.th = 0;

    double robot_x = kinectPoint.z;
    double robot_y = -kinectPoint.x;

    double cosTheta = cos(poseOfRobot.th);
    double sinTheta = sin(poseOfRobot.th);
    worldPoint.x = robot_x * cosTheta - robot_y * sinTheta + poseOfRobot.x;
    worldPoint.y = robot_x * sinTheta + robot_y * cosTheta + poseOfRobot.y;

    return worldPoint;
}

inline double getActualZ(pcl::PointXYZ kinectPoint) {
    return -kinectPoint.y + GROUND_HEIGHT;
}

cv::Mat addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

// cv::Mat addNewMargin(cv::Mat map, GridMapPoint obstacle, int newMargin, ) {

// }