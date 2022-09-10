#include "matplotlibcpp.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;
float angle_wrap(float angle);
float DistanceLinefromPt(std::pair<float,float> pos, std::pair<float,float> p1, std::pair<float,float> p2);
struct robot_check{
    bool intersection_check;
    std::vector<std::pair<float,float>> sampled_points;
    std::pair<float,float> intersection_point;
    float intersection_heading;
};

robot_check temp_a;

class obstacle{
    public:
        // The obstacle class is supposed to represent one obstacle 
        // Assumed that points given in acw direction as required by obstacle definitions
        std::vector<std::pair<float,float>> obs_points;

        obstacle(std::vector<std::pair<float,float>> points){
            for(std::pair<float,float> p : points){
                obs_points.push_back(p);
            }
        }

        bool CheckIntersectionWObs(std::pair<float,float> pos);
};
 
class robot{
    public:
        std::pair<float, float> pos;
        float robotStep                 = 0.1;
        float heading                   = 0.0;
        float RadiusofView              = 0.1;
        float SensorResolution          = M_PI/6;

        robot(float x, float y){
            this->pos.first   =   x;
            this->pos.second  =   y;
        }
        robot_check contactSensor_alternate(std::vector<obstacle> obs, float start_scan,bool turn);
        // robot_check contactSensor(std::vector<obstacle> obs, float direction_angle);
        std::pair<float,float> minDistpt(std::vector<std::pair<float,float>> trajectory,std::pair<float,float>Pos);
        std::vector<std::pair<float,float>> Bug2MainLoop(std::pair<float,float> iniPos, std::pair<float,float> goalPos, std::vector<obstacle> obs,bool turn);
        bool inFront(float intersectionHeading);
};


float angle_wrap(float angle){
    if(angle > M_PI){
        return -1*(2*M_PI-angle);
    }
    else if(angle < -M_PI){
        return -1*(-2*M_PI-angle);
    }
    return angle;
}


float bug2_obstacle1Driver(bool turn){
    // Obstacle Definition
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{1,1},std::pair<float,float>{2,1},std::pair<float,float>{2,5},std::pair<float,float>{1,5},std::pair<float,float>{1,1}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{3,3},std::pair<float,float>{4,3},std::pair<float,float>{4,12},std::pair<float,float>{3,12},std::pair<float,float>{3,3}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{3,12},std::pair<float,float>{12,12},std::pair<float,float>{12,13},std::pair<float,float>{3,13},std::pair<float,float>{3,12}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{12,5},std::pair<float,float>{13,5},std::pair<float,float>{13,13},std::pair<float,float>{12,13},std::pair<float,float>{12,5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{6,5},std::pair<float,float>{12,5},std::pair<float,float>{12,6},std::pair<float,float>{6,6},std::pair<float,float>{6,5}};
    std::vector<std::pair<float,float>>p_merged = {std::pair<float,float>{3,3},std::pair<float,float>{3,13},std::pair<float,float>{13,13},std::pair<float,float>{13,5},std::pair<float,float>{6,5},
                                                    std::pair<float,float>{6,6},std::pair<float,float>{12,6},std::pair<float,float>{12,12},std::pair<float,float>{4,12},std::pair<float,float>{4,3},std::pair<float,float>{3,3}};

    std::vector<std::vector<float>> plot_x  = {};
    std::vector<std::vector<float>> plot_y  = {};
    std::vector<float> PlotSampledX         = {};
    std::vector<float> PlotSampledY         = {};
    std::vector<float> px,py;
    for(int i = 0; i < 25; i++){
        if(i < 5){
            px.push_back(p0[i].first);
            py.push_back(p0[i].second);
            if(i == 4){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 5 && i < 10){
            px.push_back(p1[i-5].first);
            py.push_back(p1[i-5].second);
            if(i == 9){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 10 && i < 15){
            px.push_back(p2[i-10].first);
            py.push_back(p2[i-10].second);
            if(i == 14){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 15 && i < 20){
            px.push_back(p3[i-15].first);
            py.push_back(p3[i-15].second);
            if(i == 19){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 20 && i < 25){
            px.push_back(p4[i-20].first);
            py.push_back(p4[i-20].second);
            if(i == 24){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);
    obstacle obs_mergerd(p_merged);
    std::pair<float,float> iniPos = {0.0,0.0};
    std::pair<float,float> goalPos = {10.0,10.0};
    robot r(iniPos.first,iniPos.second);
    std::vector<obstacle> Union_obstacle;
    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    std::vector<obstacle> Union_obstacle_merged;
    Union_obstacle_merged.push_back(obs);
    Union_obstacle_merged.push_back(p_merged);
    std::vector<std::pair<float,float>> Bug1Trajectory;
    std::vector<float> Bug1X;
    std::vector<float> Bug1Y;
    std::vector<float> losX;
    std::vector<float> losY;

    Bug1Trajectory = r.Bug2MainLoop(r.pos, goalPos, Union_obstacle,turn);
    
    for(int i = 0; i < Bug1Trajectory.size(); i++){
        Bug1X.push_back(Bug1Trajectory[i].first);
        Bug1Y.push_back(Bug1Trajectory[i].second);
    }

    for(int i = 0; i < plot_x.size(); i++){
        plt::named_plot("Obj " + std::to_string(i),plot_x[i],plot_y[i],"*--");
    }
    for(float t = 0; t <= 1.0; t += 0.1){
        losX.push_back(t*iniPos.first + (1-t)*goalPos.first);
        losY.push_back(t*iniPos.second + (1-t)*goalPos.second);
    }
    plt::named_plot("LOS", losX, losY,"--");
    plt::named_plot("Trajectory", Bug1X, Bug1Y,"*b");
    plt::grid(true);
    plt::legend();
    plt::xlabel("X axes(m)");
    plt::ylabel("Y axes(m)");
    plt::title("Bug2 Obstacle 1");
	plt::show();
    
    float len = 0;

    for(int i = 0; i < Bug1Trajectory.size()-1; i++){
        len += std::sqrt(std::pow(Bug1Trajectory[i].second - Bug1Trajectory[i+1].second,2) + std::pow(Bug1Trajectory[i].first - Bug1Trajectory[i+1].first,2));
    }

    return len;
}

float bug2_obstacle2Driver(bool turn){
    // Obstacle Definition
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{-6,-6},std::pair<float,float>{25,-6},std::pair<float,float>{25,-5},std::pair<float,float>{-6,-5},std::pair<float,float>{-6,-6}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{-6,5},std::pair<float,float>{30,5},std::pair<float,float>{30,6},std::pair<float,float>{-6,6},std::pair<float,float>{-6,5}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{-6,-5},std::pair<float,float>{-5,-5},std::pair<float,float>{-5,5},std::pair<float,float>{-6,5},std::pair<float,float>{-6,-5}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{4,-5},std::pair<float,float>{5,-5},std::pair<float,float>{5,1},std::pair<float,float>{4,1},std::pair<float,float>{4,-5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{9,0},std::pair<float,float>{10,0},std::pair<float,float>{10,5},std::pair<float,float>{9,5},std::pair<float,float>{9,0}};
    std::vector<std::pair<float,float>> p5  = {std::pair<float,float>{14,-5},std::pair<float,float>{15,-5},std::pair<float,float>{15,1},std::pair<float,float>{14,1},std::pair<float,float>{14,-5}};
    std::vector<std::pair<float,float>> p6  = {std::pair<float,float>{19,0},std::pair<float,float>{20,0},std::pair<float,float>{20,5},std::pair<float,float>{19,5},std::pair<float,float>{19,0}};
    std::vector<std::pair<float,float>> p7  = {std::pair<float,float>{24,-5},std::pair<float,float>{25,-5},std::pair<float,float>{25,1},std::pair<float,float>{24,1},std::pair<float,float>{24,-5}};
    std::vector<std::pair<float,float>> p8  = {std::pair<float,float>{29,0},std::pair<float,float>{30,0},std::pair<float,float>{30,5},std::pair<float,float>{29,5},std::pair<float,float>{29,0}};


    std::vector<std::vector<float>> plot_x  = {};
    std::vector<std::vector<float>> plot_y  = {};
    std::vector<float> PlotSampledX         = {};
    std::vector<float> PlotSampledY         = {};
    std::vector<float> px,py;
    for(int i = 0; i < 40; i++){
        if(i < 5){
            px.push_back(p0[i].first);
            py.push_back(p0[i].second);
            if(i == 4){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 5 && i < 10){
            px.push_back(p1[i-5].first);
            py.push_back(p1[i-5].second);
            if(i == 9){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 10 && i < 15){
            px.push_back(p2[i-10].first);
            py.push_back(p2[i-10].second);
            if(i == 14){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 15 && i < 20){
            px.push_back(p3[i-15].first);
            py.push_back(p3[i-15].second);
            if(i == 19){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 20 && i < 25){
            px.push_back(p4[i-20].first);
            py.push_back(p4[i-20].second);
            if(i == 24){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 25 && i < 30){
            px.push_back(p5[i-25].first);
            py.push_back(p5[i-25].second);
            if(i == 29){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 30 && i < 35){
            px.push_back(p6[i-30].first);
            py.push_back(p6[i-30].second);
            if(i == 34){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 35 && i < 40){
            px.push_back(p7[i-35].first);
            py.push_back(p7[i-35].second);
            if(i == 39){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
        else if(i >= 40 && i < 45){
            px.push_back(p8[i-40].first);
            py.push_back(p8[i-40].second);
            if(i == 44){
                plot_x.push_back(px);
                plot_y.push_back(py);
                px.clear();
                py.clear();
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);
    obstacle obs5(p5);
    obstacle obs6(p6);
    obstacle obs7(p7);
    obstacle obs8(p8);
    std::pair<float,float> iniPos = {0.0,0.0};
    std::pair<float,float> goalPos = {35.0,0.0};
    robot r(iniPos.first,iniPos.second);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    Union_obstacle.push_back(obs5);
    Union_obstacle.push_back(obs6);
    Union_obstacle.push_back(obs7);
    Union_obstacle.push_back(obs8);

    std::vector<std::pair<float,float>> Bug1Trajectory;
    std::vector<float> Bug1X;
    std::vector<float> Bug1Y;
    std::vector<float> losX;
    std::vector<float> losY;

    Bug1Trajectory = r.Bug2MainLoop(r.pos, goalPos, Union_obstacle,turn);
    
    for(int i = 0; i < Bug1Trajectory.size(); i++){
        Bug1X.push_back(Bug1Trajectory[i].first);
        Bug1Y.push_back(Bug1Trajectory[i].second);
    }

    for(int i = 0; i < plot_x.size(); i++){
        plt::named_plot("Obj " + std::to_string(i),plot_x[i],plot_y[i],"*--");
    }
    for(float t = 0; t <= 1.0; t += 0.1){
        losX.push_back(t*iniPos.first + (1-t)*goalPos.first);
        losY.push_back(t*iniPos.second + (1-t)*goalPos.second);
    }
    plt::named_plot("LOS", losX, losY,"--");
    plt::named_plot("Trajectory", Bug1X, Bug1Y,"*b");
    plt::grid(true);
    plt::legend();
    plt::xlabel("X axes(m)");
    plt::ylabel("Y axes(m)");
    plt::title("Bug2 Obstacle 2");
	plt::show();
    float len = 0;

    for(int i = 0; i < Bug1Trajectory.size()-1; i++){
        len += std::sqrt(std::pow(Bug1Trajectory[i].second - Bug1Trajectory[i+1].second,2) + std::pow(Bug1Trajectory[i].first - Bug1Trajectory[i+1].first,2));
    }

    return len;

}

int main(){
    bool turn = false;
    std::cout << "Do you want to take left turns? Enter 0(false)/1(true)" << std::endl;
    std::cin >> turn;

    float i = bug2_obstacle1Driver(turn);

    float j = bug2_obstacle2Driver(turn);

    std::cout << "Distance covered by bug for obstacle course 1: " << i << std::endl;
    std::cout << "Distance covered by bug for obstacle course 2: " << j << std::endl;

    return 0;
}


bool obstacle::CheckIntersectionWObs(std::pair<float,float> pos){
    // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
    int n                   = this->obs_points.size();
    float my_sum            = 0;
    bool intersection       = false;
    float prev_min          = INFINITY;
    float dist_from_line    = 0;
    int line_cnt            = 0;
    float xmin, ymin, xmax, ymax;
    xmin = INFINITY; ymin = INFINITY; xmax = -INFINITY; ymax = -INFINITY;

    for(int i = 0; i < obs_points.size(); i++){
        xmin = std::min(obs_points[i].first,xmin);
        ymin = std::min(obs_points[i].second,ymin);
        xmax = std::max(obs_points[i].first,xmax);
        ymax = std::max(obs_points[i].second,ymax);
    }

    if(pos.first >= xmin && pos.second >= ymin && pos.first <= xmax && pos.second <= ymax){
        return true;
    }
    else{
        return false;
    }
    return intersection;
}

// robot_check robot::contactSensor(std::vector<obstacle> obs, float start_scan){
//     /*        Generate points for sampling around the robot       */
//     std::vector<std::pair<float,float>> sample_points;
//     std::pair<float,float> intersection_point;
//     bool intersection_check = false;

//     // Generating Sampling points for the sensor
//     float i = start_scan - M_PI;
//     while(i < M_PI + start_scan){
//         float x     = this->pos.first     + RadiusofView * std::cos(i);
//         float y     = this->pos.second    + RadiusofView * std::sin(i);
//         sample_points.push_back(std::pair<float,float>{x,y});
//         i += SensorResolution;
//     }
//     // These loops check intersection with each obstacle present in the environment
//     for(obstacle o : obs){
//         for(std::pair<float, float> p : sample_points){
//             // std::cout << o.obs_points[0].first << " " << o.obs_points[1].first << " " << o.obs_points[2].first << " " << o.obs_points[3].first << "\n";
//             intersection_check = o.CheckIntersectionWObs(p);
//             if(intersection_check){
//                 intersection_point = p;
//                 robot_check ans;
//                 ans.sampled_points      = sample_points;
//                 ans.intersection_point  = intersection_point;
//                 ans.intersection_heading= std::atan2(intersection_point.second - this->pos.second, intersection_point.first - this->pos.first);
//                 return ans;
//             }
//         }
//     }
//     robot_check ans;
//     ans.intersection_check  = intersection_check;
//     ans.sampled_points      = sample_points;
//     ans.intersection_point  = intersection_point;
//     ans.intersection_heading= std::atan2(intersection_point.second - this->pos.second, intersection_point.first - this->pos.first);
//     return ans;
// }

robot_check robot::contactSensor_alternate(std::vector<obstacle> obs, float start_scan,bool turn){
    /*        Generate points for sampling around the robot       */

    std::vector<std::pair<float,float>> sample_points;
    std::pair<float,float> intersection_beam;
    bool intersection_check,intersection_check1; // In ACW fashion
    std::vector<std::pair<float,float>> temp_check;
    std::vector<float> angles;
    robot_check ans;
    bool transition = false;

    // Generating Sampling points for the sensor
    float i = start_scan - M_PI;

    while(i < start_scan + M_PI){
        float x     = this->pos.first     + RadiusofView * std::cos(i);
        float y     = this->pos.second    + RadiusofView * std::sin(i);
        sample_points.push_back(std::pair<float,float>{x,y});
        angles.push_back(i);
        i += SensorResolution;
    }
    if(!turn){
        std::reverse(sample_points.begin(),sample_points.end());
        std::reverse(angles.begin(),angles.end());
    }

    ans.intersection_check  = false;
    ans.sampled_points      = sample_points;
    ans.intersection_point  = sample_points[0];
    ans.intersection_heading= angles[0];

    // These loops check intersection with each obstacle present in the environment
    for(int i = 0; i < sample_points.size()-1; i++){
        for(int j = 0; j < obs.size(); j++){
            intersection_check = obs[j].CheckIntersectionWObs(sample_points[i]);
            intersection_check1 = obs[j].CheckIntersectionWObs(sample_points[i+1]);
            bool c1 = intersection_check;
            bool c2 = intersection_check1;
            if(c1&&!c2){
                        ans.intersection_check  = intersection_check;
                        ans.sampled_points      = sample_points;
                        ans.intersection_point  = sample_points[i+1];
                        ans.intersection_heading= angles[i+1];
                        break;
            }
            else if(!c1 && c2){
                        ans.intersection_check = intersection_check1;
                        ans.sampled_points      = sample_points;
                        ans.intersection_point  = sample_points[i];
                        ans.intersection_heading= angles[i];
                        break;
            }
        }
    }
    return ans;
}




std::vector<std::pair<float,float>> robot::Bug2MainLoop(std::pair<float,float> iniPos, std::pair<float,float> goalPos, std::vector<obstacle> obs,bool turn){

    /*  STEPS for Doing bug1 algorithm
    *  Move in the direction of goal
    *  if encounter obstacle i) save contact point(Q_h) ii) start move around obstacle
    *  ii) move around obstacle unless you come across contanct(Q_h)
    *  iii) move to smallest distance point(Q_l). 
    *  if smallest dist(Q_l,robot) < 0.1: Move in the direction of goal
    */

    bool obstacleFollow     = false;
    bool obstacleCircle     = false;
    bool gotoQl             = false;
    bool headtoGoal         = false; 
    bool inFrontObs         = false;
    int q_h_count           =  0; //  0 if prev state was head to goal, 1 if prev state was wall follow
    int inicnt              =  0;
    float distancefrmLOS    =  0; 
     //goal line constants
    std::pair<float,float> q_l, q_h;
    std::pair<float,float> StartingPos = this->pos;
    q_l = std::pair<float,float>{INFINITY,INFINITY};
    q_h = std::pair<float,float>{INFINITY,INFINITY};
    std::vector<std::pair<float,float>> trajectory;
    std::vector<std::pair<float,float>> obsTrajectory;

    std::vector<float> PlotSampledX         = {};
    std::vector<float> PlotSampledY         = {};

    // Debugging variables 
    int count = 0;
    int max_c = 2000;

    while(count < max_c){
        count ++;
        // std::cout << "x: " <<  this->pos.first << " y: " << this->pos.second << std::endl; 
        trajectory.push_back(this->pos);
        // if dist from goal < 0.5m say algorithm is complete
        if(std::sqrt(std::pow(this->pos.first-goalPos.first,2) + std::pow(this->pos.second-goalPos.second,2)) < 0.5){
            break;
        } 
        
        robot_check a = this->contactSensor_alternate(obs, this->heading,turn);            
        distancefrmLOS    = DistanceLinefromPt(this->pos, StartingPos, goalPos);
        // Flag setting procedure
        if(!a.intersection_check){
            if(headtoGoal){
                q_h_count       = 0;
            }
            else if(obstacleFollow){
                q_h_count       = 1;
            }
            obstacleCircle  = false;
            obstacleFollow  = false;
            headtoGoal      = true;
        }
        else{
            if(headtoGoal){
                q_h_count       = 0;
            }
            else if(obstacleFollow){
                q_h_count       = 1;
            }
            q_h                 = std::pair<float,float>{pos.first,pos.second};
            obstacleFollow      = true;
            obstacleCircle      = true;
            headtoGoal          = false;
        }

        // 
        if(obstacleFollow){
            if(obstacleCircle){
                this->heading = a.intersection_heading;
                this->pos.first     += this->robotStep*std::cos(this->heading);
                this->pos.second    += this->robotStep*std::sin(this->heading);
                if(distancefrmLOS < 0.0038){
                    if(q_h_count == 0){
                        continue;
                    }
                    else if(q_h_count == 1){
                    // std::cout << "Was In collision " << distancefrmLOS << std::endl;
                    int temp_cnt = 0;
                        while(temp_cnt < 5){
                            obstacleFollow      = false;
                            this->heading       = std::atan2(goalPos.second - this->pos.second, goalPos.first - this->pos.first);
                            this->pos.first     += this->robotStep*std::cos(this->heading);
                            this->pos.second    += this->robotStep*std::sin(this->heading);
                            temp_cnt++;
                        }
                        q_h_count = 0;
                    }
                    headtoGoal = true;
                }
            }
        }
        else if(headtoGoal){
            obstacleFollow      = false;
            this->heading       = std::atan2(goalPos.second - this->pos.second, goalPos.first - this->pos.first);
            this->pos.first     += this->robotStep*std::cos(this->heading);
            this->pos.second    += this->robotStep*std::sin(this->heading);
        }
        if(count == max_c){
            a = this->contactSensor_alternate(obs, this->heading,turn);   
            for(int i = 0; i < a.sampled_points.size(); i++){
                PlotSampledX.push_back(a.sampled_points[i].first);
                PlotSampledY.push_back(a.sampled_points[i].second);
            }
            plt::named_plot("Sensor_samples",PlotSampledX,PlotSampledY,"*g");
            plt::named_plot("Intersection Point", std::vector<float>{a.intersection_point.first}, std::vector<float>{a.intersection_point.second},"^r");
        }
    }

    return trajectory;
}

std::pair<float,float> robot::minDistpt(std::vector<std::pair<float,float>> trajectory,std::pair<float,float>Pos){
    float minD             = INFINITY;
    int   minCount         = 0;
    std::pair<float,float> minDpos;
    for(int i = 0; i < trajectory.size(); i++){
        float dist = std::sqrt(std::pow(Pos.first-trajectory[i].first,2)+std::pow(Pos.second-trajectory[i].second,2));
        if(dist < minD){
            std::cout << "mindD in if: " << minCount << " " << minCount << std::endl;
            minCount = i;
            minD    = dist;
        }
    }
    minDpos.first = trajectory[minCount].first;
    minDpos.second = trajectory[minCount].second;
    std::cout << "mindD: " << minCount << " " << minCount << std::endl;
    return minDpos;
}

bool robot::inFront(float intersectionHeading){
    if((intersectionHeading < M_PI/3 || intersectionHeading > 3*M_PI/3)){
        return true;
    }
    else{
        return false;
    }
}

float DistanceLinefromPt(std::pair<float,float> pos, std::pair<float,float> p1, std::pair<float,float> p2){
    float m = ((p2.second - p1.second)/(p2.first - p1.first));
    float c =  p1.second - ((m)*p1.first);
    return std::abs(pos.first - m*pos.second - c)/std::sqrt(1 + std::pow(m,2));
}
