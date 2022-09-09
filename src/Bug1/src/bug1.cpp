#include "matplotlibcpp.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;
float angle_wrap(float angle);

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

        float DistanceLinefromPt(std::pair<float,float> pos,float a,float b,float c);
        float DistanceLinesegmentfromPt(std::pair<float, float> pos, float x1, float y1, float x2, float y2);
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
        robot_check contactSensor_alternate(std::vector<obstacle> obs, float start_scan);
        robot_check contactSensor(std::vector<obstacle> obs, float direction_angle);
        std::pair<float,float> minDistpt(std::vector<std::pair<float,float>> trajectory,std::pair<float,float>Pos);
        std::vector<std::pair<float,float>> Bug1MainLoop(std::pair<float,float> iniPos, std::pair<float,float> goalPos, std::vector<obstacle> obs);
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


int bug1Driver(){
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
    robot r(0.0,0.0);
    std::vector<obstacle> Union_obstacle;
    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    std::vector<obstacle> Union_obstacle_merged;
    Union_obstacle_merged.push_back(obs);
    Union_obstacle_merged.push_back(p_merged);
    std::pair<float,float> goalPos = {10.0,10.0};
    std::vector<std::pair<float,float>> Bug1Trajectory;
    std::vector<float> Bug1X;
    std::vector<float> Bug1Y;

    Bug1Trajectory = r.Bug1MainLoop(r.pos, goalPos, Union_obstacle_merged);
    
    for(int i = 0; i < Bug1Trajectory.size(); i++){
        Bug1X.push_back(Bug1Trajectory[i].first);
        Bug1Y.push_back(Bug1Trajectory[i].second);
    }

    for(int i = 0; i < plot_x.size(); i++){
        plt::named_plot("Obj " + std::to_string(i),plot_x[i],plot_y[i],"*--");
    }
    plt::named_plot("Trajectory", Bug1X, Bug1Y,"*b");
    plt::grid(true);
    plt::legend();
	plt::show();

    return 0;
}

int main(){
    int i = bug1Driver();
    return 0;
}


float obstacle::DistanceLinefromPt(std::pair<float,float> pos,float a,float b,float c){
    return std::abs(a*pos.first + b*pos.second + c)/std::sqrt(std::pow(a,2) + std::pow(b,2));
}

float obstacle::DistanceLinesegmentfromPt(std::pair<float, float> pos, float x1, float y1, float x2, float y2){
    float a                 = pos.first - x1;
    float b                 = pos.second - y1;
    float c                 = x2 - x1;
    float d                 = y2 - y1;

    float dot               = a * c + b * d;
    float lenSq             = c * c + d * d;
    float param             = -1;
    if(lenSq != 0.0){
        param               = dot/lenSq;
    }
    float xx, yy; // Projection points on the line. // triangular distance from this is distance from point
    if(param < 0.0){
        xx                  = x1;
        yy                  = y1;                
    }
    else if(param > 1){
        xx                  = x2;
        yy                  = y2;
    }
    else{
        xx                  = x1 + param * c;
        yy                  = y1 + param * d;
    }
    return std::sqrt(std::pow(pos.first-xx,2) + std::pow(pos.second - yy,2));
}

bool obstacle::CheckIntersectionWObs(std::pair<float,float> pos){
    // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
    int n                   = this->obs_points.size();
    float my_sum            = 0;
    bool intersection       = false;
    float prev_min          = INFINITY;
    float dist_from_line    = 0;
    int line_cnt            = 0;

    for(int i = 0; i < this->obs_points.size(); i++){
        // my_sum = sum of all interior angles; interior angles = angle of pos.point vec  - angle of pos.next point vec
        float ang           = std::atan2(this->obs_points[(i+1)%n].second - pos.second, this->obs_points[(i+1)%n].first - pos.first) 
                            - std::atan2(this->obs_points[i].second - pos.second, this->obs_points[i].first - pos.first);
        ang                 = angle_wrap(ang);
        my_sum              += ang;
    }
    if (std::abs(my_sum)    >= M_PI){
        intersection        = true;
    } 
    return intersection;

    // std::vector<std::pair<float,float>> ans;
    // if(intersection){
    //     ans.push_back(this->obs_points[line_cnt]);
    //     ans.push_back(this->obs_points[(line_cnt+1)%n]);
    // }
    // else{
    //     ans.push_back(std::pair<float,float>(-INFINITY,-INFINITY));
    //     ans.push_back(std::pair<float,float>(-INFINITY,-INFINITY));
    // }
    // return ans;
}


robot_check robot::contactSensor(std::vector<obstacle> obs, float start_scan){
    /*        Generate points for sampling around the robot       */
    std::vector<std::pair<float,float>> sample_points;
    std::pair<float,float> intersection_point;
    bool intersection_check = false;

    // Generating Sampling points for the sensor
    float i = start_scan - M_PI;
    while(i < M_PI + start_scan){
        float x     = this->pos.first     + RadiusofView * std::cos(i);
        float y     = this->pos.second    + RadiusofView * std::sin(i);
        sample_points.push_back(std::pair<float,float>{x,y});
        i += SensorResolution;
    }
    // These loops check intersection with each obstacle present in the environment
    for(obstacle o : obs){
        for(std::pair<float, float> p : sample_points){
            // std::cout << o.obs_points[0].first << " " << o.obs_points[1].first << " " << o.obs_points[2].first << " " << o.obs_points[3].first << "\n";
            intersection_check = o.CheckIntersectionWObs(p);
            if(intersection_check){
                intersection_point = p;
                robot_check ans;
                ans.sampled_points      = sample_points;
                ans.intersection_point  = intersection_point;
                ans.intersection_heading= std::atan2(intersection_point.second - this->pos.second, intersection_point.first - this->pos.first);
                return ans;
            }
        }
    }
    robot_check ans;
    ans.intersection_check  = intersection_check;
    ans.sampled_points      = sample_points;
    ans.intersection_point  = intersection_point;
    ans.intersection_heading= std::atan2(intersection_point.second - this->pos.second, intersection_point.first - this->pos.first);
    return ans;
}

robot_check robot::contactSensor_alternate(std::vector<obstacle> obs, float start_scan){
    /*        Generate points for sampling around the robot       */

    std::vector<std::pair<float,float>> sample_points;
    std::pair<float,float> intersection_beam;
    bool intersection_check,intersection_check1; // In ACW fashion
    std::vector<std::pair<float,float>> temp_check;
    std::vector<float> angles;
    robot_check ans,ans1;
    bool transition = false;

    // Generating Sampling points for the sensor

    float i = M_PI/2 + 0.3 + start_scan;
    while(i > start_scan - M_PI/2 - 0.3){
        float x     = this->pos.first     + RadiusofView * std::cos(i);
        float y     = this->pos.second    + RadiusofView * std::sin(i);
        sample_points.push_back(std::pair<float,float>{x,y});
        angles.push_back(i);
        i -= SensorResolution;
    }

    ans.intersection_check  = false;
    ans.sampled_points      = sample_points;
    ans.intersection_point  = sample_points[0];
    ans.intersection_heading= angles[0];

    ans1.intersection_check  = false;
    ans1.sampled_points      = sample_points;
    ans1.intersection_point  = sample_points[sample_points.size() -1];
    ans1.intersection_heading= angles[sample_points.size() -1];

    // These loops check intersection with each obstacle present in the environment
    for(int j = 0; j < obs.size(); j++){
        for(int i = 0; i < sample_points.size()-1; i++){

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




std::vector<std::pair<float,float>> robot::Bug1MainLoop(std::pair<float,float> iniPos, std::pair<float,float> goalPos, std::vector<obstacle> obs){

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
    int q_h_count           = -1;
    int inicnt              =  0;
    std::pair<float,float> q_l, q_h;
    std::vector<std::pair<float,float>> trajectory;
    std::vector<std::pair<float,float>> obsTrajectory;
    std::vector<float> PlotSampledX         = {};
    std::vector<float> PlotSampledY         = {};

    int count = 0;
    int max_c = 163;
    while(count < max_c){
        count ++;
        // std::cout << "x: " <<  this->pos.first << " y: " << this->pos.second << std::endl; 
        trajectory.push_back(this->pos);
        // if dist from goal < 1m say algorithm is complete
        if(std::sqrt(std::pow(this->pos.first-goalPos.first,2) + std::pow(this->pos.second-goalPos.second,2)) < 1){
            break;
        } 

        ::temp_a                   = this->contactSensor_alternate(obs, this->heading);
        robot_check a = ::temp_a;
        if(count == max_c - 1){
            for(int i = 0; i < a.sampled_points.size(); i++){
                PlotSampledX.push_back(a.sampled_points[i].first);
                PlotSampledY.push_back(a.sampled_points[i].second);
            }
            plt::named_plot("Sensor_samples",PlotSampledX,PlotSampledY,"*g");

        }

        // Flag setting procedure
        if(!a.intersection_check){
            // obstacleFollow  = false;
            obstacleCircle  = false;
            gotoQl          = false;
            headtoGoal      = true;
        }
        else{
            // if(!obstacleFollow && q_h_count == -1){
            if(headtoGoal){
                // std::cout << "hit obs count: " << q_h_count << " obstacle circling?: " << obstacleCircle << " Obstacle following? " << obstacleFollow << std::endl;
                q_h             = std::pair<float,float>{pos.first,pos.second};
                q_h_count++;
            }
            // obstacleFollow      = true;
            if(!gotoQl){
                obstacleCircle  = true;
                gotoQl          = false;
            }
            else{
                obstacleCircle  = false;
                gotoQl          = true;
            }
            headtoGoal          = false;
            inFrontObs          = true; //checks if obstacle is directly in front or not, based on this we take left turn or just follow wall right or left.
        }

        // 
        if(headtoGoal){
            obstacleCircle      = false;
            gotoQl              = false;
            // obstacleFollow      = false;
            this->heading       = std::atan2(goalPos.second - this->pos.second, goalPos.first - this->pos.first);
            this->pos.first     += this->robotStep*std::cos(this->heading);
            this->pos.second    += this->robotStep*std::sin(this->heading);
        }
        else{

        // if(obstacleFollow){
            float distQl    = std::sqrt(std::pow(this->pos.first-q_l.first,2)+std::pow(this->pos.second-q_l.second,2));
            float distQh    = std::sqrt(std::pow(this->pos.first-q_h.first,2)+std::pow(this->pos.second-q_h.second,2));

            if(obstacleCircle){
                if(distQh < 0.1 && inicnt > 10){
                    // obstacleFollow = true;
                    obstacleCircle = false;
                    gotoQl         = true;
                    q_l            = minDistpt(obsTrajectory,goalPos); // finds the minimum distance point to the system
                    obsTrajectory.clear();
                    --q_h_count;
                    std::cout << "In Qh " << " qlx: "  << q_l.first << " qly: " << q_l.second << " count: " << count << std::endl;
                    std::cout << " qhx: "  << q_h.first << " qhy: " << q_h.second << " q_h_count " << q_h_count << std::endl;
                }
                this->heading = a.intersection_heading;
                // if(inFrontObs){
                //     this->heading        = std::atan2(a.intersection_edge[0].second - a.intersection_edge[1].second, a.intersection_edge[0].first - a.intersection_edge[1].first);
                // }
                // else{
                //     this->heading        = std::atan2(a.intersection_edge[1].second - a.intersection_edge[0].second, a.intersection_edge[1].first - a.intersection_edge[0].first); // Added M_PI for left turning
                // }
                inicnt++;
                this->pos.first     += this->robotStep*std::cos(this->heading);
                this->pos.second    += this->robotStep*std::sin(this->heading);
                obsTrajectory.push_back(this->pos);
            }

            if(gotoQl){
                if(distQl < 0.5){
                    // obstacleFollow = false;
                    obstacleCircle = false;
                    gotoQl         = false;
                    headtoGoal     = true;
                    --q_h_count;
                    inicnt = 0;
                    std::cout << "Done with obstacle 1" << " count " << count << std::endl;
                    std::cout << " qhx: "  << q_h.first << " qhy: " << q_h.second << " q_h_count " << q_h_count << std::endl;
                }

                this->heading = a.intersection_heading;
                // std::cout << " qhx: "  << q_h.first << " qhy: " << q_h.second << std::endl;
                // std::cout << "distQh: " << distQh << " q_h_count " << q_h_count << " count: " << count << std::endl;
                std::cout << " qlx: "  << q_l.first << " qly: " << q_l.second << std::endl;
                // if(inFrontObs){
                //     this->heading        = std::atan2(a.intersection_edge[0].second - a.intersection_edge[1].second, a.intersection_edge[0].first - a.intersection_edge[1].first);
                // }
                // else{
                //     this->heading        = std::atan2(a.intersection_edge[1].second - a.intersection_edge[0].second, a.intersection_edge[1].first - a.intersection_edge[0].first); // Added M_PI for left turning
                // }

                this->pos.first     += this->robotStep*std::cos(this->heading);
                this->pos.second    += this->robotStep*std::sin(this->heading);
            }
        }
        // }
    }

    return trajectory;
}

std::pair<float,float> robot::minDistpt(std::vector<std::pair<float,float>> trajectory,std::pair<float,float>Pos){
    float minD             = INFINITY;
    std::pair<float,float> minDpos;
    for(int i = 0; i < trajectory.size(); i++){
        float dist = std::sqrt(std::pow(Pos.first-trajectory[i].first,2)+std::pow(Pos.second-trajectory[i].second,2));
        if(dist < minD){
            minDpos = trajectory[i];
            minD    = dist;
        }
    }
    return minDpos;
}

bool robot::inFront(float intersectionHeading){
    if(intersectionHeading > M_PI/6 && intersectionHeading < -M_PI/6){
        return true;
    }
    else{
        return false;
    }
}