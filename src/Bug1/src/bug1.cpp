#include "matplotlibcpp.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;

float angle_wrap(float angle);

struct robot_check{
    std::vector<std::pair<float,float>> intersection_edge;
    std::vector<std::pair<float,float>> sampled_points;
};

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
        std::vector<std::pair<float,float>> CheckIntersectionWObs(std::pair<float,float> pos);


};
 
class robot{
    public:
        std::pair<float, float> pos;
        float RadiusofView              = 0.1;
        float SensorResolution          = M_PI/16;

        robot(float x, float y){
            this->pos.first   =   x;
            this->pos.second  =   y;
        }
        robot_check contactSensor(std::vector<obstacle> obs);
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

int robot_test_main(){
    ////////////////////////////////////Obstacle Definition and plotting inputs/////////////////
    std::vector<std::pair<float,float>> p   = {std::pair<float,float>{1,1},std::pair<float,float>{2,1},std::pair<float,float>{1.5,2},std::pair<float,float>{1,1}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{1,1},std::pair<float,float>{1.5,-2},std::pair<float,float>{2,1},std::pair<float,float>{1,1}};
    std::vector<float> plot_x               = {};
    std::vector<float> plot_y               = {};
    std::vector<float> PlotSampledX         = {};
    std::vector<float> PlotSampledY         = {};

    for(int i = 0; i < 8; i++){
        if(i < 4){
            plot_x.push_back(p[i].first);
            plot_y.push_back(p[i].second);
        }
        else{
            plot_x.push_back(p1[i-4].first);
            plot_y.push_back(p1[i-4].second);
        }
        std::cout << plot_x[i] << " " << plot_y[i] << " " << std::endl;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    obstacle obs(p);
    obstacle obs1(p1);
    robot r(1.5,-2.05);
    std::vector<obstacle> Union_obstacle;
    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    robot_check a; // A temporary variable to check intersections
    a = r.contactSensor(Union_obstacle);

    if(a.intersection_edge[0].first == -INFINITY && a.intersection_edge[0].second == -INFINITY){
        std::cout << "no intersection" << std::endl;
    }
    else{
        std::cout << "Intersection with points\n" << a.intersection_edge[0].first << " " << a.intersection_edge[0].second << " ; " << a.intersection_edge[1].first << " " << a.intersection_edge[1].second << std::endl;
    }

    for(int i = 0; i < a.sampled_points.size(); i++){
        PlotSampledX.push_back(a.sampled_points[i].first);
        PlotSampledY.push_back(a.sampled_points[i].second);
    }


    plt::named_plot("Polygon_points",plot_x,plot_y,"*-");
    plt::named_plot("Sensor_samples",PlotSampledX,PlotSampledY,"*g");
    plt::named_plot("Obstacle_detected", std::vector<float>{r.pos.first},std::vector<float>{r.pos.second},"*r");
    plt::named_plot("Nearest_edge", std::vector<float>{a.intersection_edge[0].first,a.intersection_edge[1].first},
                                    std::vector<float>{a.intersection_edge[0].second,a.intersection_edge[1].second});	
    plt::grid(true);
    plt::legend();
	plt::show();

    return 0;
}

int obstacle_test_main(){
    ////////////////////////////////////Obstacle Definition and plotting inputs/////////////////
    std::vector<std::pair<float,float>> p;
    std::vector<float> plot_x       = {};
    std::vector<float> plot_y       = {};
    for(int i = 0; i < 3; i++){
        std::pair<float, float> p1;
        std::cout << "Point " << i+1  << " : "<< std::endl;
        std::cin >> p1.first >> p1.second;
        plot_x.push_back(p1.first);
        plot_y.push_back(p1.second);
        p.push_back(p1);
    }
    std::reverse(p.begin(),p.end());
    p.push_back(std::pair<float,float>{p[p.size()-1].first,p[p.size()-1].second});
    plot_x.push_back(p[p.size()-1].first); plot_y.push_back(p[p.size()-1].second);
    ///////////////////////////////////////////////////////////////////////////////////////////
    obstacle obs(p);
    std::pair<float,float> p_check;
    std::vector<std::pair<float, float>> a; // A temporary variable to check intersections
    p_check.first                   = 1.5;
    p_check.second                  = 1.1;

    a                               = obs.CheckIntersectionWObs(p_check);

    if(a[0].first == -INFINITY && a[0].second == -INFINITY){
        std::cout << "no intersection" << std::endl;
    }
    else{
        std::cout << "Intersection with points\n" << a[0].first << " " << a[0].second << " ; " << a[1].first << " " << a[1].second << std::endl;
    }

    plt::named_plot("Polygon_points",plot_x,plot_y,"*-");
    plt::named_plot("Obstacle_detected", std::vector<float>{p_check.first},std::vector<float>{p_check.second},"*r");
    plt::named_plot("Nearest_edge", std::vector<float>{a[0].first,a[1].first},std::vector<float>{a[0].second,a[1].second});	
    plt::grid(true);
    plt::legend();
	plt::show();

    return 0;
}

int main(){
    // obstacle_test_main();
    robot_test_main();
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

std::vector<std::pair<float,float>> obstacle::CheckIntersectionWObs(std::pair<float,float> pos){
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
    if (std::abs(my_sum) >= M_PI){
        intersection        = true;
    } 

    for(int i = 0; i < this->obs_points.size(); i++){
        dist_from_line      = DistanceLinesegmentfromPt(pos,this->obs_points[(i+1)%n].first,this->obs_points[(i+1)%n].second,
                                                        this->obs_points[i].first, this->obs_points[i].second);

        if(dist_from_line < prev_min){
            prev_min        = dist_from_line;
            line_cnt        = i;
        }
    }
    std::vector<std::pair<float,float>> ans;
    if(intersection){
        ans.push_back(this->obs_points[line_cnt]);
        ans.push_back(this->obs_points[(line_cnt+1)%n]);
    }
    else{
        ans.push_back(std::pair<float,float>(-INFINITY,-INFINITY));
        ans.push_back(std::pair<float,float>(-INFINITY,-INFINITY));
    }
    return ans;
}


robot_check robot::contactSensor(std::vector<obstacle> obs){
    /*        Generate points for sampling around the robot       */
    std::vector<std::pair<float,float>> sample_points;
    std::pair<float,float> intersection_point;
    std::vector<std::pair<float,float>> intersection_edge; // In ACW fashion

    // Generating Sampling points for the sensor
    float i = M_PI_2;
    while(i < 5*M_PI_2){
        float x     = this->pos.first     + RadiusofView * std::cos(i);
        float y     = this->pos.second    + RadiusofView * std::sin(i);
        sample_points.push_back(std::pair<float,float>{x,y});
        i += SensorResolution;
    }

    // These loops check intersection with each obstacle present in the environment
    for(obstacle o : obs){
        for(std::pair<float, float> p : sample_points){
            std::cout << o.obs_points[0].first << " " << o.obs_points[1].first << " " << o.obs_points[2].first << " " << o.obs_points[3].first << "\n";
            intersection_edge = o.CheckIntersectionWObs(p);
            if(intersection_edge[0].first != -INFINITY && intersection_edge[0].second != -INFINITY){
                intersection_point = p;
                robot_check ans;
                ans.intersection_edge = intersection_edge;
                ans.sampled_points    = sample_points;
                return ans;

            }
        }
    }
    robot_check ans;
    ans.intersection_edge = intersection_edge;
    ans.sampled_points    = sample_points;
    return ans;
}