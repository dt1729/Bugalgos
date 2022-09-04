#include "matplotlibcpp.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;
float angle_wrap(float angle);
class obstacle{
    public:
    // Assumed that points given in acw direction as required by obstacle definitions
        std::vector<std::pair<float,float>> rectangle_obs;

        obstacle(std::pair<float,float> point1, std::pair<float, float> point2, std::pair<float, float> point3, std::pair<float, float> point4){
            rectangle_obs.push_back(point1);
            rectangle_obs.push_back(point2);
            rectangle_obs.push_back(point3);
            rectangle_obs.push_back(point4);
        }

        float DistanceLinefromPt(std::pair<float,float> pos,float a,float b,float c){
            return std::abs(a*pos.first + b*pos.second + c)/std::sqrt(std::pow(a,2) + std::pow(b,2));
        }

        std::vector<std::pair<float,float>> CheckIntersectionWLine(std::pair<float,float> pos, obstacle obs){
            // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
            int n = obs.rectangle_obs.size();
            float my_sum = 0;
            bool intersection = false;
            float prev_min = INFINITY;
            float dist_from_line = 0;
            int line_cnt = 0;
            for(int i = 0; i < obs.rectangle_obs.size(); i++){
                // my_sum = sum of all interior angles; interior angles = angle of pos.point vec  - angle of pos.next point vec
                std::cout << std::atan2(obs.rectangle_obs[(i+1)%n].second - pos.second, obs.rectangle_obs[(i+1)%n].first - pos.first) << " " << std::atan2(obs.rectangle_obs[i].second - pos.second, obs.rectangle_obs[i].first - pos.first) << "\n";
                float ang = std::atan2(obs.rectangle_obs[(i+1)%n].second - pos.second, obs.rectangle_obs[(i+1)%n].first - pos.first) - std::atan2(obs.rectangle_obs[i].second - pos.second, obs.rectangle_obs[i].first - pos.first) ;
                std::cout << ang << "\n";
                ang = angle_wrap(ang);
                my_sum += ang;
                // my_sum = angle_wrap(my_sum);
            }
            std::cout << my_sum;
            if (std::abs(my_sum) >= M_PI){
                intersection = true;
            } 

            for(int i = 0; i < obs.rectangle_obs.size(); i++){
                float a = -(obs.rectangle_obs[(i+1)%n].second - obs.rectangle_obs[i].second);
                float b = obs.rectangle_obs[(i+1)%n].first - obs.rectangle_obs[i].first;
                float c = obs.rectangle_obs[i].first*(obs.rectangle_obs[(i+1)%n].second - obs.rectangle_obs[i].second) -  obs.rectangle_obs[i].second*(obs.rectangle_obs[(i+1)%n].first - obs.rectangle_obs[i].first);
                dist_from_line = DistanceLinefromPt(pos,a,b,c);
                std::cout << a << " " << b << " " << c << " " << dist_from_line << "\n";
                if(dist_from_line < prev_min){
                    prev_min = dist_from_line;
                    line_cnt = i;
                }
            }
            std::vector<std::pair<float,float>> ans;
            if(intersection){
                ans.push_back(obs.rectangle_obs[line_cnt]);
                ans.push_back(obs.rectangle_obs[(line_cnt+1)%n]);
            }
            else{
                ans.push_back(std::pair<float,float>(-INFINITY,-INFINITY));
                ans.push_back(std::pair<float,float>(-INFINITY,-INFINITY));
            }
            return ans;
        }
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

int main(){
    std::pair<float,float> p1, p2, p3, p4;
    p1.first = 1; p1.second = 1;
    p2.first = 3; p2.second = 3;
    p3.first = 2; p3.second = 5;
    p4.first = 0.5; p4.second = 4;

    obstacle obs(p1,p2,p3,p4);
    std::pair<float,float> p_check;
    std::vector<std::pair<float, float>> a;
    p_check.first = 1.01;
    p_check.second = 1.025;
    a = obs.CheckIntersectionWLine(p_check,obs);
    if(a[0].first == -INFINITY && a[0].second == -INFINITY){
        std::cout << "no intersection" << std::endl;
    }
    else{
        std::cout << "Intersection with points\n" << a[0].first << " " << a[0].second << " ; " << a[1].first << " " << a[1].second << std::endl;
    }
    std::vector<float> plot_x = {p1.first,p2.first,p3.first,p4.first,p1.first};
    std::vector<float> plot_y = {p1.second,p2.second,p3.second,p4.second,p1.second};
    plt::named_plot("Polygon_points",plot_x,plot_y,"*-");
    plt::named_plot("Obstacle_detected", std::vector<float>{p_check.first},std::vector<float>{p_check.second},"*r");
    plt::named_plot("Nearest_edge", std::vector<float>{a[0].first,a[1].first},std::vector<float>{a[0].second,a[1].second});	
    plt::legend();
	plt::show();
    return 0;
}