int robot_test_main(){
    ////////////////////////////////////Obstacle Definition and plotting inputs/////////////////
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{1,1},std::pair<float,float>{2,1},std::pair<float,float>{2,5},std::pair<float,float>{1,5},std::pair<float,float>{1,1}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{3,3},std::pair<float,float>{4,3},std::pair<float,float>{4,12},std::pair<float,float>{3,12},std::pair<float,float>{3,3}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{3,12},std::pair<float,float>{12,12},std::pair<float,float>{12,13},std::pair<float,float>{3,13},std::pair<float,float>{3,12}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{12,5},std::pair<float,float>{13,5},std::pair<float,float>{13,13},std::pair<float,float>{12,13},std::pair<float,float>{12,5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{6,5},std::pair<float,float>{12,5},std::pair<float,float>{12,6},std::pair<float,float>{6,6},std::pair<float,float>{6,5}};
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
    robot r(1.0,3.0);
    std::vector<obstacle> Union_obstacle;
    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    robot_check a; // A temporary variable to check intersections
    a = r.contactSensor(Union_obstacle,0);

    if(a.intersection_check){
        std::cout << "Inside obstacle" << std::endl;
    }

    for(int i = 0; i < a.sampled_points.size(); i++){
        PlotSampledX.push_back(a.sampled_points[i].first);
        PlotSampledY.push_back(a.sampled_points[i].second);
    }

    for(int i = 0; i < plot_x.size(); i++){
        plt::named_plot("Obj " + std::to_string(i),plot_x[i],plot_y[i],"*--");
    }
    
    plt::named_plot("Sensor_samples",PlotSampledX,PlotSampledY,"*g");
    plt::named_plot("Obstacle_detected", std::vector<float>{r.pos.first},std::vector<float>{r.pos.second},"*r");
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
    bool a; // A temporary variable to check intersections
    p_check.first                   = 0.65;
    p_check.second                  = 0.7;

    a                               = obs.CheckIntersectionWObs(p_check);

    if(a){
        std::cout << "Inside the obstacle" << std::endl;
    }


    plt::named_plot("Polygon_points",plot_x,plot_y,"*-");
    plt::named_plot("Obstacle_detected", std::vector<float>{p_check.first},std::vector<float>{p_check.second},"*r");
    plt::grid(true);
    plt::legend();
	plt::show();

    return 0;
}
