#include "matplotlibcpp.h"
#include <iostream>
namespace plt = matplotlibcpp;



void subplots(std::vector<int> a, std::vector<int> b){
	plt::suptitle("first_subplot");
	plt::subplot(1,2,1);
	plt::plot(a,b,"r-");
	plt::subplot(1,2,2);
	plt::plot(b,a,"*");
	plt::text(100,90,"Hello");
	plt::show();
}


int main(){
	int n	=	1000;
	std::vector<int> x(n),y(n);
	for(int i = 0; i < n; i++){
		x.at(i) = i;
		y.at(i) = 1;
	}

	plt::named_plot("first_plot",x,y,"-o");
	plt::named_plot("second_plot",y,x,"-r");
	plt::title("Sample figure");
	plt::legend();
	plt::show();
	subplots(x,y);
	std::cout << "Imported matplotlib successfully" << std::endl;

}
