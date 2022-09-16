# Bugalgos
Implementation of Bug1 and Bug2 algorithms in real space

To install Matplotlib.cpp run the following lines

```
sudo apt-get install python-matplotlib python-numpy python3.8-dev 

cd <this_repo>/src
git clone https://github.com/lava/matplotlib-cpp
cd matplotlib-cpp
mkdir build
cd build
cmake .. && make -j4
```

If this runs without any errors that means matplotlibcpp is build. 
Now in this repository I've already placed matplotlibcpp.h in the ```src/Bug1/src```
Next run the following lines to make an a.out file and run the programs
```
cd <this_repo>/src/Bug1/src
#to build Bug1
g++ bug1.cpp -std=c++11 -I/usr/include/python3.8 -lpython3.8
#to run Bug1
./a.out
```
Similarly for Bug2 replace bug1.cpp with bug2.cpp to run the code.
