#include <clean_robot_facilities.h>
#include "utils/point.h"
#include "sensor/lidar_sensor/lidarsensor.h"


int main ( int argc, char** argv ) {
    //your code goes here
    SLAM::point<int> p(1,2);
    cout << p.x << "  " << p.y << endl;
    SLAM::point<int> p1(2,3);
    SLAM::point<int> p2 = p + p1;
    cout << p2.x << "  " << p2.y << endl;
    p2 = p1 - p;
    cout << p2.x << "  " << p2.y << endl;
    p2 = p * 2;
    cout << p2.x << "  " << p2.y << endl;
    p2 = 3 * p1;
    cout << p2.x << "  " << p2.y << endl;
    cout <<  "C++ with KDevelop is fun!" << endl;

    return ( 0 );
}
