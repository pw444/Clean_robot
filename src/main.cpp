#include <clean_robot_facilities.h>
#include "utils/point.h"
#include "map/array2d.h"
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

    typedef SLAM::array2D<double> DoubleArray;
    DoubleArray d_array(3,3);
    cout << d_array.getXSize() << "  " << d_array.getYSize() << endl;
    cout << d_array.isInside(5,5) << "  " << d_array.isInside(2,2) << endl;
    cout << d_array.cell(2,2) << endl;
    d_array.resize(6,6);
    cout << d_array.isInside(5,5) << "  " << d_array.isInside(2,2) << endl;
    cout << d_array.cellState(SLAM::IntPoint(1,1)) << endl;
    return ( 0 );
}
