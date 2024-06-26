#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v


    //官方文档 http://eigen.tuxfamily.org/dox

    //typedef Matrix<float, 3, 1> Vector3f;
    //typedef Matrix<int, 1, 2> RowVector2i;

    //Homework 0:给定一个点 P=(2,1), 将该点绕原点先逆时针旋转45◦，再平移(1,2), 计算出变换后点的坐标（要求用齐次坐标进行计算）。
    Eigen::Vector3d p(2, 1, 1);

    double ang = M_PI / 4;

    Eigen::Matrix<double, 3, 3> rot45;
    rot45 << cos(ang), -sin(ang), 0, 
             sin(ang), cos(ang), 0,
             0, 0, 1;

    double tx = 1, ty = 2;
    Eigen::Matrix<double, 3, 3> trans;
    trans << 1, 0, tx,
             0, 1, ty,
             0, 0, 1;

    std::cout << p << "\n";
    p = rot45 * p;
    std::cout << p << "\n";
    p = trans * p;
    std::cout << p << "\n";



    return 0;
}