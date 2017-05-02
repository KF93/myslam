/*************************************************************************
	> File Name: main.cpp
	> Author:李小二 
	> Mail: 
	> Created Time: 2017年04月30日 星期日 14时12分50秒
 ************************************************************************/

#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/feature2d/feature2d.hpp>
#include<boost/concept_check.hpp>
#include<g2o/core/sparse_optimizer.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/robust_kernel.h>
#include<g2o/core/robust_kernel_impl.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/cholmod/linear_solver_cholmod.h>
#include<g2o/types/slam3d/se3quat.h>
#include<g2o/types/sba/types_six_dof_expmap.h>
using namespace std;
int findCorrespondingPoints(const cv::Mat& img1,const cv::Mat& img2,vector<cv::Point2f>& points1,vector<cv::Point2f>& points2);
double cx=325.5;
double cy=253.5;
double fx=518.0;
double fy=519.0;
 
int main(int argc, char** argv)
{
   if(grac!=3) 
    {
        cout<<"Usage: ba_example img1,img2"<<endl;
        exit(1);
    }
    cv::Mat img1=cv::imread(argv[1]);
    cv::Mat img2=cv::imread(argv[2]);
    vector<cv::Point2f>pts1,pts2;
    if(findCorrespondingPoints(img1,img2,pts1,pts2)==false)
    {
        cout<<"匹配点不够"<<endl;
        return 0;
    }
    cout<<"找到了"<<pts1.size()<<"组对应特征点"<<endl;
    g2o::SparseOptimizer optimizer;


}

