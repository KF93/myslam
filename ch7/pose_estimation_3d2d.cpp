/*************************************************************************
	> File Name: pose_estimation_3d2d.cpp
	> Author:李小二 
	> Mail: 
	> Created Time: 2017年05月05日 星期五 16时55分10秒
 ************************************************************************/

#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/csparse/linear_solver_csparse.h>
#include<g2o/types/sba/types_six_dof_expmap.h>
#include<chrono>
#include <boost/concept_check.hpp>
using namespace std;
using namespace cv;
void find_feature_matches(
  const Mat& img_1,const Mat& img_2,
  std::vector<KeyPoint>& keypoints_1,
  std::vector<KeyPoint>& keypoints_2,
  std::vector<DMatch>& matches
);
Point2d pixel2cam(const Point2d& p,const Mat& K);
void bundleAdjustment(
  const vector<Point2f> points_3d,
  const vector<Point2f> points_2d,
  const Mat& K,
  Mat& R,Mat& t
);
int main(int argc,char** argv)
{
  if(argc!=5)
  {
    cout<<"usage: pose_estimation_3d2d img1 img2 depth1 depth2"<<endl;
    return 1;
  }
 Mat img_1=imread(argv[1],CV_LOAD_IMAGE_COLOR);
 Mat img_2=imread(argv[2],CV_LOAD_IMAGE_COLOR);
 vector<KeyPoint> keypoints_1,keypoints_2;
 vector<DMatch> matches;
 find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
 cout<<"一共找到了"<<matches.size()<<"组匹配点"<<endl;
 Mat d1=imread(argv[3],CV_LOAD_IMAGE_UNCHANGED);
 Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
 vector<Point3f> pts_3d;
 vector<Point2f> pts_2d;
 for(DMatch m:matches)
 {
   ushort d=d1.ptr<unsigned short> (int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
   if(d==0)
     continue;
   float dd=d/1000.0;
   Point2d p1=pixel2cam(keypoints_1[m.queryIdx].pt,K);
   pts_3d.push_back(Point3f(p1.x*dd,p1.y*dd,dd));
   pts_2d.push_back(keypoints_2[m.trainIdx].pt);
 }
 cout<<"3d-2d pairs: "<<pts_3d.size()<<endl;
 Mat r,t;
 solvePnP(pts_3d,pts_2d,K,Mat(),r,t,false);
 Mat R;
 cv::Rodrigues(r,R);
 cout<<"R="<<endl<<R<<endl;
 cout<<"t="<<endl<<t<<endl;
 cout<<"calling bundle adjustment"<<endl;
 bundleAdjustment(pts_3d,pts_2d,K,R,t);
}
void find_feature_matches(
  const Mat& img_1,const Mat& img_2,
  std::vector<KeyPoint>& keypoints_1,
  std::vector<KeyPoint>& keypoints_2,
  std::vector<DMatch>& matches
                         )
{
  Mat descriptor_1,descriptor_2;
  Ptr<FeatureDetector> detector=ORB::create();
  Ptr<DescriptorExtractor> descriptor=ORB::create();
  Ptr<DescriptorMatcher>matcher=DescriptorMatcher::create("BruteForce-Hamming");
  detector->detect(img_1,keypoints_1);
  detector->detect(img_2,keypoints_2);
  descriptor->compute(img_1,keypoints_1,descriptor_1);
  descriptor->compute(img_2,keypoints_2,descriptor_2);
  vector<DMatch> match;
  matcher->match(descriptor_1,descriptor_2,match);
  double min_dist=100000,max_dist=0;
  for(int i=0;i<descriptor_1.rows;i++)
  {
    double dist=match[i].distance;
    if(dist<min_dist) min_dist=dist;
    if(dist>max-dist) max_dist=dist;
  }
  printf("--Max dist: %f \n",max_dist);
  printf("--Min dist: %f \n",min_dist);
  for(int i=0;i<descriptor_1.rows;i++)
  {
    if(match[i].distance<=max(2*min_dist,30.0))
    {
      match.push_back(match[i]);
    }
  }
}
Point2d pixel2cam(const Point2d& p,const Mat& K )
{
  return Point2d
  (
    (p.x-K.at<double>(0,2))/K.at<double>(0,0),
    (p.y-K.at<double>(1,2))/K.at<double>(1,1)
  );
}
void bundleAdjustment(const vector< Point2f > points_3d, const vector< Point2f > points_2d, const Mat& K, Mat& R, Mat& t)
{
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3> > Block;
  Block::LinearSolverType* linearSolver=new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
  Block* solver_ptr=new Block(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver=new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  g2o::VertexSE3Expmap* pose=new g2o::VertexSE3Expmap();
  Eigen::Matrix3d R_mat;
}

