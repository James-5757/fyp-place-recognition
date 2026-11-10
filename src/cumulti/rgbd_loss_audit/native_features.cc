// Offline feature diagnostics ONLY. Never constructs System or calls TrackRGBD.
#include <ORBextractor.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <algorithm>
#include <cmath>
double median(std::vector<double> x){if(x.empty())return -1;std::sort(x.begin(),x.end());return x[x.size()/2];}
int main(int argc,char**argv){
 if(argc!=4){std::cerr<<"sequence.txt metrics.csv overlay-directory\n";return 2;}cv::setNumThreads(1);
 std::ifstream input(argv[1]);std::ofstream out(argv[2]);out<<std::setprecision(12);
 out<<"robot,capacity,index,time_s,native_features,valid_depth_features,close_depth_features,stable_depth_patch_features,valid_depth_top_half,valid_depth_bottom_half,previous_index,mutual_matches,fundamental_inliers,pairs_with_depth,median_motion_px,median_depth_delta_m,gray_mean_abs_difference,floor_round_depth_presence_disagreements\n";
 std::string robot,rgbpath,dpath,cfgpath,oldrobot;int capacity,index,overlay,oldcapacity=-1,prevIndex=-100;double time;
 std::unique_ptr<ORB_SLAM3::ORBextractor> extractor;cv::Mat K,D,prevDesc,prevGray;std::vector<cv::KeyPoint> prevKeys;std::vector<float> prevDepth;float close=2;
 while(input>>robot>>capacity>>index>>time>>rgbpath>>dpath>>cfgpath>>overlay){
  if(robot!=oldrobot||capacity!=oldcapacity){
   cv::FileStorage f(cfgpath,cv::FileStorage::READ);if(!f.isOpened())return 3;
   if((int)f["ORBextractor.nFeatures"]!=capacity)return 4;
   extractor.reset(new ORB_SLAM3::ORBextractor(capacity,(float)f["ORBextractor.scaleFactor"],(int)f["ORBextractor.nLevels"],(int)f["ORBextractor.iniThFAST"],(int)f["ORBextractor.minThFAST"]));
   K=(cv::Mat_<double>(3,3)<<(double)f["Camera1.fx"],0,(double)f["Camera1.cx"],0,(double)f["Camera1.fy"],(double)f["Camera1.cy"],0,0,1);
   D=(cv::Mat_<double>(1,5)<<(double)f["Camera1.k1"],(double)f["Camera1.k2"],(double)f["Camera1.p1"],(double)f["Camera1.p2"],(double)f["Camera1.k3"]);
   close=(float)f["Stereo.b"]*(float)f["Stereo.ThDepth"];oldrobot=robot;oldcapacity=capacity;prevIndex=-100;
  }
  cv::Mat color=cv::imread(rgbpath),depth=cv::imread(dpath,cv::IMREAD_UNCHANGED),gray;cv::cvtColor(color,gray,cv::COLOR_BGR2GRAY);
  if(color.empty()||depth.type()!=CV_16UC1)return 5;
  std::vector<cv::KeyPoint> keys;cv::Mat desc;std::vector<int> overlap={0,0};(*extractor)(gray,cv::noArray(),keys,desc,overlap);
  int valid=0,near=0,stable=0,top=0,bottom=0,disagree=0;std::vector<float> depths;
  for(auto&key:keys){
   int x=(int)key.pt.x,y=(int)key.pt.y;float z=depth.at<unsigned short>(y,x)/1000.f;depths.push_back(z);
   int xr=std::min(depth.cols-1,(int)std::floor(key.pt.x+.5)),yr=std::min(depth.rows-1,(int)std::floor(key.pt.y+.5));
   disagree+=(z>0)!=(depth.at<unsigned short>(yr,xr)>0);
   if(z>0){valid++;near+=z<close;if(y<gray.rows/2)top++;else bottom++;
    if(x>0&&y>0&&x+1<depth.cols&&y+1<depth.rows){double lo,hi;cv::minMaxLoc(depth(cv::Rect(x-1,y-1,3,3)),&lo,&hi);stable+=lo>0&&hi-lo<=100;}
   }
   if(overlay)cv::circle(color,key.pt,2,z>0?cv::Scalar(40,240,70):cv::Scalar(80,80,240),1);
  }
  int matches=-1,inliers=-1,pairs=-1;double motion=-1,depthDelta=-1,grayDiff=-1;
  if(prevIndex==index-1&&!prevDesc.empty()&&!desc.empty()){
   cv::BFMatcher matcher(cv::NORM_HAMMING);std::vector<std::vector<cv::DMatch>> a,b;matcher.knnMatch(prevDesc,desc,a,2);matcher.knnMatch(desc,prevDesc,b,2);
   std::map<int,int> reverse;for(auto&v:b)if(v.size()==2&&v[0].distance<.75*v[1].distance)reverse[v[0].queryIdx]=v[0].trainIdx;
   std::vector<cv::Point2f> p,q;std::vector<double> moves,delta;matches=0;pairs=0;
   for(auto&v:a)if(v.size()==2&&v[0].distance<.75*v[1].distance&&reverse.count(v[0].trainIdx)&&reverse[v[0].trainIdx]==v[0].queryIdx){
    int i=v[0].queryIdx,j=v[0].trainIdx;matches++;p.push_back(prevKeys[i].pt);q.push_back(keys[j].pt);moves.push_back(cv::norm(prevKeys[i].pt-keys[j].pt));
    if(prevDepth[i]>0&&depths[j]>0){pairs++;delta.push_back(std::abs(prevDepth[i]-depths[j]));}
   }
   motion=median(moves);depthDelta=median(delta);grayDiff=cv::norm(gray,prevGray,cv::NORM_L1)/gray.total();
   if(p.size()>=8){std::vector<cv::Point2f> pu,qu;cv::undistortPoints(p,pu,K,D,cv::noArray(),K);cv::undistortPoints(q,qu,K,D,cv::noArray(),K);cv::Mat mask;cv::setRNGSeed(20261110);auto fundamental=cv::findFundamentalMat(pu,qu,cv::FM_RANSAC,1.5,.99,mask);inliers=fundamental.empty()?0:cv::countNonZero(mask);}
  }
  out<<robot<<","<<capacity<<","<<index<<","<<time<<","<<keys.size()<<","<<valid<<","<<near<<","<<stable<<","<<top<<","<<bottom<<","<<prevIndex<<","<<matches<<","<<inliers<<","<<pairs<<","<<motion<<","<<depthDelta<<","<<grayDiff<<","<<disagree<<"\n";
  if(overlay){cv::putText(color,"GREEN measured depth / RED no depth; native ORB "+std::to_string(capacity),{8,18},cv::FONT_HERSHEY_SIMPLEX,.4,{255,255,255},1);cv::imwrite(std::string(argv[3])+"/"+robot+"_"+std::to_string(capacity)+"_"+std::to_string(index)+".jpg",color);}
  prevIndex=index;prevDesc=desc;prevKeys=keys;prevDepth=depths;prevGray=gray;
 }
 return 0;
}
