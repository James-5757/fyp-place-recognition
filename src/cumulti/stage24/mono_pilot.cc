// Original Stage24 headless adapter to the separately installed GPLv3 ORB-SLAM3.
// Linkage provenance/license are recorded; no ORB-SLAM3 source is modified.
#include <System.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <iostream>
int main(int argc,char**argv){
 if(argc!=6){std::cerr<<"vocabulary settings sequence.txt poses.csv keyframes.tum\n";return 2;}
 ORB_SLAM3::System slam(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);
 std::ifstream in(argv[3]);std::ofstream out(argv[4]);
 out<<"timestamp_s,tracking_state,tracked_points,track_ms,valid_pose,tx,ty,tz,qx,qy,qz,qw\n"<<std::setprecision(16);
 double t;std::string path;
 while(in>>t>>path){
  cv::Mat image=cv::imread(path,cv::IMREAD_COLOR);if(image.empty())return 3;
  auto begin=std::chrono::steady_clock::now();auto tcw=slam.TrackMonocular(image,t);
  auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  int state=slam.GetTrackingState(),points=0;for(auto*p:slam.GetTrackedMapPoints())if(p)points++;
  bool valid=state==2&&tcw.matrix().allFinite();out<<t<<","<<state<<","<<points<<","<<ms<<","<<valid;
  if(valid){auto twc=tcw.inverse();auto x=twc.translation();auto q=twc.unit_quaternion();
   out<<","<<x.x()<<","<<x.y()<<","<<x.z()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w();
  }else out<<",,,,,,,";
  out<<"\n";out.flush();
 }
 slam.Shutdown();slam.SaveKeyFrameTrajectoryTUM(argv[5]);return 0;
}
