// Isolated native-rate RGB-D observer. No ORB-SLAM3 library source is changed.
#include <System.h>
#include <MapPoint.h>
#include <Map.h>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <filesystem>
int main(int argc,char**argv){
 if(argc!=6){std::cerr<<"vocabulary settings sequence rows.csv corrected_ns.tum\n";return 2;}
 ORB_SLAM3::System slam(argv[1],argv[2],ORB_SLAM3::System::RGBD,false);
 std::ifstream in(argv[3]);std::ofstream out(argv[4]);
 out<<"index,relative_ns,tracking_state,tracked_points,map_id,map_init_kf,map_vote_fraction,track_ms,valid_pose,tx,ty,tz,qx,qy,qz,qw\n"<<std::setprecision(16);
 std::int64_t ns;std::string rgbpath,depthpath;int index=0;
 std::set<std::pair<unsigned long,unsigned long>> origins;ORB_SLAM3::Map*lastMap=nullptr;
 auto wall=std::chrono::steady_clock::now();bool first=true;
 while(in>>ns>>rgbpath>>depthpath){
  if(first){wall=std::chrono::steady_clock::now();first=false;}
  std::this_thread::sleep_until(wall+std::chrono::nanoseconds(ns));
  auto waiting=std::chrono::steady_clock::now();
  while(!std::filesystem::exists(depthpath)){
   if(std::chrono::steady_clock::now()-waiting>std::chrono::minutes(10))return 4;
   std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  auto rgb=cv::imread(rgbpath,cv::IMREAD_COLOR),depth=cv::imread(depthpath,cv::IMREAD_UNCHANGED);
  if(rgb.empty()||depth.type()!=CV_16UC1||rgb.size()!=depth.size())return 3;
  auto start=std::chrono::steady_clock::now();auto tcw=slam.TrackRGBD(rgb,depth,double(ns)/1e9);
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  int state=slam.GetTrackingState(),points=0,best=0;std::map<ORB_SLAM3::Map*,int> votes;
  for(auto*p:slam.GetTrackedMapPoints())if(p&&!p->isBad()&&p->GetMap()){votes[p->GetMap()]++;points++;}
  ORB_SLAM3::Map*map=nullptr;for(auto&v:votes)if(v.second>best){best=v.second;map=v.first;}
  long id=map?map->GetId():-1,init=map?map->GetInitKFid():-1;
  double fraction=points?double(best)/points:0;
  bool valid=state==2&&tcw.matrix().allFinite()&&map&&fraction>=.8;
  out<<index<<","<<ns<<","<<state<<","<<points<<","<<id<<","<<init<<","<<fraction<<","<<ms<<","<<valid;
  if(valid){origins.emplace(id,init);lastMap=map;auto twc=tcw.inverse();auto t=twc.translation();auto q=twc.unit_quaternion();
   out<<","<<t.x()<<","<<t.y()<<","<<t.z()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w();}
  else out<<",,,,,,,";
  out<<"\n";out.flush();index++;
  if(index%100==0)std::cout<<"PROGRESS frames="<<index<<" sensor_s="<<double(ns)/1e9<<std::endl;
 }
 slam.Shutdown();
 // Only a single witnessed native coordinate frame can supply final-BA poses.
 // The Python audit further excludes every online-untracked frame from this file.
 if(false)slam.SaveTrajectoryEuRoC(argv[5],lastMap);
 else{std::ofstream empty(argv[5]);}
 std::cout<<"FINISHED frames="<<index<<" coordinate_frames="<<origins.size()<<std::endl;
 return 0;
}
