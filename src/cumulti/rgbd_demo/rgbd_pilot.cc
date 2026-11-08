// Camera-only headless RGB-D adapter; the separately installed ORB-SLAM3 is unchanged.
#include <System.h>
#include <MapPoint.h>
#include <Map.h>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
int main(int argc, char** argv) {
    if(argc != 7) { std::cerr << "vocabulary settings sequence poses final.tum paced|unpaced\n"; return 2; }
    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::RGBD, false);
    std::ifstream in(argv[3]); std::ofstream out(argv[4]);
    out << "timestamp_s,tracking_state,tracked_points,map_id,tracking_epoch,track_ms,valid_pose,tx,ty,tz,qx,qy,qz,qw\n" << std::setprecision(16);
    double stamp,firstStamp=0; std::string rgbpath, depthpath; bool anyValid=false,wasValid=false,first=true;
    int epoch=0;auto wallStart=std::chrono::steady_clock::now();
    while(in >> stamp >> rgbpath >> depthpath) {
        if(first){firstStamp=stamp;wallStart=std::chrono::steady_clock::now();first=false;}
        if(std::string(argv[6])=="paced") std::this_thread::sleep_until(wallStart+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(stamp-firstStamp)));
        auto rgb=cv::imread(rgbpath,cv::IMREAD_COLOR);
        auto depth=cv::imread(depthpath,cv::IMREAD_UNCHANGED);
        if(rgb.empty() || depth.empty() || depth.type()!=CV_16UC1 || rgb.size()!=depth.size()) return 3;
        auto start=std::chrono::steady_clock::now();
        auto tcw=slam.TrackRGBD(rgb,depth,stamp);
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        int state=slam.GetTrackingState(), count=0; long map=-1;
        for(auto* point:slam.GetTrackedMapPoints()) if(point) { count++; if(point->GetMap()) map=point->GetMap()->GetId(); }
        bool valid=state==2 && tcw.matrix().allFinite() && map>=0;
        // Conservative independent epochs after every tracking gap. Map IDs alone
        // are insufficient: ORB-SLAM3 can reset and reuse the active map's ID.
        if(valid&&!wasValid&&anyValid)epoch++;
        anyValid=anyValid||valid;wasValid=valid;
        out<<stamp<<","<<state<<","<<count<<","<<map<<","<<epoch<<","<<ms<<","<<valid;
        if(valid) {
            auto twc=tcw.inverse(); auto t=twc.translation(); auto q=twc.unit_quaternion();
            out<<","<<t.x()<<","<<t.y()<<","<<t.z()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w();
        } else out<<",,,,,,,";
        out<<"\n";out.flush();
    }
    slam.Shutdown();
    if(anyValid) slam.SaveTrajectoryTUM(argv[5]);
    else { std::ofstream empty(argv[5]); }
    return 0;
}
