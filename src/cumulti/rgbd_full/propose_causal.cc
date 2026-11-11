// Offline RGB-only BoW proposals. No System, poses, LiDAR or ground truth is read.
#include <ORBextractor.h>
#include <ORBVocabulary.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
struct Frame {
 int index;
 long long time;
 DBoW2::BowVector bow;
};
int main(int argc,char**argv){
 if(argc!=4)return 2;
 cv::setNumThreads(1);
 ORB_SLAM3::ORBVocabulary vocabulary;
 if(!vocabulary.loadFromTextFile(argv[1]))return 3;
 ORB_SLAM3::ORBextractor extractor(1000,1.2,8,20,7);
 std::ifstream input(argv[2]);
 std::ofstream out(argv[3]);
 if(!input || !out)return 5;
 std::vector<Frame> arrived[2];
 out<<"query_robot,query_index,candidate_index,rank,bow_score,arrival_ns,candidate_ns\n";
 std::string robot,path;
 int index,query;
 long long time;
 while(input>>robot>>index>>time>>query>>path){
  int side=robot=="robot1"?0:1;
  auto image=cv::imread(path,cv::IMREAD_GRAYSCALE);
  if(image.empty())return 4;
  std::vector<cv::KeyPoint> keys;
  cv::Mat descriptors;
  std::vector<int> overlap={0,0};
  extractor(image,cv::noArray(),keys,descriptors,overlap);
  std::vector<cv::Mat> rows;
  for(int i=0;i<descriptors.rows;i++)rows.push_back(descriptors.row(i));
  Frame current{index,time,{}};
  vocabulary.transform(rows,current.bow);
  if(query){
   auto& other=arrived[1-side];
   std::vector<std::pair<double,size_t>> scores;
   for(size_t i=0;i<other.size();i++)scores.emplace_back(vocabulary.score(current.bow,other[i].bow),i);
   std::sort(scores.begin(),scores.end(),[](auto&a,auto&b){return a.first!=b.first?a.first>b.first:a.second<b.second;});
   for(size_t k=0;k<std::min(size_t(3),scores.size());k++){
    const auto& candidate=other[scores[k].second];
    out<<robot<<","<<index<<","<<candidate.index<<","<<k+1<<","<<scores[k].first<<","<<time<<","<<candidate.time<<"\n";
   }
  }
  arrived[side].push_back(std::move(current));
 }
}
