// Offline RGB-only BoW proposals. No System, poses, LiDAR or ground truth is read.
#include <ORBextractor.h>
#include <ORBVocabulary.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
struct Frame{std::string robot;int index;DBoW2::BowVector bow;};
int main(int argc,char**argv){
 if(argc!=4)return 2;cv::setNumThreads(1);ORB_SLAM3::ORBVocabulary vocabulary;
 if(!vocabulary.loadFromTextFile(argv[1]))return 3;
 ORB_SLAM3::ORBextractor extractor(1000,1.2,8,20,7);
 std::ifstream input(argv[2]);std::ofstream out(argv[3]);std::vector<Frame> query,database;
 std::string robot,path;int index;
 while(input>>robot>>index>>path){
  auto image=cv::imread(path,cv::IMREAD_GRAYSCALE);if(image.empty())return 4;
  std::vector<cv::KeyPoint> keys;cv::Mat descriptors;std::vector<int> overlap={0,0};extractor(image,cv::noArray(),keys,descriptors,overlap);
  std::vector<cv::Mat> rows;for(int i=0;i<descriptors.rows;i++)rows.push_back(descriptors.row(i));Frame f{robot,index,{}};vocabulary.transform(rows,f.bow);
  (robot=="robot1"?query:database).push_back(f);
 }
 out<<"query_index,candidate_index,rank,bow_score\n";
 for(auto&q:query){std::vector<std::pair<double,int>> scores;for(auto&d:database)scores.emplace_back(vocabulary.score(q.bow,d.bow),d.index);
  std::sort(scores.begin(),scores.end(),[](auto&a,auto&b){return a.first!=b.first?a.first>b.first:a.second<b.second;});
  for(int k=0;k<std::min(3,int(scores.size()));k++)out<<q.index<<","<<scores[k].second<<","<<k+1<<","<<scores[k].first<<"\n";
 }
 return 0;
}
