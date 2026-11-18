// Stage32 independent feature/depth probe. Original SDK and tracker are untouched.
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    std::ifstream inputs(argv[1]);
    std::ofstream out(argv[2]);
    if (!inputs || !out) return 3;
    out << "index,extracted_keypoints,depth_supported_keypoints,missing_depth_keypoints,"
           "features_near_depth_discontinuity";
    for (const auto &prefix : {"orb_region_", "depth_orb_region_"})
        for (int region = 0; region < 16; ++region) out << "," << prefix << region;
    out << ",depth_keypoint_min_m,depth_keypoint_median_m,depth_keypoint_p95_m,depth_keypoint_max_m,close_depth_keypoints\n";
    ORB_SLAM3::ORBextractor extractor(1000, 1.2f, 8, 20, 7);
    int index;
    std::string rgbPath, depthPath;
    while (inputs >> index >> rgbPath >> depthPath) {
        cv::Mat bgr = cv::imread(rgbPath, cv::IMREAD_COLOR);
        cv::Mat depth = cv::imread(depthPath, cv::IMREAD_UNCHANGED);
        if (bgr.size() != cv::Size(640, 400) || depth.type() != CV_16UC1 ||
            depth.size() != bgr.size()) return 4;
        cv::Mat gray, meters, descriptors;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        depth.convertTo(meters, CV_32F, 1.0f / 1000.0f);
        std::vector<cv::KeyPoint> keys;
        std::vector<int> overlap{0, 0};
        extractor(gray, cv::Mat(), keys, descriptors, overlap);
        int supported = 0, discontinuity = 0, close = 0;
        std::vector<float> depths;
        std::vector<int> regions(16), supportedRegions(16);
        for (const auto &key : keys) {
            // Matches native Frame::ComputeStereoFromRGBD truncation.
            const int u = static_cast<int>(key.pt.x), v = static_cast<int>(key.pt.y);
            if (u < 0 || u >= 640 || v < 0 || v >= 400) return 5;
            const int region = std::min(3, v / 100) * 4 + std::min(3, u / 160);
            ++regions[region];
            const float z = meters.at<float>(v, u);
            if (z <= 0) continue;
            ++supported;
            depths.push_back(z);
            if (z < 2.f) ++close;
            ++supportedRegions[region];
            float low = z, high = z;
            bool missing = false;
            for (int y = std::max(0, v - 1); y <= std::min(399, v + 1); ++y)
                for (int x = std::max(0, u - 1); x <= std::min(639, u + 1); ++x) {
                    const float neighbor = meters.at<float>(y, x);
                    if (neighbor <= 0) missing = true;
                    else { low = std::min(low, neighbor); high = std::max(high, neighbor); }
                }
            if (missing || high - low > .3f) ++discontinuity;
        }
        out << index << "," << keys.size() << "," << supported << ","
            << keys.size() - supported << "," << discontinuity;
        for (const auto &counts : {regions, supportedRegions})
            for (int value : counts) out << "," << value;
        std::sort(depths.begin(), depths.end());
        if (!depths.empty())
            out << "," << depths.front() << "," << depths[(depths.size() - 1) / 2]
                << "," << depths[static_cast<size_t>((depths.size() - 1) * .95)] << "," << depths.back();
        else out << ",,,,";
        out << "," << close << "\n";
    }
    return out ? 0 : 6;
}
