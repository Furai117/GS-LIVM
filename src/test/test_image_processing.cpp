#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    // Load test images
    cv::Mat img1_color = cv::imread("src/test/data/frame1.pgm", cv::IMREAD_COLOR);
    cv::Mat img2_color = cv::imread("src/test/data/frame2.pgm", cv::IMREAD_COLOR);
    if (img1_color.empty() || img2_color.empty()) {
        std::cerr << "Failed to load test images" << std::endl;
        return -1;
    }
    cv::Mat img1_gray, img2_gray;
    cv::cvtColor(img1_color, img1_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2_color, img2_gray, cv::COLOR_BGR2GRAY);

    // --- Verify undistortion map generation ---
    cv::Mat K = (cv::Mat_<double>(3,3) << 5.0, 0.0, img1_color.cols/2.0,
                                          0.0, 5.0, img1_color.rows/2.0,
                                          0.0, 0.0, 1.0);
    cv::Mat dist = (cv::Mat_<double>(1,5) << 0.1, -0.05, 0.0, 0.0, 0.0);

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(K, dist, cv::Mat(), K,
                                img1_color.size(), CV_32FC1, map1, map2);

    if (map1.empty() || map2.empty()) {
        std::cerr << "Undistortion map generation failed" << std::endl;
        return -1;
    }

    cv::Point2f mapped_corner(map1.at<float>(0, 0), map2.at<float>(0, 0));
    if (cv::norm(mapped_corner - cv::Point2f(0.0f, 0.0f)) < 1e-5) {
        std::cerr << "Undistortion map appears to be identity" << std::endl;
        return -1;
    }

    // --- Optical flow tracking ---
    std::vector<cv::Point2f> points0;
    cv::goodFeaturesToTrack(img1_gray, points0, 50, 0.01, 1.0);
    if (points0.size() < 10) {
        std::cerr << "Not enough features detected: " << points0.size() << std::endl;
        return -1;
    }

    std::vector<cv::Point2f> points1;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(img1_gray, img2_gray, points0, points1, status, err);

    int tracked = 0;
    for (size_t i = 0; i < points0.size(); ++i) {
        if (!status[i]) continue;
        cv::Point2f expected = points0[i] + cv::Point2f(1.0f, 0.0f);
        if (cv::norm(points1[i] - expected) > 0.5f) {
            std::cerr << "Feature " << i << " moved to " << points1[i]
                      << " expected " << expected << std::endl;
            return -1;
        }
        ++tracked;
    }

    if (tracked < 10) {
        std::cerr << "Tracked features fewer than expected: " << tracked << std::endl;
        return -1;
    }

    std::cout << "Image processing tests passed" << std::endl;
    return 0;
}

