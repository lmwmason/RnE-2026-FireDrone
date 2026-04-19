#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

using namespace msr::airlib;

int main() {
    MultirotorRpcLibClient client;
    client.confirmConnection();

    int img_size = 800;
    float scale = 20.0f;
    cv::Mat display = cv::Mat::zeros(img_size, img_size, CV_8UC3);

    while (true) {
        auto lidarData = client.getLidarData("MyLidar");
        const auto& points = lidarData.point_cloud;

        display = cv::Scalar(0, 0, 0);
        cv::circle(display, cv::Point(img_size / 2, img_size / 2), 5, cv::Scalar(0, 0, 255), -1);

        if (points.size() >= 3) {
            for (size_t i = 0; i < points.size(); i += 3) {
                float x = points[i];
                float y = points[i + 1];

                int img_x = (img_size / 2) + static_cast<int>(y * scale);
                int img_y = (img_size / 2) - static_cast<int>(x * scale);

                if (img_x >= 0 && img_x < img_size && img_y >= 0 && img_y < img_size) {
                    cv::circle(display, cv::Point(img_x, img_y), 2, cv::Scalar(0, 255, 0), -1);
                }
            }
        }

        cv::imshow("LiDAR 2D View", display);
        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}