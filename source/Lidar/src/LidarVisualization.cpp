#include <opencv2/opencv.hpp>
#include <atomic>
#include <iostream>
#include <csignal>
#include <cmath>
#include <sl_lidar.h>
#include <sl_lidar_driver.h>

using namespace sl;

std::atomic<bool> IsQuit;
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        IsQuit.store(true);
    }
}

int main()
{
    IsQuit.store(false);
    std::signal(SIGINT, signalHandler);

    Result<IChannel*> _channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
    ILidarDriver* drv = *createLidarDriver();

    if (!drv || !(_channel) || SL_IS_FAIL(drv->connect(*_channel)))
    {
        printf("[Err] Lidar Connection Fail\n");
        return -1;
    }

    drv->setMotorSpeed();
    drv->startScan(0, 1);

    cv::Mat img = cv::Mat::zeros(800, 800, CV_8UC3);
    sl_lidar_response_measurement_node_hq_t lidarNodes[8192];

    while (!IsQuit.load())
    {
        size_t lidarPointCnt = 8192;
        sl_result readVal = drv->grabScanDataHq(lidarNodes, lidarPointCnt);

        if (SL_IS_OK(readVal))
        {
            img = cv::Scalar(0, 0, 0);
            float scale = 0.1f;
            int center = 400;

            for (size_t i = 0; i < lidarPointCnt; ++i)
            {
                float dist = lidarNodes[i].dist_mm_q2 / 4.0f;
                if (dist > 0)
                {
                    float angle = lidarNodes[i].angle_z_q14 * 90.f / 16384.f;
                    float radian = angle * M_PI / 180.0f;

                    int x = center + static_cast<int>(std::sin(radian) * dist * scale);
                    int y = center - static_cast<int>(std::cos(radian) * dist * scale);

                    if (x >= 0 && x < 800 && y >= 0 && y < 800)
                    {
                        cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
                    }
                }
            }
            cv::imshow("Lidar Visualization", img);
        }

        if (cv::waitKey(1) == 27 || IsQuit.load())
        {
            break;
        }
    }

    drv->stop();
    drv->disconnect();
    delete drv;
    cv::destroyAllWindows();

    return 0;
}