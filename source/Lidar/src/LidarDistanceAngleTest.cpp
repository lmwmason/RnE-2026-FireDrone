/*#include <atomic>
#include <iostream>
#include <csignal>
#include <sl_lidar.h>
#include <sl_lidar_driver.h>

using namespace sl;

// Quit with ctrl+c
std::atomic<bool> IsQuit(false);
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        IsQuit.store(true);
    }
}


int main()
{
    std::signal(SIGINT, signalHandler);

    Result<IChannel*> _channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
    ILidarDriver* drv = *createLidarDriver();

    if (!drv || !(_channel) || SL_IS_FAIL(drv->connect(*_channel)))
    {
        printf("[Err] Lidar Connection Fail");
        return -1;
    }

    drv->setMotorSpeed();
    drv->startScan(0,1);

    sl_lidar_response_measurement_node_hq_t lidarNodes[8192];

    while (!IsQuit.load())
    {
        size_t lidarPointCnt = 8192;
        sl_result readVal = drv->grabScanDataHq(lidarNodes, lidarPointCnt);

        if (SL_IS_OK(readVal))
        {
            drv->ascendScanData(lidarNodes, lidarPointCnt);
            if (lidarPointCnt>0)
            {
                float lidarReadAngle = lidarNodes[0].angle_z_q14 * 90.f / 16384.f;
                float lidarReadDistance = lidarNodes[0].dist_mm_q2 /4.0f;

                printf("[Result] Lidar Read Angle: %.2f\tLidar Read Distance: %.2f\n", lidarReadAngle, lidarReadDistance);
            }
        }
    }

    drv->stop();
    drv->disconnect();

    delete drv;

    return 0;
}*/