/**
 * @file      ascamera_node.cpp
 * @brief     angstrong camera ROS demo program.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/02/15
 * @version   1.0

 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <thread>
#include <sys/time.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>
#include <iostream>

#include "ascamera_node.h"
#include "CameraPublisher.h"
#include "Logger.h"

#include "ros/ros.h"

static CameraPublisher *cameraPublisher;

int main(int argc, char *argv[])
{
    int ret = 0;
    LOG(INFO) << "Hello, angstrong camera node" << std::endl;
    ros::init(argc, argv, "ascamera");

    cameraPublisher = new CameraPublisher();

    ret = cameraPublisher->start();
    if (ret != 0) {
        LOG(ERROR) << "start camera publisher failed" << std::endl;
        return -1;
    }

    std::thread cmdThread([&] {
        fd_set readFdSet;
        struct timeval timeout;
        while (ros::ok())
        {
            FD_ZERO(&readFdSet);
            FD_SET(fileno(stdin), &readFdSet);
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            select(fileno(stdin) + 1, &readFdSet, nullptr, nullptr, &timeout);
            if (FD_ISSET(fileno(stdin), &readFdSet)) {
                char ch = getchar();
                if (ch == 's') {
                    cameraPublisher->saveImage();
                } else if (ch == 'q') {
                    ros::shutdown();
                    break;
                } else if (ch == 'f') {
                    cameraPublisher->logFps(!cameraPublisher->getLogFps());
                } else if (ch == 'l') {
                    cameraPublisher->logCfgParameter();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    ros::spin();
    if (cmdThread.joinable()) {
        cmdThread.join();
    }

    cameraPublisher->stop();
    delete cameraPublisher;

    return 0;
}
