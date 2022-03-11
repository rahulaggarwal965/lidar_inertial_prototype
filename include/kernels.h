#ifndef KERNELS_H
#define KERNELS_H

#include <opencv2/opencv.hpp>

// https://github.com/PRBonn/depth_clustering
static cv::Mat savitsky_golay_kernel(int window_size) {

    // below are no magic constants. See Savitsky-golay filter.
    cv::Mat kernel;
    switch (window_size) {
        case 5:
            kernel = cv::Mat::zeros(window_size, 1, CV_32F);
            kernel.at<float>(0, 0) = -3.0f;
            kernel.at<float>(0, 1) = 12.0f;
            kernel.at<float>(0, 2) = 17.0f;
            kernel.at<float>(0, 3) = 12.0f;
            kernel.at<float>(0, 4) = -3.0f;
            kernel /= 35.0f;
            return kernel;
        case 7:
            kernel = cv::Mat::zeros(window_size, 1, CV_32F);
            kernel.at<float>(0, 0) = -2.0f;
            kernel.at<float>(0, 1) = 3.0f;
            kernel.at<float>(0, 2) = 6.0f;
            kernel.at<float>(0, 3) = 7.0f;
            kernel.at<float>(0, 4) = 6.0f;
            kernel.at<float>(0, 5) = 3.0f;
            kernel.at<float>(0, 6) = -2.0f;
            kernel /= 21.0f;
            return kernel;
        case 9:
            kernel = cv::Mat::zeros(window_size, 1, CV_32F);
            kernel.at<float>(0, 0) = -21.0f;
            kernel.at<float>(0, 1) = 14.0f;
            kernel.at<float>(0, 2) = 39.0f;
            kernel.at<float>(0, 3) = 54.0f;
            kernel.at<float>(0, 4) = 59.0f;
            kernel.at<float>(0, 5) = 54.0f;
            kernel.at<float>(0, 6) = 39.0f;
            kernel.at<float>(0, 7) = 14.0f;
            kernel.at<float>(0, 8) = -21.0f;
            kernel /= 231.0f;
            return kernel;
        case 11:
            kernel = cv::Mat::zeros(window_size, 1, CV_32F);
            kernel.at<float>(0, 0) = -36.0f;
            kernel.at<float>(0, 1) = 9.0f;
            kernel.at<float>(0, 2) = 44.0f;
            kernel.at<float>(0, 3) = 69.0f;
            kernel.at<float>(0, 4) = 84.0f;
            kernel.at<float>(0, 5) = 89.0f;
            kernel.at<float>(0, 6) = 84.0f;
            kernel.at<float>(0, 7) = 69.0f;
            kernel.at<float>(0, 8) = 44.0f;
            kernel.at<float>(0, 9) = 9.0f;
            kernel.at<float>(0, 10) = -36.0f;
            kernel /= 429.0f;
            return kernel;
    }
    return kernel;
}

#endif
