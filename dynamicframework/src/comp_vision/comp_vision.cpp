#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "comp_vision.hpp"

using namespace std::chrono_literals;

namespace DF
{
    cv::Mat CompVision::image;

    void CompVision::camOn(cv::Mat image)
    {
        cv::imshow("Display window", image);
        int k = cv::waitKey(1);
    }

    void CompVision::camOn(sensor_msgs::msg::Image::SharedPtr imageSensorMsg)
    {
        cv::Mat image = CompVision::convertToMatFromSensorMsgs(imageSensorMsg);
        CompVision::camOn(image);
    }

    cv::Mat CompVision::makeOpening(cv::Mat image)
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat erodes;
        cv::erode(image, erodes, kernel, cv::Point(-1, -1), 1);
        cv::Mat opening;
        cv::dilate(erodes, opening, kernel, cv::Point(-1, -1), 1);
        return opening;
    }

    cv::Mat CompVision::makeClosing(cv::Mat image)
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat dilates;
        cv::dilate(image, dilates, kernel, cv::Point(-1, -1), 1);
        cv::Mat closing;
        cv::erode(dilates, closing, kernel, cv::Point(-1, -1), 1);
        return closing;
    }

    cv::Mat CompVision::findByColorRange(cv::Mat image, cv::Scalar lower, cv::Scalar upper)
    {
        cv::Mat result;
        cv::inRange(opening, lower, upper, result);
        return result;
    }

    cv::Mat CompVision::findCenterOfRange(cv::Mat image, cv::Scalar lower, cv::Scalar upper)
    {
        cv::Mat result;
        cv::inRange(image, lower, upper, result);
    }

    cv::Mat CompVision::convertToMatFromSensorMsgs(sensor_msgs::msg::Image::SharedPtr imageSensorMsg)
    {
        return cv_bridge::toCvCopy(imageSensorMsg, "bgr8")->image;
    }

}