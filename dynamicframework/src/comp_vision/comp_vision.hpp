#ifndef COMP_VISION_HPP
#define COMP_VISION_HPP

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

namespace DF
{

    class CompVision
    {
        public:
            /*Вывод изображения + перегрузка для необработанного топика камеры*/
            static void camOn(cv::Mat image);
            static void camOn(sensor_msgs::msg::Image::SharedPtr imageSensorMsg);
            /*Преобразование в матрицу для вывода необработанного топика камеры*/
            static cv::Mat convertToMatFromSensorMsgs(sensor_msgs::msg::Image::SharedPtr imageConvert);
            /*Выделяем по диапозону цвета*/
            static cv::Mat findByColorRange(cv::Mat image, cv::Scalar lower, cv::Scalar upper);
            /*Находим центр объекта (белой зоны)*/
            static cv::Mat findCenterOfWniteZone(cv::Mat image);
            /*Рисуме на изображении в заданном диапозоне*/
            static cv::Mat drawRange(cv::Mat image);
            /*Создание размыкания изображения*/
            static cv::Mat CompVision::makeOpening(cv::Mat image)
            /*Создание замыкания изображения*/
            static cv::Mat CompVision::makeClosing(cv::Mat image)

            /*Методы доступа к статическому приватному полю image (получение объекта)*/
            static cv::Mat getImage()
            {
                return image;
            }
            
            /*Методы доступа к статическому приватному полю image (задание объекта)*/
            static void setImage(cv::Mat value)
            {
                image = value;
            }

        private:
            static cv::Mat image;
    };
}

#endif