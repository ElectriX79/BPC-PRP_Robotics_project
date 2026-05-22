#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

    class ArucoDetector {
    public:
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;
        };

        ArucoDetector() {
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        }

        ~ArucoDetector() = default;

        std::vector<Aruco> detect(cv::Mat frame) {
            std::vector<Aruco> arucos;
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;

            cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

            if (!marker_ids.empty()) {
                std::cout << "Arucos found: ";
                for (size_t i = 0; i < marker_ids.size(); i++) {
                    std::cout << marker_ids[i] << " ";
                    arucos.emplace_back(Aruco{marker_ids[i], marker_corners[i]});
                }
                std::cout << std::endl;
            }

            return arucos;
        }

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };
}