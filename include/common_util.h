#ifndef COMMON_UTIL
#define COMMON_UTIL

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <corecrt_math_defines.h>
#include <iostream>
#include <string>
#include <chrono>
#include <ctime>
#include <Windows.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <json/reader.h>

namespace fs = boost::filesystem;

namespace std {

    template <typename Scalar, int Rows, int Cols>
    struct hash<Eigen::Matrix<Scalar, Rows, Cols>>
	{
        // https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
        size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const
    	{
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i)
            {
                Scalar elem = *(matrix.data() + i);
                seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    inline bool operator < (const cv::Vec3b& s1, const cv::Vec3b& s2)
    {
        return s1[0] < s2[0] || (s1[0] == s2[0] && s1[1] < s2[1]) || (s1[0] == s2[0] && s1[1] == s2[1] && s1[2] < s2[2]);
    }

}  // namespace std

namespace comutil
{
    std::string timeToString(std::chrono::system_clock::time_point t);

    // If folder exists, delete it and make a empty new folder.
    void checkFolder(const fs::path& folder);

    // If folder doesn't exist, make a new folder.
	void safeCheckFolder(const fs::path& folder);

    // Return a point in time.
    std::chrono::steady_clock::time_point recordTime();

    // Attention: This function will change the time "now".
    // If checkpoint is true, print the time interval between this checkpoint and the last.
    // vTip is used for outputting some msg.
    void checkpointTime(std::chrono::steady_clock::time_point& now, std::string vTip = "", bool checkpoint = true);

    /*
     * Delay a period of time to fill the interval.
     */
    bool fillTimeInterval(
        const std::chrono::steady_clock::time_point& lastTimePoint, const double& interval
    );

    // Attention: This function will change the time "now".
	// Return the time interval between "now" parameter and the time this function is executed.
	double getTimeInterval(std::chrono::steady_clock::time_point& now);

    void debug_img();
	void debug_img(std::vector<cv::Mat>& vImgs);

    // Sleep x second.
    void override_sleep(double seconds);
    std::vector<cv::Vec3b> get_color_table_bgr();
    std::vector<cv::Vec3b> get_color_table_bgr2();


}
#endif
