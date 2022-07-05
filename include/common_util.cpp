#include "common_util.h"

namespace comutil
{
    std::string timeToString(std::chrono::system_clock::time_point t)
    {
        std::time_t time = std::chrono::system_clock::to_time_t(t);
        std::string time_str = std::ctime(&time);
        time_str.resize(time_str.size() - 1);
        replace(time_str.begin(), time_str.end(), ':', ' ');
        return time_str;
    }

    void debug_img()
	{
        std::vector<cv::Mat> vImgs{ cv::Mat(1, 1, CV_8UC3) };

        cv::namedWindow("Debug", cv::WINDOW_NORMAL);
        cv::resizeWindow("Debug", 800 * vImgs.size(), 800);
        cv::Mat totalImg;

        cv::hconcat(vImgs, totalImg);

        cv::imshow("Debug", totalImg);
        cv::waitKey(0);
        cv::destroyWindow("Debug");
    }

    void checkFolder(const fs::path& folder)
	{
        if (fs::is_directory(folder))
        {
            fs::remove_all(folder);
        }
        fs::create_directories(folder);
    }

    void safeCheckFolder(const fs::path& folder)
	{
        if (!fs::is_directory(folder)) 
        {
            fs::create_directories(folder);
        }
    }

    std::chrono::steady_clock::time_point recordTime() {
        std::chrono::steady_clock::time_point  now = std::chrono::steady_clock::now();
        return now;
    }

    void checkpointTime(std::chrono::steady_clock::time_point& now, std::string vTip, bool checkpoint) {
        if (!checkpoint)
            return;
        const auto t2 = std::chrono::steady_clock::now();
        const auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - now);
        std::cout << vTip << ": " << time_span.count() << std::endl;
        now = std::chrono::steady_clock::now();
    }

    bool fillTimeInterval(
        const std::chrono::steady_clock::time_point& lastTimePoint, const double& interval
    )
    {
        const auto t2 = std::chrono::steady_clock::now();
        const auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - lastTimePoint);
        if (timeSpan.count() > interval)
        {
            return false;
        }
        const unsigned long restTime = static_cast<unsigned long>((interval - timeSpan.count()) * 1000.);
        Sleep(restTime);
    	return true;
    }

    double getTimeInterval(std::chrono::steady_clock::time_point& now)
    {
        auto t2 = std::chrono::steady_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - now);
        now = std::chrono::steady_clock::now();
    	return time_span.count();
    }

    void debug_img(std::vector<cv::Mat>& vImgs)
	{
        cv::namedWindow("Debug", cv::WINDOW_NORMAL);
        cv::resizeWindow("Debug", 800 * vImgs.size(), 800);
        cv::Mat totalImg;

        cv::hconcat(vImgs, totalImg);

        cv::imshow("Debug", totalImg);
        cv::waitKey(0);
        cv::destroyWindow("Debug");
    }

    void override_sleep(double seconds)
    {
#ifdef _WIN32
        _sleep(static_cast<long>(seconds * 1000));
#else
        sleep(seconds);
#endif
    }

    std::vector<cv::Vec3b> get_color_table_bgr()
    {
        std::vector<cv::Vec3b> color_table;

        color_table.emplace_back(153, 255, 204);
        color_table.emplace_back(255, 204, 153);
        color_table.emplace_back(153, 255, 255);
        color_table.emplace_back(253, 196, 225);
        color_table.emplace_back(0, 182, 246);

        return color_table;
    }


    std::vector<cv::Vec3b> get_color_table_bgr2()
    {
        std::vector<cv::Vec3b> color_table;
        color_table.emplace_back(197, 255, 255);
        color_table.emplace_back(226, 226, 255);
        color_table.emplace_back(255, 226, 197);
        color_table.emplace_back(197, 255, 226);
        color_table.emplace_back(2, 0, 160);
        color_table.emplace_back(0, 12, 79);
        color_table.emplace_back(105, 72, 129);
        color_table.emplace_back(153, 0, 102);
        color_table.emplace_back(153, 150, 102);
        color_table.emplace_back(153, 255, 204);
        color_table.emplace_back(255, 204, 153);
        color_table.emplace_back(153, 255, 255);
        color_table.emplace_back(253, 196, 225);
        color_table.emplace_back(0, 182, 246);

        return color_table;
    }
}

namespace Log
{
	LogSys::LogSys(const string& programName, const string& configPath)
	{
        std::cout << "Read config " << configPath << std::endl;
        try
        {
            std::ifstream in(configPath);
            if (!in.is_open())
            {
                LOG(ERROR) << "Error opening file" << configPath << std::endl;
                throw;
            }
            
            if (!(in >> args))
            {
                LOG(ERROR) << "Error parse config file" << configPath << std::endl;
                throw;
            }
            in.close();
        }
        catch (const std::runtime_error& err)
        {
            LOG(INFO) << err.what();
            exit(0);
        }

        stage = args["stage"];
        logPath = args["logPath"];
        const string prjName = args["projectName"];
        logPath += "//" + prjName;
        comutil::safeCheckFolder(logPath);

        auto logt = comutil::timeToString(std::chrono::system_clock::now());
        logPath += "//" + logt + " STAGE " + std::to_string(stage);

        fs::path lop(logPath);
        logPath += "/";
        create_directory(lop);

        LOG(INFO) << "Stage: " << stage << "\n";
        LOG(INFO) << "Log Path: " << logPath << "\n";

        std::vector<string> subdir = args["subdiretories"];
        for (const auto& dirName : subdir)
        {
            fs::create_directories(logPath + dirName);
        }

	}

}
