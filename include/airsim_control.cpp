#pragma once

#include "airsim_control.h"
#include <glog/logging.h>

std::map<std::string, cv::Mat> Airsim_tools::get_images()
{
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    vector<ImageRequest> request = {
        ImageRequest("0", ImageType::Scene, false, false),
        ImageRequest("0", ImageType::DepthPlanar, true,false),
        ImageRequest("0", ImageType::Segmentation, false,false),
        //ImageRequest("0", ImageType::DepthPerspective, true,false),
    };
    const vector<ImageResponse>& response = m_agent->simGetImages(request);

    std::map<std::string, cv::Mat> images;
    if (response.size() > 0) {
        for (const ImageResponse& image_info : response) {
            if (image_info.image_type == ImageType::Scene) {
                cv::Mat rgb = cv::Mat(image_info.height, image_info.width, CV_8UC3,
                    (unsigned*)image_info.image_data_uint8.data()).clone();
                if (rgb.dims == 0)
                    return std::map<std::string, cv::Mat>();
                cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
                images.insert(std::pair<std::string, cv::Mat>("rgb", rgb));
            }
            else if (image_info.image_type == ImageType::DepthPlanar) {
                cv::Mat depth = cv::Mat(image_info.height, image_info.width, CV_32FC1,
                    (float*)image_info.image_data_float.data()).clone();
                if (depth.dims == 0)
                    return std::map<std::string, cv::Mat>();
                images.insert(std::pair<std::string, cv::Mat>("depth_planar", depth));

            }
            else if (image_info.image_type == ImageType::Segmentation) {
                cv::Mat seg = cv::Mat(image_info.height, image_info.width, CV_8UC3,
                    (unsigned*)image_info.image_data_uint8.data()).clone();
                if (seg.dims == 0)
                    return std::map<std::string, cv::Mat>();
                cv::cvtColor(seg, seg, cv::COLOR_BGR2RGB);
                images.insert(std::pair<std::string, cv::Mat>("segmentation", seg));
            }
            else if (image_info.image_type == ImageType::DepthPerspective) {
                cv::Mat depth = cv::Mat(image_info.height, image_info.width, CV_32FC1,
                    (float*)image_info.image_data_float.data()).clone();
                if (depth.dims == 0)
                    return std::map<std::string, cv::Mat>();
                images.insert(std::pair<std::string, cv::Mat>("depth_perspective", depth));

            }
        }
    }
    return images;
}

void Airsim_tools::adjust_pose(const Pos_Pack& v_pos_pack){
    Eigen::Quaternionf directionQuaternion;
    directionQuaternion =
        Eigen::AngleAxisf(-v_pos_pack.yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(-v_pos_pack.pitch, Eigen::Vector3f::UnitY());

    m_agent->simSetVehiclePose(
        msr::airlib::Pose(v_pos_pack.pos_airsim.cast<float>(), directionQuaternion), true
    );
   
}

std::map<cv::Vec3b, std::string> Airsim_tools::reset_color(std::function<bool(std::string)> v_func)
{
    std::map<cv::Vec3b, std::string> color_map;
    int num = 1;
    auto a = m_agent->simListSceneObjects();
    for (int i = 0; i < a.size(); i++)
    {
        bool result;
        if (v_func(a[i]))
        {
            result = m_agent->simSetSegmentationObjectID(a[i], num);
            cv::Vec3b color = m_color_map.at<cv::Vec3b>(num * 4);
            color_map.insert(std::make_pair(color, a[i]));
            if (!result)
                LOG(ERROR) << "Set " << a[i] << " color " << num << " failed";
            num += 1;
        }
        else
        {
            result = m_agent->simSetSegmentationObjectID(a[i], 0);
            if (!result)
                LOG(ERROR) << "Set " << a[i] << " background color" << " failed";
            //std::cout << a[i] << " set 0" << std::endl;
            if (a[i] == "stupid_floor")
            {
                std::cout << "here" << std::endl;
            }
        }
    }
    return color_map;
}

std::map<cv::Vec3b, std::string> Airsim_tools::reset_color(const std::string& v_key_words) {
    std::map<cv::Vec3b, std::string> color_map;
    int num = 1;
    auto a = m_agent->simListSceneObjects();
    for(int i = 0; i < a.size(); i++)
    {
        std::string temp = a[i];
        //std::cout << a[i].length() << a[i].size() << std::endl;
        //if (a[i].length() <= 4)
        bool result;
        if ((v_key_words.size() > 0 && a[i].find(v_key_words) != a[i].npos)|| v_key_words.size()==0)
        {
            result=m_agent->simSetSegmentationObjectID(a[i], num);
            color_map.insert(std::make_pair(m_color_map.at<cv::Vec3b>(num * 4), a[i]));
            if (!result)
                LOG(ERROR) << "Set " << a[i] << " color " << num << " failed";
            num += 1;
        }
        else
        {
            result = m_agent->simSetSegmentationObjectID(a[i], 0);
            if (!result)
                LOG(ERROR) << "Set " << a[i] << " background color" << " failed";
            //std::cout << a[i] << " set 0" << std::endl;
            if (a[i] == "stupid_floor")
            {
                std::cout << "here" << std::endl;
            }
        }
    }
    return color_map;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> demo_move_to_next(
    msr::airlib::MultirotorRpcLibClient& v_agent, const Eigen::Vector3d& v_next_pos_airsim,
    float angle, const float v_speed, bool is_forward)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> poses;
    Pose pose = v_agent.simGetVehiclePose();
    Eigen::Vector3f pos_cur = pose.position;
    const Eigen::Vector3f nextPos(v_next_pos_airsim.x(), v_next_pos_airsim.y(), v_next_pos_airsim.z());

    while (true)
    {
        pose = v_agent.simGetVehiclePose();
        pos_cur = pose.position;
    	
        Eigen::Vector3f direction = nextPos - pos_cur;
        direction.normalize();
        direction = direction * v_speed;

    	if(is_forward)
			v_agent.moveByVelocityAsync(direction[0], direction[1], direction[2], 20,
				DrivetrainType::ForwardOnly, YawMode(false, 0));
        else
            v_agent.moveByVelocityAsync(direction[0], direction[1], direction[2], 20);
            
        pos_cur = pose.position;
        if ((pos_cur - nextPos).norm() < 2)
        {
            v_agent.rotateToYawAsync(angle);
            break;
        }
        poses.emplace_back(std::make_pair(pos_cur.cast<double>(), Eigen::Vector3d(0, 0, -1)));
        comutil::override_sleep(0.05);
    }

    return poses;
}
