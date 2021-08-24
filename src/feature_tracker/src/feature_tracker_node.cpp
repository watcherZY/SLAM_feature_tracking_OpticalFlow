/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Original Author: Qin Tong (qintonguav@gmail.com)
 * Remodified Author: Hu(rhuag@connect.ust.hk) at HKUST, https://blog.csdn.net/iwanderu
 *******************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "../include/feature_tracker.h"

#define SHOW_UNDISTORTION 0

bool ASSISTED_MATCH = 1;//辅助匹配标志位


vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

queue<sensor_msgs::ImuConstPtr> imu_buf;//用于存放IMU数据
////std::condition_variable con;//线程相关
//std::mutex m_buf;//用于queue操作的线程锁


ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData;//关于特征匹配的对象，包含图片，特征点，操作函数

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;


void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // first image process
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }

    // detect unstable camera stream
    /* 当时间比上一次大一秒以上，或小与上一次，则有问题 */
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }

    // renew time stamp
    last_image_time = img_msg->header.stamp.toSec();

    // frequency control FREQ默认20HZ
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    // change the format of the image
    cv_bridge::CvImageConstPtr ptr;//把img_msg的数据导给ptr
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;//header 里面有时间戳，id，点的位置
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    // read image and track features
    cv::Mat show_img = ptr->image;
    TicToc t_r;
    ROS_DEBUG("processing camera %d", 0);

    /* 特征跟踪的任务在此完成 */
    /////////////////////////////////////////////////
    trackerData.readImage(ptr->image, img_msg->header.stamp.toSec());
    /////////////////////////////////////////////////
        

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif


    // renew global feature ids
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= trackerData.updateID(i);
        if (!completed)
            break;
    }

    // publish features 在频率控制范围的数据进行发送
   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;//点的编号
        sensor_msgs::ChannelFloat32 u_of_point;//特征点的像素坐标
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;//特征点在像素上的速度
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(1);//初始化长度为1
        auto &un_pts = trackerData.cur_un_pts;
        auto &cur_pts = trackerData.cur_pts;
        auto &ids = trackerData.ids;
        auto &pts_velocity = trackerData.pts_velocity;
        for (unsigned int j = 0; j < ids.size(); j++)
        {
            if (trackerData.track_cnt[j] > 1)
            {
                int p_id = ids[j];
                hash_ids[0].insert(p_id);
                geometry_msgs::Point32 p;
                p.x = un_pts[j].x;
                p.y = un_pts[j].y;
                p.z = 1;

                feature_points->points.push_back(p);
                id_of_point.values.push_back(p_id  );
                u_of_point.values.push_back(cur_pts[j].x);
                v_of_point.values.push_back(cur_pts[j].y);
                velocity_x_of_point.values.push_back(pts_velocity[j].x);
                velocity_y_of_point.values.push_back(pts_velocity[j].y);
            }
        }
        
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);//发布特征数据，包含编号，像素点，移动速度

        // show track
        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            cv::Mat mono_img = ptr->image;
            cv::Mat tmp_img = mono_img;
            cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

            for (unsigned int j = 0; j < trackerData.cur_pts.size(); j++)
            {
                double len = std::min(1.0, 1.0 * trackerData.track_cnt[j] / WINDOW_SIZE);
                cv::circle(tmp_img, trackerData.cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                //draw speed line
                Vector2d tmp_cur_un_pts (trackerData.cur_un_pts[j].x, trackerData.cur_un_pts[j].y);
                Vector2d tmp_pts_velocity (trackerData.pts_velocity[j].x, trackerData.pts_velocity[j].y);
                Vector3d tmp_prev_un_pts;
                tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                tmp_prev_un_pts.z() = 1;
                Vector2d tmp_prev_uv;
                trackerData.m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                cv::line(tmp_img, trackerData.cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                //char name[10];
                //sprintf(name, "%d", trackerData[i].ids[j]);
                //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
            //cv::imshow("vis", mono_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());//发布图片数据，用来绘图
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
//IMU的回调函数
/* void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();//唤醒一个等待的线程

    //predict(imu_msg);
} */
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    ROS_INFO("loaded readParameters");
    trackerData.readIntrinsicParameter(CAM_NAMES[0]);
    ROS_INFO("loaded IntrinsicParameter");
        
    if(FISHEYE)
    {     
            trackerData.fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData.fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
    }

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    /* 在feature输入IMU数据以完成辅助特征匹配
    ros::TransportHints().tcpNoDelay()的功能是用来数据的同步获取 */
    //ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());


    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?