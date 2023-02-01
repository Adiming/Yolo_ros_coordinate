// Subscibe the data stream from BoundingBox and depth
// BoudingBox topic: /darknet_ros/bounding_boxes ; depth topic: /camera/depth/image_rect_raw
// ......
// Calculate the coordinate of target 
// Publish topic of coordinate: coordinate_target

#include <ros/ros.h>
#include <math.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include <coordinate_target/coordinate.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv4/opencv2/opencv_modules.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


static ros::Publisher pub;
// set the detectiong target (generate and publish its coordinate)
std::string goal = "platform";
// std::string goal = "scissors";
// std::string goal = "person";
// image size for the calculation of coordinate
int res_w = 640;
int res_h = 480;
// focal length under 640x480 resolution for the calculation of coordinate
float f_x = 618.58;
float f_y = 618.95;

void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb_holder,
                const sensor_msgs::Image::ConstPtr& depth_holder)
{
    // declare a published variable
    coordinate_target::coordinate c;

    // covert the depth form into cv form
    cv::Mat depth_image = cv::Mat::zeros(cv::Size(res_w,res_h),CV_16UC1);  // create an empty space

    depth_image = cv_bridge::toCvCopy(depth_holder, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    // find target and acquire its coordinate
    for(auto item = bb_holder->bounding_boxes.begin();item != bb_holder->bounding_boxes.end();++item)
    {
        // check whether it is a target class and is it has enough prosibility
        if(item->Class == goal && item->probability>=0.7)
        {
            ROS_INFO("The target class is %s",item->Class.c_str());
            ROS_INFO("The target class probability is %.2f",item->probability);
            // get pixel coordinate of image
            int x = (item->xmax - item->xmin)/2 + item->xmin;
            int y = (item->ymax - item->ymin)/2 + item->ymin;
            
            // get the depth value from target pixel
            float depth = depth_image.at<uint16_t>(y,x);

            // ROS_INFO("the number is %d", std::isdigit(std::trunc(x)));
            // ROS_INFO("the number is %f", std::trunc(x));

            if(depth>0)
            {
                c.x = std::ceil(depth*(res_w/2 - x)/f_x*100.0)/100.0 + 33.5;
                c.y = std::ceil(depth*(res_h/2 - y)/f_y*100.0)/100.0;
                c.z = std::ceil(sqrt(depth*depth - c.x*c.x - c.y*c.y)*100.0)/100.0;

                ROS_INFO("The target coordinate is x: %.2f,y: %.2f,z: %.2f",c.x,c.y,c.z);
                pub.publish(c);
            }
        }
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_target");

    ros::NodeHandle nh;

    pub = nh.advertise<coordinate_target::coordinate>("coordinate/coordinate_target",1);

    ROS_INFO("Generating the data, start to publish coordinate");

    //subsciber for bounding box and depth data
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb(nh, "darknet_ros/bounding_boxes",1);
    message_filters::Subscriber<sensor_msgs::Image> depth(nh,"camera/aligned_depth_to_color/image_raw",1);

    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), bb, depth);
    sync.registerCallback(boost::bind(&callback,_1,_2));


    ros::spin();
    return 0;
}
