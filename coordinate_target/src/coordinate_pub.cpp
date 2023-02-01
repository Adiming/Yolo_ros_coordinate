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

class CoordinateClass
{
 public:
  CoordinateClass()
  {
    pub_ = nh_.advertise<coordinate_target::coordinate>("coordinate/coordinate_target",1);
    bb_.subscribe(nh_, "darknet_ros/bounding_boxes", 1);
    depth_.subscribe(nh_, "camera/aligned_depth_to_color/image_raw", 1);

    // redefine the synchronizer
    sync_.reset(new Sync(MySyncPolicy(10), bb_, depth_));
    sync_->registerCallback(boost::bind(&CoordinateClass::callback, this, _1, _2));
  }

  void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bb_holder, const sensor_msgs::ImageConstPtr &depth_holder)
  {
    // ROS_INFO("Synchronization successful");
    // declare a published variable
    coordinate_target::coordinate c;

    // covert the depth form into cv form
    cv::Mat depth_image = cv::Mat::zeros(cv::Size(res_w_,res_h_),CV_16UC1);  // create an empty space

    depth_image = cv_bridge::toCvCopy(depth_holder, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    // find target and acquire its coordinate
    for(auto item = bb_holder->bounding_boxes.begin();item != bb_holder->bounding_boxes.end();++item)
    {
        // check whether it is a target class and is it has enough prosibility
        if(item->Class == goal_ && item->probability>=0.7)
        {
            ROS_INFO("The target class is %s",item->Class.c_str());
            // ROS_INFO("The target class probability is %.2f",item->probability);
            // get pixel coordinate of image
            int x = (item->xmax - item->xmin)/2 + item->xmin;
            int y = (item->ymax - item->ymin)/2 + item->ymin;
            
            // get the depth value from target pixel
            float depth = depth_image.at<uint16_t>(y,x);

            if(depth>0)
            {
                c.x = std::ceil(depth*(res_w_/2 - x)/f_x_*100.0)/100.0 + 33.5;
                c.y = std::ceil(depth*(res_h_/2 - y)/f_y_*100.0)/100.0;
                c.z = std::ceil(sqrt(depth*depth - c.x*c.x - c.y*c.y)*100.0)/100.0;

                ROS_INFO("The target coordinate is x: %.2f,y: %.2f,z: %.2f",c.x,c.y,c.z);
                pub_.publish(c);
            }
        }
    }
  }

 private:
    ros::NodeHandle nh_;
    // subscribe message
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_;
    message_filters::Subscriber<sensor_msgs::Image> depth_;

    // parameter for coordinate calculation 
    int res_w_ = 640;       // resolution of the camera, w
    int res_h_ = 480;       // resolution of the camera, h
    float f_x_ = 618.58;    // focal length of the camera under this resolution, x
    float f_y_ = 618.95;    // focal length of the camera under this resolution, y
    std::string goal_ = "platform"; // detected goal

    ros::Publisher pub_ ;

    // define the synchronize policy -> ApproximateTime
    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // set a share pointer with data type Sync
    boost::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate_pub");

  CoordinateClass coordinate;

  ros::spin();
  
  return 0;
}