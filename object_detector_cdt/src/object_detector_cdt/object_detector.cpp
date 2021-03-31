#include <object_detector_cdt/object_detector.h>


ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe(input_image_topic_, 1, &ObjectDetector::imageCallback, this);

    // Setup publisher
    objects_pub_ = nh.advertise<cdt_msgs::ObjectList>(output_objects_topic_, 10);

    // Extrinsic calibration. This must be updated accordingly
    camera_extrinsic_x_ = 0.2;
    camera_extrinsic_y_ = 0.2;
    camera_extrinsic_z_ = 0.0;

    // Intrinsic calibration
    camera_fx_ = 381.3;
    camera_fy_ = 381.3;
    camera_cx_ = 320.5;
    camera_cy_ = 240.5;

    // Real heights of objects
    barrel_real_height_     = 1.2;   // meters 
    barrow_real_height_     = 0.7;   // meters, note: includes the wheel and frame 
    computer_real_height_   = 0.5;   // meters 
    dog_real_height_        = 0.418; // meters, note: includes legs 
}

void ObjectDetector::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_image_topic", input_image_topic_))
    {
        ROS_ERROR("Could not read parameter `input_topic`.");
        exit(-1);
    }

    if (!nh.getParam("input_base_frame", base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_fixed_frame", fixed_frame_))
    {
        ROS_ERROR("Could not read parameter `goal_frame`.");
        exit(-1);
    }   

    // output topic is optional. It will use '/detected_objects' by default
    nh.param("output_objects_topic", output_objects_topic_, std::string("/detected_objects"));
}

void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr &in_msg)
{
    ROS_DEBUG("New image received!");

    // Preallocate some variables
    cv::Mat image;
    ros::Time timestamp;

    double x, y, theta;
    getRobotPose(x, y, theta);

    // Convert message to OpenCV image
    convertMessageToImage(in_msg, image, timestamp);

    // Recognize object
    if(!wasObjectDetected("dog"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeDog(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    if(!wasObjectDetected("barrow"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeBarrow(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    if(!wasObjectDetected("barrel"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeBarrel(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    if(!wasObjectDetected("computer"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeComputer(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    // Publish list of objects detected so far
    objects_pub_.publish(detected_objects_);
}

void ObjectDetector::convertMessageToImage(const sensor_msgs::ImageConstPtr &in_msg, cv::Mat &out_image, ros::Time &out_timestamp)
{
    // Convert Image message to cv::Mat using cv_bridge
    out_image = cv_bridge::toCvShare(in_msg, "bgr8")->image;

    // Extract timestamp from header
    out_timestamp = in_msg->header.stamp;
}

cv::Mat ObjectDetector::applyColourFilter(const cv::Mat &in_image_bgr, const Colour &colour)
{
    assert(in_image_bgr.type() == CV_8UC3);

    cv::Mat in_image_hsv;
    cv::cvtColor(in_image_bgr, in_image_hsv, CV_BGR2HSV);

    // Here you should apply some binary threhsolds on the image to detect the colors
    // The output should be a binary mask indicating where the object of a given color is located
    cv::Mat mask;
    if (colour == Colour::RED) {
        inRange(in_image_hsv, cv::Scalar(  0,  5,  0), cv::Scalar( 1, 255, 255), mask);
    } else if (colour == Colour::YELLOW) {
        inRange(in_image_hsv, cv::Scalar(  30,  75,  25), cv::Scalar( 35, 255, 255), mask);
    } else if (colour == Colour::GREEN) {
        inRange(in_image_hsv, cv::Scalar(  35,  100,  25), cv::Scalar( 85, 255, 255), mask);
    } else if (colour == Colour::BLUE) {
        inRange(in_image_hsv, cv::Scalar(  110,  25,  25), cv::Scalar( 130, 255, 255), mask);
    } else {
        // Report color not implemented
        ROS_ERROR_STREAM("[ObjectDetector::colourFilter] colour (" << colour << "  not implemented!");
    }

    cv::Mat eroded_mask, dilated_mask;
    cv::erode(mask, eroded_mask, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));
    cv::dilate(eroded_mask, dilated_mask, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));

    cv::imwrite("/home/cdt2021/input_img_hsv.png", in_image_hsv);
    cv::imwrite("/home/cdt2021/mask.png", dilated_mask);

    // We return the mask, that will be used later
    return dilated_mask;
}

cv::Mat ObjectDetector::applyBoundingBox(const cv::Mat1b &in_mask, double &x, double &y, double &width, double &height) {

    cv::Mat drawing; // it could be useful to fill this image if you want to debug

    // You need to return the center of the object in image coordinates, as well as a bounding box indicating its height and width (in pixels)
    int top_left_x, top_left_y, bottom_right_x, bottom_right_y;
    top_left_x = in_mask.rows;
    top_left_y = in_mask.cols;
    bottom_right_x = 0;
    bottom_right_y = 0;

    for (int i = 0; i < in_mask.rows; ++i)
    {
        for (int j = 0; j < in_mask.cols; ++j)
        {
            int value = in_mask.at<uchar>(i, j);
            if (value == 255)
            {
                if (j < top_left_x) {
                    top_left_x = j;
                }
                if (j > bottom_right_x) {
                    bottom_right_x = j;
                }

                if (i < top_left_y) {
                    top_left_y = i;
                }
                if (i > bottom_right_y) {
                    bottom_right_y = i;
                }
            }
        }
    }

    x = (top_left_x + bottom_right_x) / 2;
    y = (top_left_y + bottom_right_y) / 2;
    width = bottom_right_x - top_left_x;
    height = bottom_right_y - top_left_y;

    // debugging bbox
    cv::Point top_left(top_left_x, top_left_y);
    cv::Point bottom_right(bottom_right_x, bottom_right_y); 
    cv::rectangle(in_mask, top_left, bottom_right, cv::Scalar(255, 255, 255));
    cv::imwrite("/home/cdt2021/bbox.png", in_mask);

    return drawing;
}

void ObjectDetector::computePoseFromBBox(double &x, double &y, double &z, double &depth,
                                         const double &obj_image_center_x, const double &obj_image_center_y,
                                         const double &obj_image_height, const double &obj_image_width,
                                         const double& robot_x, const double& robot_y,
                                         const double& robot_theta, std::string object_name) {
    double real_object_height;
    if(object_name == "dog") {
        real_object_height = dog_real_height_;
    } else if(object_name == "barrow") {
        real_object_height = barrow_real_height_;
    } else if(object_name == "barrel") {
        real_object_height = barrel_real_height_;
    } else if(object_name == "computer") {
        real_object_height = computer_real_height_;
    } else {
        ROS_ERROR_STREAM("[ObjectDetector::computePoseFromBBox] object (" << object_name << "  not implemented!");
    }

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    depth = real_object_height / obj_image_height * camera_fy_;

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double obj_position_camera_x = depth / camera_fx_ * (obj_image_center_x - camera_cx_);
    double obj_position_camera_y = depth / camera_fy_ * (obj_image_center_y - camera_cy_);
    double obj_position_camera_z = depth;

    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z 
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though
    
    double obj_position_base_x = (camera_extrinsic_x_ +  obj_position_camera_z);
    double obj_position_base_y = (camera_extrinsic_y_ + -obj_position_camera_x);

    x = robot_x +  cos(robot_theta)*obj_position_base_x + sin(-robot_theta) * obj_position_base_y;
    y = robot_y +  sin(robot_theta)*obj_position_base_x + cos(robot_theta) * obj_position_base_y;
    z = 0.0     + camera_extrinsic_z_ + -obj_position_camera_y;
}

bool ObjectDetector::recognizeObject(const Colour colour, const std::string object_name, const cv::Mat &in_image,
                                     const ros::Time &in_timestamp, const double& robot_x, const double& robot_y,
                                     const double& robot_theta, cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double obj_image_center_x;
    double obj_image_center_y;
    double obj_image_height;
    double obj_image_width;
    double x, y, z, depth;

    cv::Mat in_image_red = applyColourFilter(in_image, colour);
    cv::Mat in_image_bounding_box = applyBoundingBox(in_image_red, obj_image_center_x, obj_image_center_y, obj_image_width, obj_image_height);

    // Note: Almost everything below should be kept as it is
    if (obj_image_width < MIN_BBOX_WIDTH || obj_image_height < MIN_BBOX_HEIGHT) {
        return false;
    }
    
    computePoseFromBBox(x, y, z, depth, obj_image_center_x, obj_image_center_y, obj_image_height, obj_image_width, robot_x, robot_y, robot_theta, object_name);

    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = object_name;
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = x;
    out_new_object.position.y = y;
    out_new_object.position.z = z;

    if(object_name == "barrel") {
        return depth < MIN_BARREL_DEPTH;
    }
    return depth < MIN_OBJECT_DEPTH;
}

bool ObjectDetector::recognizeDog(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    return recognizeObject(Colour::RED, "dog", in_image, in_timestamp, robot_x, robot_y, robot_theta, out_new_object);
}

bool ObjectDetector::recognizeBarrow(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    return recognizeObject(Colour::GREEN, "barrow", in_image, in_timestamp, robot_x, robot_y, robot_theta, out_new_object);
}

bool ObjectDetector::recognizeBarrel(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    return recognizeObject(Colour::YELLOW, "barrel", in_image, in_timestamp, robot_x, robot_y, robot_theta, out_new_object);
}

bool ObjectDetector::recognizeComputer(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    return recognizeObject(Colour::BLUE, "computer", in_image, in_timestamp, robot_x, robot_y, robot_theta, out_new_object);
}

// Utils
void ObjectDetector::getRobotPose(double &x, double &y, double &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(fixed_frame_, base_frame_,  ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(fixed_frame_, base_frame_, ros::Time(0), base_to_map_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Extract components from robot pose
    x = base_to_map_transform.getOrigin().getX();
    y = base_to_map_transform.getOrigin().getY();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // First we create an Eigen quaternion
    Eigen::Quaterniond q(base_to_map_transform.getRotation().getW(),
                         base_to_map_transform.getRotation().getX(),
                         base_to_map_transform.getRotation().getY(),
                         base_to_map_transform.getRotation().getZ());
    // We convert it to an Axis-Angle representation
    // This representation is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    theta = axis_angle.axis().z() * axis_angle.angle();
}

bool ObjectDetector::wasObjectDetected(std::string object_name)
{
    bool detected = false;
    for(auto obj : detected_objects_.objects)
    {
        if(obj.id == object_name)
            detected = true;
    }

    return detected;
}