
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <resource_retriever/retriever.h>
#include <dynamic_reconfigure/server.h>
#include <fakecam/FakeCamConfig.h>
#include <string.h>
#include <stdio.h>

using namespace boost;

class FakeCamParameters {
private:
  dynamic_reconfigure::Server<fakecam::FakeCamConfig> server_;

public:
  std::string base_frame_id;
  double fov;
  double separation;
  int convergence_method;
  double convergence_angle;
  double convergence_distance;
  std::string convergence_frame_id;

  void reconfigure_cb(fakecam::FakeCamConfig &config, uint32_t level) {
    base_frame_id = config.base_frame_id;
    fov = config.fov;
    separation = config.separation;
    convergence_method = config.convergence_method;
    convergence_angle = config.convergence_angle;
    convergence_frame_id = config.convergence_frame_id;

    char buffer [50];
	sprintf(buffer, "%f", config.fov);
	ROS_ERROR(buffer);
  }

  FakeCamParameters() {
    server_.setCallback(boost::bind(&FakeCamParameters::reconfigure_cb, this, _1, _2));
  }
};

sensor_msgs::Image get_image_msg(char* name) {
	// The image never changes
	sensor_msgs::Image msg;
	msg.width = 640;
	msg.height = 480;
	msg.encoding = "rgb8";
	msg.is_bigendian = false;
	msg.step = msg.width * 3;
	msg.data.resize(msg.height * msg.step);
	for (unsigned int i = 0; i < msg.data.size(); i++) {
		msg.data[i] = 0;
	}

	// Publish images
	msg.header.stamp = ros::Time::now();
	if (strcmp(name, "right")==0)
		msg.header.frame_id = "/fake_cam_right_optical_link";
	else if (strcmp(name, "left")==0)
		msg.header.frame_id = "/fake_cam_left_optical_link";

	return msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fakecam_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Construct dynamic reconfigure server / structure
  FakeCamParameters parameters;

  // We could probably do something with the camera name, check
  // errors or something, but at the moment, we don't care.
  std::string calibration_file_uri_left, calibration_file_uri_right;

  std::string camera_name;
  sensor_msgs::CameraInfo camera_info_left, camera_info_right;


  nh_private.getParam("calibration_file_uri_left",calibration_file_uri_left);
  nh_private.getParam("calibration_file_uri_right",calibration_file_uri_right);
  nh_private.getParam("camera_name",camera_name);

  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource resource_left, resource_right;

  try {
    resource_left = retriever.get(calibration_file_uri_left);
  } catch (resource_retriever::Exception& e) {
    ROS_FATAL_STREAM("Left : Failed to retrieve camera calibration file from \""<<calibration_file_uri_left<<"\" error: "<<e.what());
    return -1;
  }

  if (camera_calibration_parsers::parseCalibrationIni(
        std::string((const char*)resource_left.data.get(),resource_left.size),
        camera_name,
        camera_info_left))
  {
    ROS_INFO_STREAM("Successfully parsed camera calibration from \""<<calibration_file_uri_left<<"\"");
  } else {
    ROS_FATAL_STREAM("Could not parse camera calibration from \""<<calibration_file_uri_left<<"\"");
    return -1;
  }

  try {
      resource_right = retriever.get(calibration_file_uri_right);
    } catch (resource_retriever::Exception& e) {
      ROS_FATAL_STREAM("Right : Failed to retrieve camera calibration file from \""<<calibration_file_uri_right<<"\" error: "<<e.what());
      return -1;
    }

    if (camera_calibration_parsers::parseCalibrationIni(
          std::string((const char*)resource_right.data.get(),resource_right.size),
          camera_name,
          camera_info_right))
    {
      ROS_INFO_STREAM("Successfully parsed camera calibration from \""<<calibration_file_uri_right<<"\"");
    } else {
      ROS_FATAL_STREAM("Could not parse camera calibration from \""<<calibration_file_uri_right<<"\"");
      return -1;
    }

  // create image transport and publishers
  image_transport::ImageTransport it(nh_private);
  image_transport::CameraPublisher pub_left = it.advertiseCamera("fake_image_left", 1);
  image_transport::CameraPublisher pub_right = it.advertiseCamera("fake_image_right", 1);


  // Create TF broadcaster
  tf::TransformBroadcaster broadcaster;
  tf::Transform left_frame, right_frame;


  ros::Rate rate(10.0);

  ROS_INFO_STREAM("Publishing fake cameras...");

  tf::TransformListener listener;
  tf::StampedTransform convergence_frame;
  double convergence_angle = 0.0;
  double B =  -(camera_info_right.P[3]/camera_info_right.P[0]);// Baseline, Shahab was here
  while(ros::ok()) {
//    switch(parameters.convergence_method) {
//      case fakecam::FakeCam_Frame:
//        try{
//          listener.lookupTransform(
//              parameters.base_frame_id, parameters.convergence_frame_id,
//              ros::Time(0), convergence_frame);
//        } catch (tf::TransformException ex) {
//          ROS_ERROR("Could not lookup convergence point: %s",ex.what());
//        }
//
//        break;
//      case fakecam::FakeCam_Distance:
//        // Use manual convergence distance
//        convergence_angle = atan2(parameters.convergence_distance, parameters.separation/2.0);
//
//        break;
//      case fakecam::FakeCam_Angle:
//        // Use manual convergence angle
//        convergence_angle = parameters.convergence_angle;
//        break;
//      default:
//        ROS_ERROR("No convergence method set!");
//
//    }
	tf::Quaternion convergence_quat_right(0,0,0), convergence_quat_left(0,0,0);
//	convergence_quat_left.setValue(-9.93511576e-04,  -2.28032537e-02,   2.31335907e-03, 9.99736715e-01);
//	convergence_quat_right.setValue(9.93375918e-04,  -1.47113648e-02,   6.00476115e-03, 9.99873242e-01);
	boost::array<double, 9> RL = camera_info_left.R;
	boost::array<double, 9> RR = camera_info_right.R;

	tf::Matrix3x3 Rect_left(RL[0], RL[1], RL[2], RL[3], RL[4], RL[5], RL[6], RL[7], RL[8]), Rect_right(RR[0], RR[1], RR[2], RR[3], RR[4], RR[5], RR[6], RR[7], RR[8]);
	double R, P, Y;
	Rect_left.getRPY(R,P,Y);
	convergence_quat_left = tf::createQuaternionFromRPY(R,P,Y);

	Rect_right.getRPY(R,P,Y);
	convergence_quat_right = tf::createQuaternionFromRPY(R,P,Y);

    left_frame.setRotation( tf::Quaternion(0,0,0));
    right_frame.setRotation( tf::Quaternion(0,0,0));

    // Publish optical frames

    // Publish frames
    std::vector<tf::StampedTransform> transforms;

    left_frame.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transforms.push_back(tf::StampedTransform(left_frame, ros::Time::now(), parameters.base_frame_id, "fake_cam_left_optical_link"));

    right_frame.setOrigin( tf::Vector3(-B, 0.0, 0.0) );
    transforms.push_back(tf::StampedTransform(right_frame, ros::Time::now(), parameters.base_frame_id, "fake_cam_right_optical_link"));


    broadcaster.sendTransform(transforms);

    sensor_msgs::Image msg_left, msg_right;

    msg_left = get_image_msg("left");
    msg_right = get_image_msg("right");

    camera_info_left.header = msg_left.header;
    camera_info_right.header = msg_right.header;
    // Shahab commented out the following lines:
//    camera_info.P[0] = parameters.fov;
//    camera_info.P[5] = parameters.fov;
    char buffer [50];
    sprintf(buffer, "separation = %f", B);
//    ROS_ERROR(buffer);

    pub_left.publish(msg_left, camera_info_left);

    pub_right.publish(msg_right, camera_info_right);

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}


