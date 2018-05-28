/* Safety Control Node
   This node provides basic collision / cliff detection and prevention
   To simplify behavior, potential collisions are detected by "zones" around the robot
   as defined in SafetySensorSummary

   - Tune how long the robot stays stopped by modifying "timeout" in param/cmd_vel_mux.yaml
   - Make sure you remap /cmd_vel in your launch file
*/

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "boost/lexical_cast.hpp"
#include <assert.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>    // std::min
#include <exception>      // std::exception

// LASER LISTENER
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"


// DIRECT POINTCLOUD
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h> // for testing only
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// NOTE: you must install TF2 Sensor Messages: sudo apt-get install ros-kinetic-tf2-sensor-msgs

//#include <trajectory_msgs/JointTrajectory.h>

// ROS REP103: "By the right hand rule, the yaw component of orientation increases 
// as the child frame rotates counter-clockwise, 
// and for geographic poses, yaw is zero when pointing east."  (left is positive, right is negative)         


enum ZONES {
  LEFT_REAR_ZONE = 0, 		
  LEFT_SIDE_ZONE,
  LEFT_FRONT_SIDE_ZONE, // In front of robot, to the side
  LEFT_FRONT_ZONE,
  RIGHT_FRONT_ZONE,
  RIGHT_FRONT_SIDE_ZONE, // In front of robot, to the side
  RIGHT_SIDE_ZONE,
  RIGHT_REAR_ZONE		
};
#define NUMBER_OF_ZONES    8

enum FIELDS {
  LIDAR = 0, 		
  FRONT_DEPTH_CAMERA,
  REAR_DEPTH_CAMERA,
  FRONT_CLIFF, // track cliffs separately
  REAR_CLIFF,
  CONFIDENCE, // overall confidence of threat
};
#define NUMBER_OF_FIELDS   6

typedef struct
{
	float	  ax; // bottom right corner
	float	  ay; 
	float	  bx; // top left corner
	float	  by; 
} BOUNDING_BOX_T;

// Global
const int DEFAULT_LIDAR_CONFIDENCE_THRESHOLD = 4; // number of lidar points needed to indicate object
const int DEFAULT_DEPTH_CAMERA_CONFIDENCE_THRESHOLD = 50; // number of depth camera points to indicate object
const int DEFAULT_CLIFF_CONFIDENCE_THRESHOLD = 150; // number of depth camera points to indicate cliff (large to avoid noise)
const int DEFAULT_OVERALL_CONFIDENCE_THRESHOLD = 4; // number of combined points needed to indicate object


////////////////////////////////////////////////////////////////////////////////////////////////////////
class SafetySensorSummary
{
public:
				  SafetySensorSummary();		// Constructor automatically initializes defaults
  void    Increment(int zone, int field);
  void    SetValue(int zone, int field, int value);
  int     GetValue(int zone, int field);
  void    SetConfidence(int field, double confidence_threshold);
  bool    ThreatDetectedInZone(int zone);
  void    ClearSensorReadings(int sensor); 
  void    UpdateConfidence();       // Summary of all sensor readings. called after any sensor updates

  int       zone_status_[NUMBER_OF_ZONES][NUMBER_OF_FIELDS];
  double    sensor_threshold_[NUMBER_OF_FIELDS];

};


SafetySensorSummary::SafetySensorSummary() 
{
  ROS_INFO("SafetySensorSummary: Starting... ");

  sensor_threshold_[LIDAR]              = DEFAULT_LIDAR_CONFIDENCE_THRESHOLD;
  sensor_threshold_[FRONT_DEPTH_CAMERA] = DEFAULT_DEPTH_CAMERA_CONFIDENCE_THRESHOLD;
  sensor_threshold_[REAR_DEPTH_CAMERA]  = DEFAULT_DEPTH_CAMERA_CONFIDENCE_THRESHOLD;
  sensor_threshold_[FRONT_CLIFF]        = DEFAULT_CLIFF_CONFIDENCE_THRESHOLD;
  sensor_threshold_[REAR_CLIFF]         = DEFAULT_CLIFF_CONFIDENCE_THRESHOLD;
  sensor_threshold_[CONFIDENCE]         = DEFAULT_OVERALL_CONFIDENCE_THRESHOLD;

  for(int zone=0; zone<NUMBER_OF_ZONES; zone++)
  {
    for(int field=0; field<NUMBER_OF_FIELDS; field++)
    zone_status_[zone][field] = 0;
  }

}

void SafetySensorSummary::ClearSensorReadings(int sensor)
{
  for(int zone=0; zone<NUMBER_OF_ZONES; zone++)
  {
    zone_status_[zone][sensor] = 0;
  }
}

void SafetySensorSummary::UpdateConfidence()
{
  for(int zone=0; zone<NUMBER_OF_ZONES; zone++)
  {
    int lidar_confidence = 0;
    int front_depth_camera_confidence = 0;
    int rear_depth_camera_confidence = 0;
    int front_cliff_confidence = 0;
    int rear_cliff_confidence = 0;

    // only include confidence if it is above the threshold for that particular type of sensor
    if( zone_status_[zone][LIDAR] > sensor_threshold_[LIDAR])
    {
      lidar_confidence = zone_status_[zone][LIDAR];
    }

    if( zone_status_[zone][FRONT_DEPTH_CAMERA] > sensor_threshold_[FRONT_DEPTH_CAMERA])
    {
      front_depth_camera_confidence = zone_status_[zone][FRONT_DEPTH_CAMERA];
    }

    if( zone_status_[zone][REAR_DEPTH_CAMERA] > sensor_threshold_[REAR_DEPTH_CAMERA])
    {
      rear_depth_camera_confidence = zone_status_[zone][REAR_DEPTH_CAMERA];
    }

    // CLIFFS
    if( zone_status_[zone][FRONT_CLIFF] > sensor_threshold_[FRONT_CLIFF])
    {
      front_cliff_confidence = zone_status_[zone][FRONT_CLIFF];
    }
    if( zone_status_[zone][REAR_CLIFF] > sensor_threshold_[REAR_CLIFF])
    {
      rear_cliff_confidence = zone_status_[zone][REAR_CLIFF];
    }

    zone_status_[zone][CONFIDENCE] = lidar_confidence + 
      front_depth_camera_confidence + rear_depth_camera_confidence + 
      front_cliff_confidence + rear_cliff_confidence;

/*
    printf ("DBG UpdateConfidence:  For Zone = %d, confidence = %d\n", zone, zone_status_[zone][CONFIDENCE]);
    printf ("                       LIDAR =              %4d\n", zone_status_[zone][LIDAR]);
    printf ("                       FRONT_DEPTH_CAMERA = %4d\n", zone_status_[zone][FRONT_DEPTH_CAMERA]);
    printf ("                       REAR_DEPTH_CAMERA =  %4d\n", zone_status_[zone][REAR_DEPTH_CAMERA]);
*/

  }
}

void SafetySensorSummary::SetConfidence(int field, double confidence_threshold)
{
  if(field < NUMBER_OF_FIELDS)
  {
    sensor_threshold_[field] = confidence_threshold;
    ROS_INFO("SafetySensorSummary: Confidence for sensor %d changed to %f", field, confidence_threshold);
  }
  else
  {
    ROS_ERROR("SafetySensorSummary::SetConfidence:  BAD FIELD NUMBER! (%d)", field);
  }
}

void SafetySensorSummary::Increment(int zone, int field)
{
  zone_status_[zone][field]++;
}

void SafetySensorSummary::SetValue(int zone, int field, int value)
{
  zone_status_[zone][field] = value;
}

int SafetySensorSummary::GetValue(int zone, int field)
{
  return zone_status_[zone][field];
}

bool SafetySensorSummary::ThreatDetectedInZone(int zone)
{
  // determine if there is an object in the zone
  //printf ("DBG ThreatDetectedInZone:  Zone = %d, confidence = %d\n", zone, zone_status_[zone][CONFIDENCE]);

  if( zone_status_[zone][CONFIDENCE] >= sensor_threshold_[CONFIDENCE] )
  {
    printf ("DBG OBJECT IN ZONE:  Zone = %d, confidence = %d\n", zone, zone_status_[zone][CONFIDENCE]);
    printf ("                     LIDAR = %d, FRONT_DEPTH_CAMERA = %d, FRONT_CLIFF = %d\n", 
      zone_status_[zone][LIDAR], zone_status_[zone][FRONT_DEPTH_CAMERA],
      zone_status_[zone][FRONT_CLIFF]);

    return true;
  }
  else
  {
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
class SafetyControl
{
public:
  SafetyControl(ros::NodeHandle n);

private:

  // FUNCTIONS
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void frontDepthScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void rearDepthScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  //void cmdVelMuxCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void DisplayCollisionMarker(int zone);
  void UpdateZonesWithSensor(int sensor_type, float x, float y, float z=0.0);
  bool IsInBoundingBox(BOUNDING_BOX_T bb, float x, float y);
  void HandleThreats(bool threat_detected);
  void CheckForThreat();
  void Stop();
  void Turn(float turnh_amount);
  bool robotIsMoving();

 //void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_in);
 void frontDepthCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_in);

  // CONSTANTS

  const char*   DEFAULT_TARGET_FRAME = "base_link";     // TF frame for sensors
  const double  DEFAULT_TF_TOLERANCE = 0.01;           // TF latency tolerance

  const float   DEFAULT_ROBOT_LENGTH = 0.4;  // meters
  const float   DEFAULT_ROBOT_WIDTH = 0.4;  

  const float   DEFAULT_FRONT_ZONE_RANGE_MIN = 0.2; 
  const float   DEFAULT_FRONT_ZONE_RANGE_SPEED_SCALER = 0.2;
  const float   DEFAULT_FRONT_ZONE_CLIFF_ADDER = 0.2; // Add extra buffer for cliffs! 
  const float   DEFAULT_FRONT_ZONE_WIDTH = (DEFAULT_ROBOT_WIDTH * 0.75) / 2.0; // a little narrower than half the robot
  const float   DEFAULT_SIDE_ZONE_RANGE = 0.05; 
  const float   DEFAULT_FRONT_SIDE_ZONE_WIDTH = ((DEFAULT_ROBOT_WIDTH / 2.0) - DEFAULT_FRONT_ZONE_WIDTH) + DEFAULT_SIDE_ZONE_RANGE;

  const float   DEFAULT_REAR_ZONE_RANGE_MIN = 0.1; 
  const float   DEFAULT_REAR_ZONE_RANGE_SPEED_SCALER = 0.2;
  const float   DEFAULT_REAR_ZONE_CLIFF_ADDER = 0.2; // Add extra buffer for cliffs! 
  const float   DEFAULT_REAR_ZONE_WIDTH = DEFAULT_ROBOT_WIDTH / 2.0;

  const float   DEFAULT_AVOID_TURN_AMOUNT = 0.8;    // radians per second
  const float   DEFAULT_OBJECT_MIN_HEIGHT = 0.0508; // avoid any objects higher than this (2 inches)
  const float   DEFAULT_OBJECT_MAX_HEIGHT = 1.0;    // ignore any objects taller than the robot (using 1 meter for now)
  const float   DEFAULT_CLIFF_MIN_HEIGHT = -0.03; //-0.0508;  // find cliffs bigger than this (2 inches)
  const float   DEFAULT_CLIFF_MAX_HEIGHT = -1.0;     // sensor reading larger than this is probably an error

  // PARAMETERS

  std::float_t  robot_length_;
  std::float_t  robot_width_;

  std::float_t  front_zone_range_min_;
  std::float_t  front_zone_range_speed_scaler_;
  std::float_t  front_zone_cliff_adder_;
  std::float_t  front_zone_width_;

  std::float_t  side_zone_range_;
  std::float_t  front_side_zone_width_;

  std::float_t  rear_zone_range_min_;
  std::float_t  rear_zone_range_speed_scaler_;
  std::float_t  rear_zone_cliff_adder_;
  std::float_t  rear_zone_width_;

  std::float_t  avoid_turn_amount_;

  std::float_t  object_min_height_;  // find any objects higher than this
  std::float_t  object_max_height_;  // ignore any objects taller than the robot (using 1 meter for now)
  std::float_t  cliff_min_height_;   // find cliffs higher than this
  std::float_t  cliff_max_height_;   // ignore any cliffs taller than this, probably an error


  // VARIABLES

  ros::NodeHandle                 nh_;
  ros::NodeHandle                 private_nh_;  
  
  std::string                     target_frame_;
  double                          tf_tolerance_; 
  tf2_ros::Buffer                 tf2_;
  tf2_ros::TransformListener      tfListener_;

  sensor_msgs::PointCloud         laser_scan_cloud_;
  laser_geometry::LaserProjection laser_scan_projector_;
  tf::TransformListener           laser_scan_listener_;

  sensor_msgs::PointCloud         front_depth_scan_cloud_;
  laser_geometry::LaserProjection front_depth_scan_projector_;
  tf::TransformListener           front_depth_scan_listener_;
 
  sensor_msgs::PointCloud         rear_depth_scan_cloud_;
  laser_geometry::LaserProjection rear_depth_scan_projector_;
  tf::TransformListener           rear_depth_scan_listener_;
 
  SafetySensorSummary             sensor_summary_;
  float                           odom_linear_velocity_;       // current velocity as reported by odometry
  float                           odom_angular_velocity_;
  //float                         cmd_mux_linear_velocity_;    // current speed requested by the mux 
  //float                         cmd_mux_angular_velocity_;   // (for whatever module has the mux)
  float                           front_zone_range_;
  float                           rear_zone_range_;



  // SUBSCRIBERS
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub; // from lidar
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;

  message_filters::Subscriber<sensor_msgs::LaserScan> front_depth_scan_sub_; // from front depth camera
  tf::MessageFilter<sensor_msgs::LaserScan> front_depth_scan_notifier_;

  message_filters::Subscriber<sensor_msgs::LaserScan> rear_depth_scan_sub_; // from front depth camera
  tf::MessageFilter<sensor_msgs::LaserScan> rear_depth_scan_notifier_;

  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_vel_sub_; // current velocity being requested by whatever node has the mux
  ros::Subscriber front_depth_cloud_sub_;

  // PUBLISHERS
  ros::Publisher laser_scan_pub_;
  ros::Publisher front_depth_scan_pub_;
  ros::Publisher rear_depth_scan_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher cmd_vel_pub_;
  //ros::Publisher voxel_pub_;


};
////////////////////////////////////////////////////////////////////////////////////////////////////////


SafetyControl::SafetyControl(ros::NodeHandle n) : 
  nh_(n),
  private_nh_("~"),
  laser_sub(nh_, "/scan", 10),
  laser_notifier(laser_sub,laser_scan_listener_, "base_link", 10),

  front_depth_scan_sub_(nh_, "/camera/scan1", 10), // from front depth camera
  front_depth_scan_notifier_(front_depth_scan_sub_, front_depth_scan_listener_, "base_link", 10),

  rear_depth_scan_sub_(nh_, "/camera/scan2", 10), // from front depth camera  ==> DAVES TODO - FIX TOPIC!!
  rear_depth_scan_notifier_(rear_depth_scan_sub_, rear_depth_scan_listener_, "base_link", 10),

  tfListener_(tf2_),

  odom_linear_velocity_(0.0),
  odom_angular_velocity_(0.0)

{

  ROS_INFO("SafetyControl: Initializing...");



  // PARAMETERS
  printf("\nSETTABLE PARAMS: \n");

  private_nh_.param<std::string>("target_frame", target_frame_, DEFAULT_TARGET_FRAME); // TF frame for sensors
  private_nh_.param<double>("transform_tolerance", tf_tolerance_, DEFAULT_TF_TOLERANCE); 

  private_nh_.param<std::float_t>("robot_length", robot_length_, DEFAULT_ROBOT_LENGTH);
  private_nh_.param<std::float_t>("robot_width", robot_width_, DEFAULT_ROBOT_WIDTH);

  private_nh_.param<std::float_t>("front_zone_range_min", front_zone_range_min_, DEFAULT_FRONT_ZONE_RANGE_MIN);
  private_nh_.param<std::float_t>("front_zone_range_speed_scaler", front_zone_range_speed_scaler_, DEFAULT_FRONT_ZONE_RANGE_SPEED_SCALER);
  private_nh_.param<std::float_t>("front_zone_cliff_adder", front_zone_cliff_adder_, DEFAULT_FRONT_ZONE_CLIFF_ADDER);
  private_nh_.param<std::float_t>("front_zone_width", front_zone_width_, DEFAULT_FRONT_ZONE_WIDTH);

  private_nh_.param<std::float_t>("side_zone_range", side_zone_range_, DEFAULT_SIDE_ZONE_RANGE);
  private_nh_.param<std::float_t>("front_side_zone_width", front_side_zone_width_, DEFAULT_FRONT_SIDE_ZONE_WIDTH);

  private_nh_.param<std::float_t>("rear_zone_range_min", rear_zone_range_min_, DEFAULT_REAR_ZONE_RANGE_MIN);
  private_nh_.param<std::float_t>("rear_zone_range_speed_scaler", rear_zone_range_speed_scaler_, DEFAULT_REAR_ZONE_RANGE_SPEED_SCALER);
  private_nh_.param<std::float_t>("rear_zone_cliff_adder", rear_zone_cliff_adder_, DEFAULT_REAR_ZONE_CLIFF_ADDER);

  private_nh_.param<std::float_t>("rear_zone_width", rear_zone_width_, DEFAULT_REAR_ZONE_WIDTH);

  private_nh_.param<std::float_t>("avoid_turn_amount", avoid_turn_amount_, DEFAULT_AVOID_TURN_AMOUNT);
  private_nh_.param<std::float_t>("object_min_height", object_min_height_, DEFAULT_OBJECT_MIN_HEIGHT);
  private_nh_.param<std::float_t>("object_max_height", object_max_height_, DEFAULT_OBJECT_MAX_HEIGHT);
  private_nh_.param<std::float_t>("cliff_min_height",  cliff_min_height_,  DEFAULT_CLIFF_MIN_HEIGHT);
  private_nh_.param<std::float_t>("cliff_max_height",  cliff_max_height_,  DEFAULT_CLIFF_MAX_HEIGHT);

  // initialize variable sensor ranges
  front_zone_range_ = front_zone_range_min_;  // robot not moving initially
  rear_zone_range_ = rear_zone_range_min_;


  // Display current settings
  printf("   target_frame:                  %s (default = '%s')\n", target_frame_.c_str(), DEFAULT_TARGET_FRAME);
  printf("   tf_tolerance:                  %f  (default = %f)\n\n", tf_tolerance_, DEFAULT_TF_TOLERANCE);

  printf("   robot_length:                  %f  (default = %f)\n", robot_length_, DEFAULT_ROBOT_LENGTH);
  printf("   robot_width:                   %f  (default = %f)\n\n", robot_width_, DEFAULT_ROBOT_WIDTH);

  printf("   front_zone_range_min:          %f  (default = %f)\n", front_zone_range_min_, DEFAULT_FRONT_ZONE_RANGE_MIN);
  printf("   front_zone_range_speed_scaler: %f  (default = %f)\n", front_zone_range_speed_scaler_, DEFAULT_FRONT_ZONE_RANGE_SPEED_SCALER);
  printf("   front_zone_cliff_adder:        %f  (default = %f)\n", front_zone_cliff_adder_, DEFAULT_FRONT_ZONE_CLIFF_ADDER);
  printf("   front_zone_width:              %f  (default = %f)\n\n", front_zone_width_, DEFAULT_FRONT_ZONE_WIDTH);

  printf("   side_zone_range:               %f  (default = %f)\n", side_zone_range_, DEFAULT_SIDE_ZONE_RANGE);
  printf("   front_side_zone_width:         %f  (default = %f)\n\n", front_side_zone_width_, DEFAULT_FRONT_SIDE_ZONE_WIDTH);

  printf("   rear_zone_range_min:           %f  (default = %f)\n", rear_zone_range_min_, DEFAULT_REAR_ZONE_RANGE_MIN);
  printf("   rear_zone_range_speed_scaler:  %f  (default = %f)\n", rear_zone_range_speed_scaler_, DEFAULT_REAR_ZONE_RANGE_SPEED_SCALER);
  printf("   rear_zone_cliff_adder:         %f  (default = %f)\n", rear_zone_cliff_adder_, DEFAULT_REAR_ZONE_CLIFF_ADDER);
  printf("   rear_zone_width:               %f  (default = %f)\n\n", rear_zone_width_, DEFAULT_REAR_ZONE_WIDTH);

  printf("   avoid_turn_amount:             %f  (default = %f)\n",   avoid_turn_amount_, DEFAULT_AVOID_TURN_AMOUNT);
  printf("   object_min_height:             %f  (default = %f)\n",   object_min_height_, DEFAULT_OBJECT_MIN_HEIGHT);
  printf("   object_max_height:             %f  (default = %f)\n", object_max_height_, DEFAULT_OBJECT_MAX_HEIGHT);
  printf("   cliff_min_height:             %f  (default = %f)\n", cliff_min_height_, DEFAULT_CLIFF_MIN_HEIGHT);
  printf("   cliff_max_height:             %f  (default = %f)\n\n", cliff_max_height_, DEFAULT_CLIFF_MAX_HEIGHT);


  printf("   THRESHOLDS:\n");

  // set optional parameters for the SensorSummary class
  int threshold;

  if ( private_nh_.param("lidar_confidence_threshold", threshold, DEFAULT_LIDAR_CONFIDENCE_THRESHOLD) )
  {
    sensor_summary_.SetConfidence(LIDAR, threshold);
  }
  printf("   lidar_confidence_threshold:        %4d  (default = %4d)\n", 
    threshold, DEFAULT_LIDAR_CONFIDENCE_THRESHOLD);

  if ( private_nh_.param("depth_camera_confidence_threshold", threshold, DEFAULT_DEPTH_CAMERA_CONFIDENCE_THRESHOLD) )
  {
    sensor_summary_.SetConfidence(FRONT_DEPTH_CAMERA, threshold);
    sensor_summary_.SetConfidence(REAR_DEPTH_CAMERA, threshold);
  }
  printf("   depth_camera_confidence_threshold: %4d  (default = %4d)\n", 
    threshold, DEFAULT_LIDAR_CONFIDENCE_THRESHOLD);

  if ( private_nh_.param("cliff_confidence_threshold", threshold, DEFAULT_CLIFF_CONFIDENCE_THRESHOLD) )
  {
    sensor_summary_.SetConfidence(FRONT_CLIFF, threshold);
    sensor_summary_.SetConfidence(REAR_CLIFF, threshold);
  }
  printf("   cliff_confidence_threshold:        %4d  (default = %4d)\n", 
    threshold, DEFAULT_CLIFF_CONFIDENCE_THRESHOLD);

  if ( private_nh_.param("overall_confidence_threshold", threshold, DEFAULT_OVERALL_CONFIDENCE_THRESHOLD) )
  {
    sensor_summary_.SetConfidence(CONFIDENCE, threshold);
  }
  printf("   overall_confidence_threshold:      %4d  (default = %4d)\n\n", 
    threshold, DEFAULT_OVERALL_CONFIDENCE_THRESHOLD);




  // SUBSCRIBERS
  laser_notifier.registerCallback(
    boost::bind(&SafetyControl::laserScanCallback, this, _1));
  laser_notifier.setTolerance(ros::Duration(0.02));

  front_depth_scan_notifier_.registerCallback(
    boost::bind(&SafetyControl::frontDepthScanCallback, this, _1));
  front_depth_scan_notifier_.setTolerance(ros::Duration(0.01));

  rear_depth_scan_notifier_.registerCallback(
    boost::bind(&SafetyControl::rearDepthScanCallback, this, _1));
  rear_depth_scan_notifier_.setTolerance(ros::Duration(0.01));

  // DAVES - DEPTH CLOUD
  front_depth_cloud_sub_ = nh_.subscribe ("/camera/points", 1, &SafetyControl::frontDepthCloudCallback, this); 


  odom_sub_ = nh_.subscribe("odom", 1000, &SafetyControl::odomCallback, this);
  //cmd_vel_sub_ = nh_.subscribe("/mobile_base/commands/velocity", 1000, &SafetyControl::cmdVelMuxCallback, this);


  // PUBLISHERS
  laser_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("safety_control/laser_cloud",1);
  front_depth_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("safety_control/laser_cloud2",1);
  rear_depth_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("safety_control/laser_cloud3",1);

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("safety_control/zone_marker", 1);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

  //voxel_pub_ = nh_.advertise<pcl::PCLPointCloud2> ("output", 1); // DAVES TODO - set correct message!

  ROS_INFO("SafetyControl: listening for topics: /scan, odom, /camera/points ");
  ROS_INFO("SafetyControl: publishing topics: /cmd_vel, safety_control/zone_marker, safety_control/laser_cloud ");

}


////////////////////////////////////////////////////////////////////////////////////////////////////////

void SafetyControl::frontDepthCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // ROS_INFO("DBG: Got FRONT DEPTH CLOUD");
    
  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;

  try
  {
    cloud.reset(new sensor_msgs::PointCloud2);
    tf2_.transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tf_tolerance_));
    cloud_out = cloud;
  }
  catch (tf2::TransformException &ex)
  {
    // ROS_WARN_STREAM("Transform failure: " << ex.what());
    return;
  }

  int total_points_found = 0;
  int points_in_boundry = 0;
  int cliff_points_found = 0;
  double dbg_total_to_average = 0.0;

  // clear prior readings
  sensor_summary_.ClearSensorReadings(FRONT_DEPTH_CAMERA); 
  sensor_summary_.ClearSensorReadings(FRONT_CLIFF); 

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float>
            iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
            iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z)
  {


    // Discard any NAN points
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      // ROS_INFO("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }
    total_points_found++;
    dbg_total_to_average += *iter_z;


    // look for objects and cliffs
    // Test height of each point, reject those too high or too low

    if (*iter_z < cliff_min_height_ && *iter_z > cliff_max_height_) // negative values (below the floor)
    {
      // CLIFF Detected
      //ROS_INFO("found cliff point: z = %f in range (%f, %f)", *iter_z, cliff_min_height_, cliff_max_height_);
      cliff_points_found++;
      UpdateZonesWithSensor(FRONT_CLIFF, *iter_x, *iter_y, *iter_z);

    }
    else if (*iter_z > object_min_height_ && *iter_z < object_max_height_)
    {
      // OBJECT Detected
      // ROS_INFO("rejected for height %f not in range (%f, %f)", *iter_z, object_min_height_, object_max_height_);
      points_in_boundry++;
      UpdateZonesWithSensor(FRONT_DEPTH_CAMERA, *iter_x, *iter_y, *iter_z);
    }
  }

  if( total_points_found != 0 )
  {
  	 double dbg_average = dbg_total_to_average/total_points_found;
     ROS_INFO(">>> total points found = %d,  averageZ = %f", total_points_found, dbg_average );
  }

  //ROS_INFO("DEPTH CLOUD: total points found = %d,  in boundry = %d", total_points_found, points_in_boundry);
  //ROS_INFO("DEPTH CLOUD: cliff points found = %d", cliff_points_found);

  // update system confidence
  sensor_summary_.UpdateConfidence();         

  // See if any objects pose a threat to the robot, and handle them
  CheckForThreat();


}


void SafetyControl::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

  // ROS_INFO("DBG: Got LIDAR Laser Scan");
  try
  {
      laser_scan_projector_.transformLaserScanToPointCloud(
        "base_link",*scan_in, laser_scan_cloud_,laser_scan_listener_);
  }
  catch (tf::TransformException& e)
  {
      std::cout << e.what();
      return;
  }
  
  // Got a valid laser scan, which has been converted to a point cloud (X,Y values, z is 0)
  // publish the laserCloud for debug purposes
  laser_scan_pub_.publish(laser_scan_cloud_);


  // clear Lidar readings
  sensor_summary_.ClearSensorReadings(LIDAR); 

  geometry_msgs::Point32 nearX, nearY;
  nearX.x = 10000;
  nearY.y = 10000;


  //Update Sensor Summary with LaserScan 
  uint32_t num_points = laser_scan_cloud_.points.size(); 
  //printf ("DBG LaserCloud: number points = %d\n", num_points);
  for( unsigned int i=0; i < num_points; i++)
  {
    //printf ("DBG laserScanCallback:  x = %f, y = %f\n", laser_scan_cloud_.points[i].x, laser_scan_cloud_.points[i].y);

    if( fabs(laser_scan_cloud_.points[i].x) < fabs(nearX.x))
    {
     nearX.x = laser_scan_cloud_.points[i].x;
     nearX.y = laser_scan_cloud_.points[i].y;
    }
    if( fabs(laser_scan_cloud_.points[i].y) < fabs(nearY.y))
    {
     nearY.x = laser_scan_cloud_.points[i].x;
     nearY.y = laser_scan_cloud_.points[i].y;
    }

    UpdateZonesWithSensor(LIDAR, laser_scan_cloud_.points[i].x, laser_scan_cloud_.points[i].y);
  }

  //printf ("DBG LaserCloud: \n   Nearest X = %f, %f  \n   Nearest Y = %f, %f\n", 
  //  nearX.x, nearX.y, nearY.x, nearY.y);


  // update system confidence
  sensor_summary_.UpdateConfidence();         

  // See if any objects pose a threat to the robot, and handle them
  CheckForThreat();

}




void SafetyControl::frontDepthScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  // WARNING - for this to work, you must launch "pointcloud_to_laser.launch"

  ROS_INFO("DBG: Got FRONT DEPTH CAMERA Laser Scan");
  try
  {
      front_depth_scan_projector_.transformLaserScanToPointCloud(
        "base_link",*scan_in, front_depth_scan_cloud_,front_depth_scan_listener_);
  }
  catch (tf::TransformException& e)
  {
      std::cout << e.what();
      return;
  }
  
  // Got a valid laser scan, which has been converted to a point cloud (X,Y values, z is 0)
  // publish the laserCloud for debug purposes
  front_depth_scan_pub_.publish(front_depth_scan_cloud_);

  // clear Lidar readings
  sensor_summary_.ClearSensorReadings(FRONT_DEPTH_CAMERA); 

  //Update Sensor Summary with LaserScan 
  uint32_t num_points = front_depth_scan_cloud_.points.size(); 
  // printf ("DBG LaserCloud: number points = %d\n", num_points);
  for( unsigned int i=0; i < num_points; i++)
  {
    UpdateZonesWithSensor(FRONT_DEPTH_CAMERA, front_depth_scan_cloud_.points[i].x, laser_scan_cloud_.points[i].y);
  }

  // update system confidence
  sensor_summary_.UpdateConfidence();         

  // See if any objects pose a threat to the robot, and handle them
  CheckForThreat();


}

void SafetyControl::rearDepthScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  ROS_INFO("DBG: Got REAR DEPTH CAMERA Laser Scan");
  try
  {
      rear_depth_scan_projector_.transformLaserScanToPointCloud(
        "base_link",*scan_in, rear_depth_scan_cloud_, rear_depth_scan_listener_);
  }
  catch (tf::TransformException& e)
  {
      std::cout << e.what();
      return;
  }
  
  // Got a valid laser scan, which has been converted to a point cloud (X,Y values, z is 0)
  // publish the laserCloud for debug purposes
  rear_depth_scan_pub_.publish(rear_depth_scan_cloud_);

  // clear Lidar readings
  sensor_summary_.ClearSensorReadings(FRONT_DEPTH_CAMERA); 

  //Update Sensor Summary with LaserScan 
  uint32_t num_points = rear_depth_scan_cloud_.points.size(); 
  // printf ("DBG LaserCloud: number points = %d\n", num_points);
  for( unsigned int i=0; i < num_points; i++)
  {
    UpdateZonesWithSensor(FRONT_DEPTH_CAMERA, rear_depth_scan_cloud_.points[i].x, laser_scan_cloud_.points[i].y);
  }

  // update system confidence
  sensor_summary_.UpdateConfidence();         

  // See if any objects pose a threat to the robot, and handle them
  CheckForThreat();

}


void SafetyControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Current velocity reported by Odometry
  // ROS_INFO("DBG Odom: Linear = %f, Angular = %f", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
  odom_linear_velocity_ = msg->twist.twist.linear.x;
  odom_angular_velocity_ = msg->twist.twist.angular.z;
  const float velocity_max = 0.4; // approx max speed of turtlebot

  // Set dynamic zone ranges.
  front_zone_range_ = front_zone_range_min_;
  rear_zone_range_ = rear_zone_range_min_;
  if(odom_linear_velocity_ > 0.0)
  {
    // Robot moving forward
    front_zone_range_ = front_zone_range_min_ + ((odom_linear_velocity_ / velocity_max) * front_zone_range_speed_scaler_);
  }
  else if(odom_linear_velocity_ < 0.0)
  {
    // Robot backing up
    rear_zone_range_ = rear_zone_range_min_ - ((odom_linear_velocity_ / velocity_max) * rear_zone_range_speed_scaler_);
  }

  //ROS_INFO("DBG Odom-> front_zone_range_: [%f], rear_zone_range_: [%f]", front_zone_range_, rear_zone_range_);

}

/*
void SafetyControl::cmdVelMuxCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Current velocity being requested by the mux
  //ROS_INFO("VelMux-> Linear: [%f], Angular: [%f]", msg->linear.x, msg->angular.z);
  cmd_mux_linear_velocity_ = msg->linear.x;
  cmd_mux_angular_velocity_ = msg->angular.z;

}
*/


void SafetyControl::Stop()
{
  // send high priority message to motor control overriding most behaviors
  geometry_msgs::Twist cmd;
  cmd.linear.x = cmd.angular.z = 0;  
  cmd_vel_pub_.publish(cmd); 

}


void SafetyControl::Turn(float turn_amount)
{
  // send high priority message to motor control overriding most behaviors
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.0; // force pivot turn.  (Cant continue at current speed, becuase safety control takes over the mux!)
  cmd.angular.z = turn_amount;     // but turn as needed
  cmd_vel_pub_.publish(cmd); 

}


void SafetyControl::CheckForThreat()
{

  // Check for Collisions or Cliffs, and display in RVIZ
  bool threat_detected = false;
  for(int zone_number=0; zone_number<NUMBER_OF_ZONES; zone_number++)
  {
    if( sensor_summary_.ThreatDetectedInZone(zone_number) )
    {
      // printf ("DBG CheckForThreat:  OBJECT DETECTED in Zone %d\n", zone_number);
      threat_detected = true;
      DisplayCollisionMarker(zone_number); 
    }
  }

  // Handle any threats
  HandleThreats(threat_detected);

}

bool SafetyControl::robotIsMoving()
{
  return (0.0 != odom_linear_velocity_) || (0.0 != odom_angular_velocity_);
}


void SafetyControl::HandleThreats(bool threat_detected)
{
  // Only publish output if a threat is found, and only to prevent movement in the "bad" direction
  // So, if threat in front, still allow movement backward and turns
  float new_turn = 0.0;

  if(!threat_detected)
  {
    return; // for now, just return if no threats.  In future, we may need to do something after the threat has passed?
  } 

  // ROS_INFO("--- HandleThreats ---");
  // is SafetyControl enabled?
  // TODO - use a "state" message, service, or parameter server?

  // Check if robot is moving
  if( !robotIsMoving() )
  {
    return; // Not moving. Don't do collision avoidance if we are not moving
    // for example, prevent robot from backing up if a person stands next to robot
  }

  // Check if robot is backing up
  if(odom_linear_velocity_ < 0.0)
  {
    // robot is backing up
    if( sensor_summary_.ThreatDetectedInZone(LEFT_REAR_ZONE) || sensor_summary_.ThreatDetectedInZone(RIGHT_REAR_ZONE) )
    {
      // just force stop
      Stop();
      ROS_INFO("Safety_control: Object while backing up, Robot stopping.");
    }
    return;
  }

  // Look for potential collisions
  else if( (sensor_summary_.ThreatDetectedInZone(LEFT_FRONT_ZONE) && sensor_summary_.ThreatDetectedInZone(RIGHT_FRONT_ZONE)) || // dead ahead
    (sensor_summary_.ThreatDetectedInZone(LEFT_FRONT_SIDE_ZONE) && sensor_summary_.ThreatDetectedInZone(RIGHT_FRONT_SIDE_ZONE)) || // both front side sensors
    (sensor_summary_.ThreatDetectedInZone(LEFT_SIDE_ZONE) && sensor_summary_.ThreatDetectedInZone(RIGHT_SIDE_ZONE)) ) // both side sensors!
  {
    // Object dead ahead, or on both sides!  Just force stop
    if(odom_linear_velocity_ > 0.0)
    {
      Stop(); // block robot from moving forward.  Turns are OK.
      ROS_INFO("Safety_control: Object ahead, Robot stopping");
      return;
    }

  }
  else if(sensor_summary_.ThreatDetectedInZone(LEFT_FRONT_ZONE) || sensor_summary_.ThreatDetectedInZone(LEFT_FRONT_SIDE_ZONE) || sensor_summary_.ThreatDetectedInZone(LEFT_SIDE_ZONE))
  {
    // object to the left, steer away from it
    // if strong enough turn already being requested, do nothing
    //if(cmd_mux_angular_velocity_ > (-avoid_turn_amount_))
    {
      // stop and turn right
      Turn(-avoid_turn_amount_); // RadiansPerSecond
      ROS_INFO("Safety_control: Object left, turning right");
    }
  }
  else if(sensor_summary_.ThreatDetectedInZone(RIGHT_FRONT_ZONE) || sensor_summary_.ThreatDetectedInZone(RIGHT_FRONT_SIDE_ZONE) || sensor_summary_.ThreatDetectedInZone(RIGHT_SIDE_ZONE))
  {
    // object to the right, steer away from it
    // if strong enough turn already being requested, do nothing
    //if(cmd_mux_angular_velocity_ < avoid_turn_amount_)
    {
      // stop and turn left
      Turn(avoid_turn_amount_); // RadiansPerSecond
      ROS_INFO("Safety_control: Object right, turning left");
    }
  }

}


bool SafetyControl::IsInBoundingBox(BOUNDING_BOX_T bb, float x, float y)
{
  // printf ("DBG IsInBoundingBox: bb.bx = %f, bb.ax = %f,  bb.by = %f, bb.ay = %f\n", bb.bx, bb.ax, bb.by, bb.ay);
  // printf ("DBG IsInBoundingBox: x = %f, y = %f", x, y);

  if( x <= bb.bx && bb.ax <= x && y <= bb.by && bb.ay <= y )
  {
    // printf ("  TRUE\n");
    return true;
  }
  else
  {
    // printf ("  FALSE\n");
    return false;
  }
}

void SafetyControl::UpdateZonesWithSensor(int sensor_type, float x, float y, float z)
{
  // for each sensor reading (eg., a point in the laser scan), see if it intersects a safety zone.
  // add up all the hits (which increase confidence that a hit actually occured)
  // Bounding box: (bx,by) are top-left coordinates, (ax,ay) are bottom-right coordinates. 

  double front_zone_range = front_zone_range_;
  double rear_zone_range  = rear_zone_range_;

  // Add additional safety distance for cliffs (it's very bad to fall off a cliff!)
  if(FRONT_CLIFF == sensor_type)
  {
    front_zone_range += front_zone_cliff_adder_;
    //printf ("DBG UpdateZonesWithSensor: CLIFF DEBUG: x = %f, y = %f, z = %f\n", x, y, z);
  }
  else if(REAR_CLIFF == sensor_type)
  {
    rear_zone_range += rear_zone_cliff_adder_;
  }

  // LEFT_FRONT_ZONE:
  BOUNDING_BOX_T left_front_zone;
  left_front_zone.ax = robot_length_ / 2.0;
  left_front_zone.ay = 0.0;
  left_front_zone.bx = left_front_zone.ax + front_zone_range;
  left_front_zone.by = front_zone_width_;
  //printf ("DBG UpdateZonesWithSensor: robot_length_ = %f, front_zone_range = %f\n", robot_length_, front_zone_range);
  //printf ("DBG UpdateZonesWithSensor: left_front_zone.bx = %f, left_front_zone.ax = %f,  left_front_zone.by = %f, left_front_zone.ay = %f\n", left_front_zone.bx, left_front_zone.ax, left_front_zone.by, left_front_zone.ay);
  //printf ("DBG UpdateZonesWithSensor: LEFT_FRONT_ZONE: x = %f, y = %f\n", x, y);
  if( IsInBoundingBox(left_front_zone, x, y) )
  {
    // point is in the box
    // if(FRONT_CLIFF == sensor_type)
      //printf (">>>>>> DBG UpdateZonesWithSensor: IN LEFT_FRONT_ZONE: sensor = %d, x = %f, y = %f, z = %f, x-ax = %f\n", sensor_type, x, y, z, (x-left_front_zone.ax) );

    sensor_summary_.Increment(LEFT_FRONT_ZONE, sensor_type);
  }

  // RIGHT_FRONT_ZONE
  BOUNDING_BOX_T right_front_zone;
  right_front_zone.ax = robot_length_ / 2.0;
  right_front_zone.ay = -1.0 * front_zone_width_;
  right_front_zone.bx = right_front_zone.ax + front_zone_range;
  right_front_zone.by = 0.0;
  //printf ("DBG UpdateZonesWithSensor: RIGHT_FRONT_ZONE: x = %f, y = %f\n", x, y);
  if( IsInBoundingBox(right_front_zone, x, y) )
  {
    // point is in the box
    //printf ("DBG UpdateZonesWithSensor: IN RIGHT_FRONT_ZONE: x = %f, y = %f\n", x, y);
    //printf ("DBG UpdateZonesWithSensor: IN RIGHT_FRONT_ZONE: sensor = %d, x = %f, y = %f, z = %f, x-ax = %f\n", sensor_type, x, y, z, (x-left_front_zone.ax) );
    sensor_summary_.Increment(RIGHT_FRONT_ZONE, sensor_type);
  }

  // LEFT_REAR_ZONE:
  BOUNDING_BOX_T left_rear_zone;
  left_rear_zone.bx = -1.0 * robot_length_ / 2.0;
  left_rear_zone.by = rear_zone_width_;
  left_rear_zone.ax = left_rear_zone.bx - rear_zone_range;
  left_rear_zone.ay = 0.0;
  if( IsInBoundingBox(left_rear_zone, x, y) )
  {
    // point is in the box
    sensor_summary_.Increment(LEFT_REAR_ZONE, sensor_type);
  }

  // RIGHT_REAR_ZONE
  BOUNDING_BOX_T right_rear_zone;
  right_rear_zone.bx = -1.0 * robot_length_ / 2.0;
  right_rear_zone.by = 0.0;
  right_rear_zone.ax = right_rear_zone.bx - rear_zone_range;
  right_rear_zone.ay = -1.0 * rear_zone_width_;
  if( IsInBoundingBox(right_rear_zone, x, y) )
  {
    // point is in the box
    sensor_summary_.Increment(RIGHT_REAR_ZONE, sensor_type);
  }

  // LEFT_SIDE_ZONE:
  BOUNDING_BOX_T left_side_zone;
  left_side_zone.ax = -1.0 * robot_length_ / 2.0;  // from the back of the robot
  left_side_zone.ay = robot_width_ / 2.0;
  left_side_zone.bx = robot_length_ / 2.0;         // to the front of the robot
  left_side_zone.by = left_side_zone.ay + side_zone_range_;
  if( IsInBoundingBox(left_side_zone, x, y) )
  {
    // point is in the box
    sensor_summary_.Increment(LEFT_SIDE_ZONE, sensor_type);
  }

  // RIGHT_SIDE_ZONE:
  BOUNDING_BOX_T right_side_zone;
  right_side_zone.bx = robot_length_ / 2.0;
  right_side_zone.by = -1.0 * robot_width_ / 2.0;
  right_side_zone.ax = -1.0 * robot_length_ / 2.0;
  right_side_zone.ay = right_side_zone.by - side_zone_range_;
  if( IsInBoundingBox(right_side_zone, x, y) )
  {
    // point is in the box
    sensor_summary_.Increment(RIGHT_SIDE_ZONE, sensor_type);
  }

  // LEFT_FRONT_SIDE_ZONE:
  BOUNDING_BOX_T left_front_side_zone;
  left_front_side_zone.ax = robot_length_ / 2.0;
  left_front_side_zone.ay = front_zone_width_;
  left_front_side_zone.bx = left_front_side_zone.ax + front_zone_range;
  left_front_side_zone.by = front_zone_width_ + front_side_zone_width_;
  if( IsInBoundingBox(left_front_side_zone, x, y) )
  {
    // point is in the box
    sensor_summary_.Increment(LEFT_FRONT_SIDE_ZONE, sensor_type);
  }

  // RIGHT_FRONT_SIDE_ZONE
  BOUNDING_BOX_T right_front_side_zone;
  right_front_side_zone.ax = robot_length_ / 2.0;
  right_front_side_zone.ay = -1.0 * (front_zone_width_ + front_side_zone_width_);
  right_front_side_zone.bx = right_front_side_zone.ax + front_zone_range;
  right_front_side_zone.by = -1.0 * front_zone_width_;
  if( IsInBoundingBox(right_front_side_zone, x, y) )
  {
    // point is in the box
    sensor_summary_.Increment(RIGHT_FRONT_SIDE_ZONE, sensor_type);
  }
 
}

void SafetyControl::DisplayCollisionMarker(int zone)
{
  // Display markers to indicate when robot is at risk of collision (or cliff, etc.)
  // For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

  // ROS_INFO("DBG: DisplayCollisionMarker called");
  // printf ("DBG DisplayCollisionMarker called for zone %d\n", zone);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();

  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "safety_control";
  marker.id = zone; // We use the zone number to make the id unique

  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f; 
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.pose.position.z = 0.02; // shift up to ground plane
  marker.scale.z = 0.04;  // fixed marker height
  marker.lifetime = ros::Duration(1.0); // seconds

  switch( zone ) 
  {
	  case LEFT_FRONT_ZONE:
      marker.scale.x = front_zone_range_;
      marker.scale.y = front_zone_width_;
      marker.pose.position.x = (robot_length_ + front_zone_range_) / 2.0;
      marker.pose.position.y = front_zone_width_ / 2.0;
      break;

	  case RIGHT_FRONT_ZONE:
      marker.scale.x = front_zone_range_;
      marker.scale.y = front_zone_width_;
      marker.pose.position.x = (robot_length_ + front_zone_range_) / 2.0;
      marker.pose.position.y = -1.0 * front_zone_width_ / 2.0;
      break;

    case LEFT_REAR_ZONE:
      marker.scale.x = rear_zone_range_;
      marker.scale.y = rear_zone_width_;
      marker.pose.position.x = -1.0 * (robot_length_ + rear_zone_range_) / 2.0;
      marker.pose.position.y = rear_zone_width_ / 2.0;
      break;

    case RIGHT_REAR_ZONE:
      marker.scale.x = rear_zone_range_;
      marker.scale.y = rear_zone_width_;
      marker.pose.position.x = -1.0 * (robot_length_ + rear_zone_range_) / 2.0;
      marker.pose.position.y = -1.0 * rear_zone_width_ / 2.0;
      break;

    case LEFT_SIDE_ZONE:
      marker.scale.x = robot_length_ ;
      marker.scale.y = side_zone_range_; 
      marker.pose.position.x = 0.0; // sides are aligned with center of robot
      marker.pose.position.y = (robot_width_ + side_zone_range_) / 2.0;
      break;

    case RIGHT_SIDE_ZONE:
      marker.scale.x = robot_length_ ;
      marker.scale.y = side_zone_range_; 
      marker.pose.position.x = 0.0; // sides are aligned with center of robot
      marker.pose.position.y = -1.0 * (robot_width_ + side_zone_range_) / 2.0;
      break;


	  case LEFT_FRONT_SIDE_ZONE:
      marker.scale.x = front_zone_range_;
      marker.scale.y = front_side_zone_width_;
      marker.pose.position.x = (robot_length_ + front_zone_range_) / 2.0;
      marker.pose.position.y = front_zone_width_ + (front_side_zone_width_ / 2.0);

      marker.color.r = 1.0f;
      marker.color.g = 1.0f; 
      marker.color.b = 0.0f;

      break;

	  case RIGHT_FRONT_SIDE_ZONE:
      marker.scale.x = front_zone_range_;
      marker.scale.y = front_side_zone_width_;
      marker.pose.position.x = (robot_length_ + front_zone_range_) / 2.0;
      marker.pose.position.y = -1.0 * (front_zone_width_ + (front_side_zone_width_ / 2.0));

      marker.color.r = 1.0f;
      marker.color.g = 1.0f; 
      marker.color.b = 0.0f;

      break;


	  default:
      ROS_ERROR("SafetyControl: Bad Zone!");
      return; // don't publish anyting 
      break;
  }
  //ROS_INFO("DBG: Publishing Marker");
  marker_pub_.publish(marker);

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ROS_INFO("SafetyControl: Initializing... ");
  ros::init(argc, argv, "safety_control");

  ros::NodeHandle n;
  //ros::NodeHandle n("~");  
  SafetyControl safety_control(n);
  ros::spin();
  return 0;

}


