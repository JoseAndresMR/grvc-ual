//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <string>
#include <chrono>
#include <uav_abstraction_layer/backend_mavros_fw.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <uav_abstraction_layer/geographic_to_cartesian.h>
#include <uav_abstraction_layer/Param_float.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>

namespace grvc { namespace ual {

BackendMavrosFW::BackendMavrosFW()
    : Backend()
{
    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    // float position_th_param, orientation_th_param;
    // pnh.param<float>("position_th", position_th_param, 0.33);
    // pnh.param<float>("orientation_th", orientation_th_param, 0.65);
    // position_th_ = position_th_param*position_th_param;
    // orientation_th_ = 0.5*(1 - cos(orientation_th_param));

    ROS_INFO("BackendMavrosFW constructor with id %d",robot_id_);
    // ROS_INFO("BackendMavrosFW: thresholds = %f %f", position_th_, orientation_th_);

    // Init ros communications
    ros::NodeHandle nh;
    std::string mavros_ns = "mavros";
    std::string set_mode_srv = mavros_ns + "/set_mode";
    std::string arming_srv = mavros_ns + "/cmd/arming";
    std::string get_param_srv = mavros_ns + "/param/get";
    std::string set_param_srv = mavros_ns + "/param/set";
    std::string push_mission_srv = mavros_ns + "/mission/push";
    std::string clear_mission_srv = mavros_ns + "/mission/clear";
    std::string set_pose_topic = mavros_ns + "/setpoint_position/local";
    std::string set_pose_global_topic = mavros_ns + "/setpoint_raw/global";
    std::string set_vel_topic = mavros_ns + "/setpoint_velocity/cmd_vel";
    std::string pose_topic = mavros_ns + "/local_position/pose";
    std::string geo_pose_topic = mavros_ns + "/global_position/global";
#ifdef MAVROS_VERSION_BELOW_0_29_0
    std::string vel_topic_local = mavros_ns + "/local_position/velocity";
#else
    std::string vel_topic_local = mavros_ns + "/local_position/velocity_local";
#endif
    std::string vel_topic_body = mavros_ns + "/local_position/velocity_body";
    std::string state_topic = mavros_ns + "/state";
    std::string extended_state_topic = mavros_ns + "/extended_state";
    std::string reached_mission_topic = mavros_ns + "/mission/reached";
    std::string waypoints_mission_topic = mavros_ns + "/mission/waypoints";

    flight_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(set_mode_srv.c_str());
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(arming_srv.c_str());

    mavros_ref_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(set_pose_topic.c_str(), 1);
    mavros_ref_pose_global_pub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>(set_pose_global_topic.c_str(), 1);
    mavros_ref_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>(set_vel_topic.c_str(), 1);

    mavros_cur_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1, \
        [this](const geometry_msgs::PoseStamped::ConstPtr& _msg) {
            this->cur_pose_ = *_msg;
            this->mavros_has_pose_ = true;
    });
    mavros_cur_vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(vel_topic_local.c_str(), 1, \
        [this](const geometry_msgs::TwistStamped::ConstPtr& _msg) {
            this->cur_vel_ = *_msg;
            this->cur_vel_.header.frame_id = this->uav_home_frame_id_;
#ifdef MAVROS_VERSION_BELOW_0_29_0
            this->cur_vel_body_ = *_msg;
            this->cur_vel_body_.header.frame_id = this->uav_frame_id_;
#endif
    });
    mavros_cur_vel_body_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(vel_topic_body.c_str(), 1, \
        [this](const geometry_msgs::TwistStamped::ConstPtr& _msg) {
            this->cur_vel_body_ = *_msg;
            this->cur_vel_body_.header.frame_id = this->uav_frame_id_;
    });
    mavros_cur_geo_pose_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(geo_pose_topic.c_str(), 1, \
        [this](const sensor_msgs::NavSatFix::ConstPtr& _msg) {
            this->cur_geo_pose_ = *_msg;
            if (!this->mavros_has_geo_pose_) {
                if (_msg->position_covariance[0] < 1.2 && _msg->position_covariance[0] > 0 && _msg->header.seq > 100) {
                    this->mavros_has_geo_pose_ = true;
                    ROS_INFO("Has Geo Pose! %f",_msg->position_covariance[0]);
                }
            }
    });
    mavros_cur_state_sub_ = nh.subscribe<mavros_msgs::State>(state_topic.c_str(), 1, \
        [this](const mavros_msgs::State::ConstPtr& _msg) {
            this->mavros_state_ = *_msg;
    });
    mavros_cur_extended_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>(extended_state_topic.c_str(), 1, \
        [this](const mavros_msgs::ExtendedState::ConstPtr& _msg) {
            this->mavros_extended_state_ = *_msg;
    });

    mavros_reached_wp_sub_ = nh.subscribe<mavros_msgs::WaypointReached>(reached_mission_topic.c_str(), 1, \
        [this](const mavros_msgs::WaypointReached::ConstPtr& _msg) {
            this->mavros_reached_wp_ = _msg->wp_seq;
    });

    mavros_cur_mission_sub_ = nh.subscribe<mavros_msgs::WaypointList>(waypoints_mission_topic.c_str(), 1, \
        [this](const mavros_msgs::WaypointList::ConstPtr& _msg) {
            this->mavros_cur_mission_ = *_msg;
            mission_state_ = this->mavros_cur_mission_.current_seq;
    });

    // Wait until mavros is connected
    while (!mavros_state_.connected && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // setParam("MIS_DIST_WPS",1200);
    
    // TODO: Check this and solve frames issue
    initHomeFrame();
    // Thread publishing target pose at 10Hz for mission mode
    mission_thread_ = std::thread(&BackendMavrosFW::missionThreadLoop, this);

    // Client to push missions to mavros
    push_mission_client_ = nh.serviceClient<mavros_msgs::WaypointPush>(push_mission_srv.c_str());

    //Client to clear the mission on mavros
    clear_mission_client_ = nh.serviceClient<mavros_msgs::WaypointClear>(clear_mission_srv.c_str());
    
    // Client to set parameters from mavros
    set_param_client_ = nh.serviceClient<mavros_msgs::ParamSet>(set_param_srv.c_str());

    // Client to get parameters from mavros and required default values
    get_param_client_ = nh.serviceClient<mavros_msgs::ParamGet>(get_param_srv.c_str());
    // mavros_params_["MPC_XY_VEL_MAX"]   =   2.0;  // [m/s]   Default value
    // mavros_params_["MPC_Z_VEL_MAX_UP"] =   3.0;  // [m/s]   Default value
    // mavros_params_["MPC_Z_VEL_MAX_DN"] =   1.0;  // [m/s]   Default value
    // mavros_params_["MC_YAWRATE_MAX"]   = 200.0;  // [deg/s] Default value
    // mavros_params_["MPC_TKO_SPEED"]    =   1.5;  // [m/s]   Default value
    mavros_params_["NAV_DLL_ACT"]   =   1;  // [?]   Default value
    mavros_params_["MIS_DIST_1PS"]   =   900;  // [m]   Default value
    mavros_params_["MIS_DIST_WPS"]   =   900;  // [m]   Default value
    // Updating here is non-sense as service seems to be slow in waking up

    initMission();

    ROS_INFO("BackendMavrosFW %d running!",robot_id_);

}

BackendMavrosFW::~BackendMavrosFW() {
    if (mission_thread_.joinable()) { mission_thread_.join(); }
}

void BackendMavrosFW::missionThreadLoop(){

    ros::param::param<double>("~mavros_mission_rate", mission_thread_frequency_, 30.0);

    ros::Rate rate(mission_thread_frequency_);

    while (ros::ok()) {

        if(this->mavros_state_.mode == "AUTO.MISSION"){ 
                if (std::find(takeoff_wps_on_mission_.begin(), takeoff_wps_on_mission_.end(), mavros_cur_mission_.current_seq) != takeoff_wps_on_mission_.end()){
                    calling_takeoff = true;
                }
                else if (std::find(land_wps_on_mission_.begin(), land_wps_on_mission_.end(), mavros_cur_mission_.current_seq) != land_wps_on_mission_.end()){
                    calling_land = true;
                }
                else {
                    calling_takeoff = false;
                    calling_land = false;
                }
        }

        this->state_ = guessState();

        rate.sleep();
    }
}

Backend::State BackendMavrosFW::guessState() {
    // Sequentially checks allow state deduction
    if (!this->isReady()) { return UNINITIALIZED; }
    if (!this->mavros_state_.armed) { return LANDED_DISARMED; }
    if (this->mavros_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) { return LANDED_ARMED; }  // TODO(franreal): Use LANDED_STATE_IN_AIR instead?
    if (this->calling_takeoff) { return TAKING_OFF; }
    if (this->calling_land) { return LANDING; }
    if (this->mavros_state_.mode == "AUTO.MISSION") { return FLYING_AUTO; }
    return FLYING_MANUAL;
}

void BackendMavrosFW::initMission() {

    // ROS_INFO("Flag0");
    // clearMission();
    setFlightMode("AUTO.LAND");
    arm(false);
    arm(true);
    setParam("NAV_DLL_ACT",0);
    setParam("MIS_DIST_1WP",0);
    setParam("MIS_DIST_WPS",0);
}

void BackendMavrosFW::setFlightMode(const std::string& _flight_mode) {
    mavros_msgs::SetMode flight_mode_service;
    flight_mode_service.request.base_mode = 0;
    flight_mode_service.request.custom_mode = _flight_mode;
    // Set mode: unabortable?
    while (mavros_state_.mode != _flight_mode && ros::ok()) {
        if (!flight_mode_client_.call(flight_mode_service)) {
            ROS_ERROR("Error in set flight mode [%s] service calling!", _flight_mode.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.success ? "true" : "false");
#else
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("Trying to set [%s] mode; mavros_state_.mode = [%s]", _flight_mode.c_str(), mavros_state_.mode.c_str());
    }
}

void BackendMavrosFW::arm(const bool& _arm) {
    mavros_msgs::CommandBool arm_service;
    arm_service.request.value = _arm;

    // Set mode: unabortable?
    while (mavros_state_.armed != _arm && ros::ok()) {
        if (!arming_client_.call(arm_service)) {
            ROS_ERROR("Error in [%s] service calling!", _arm ? "arming" : "disarming");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set [%s] response.success = %s", _arm ? "armed" : "disarmed", \
            arm_service.response.success ? "true" : "false");
#else
        // ROS_INFO("Set [%s] response.success = %s", _arm ? "armed" : "disarmed", \
        //     arm_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("  Trying to [%s]; mavros_state_.armed = [%s]", _arm ? "arm" : "disarm", mavros_state_.armed ? "true" : "false");
    }
}

void BackendMavrosFW::recoverFromManual() {
    if (!mavros_state_.armed || mavros_extended_state_.landed_state != 
        mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR) {
        ROS_WARN("Unable to recover from manual mode (not flying!)");
        return;
    }

    if (mavros_state_.mode != "POSCTL" &&
        mavros_state_.mode != "ALTCTL" &&
        mavros_state_.mode != "STABILIZED") {
        ROS_WARN("Unable to recover from manual mode (not in manual!)");
        return;
    }

    // Set mode to OFFBOARD and state to FLYING
    ref_pose_ = cur_pose_;
    control_mode_ = eControlMode::LOCAL_POSE;
    setFlightMode("OFFBOARD");
    ROS_INFO("Recovered from manual mode!");
}

void BackendMavrosFW::setHome(bool set_z) {
    double z_offset = set_z ? cur_pose_.pose.position.z : 0.0;
    local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x, \
        cur_pose_.pose.position.y, z_offset);
}

void BackendMavrosFW::takeOff(double _height) {
    if (_height < 0.0) {
        ROS_ERROR("Takeoff height must be positive!");
        return;
    }

    tf2::Quaternion quat(cur_pose_.pose.orientation.x,cur_pose_.pose.orientation.y,cur_pose_.pose.orientation.z,cur_pose_.pose.orientation.w);
    quat.normalize();
    double yaw = tf2::getYaw(quat);

    std::vector<uav_abstraction_layer::WaypointSet> new_mission;

    uav_abstraction_layer::WaypointSet takeoff_waypoint_set;

    takeoff_waypoint_set.type = WaypointSetType::TAKEOFF_AUX;
    uav_abstraction_layer::Param_float param; param.name = "aux_distance"; param.value = 50.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "aux_angle"; param.value = 0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "aux_height"; param.value = 5;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "minimum_pitch"; param.value = 0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "yaw_angle"; param.value = yaw;
    takeoff_waypoint_set.params.push_back( param );
    new_mission.push_back(takeoff_waypoint_set);

    uav_abstraction_layer::WaypointSet loiter_waypoint_set;
    loiter_waypoint_set.type = WaypointSetType::LOITER_UNLIMITED;
    geometry_msgs::PoseStamped loiter_posestamped;
    loiter_posestamped.pose.position.x = pose().pose.position.x;
    loiter_posestamped.pose.position.y = pose().pose.position.y;
    loiter_posestamped.pose.position.z = _height;
    loiter_waypoint_set.posestamped_list.push_back(loiter_posestamped);
    param.name = "radius"; param.value = 20;
    loiter_waypoint_set.params.push_back( param );
    param.name = "yaw_angle"; param.value = 0;
    loiter_waypoint_set.params.push_back( param );
    new_mission.push_back(loiter_waypoint_set);

    setMission(new_mission);

    ROS_INFO("Flying!");
    calling_takeoff = false;

    // Update state right now!
    this->state_ = guessState();

}

void BackendMavrosFW::land() {

    std::vector<uav_abstraction_layer::WaypointSet> new_mission;

    uav_abstraction_layer::WaypointSet takeoff_waypoint_set;

    tf2::Quaternion quat(cur_pose_.pose.orientation.x,cur_pose_.pose.orientation.y,cur_pose_.pose.orientation.z,cur_pose_.pose.orientation.w);
    quat.normalize();
    double yaw = tf2::getYaw(quat);

    takeoff_waypoint_set.type = WaypointSetType::LAND_AUX;
    geometry_msgs::PoseStamped land_posestamped;
    land_posestamped.pose.position.x = cur_pose_.pose.position.x;
    land_posestamped.pose.position.y = cur_pose_.pose.position.y;
    land_posestamped.pose.position.z = 0.0;
    takeoff_waypoint_set.posestamped_list.push_back(land_posestamped);
    uav_abstraction_layer::Param_float param; param.name = "loit_heading"; param.value = 0.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "loit_radius"; param.value = 50.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "loit_forward_moving"; param.value = 0.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "precision_mode"; param.value = 0.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "abort_alt"; param.value = 0.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "yaw_angle"; param.value = 0.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "aux_distance"; param.value = 120.0;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "aux_angle"; param.value = yaw;
    takeoff_waypoint_set.params.push_back( param );
    param.name = "aux_height"; param.value = 10.0;
    takeoff_waypoint_set.params.push_back( param );
    new_mission.push_back(takeoff_waypoint_set);

    setMission(new_mission);

}

void BackendMavrosFW::setVelocity(const Velocity& _vel) {

    ROS_WARN("setPose command is not supported by the Mavros Fixed Wing backend!");

}

bool BackendMavrosFW::isReady() const {
    if (ros::param::has("~map_origin_geo")) {
        return mavros_has_geo_pose_;
    } else {
        return mavros_has_pose_ && (fabs(this->cur_pose_.pose.position.y) > 1e-8);  // Means the filter has converged!
    }
}

void BackendMavrosFW::setPose(const geometry_msgs::PoseStamped& _world) {

    ROS_WARN("setPose command is not supported by the Mavros Fixed Wing backend!");

}

// TODO: Move from here?
// struct PurePursuitOutput {
//     geometry_msgs::Point next;
//     float t_lookahead;
// };

// // TODO: Move from here?
// PurePursuitOutput PurePursuit(geometry_msgs::Point _current, geometry_msgs::Point _initial, geometry_msgs::Point _final, float _lookahead) {
//     PurePursuitOutput out;
//     return out;
// }

void BackendMavrosFW::goToWaypoint(const Waypoint& _world) {

    std::vector<uav_abstraction_layer::WaypointSet> new_mission;

    uav_abstraction_layer::WaypointSet pass_waypoint_set;
    pass_waypoint_set.type = WaypointSetType::PASS;
    geometry_msgs::PoseStamped pass_posestamped;
    pass_posestamped.pose.position.x = _world.pose.position.x;
    pass_posestamped.pose.position.y = _world.pose.position.y;
    pass_posestamped.pose.position.z = _world.pose.position.z;
    pass_waypoint_set.posestamped_list.push_back(pass_posestamped);
    uav_abstraction_layer::Param_float param;
    param.name = "acceptance_radius"; param.value = 1.0;
    pass_waypoint_set.params.push_back( param );
    param.name = "orbit_distance"; param.value = 50.0;
    pass_waypoint_set.params.push_back( param );
    param.name = "yaw_angle"; param.value = std::nan("1.0");
    pass_waypoint_set.params.push_back( param );
    new_mission.push_back(pass_waypoint_set);

    uav_abstraction_layer::WaypointSet loiter_waypoint_set;
    loiter_waypoint_set.type = WaypointSetType::LOITER_UNLIMITED;
    geometry_msgs::PoseStamped loiter_posestamped;
    loiter_posestamped.pose.position.x = _world.pose.position.x;
    loiter_posestamped.pose.position.y = _world.pose.position.y;
    loiter_posestamped.pose.position.z = _world.pose.position.z;
    loiter_waypoint_set.posestamped_list.push_back(loiter_posestamped);
    // uav_abstraction_layer::Param_float param;
    param.name = "radius"; param.value = 40.0;
    loiter_waypoint_set.params.push_back( param );
    param.name = "yaw_angle"; param.value = std::nan("1.0");
    loiter_waypoint_set.params.push_back( param );
    new_mission.push_back(loiter_waypoint_set);

    setMission(new_mission);
    
}

void BackendMavrosFW::goToWaypointGeo(const WaypointGeo& _wp) {
    control_mode_ = eControlMode::GLOBAL_POSE; // Control in position
    
    ref_pose_global_.latitude = _wp.latitude;
    ref_pose_global_.longitude = _wp.longitude;
    ref_pose_global_.altitude = _wp.altitude;

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        ref_pose_ = cur_pose_;
    }
}

void BackendMavrosFW::setMission(const std::vector<uav_abstraction_layer::WaypointSet>& _waypoint_set_list) {

    mavros_msgs::WaypointList new_mission;
    takeoff_wps_on_mission_ = std::vector<int>();
    land_wps_on_mission_ = std::vector<int>();

    for (std::vector<int>::size_type i = 0; i != _waypoint_set_list.size(); i++) {
    // for ( auto &waypoint_set : _waypoint_set_list ) {
        uav_abstraction_layer::WaypointSet waypoint_set = _waypoint_set_list[i];

        if (waypoint_set.type == WaypointSetType::TAKEOFF_POSE || waypoint_set.type == WaypointSetType::TAKEOFF_AUX) { addTakeOffWp(new_mission,waypoint_set,i); }
        else if (waypoint_set.type == WaypointSetType::PASS) { addPassWpList(new_mission,waypoint_set,i); }
        else if (waypoint_set.type == WaypointSetType::LOITER_UNLIMITED || waypoint_set.type == WaypointSetType::LOITER_TURNS ||
                 waypoint_set.type == WaypointSetType::LOITER_TIME || waypoint_set.type == WaypointSetType::LOITER_HEIGHT) { addLoiterWpList(new_mission,waypoint_set,i); }
        else if (waypoint_set.type == WaypointSetType::LAND_POSE || waypoint_set.type == WaypointSetType::LAND_AUX) { addLandWpList(new_mission,waypoint_set,i); }
        else {ROS_ERROR("Error in [%d]-th waypoint set, field type is not correct!", static_cast<int>(i));}
    }

    clearMission();
    pushMission(new_mission);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    arm(true);
    setFlightMode("AUTO.MISSION");

}

/*void BackendMavrosFW::trackPath(const WaypointList &_path) {
    // TODO: basic imlementation, ideally different from a stack of gotos
}*/

Pose BackendMavrosFW::pose() {
        Pose out;

        out.pose.position.x = cur_pose_.pose.position.x + local_start_pos_[0];
        out.pose.position.y = cur_pose_.pose.position.y + local_start_pos_[1];
        out.pose.position.z = cur_pose_.pose.position.z + local_start_pos_[2];
        out.pose.orientation = cur_pose_.pose.orientation;

        if (pose_frame_id_ == "") {
            // Default: local pose
            out.header.frame_id = uav_home_frame_id_;
        }
        else {
            // Publish pose in different frame
            Pose aux = out;
            geometry_msgs::TransformStamped transformToPoseFrame;
            std::string pose_frame_id_map = "inv_" + pose_frame_id_;

            if ( cached_transforms_.find(pose_frame_id_map) == cached_transforms_.end() ) {
                // inv_pose_frame_id_ not found in cached_transforms_
                tf2_ros::Buffer tfBuffer;
                tf2_ros::TransformListener tfListener(tfBuffer);
                transformToPoseFrame = tfBuffer.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
                cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
            } else {
                // found in cache
                transformToPoseFrame = cached_transforms_[pose_frame_id_map];
            }

            tf2::doTransform(aux, out, transformToPoseFrame);
            out.header.frame_id = pose_frame_id_;
        }

        out.header.stamp = cur_pose_.header.stamp;
        return out;
}

Velocity BackendMavrosFW::velocity() const {
    return cur_vel_;
}

Odometry BackendMavrosFW::odometry() const {
    Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = uav_home_frame_id_;
    odom.child_frame_id = uav_frame_id_;
    odom.pose.pose.position.x = cur_pose_.pose.position.x + local_start_pos_[0];
    odom.pose.pose.position.y = cur_pose_.pose.position.y + local_start_pos_[1];
    odom.pose.pose.position.z = cur_pose_.pose.position.z + local_start_pos_[2];
    odom.pose.pose.orientation = cur_pose_.pose.orientation;
    odom.twist.twist = cur_vel_body_.twist;

    return odom;
}

Transform BackendMavrosFW::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = uav_home_frame_id_;
    out.child_frame_id = uav_frame_id_;
    out.transform.translation.x = cur_pose_.pose.position.x + local_start_pos_[0];
    out.transform.translation.y = cur_pose_.pose.position.y + local_start_pos_[1];
    out.transform.translation.z = cur_pose_.pose.position.z + local_start_pos_[2];
    out.transform.rotation = cur_pose_.pose.orientation;
    return out;
}

bool BackendMavrosFW::referencePoseReached() {

    double position_min, position_mean, position_max;
    double orientation_min, orientation_mean, orientation_max;
    // if (!position_error_.get_stats(position_min, position_mean, position_max)) { return false; }
    // if (!orientation_error_.get_stats(orientation_min, orientation_mean, orientation_max)) { return false; }
    
    double position_diff = position_max - position_min;
    double orientation_diff = orientation_max - orientation_min;
    bool position_holds = (position_diff < position_th_) && (fabs(position_mean) < 0.5*position_th_);
    bool orientation_holds = (orientation_diff < orientation_th_) && (fabs(orientation_mean) < 0.5*orientation_th_);

    // if (position_holds && orientation_holds) {  // DEBUG
    //     ROS_INFO("position: %f < %f) && (%f < %f)", position_diff, position_th_, fabs(position_mean), 0.5*position_th_);
    //     ROS_INFO("orientation: %f < %f) && (%f < %f)", orientation_diff, orientation_th_, fabs(orientation_mean), 0.5*orientation_th_);
    //     ROS_INFO("Arrived!");
    // }

    return position_holds && orientation_holds;
}

void BackendMavrosFW::initHomeFrame() {

    local_start_pos_ << 0.0, 0.0, 0.0;

    // Get frame prefix from namespace
    std::string ns = ros::this_node::getNamespace();
    uav_frame_id_ = ns + "/base_link";
    uav_home_frame_id_ = ns + "/odom";
    while (uav_frame_id_[0]=='/') {
        uav_frame_id_.erase(0,1);
    }
    while (uav_home_frame_id_[0]=='/') {
        uav_home_frame_id_.erase(0,1);
    }
    std::string parent_frame;
    ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");
    
    std::vector<double> home_pose(3, 0.0);
    if (ros::param::has("~home_pose")) {
        ros::param::get("~home_pose",home_pose);
    }
    
    else if (ros::param::has("~map_origin_geo")) {
        ROS_WARN("Be careful, you should only use this mode with RTK GPS!");
        while (!this->mavros_has_geo_pose_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::vector<double> map_origin_geo(3, 0.0);
        ros::param::get("~map_origin_geo",map_origin_geo);
        geographic_msgs::GeoPoint origin_geo, actual_coordinate_geo;       // origin_geo_ defined on .h
        origin_geo.latitude = map_origin_geo[0];
        origin_geo.longitude = map_origin_geo[1];
        origin_geo.altitude = 0; //map_origin_geo[2];
        actual_coordinate_geo.latitude = cur_geo_pose_.latitude;
        actual_coordinate_geo.longitude = cur_geo_pose_.longitude;
        actual_coordinate_geo.altitude = 0; //cur_geo_pose_.altitude;
        if(map_origin_geo[0]==0 && map_origin_geo[1]==0) {
            ROS_WARN("Map origin is set to 0. Define map_origin_geo param by a vector in format [lat,lon,alt].");
        }
        geometry_msgs::Point32 map_origin_cartesian = geographic_to_cartesian (actual_coordinate_geo, origin_geo);

        home_pose[0] = map_origin_cartesian.x;
        home_pose[1] = map_origin_cartesian.y;
        home_pose[2] = map_origin_cartesian.z;
    }
    else {
        ROS_WARN("No home pose or map origin was defined. Home frame will be equal to map.");
    }

    if (ros::param::has("~map_origin_geo")) {

        std::vector<double> map_origin_geo(3, 0.0);
        ros::param::get("~map_origin_geo",map_origin_geo);
        origin_geo_.latitude = map_origin_geo[0];
        origin_geo_.longitude = map_origin_geo[1];
        origin_geo_.altitude = 0; //map_origin_geo[2];
        
    }

    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = parent_frame;
    static_transformStamped.child_frame_id = uav_home_frame_id_;
    static_transformStamped.transform.translation.x = home_pose[0];
    static_transformStamped.transform.translation.y = home_pose[1];
    static_transformStamped.transform.translation.z = home_pose[2];

    if(parent_frame == "map" || parent_frame == "") {
        static_transformStamped.transform.rotation.x = 0;
        static_transformStamped.transform.rotation.y = 0;
        static_transformStamped.transform.rotation.z = 0;
        static_transformStamped.transform.rotation.w = 1;
    }
    else {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transform_to_map;
        transform_to_map = tfBuffer.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
        static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
    }

    static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
    static_tf_broadcaster_->sendTransform(static_transformStamped);

}

double BackendMavrosFW::updateParam(const std::string& _param_id) {
    mavros_msgs::ParamGet get_param_service;
    get_param_service.request.param_id = _param_id;
    if (get_param_client_.call(get_param_service) && get_param_service.response.success) {
        mavros_params_[_param_id] = get_param_service.response.value.integer? 
            get_param_service.response.value.integer : get_param_service.response.value.real;
        ROS_INFO("Parameter [%s] value is [%f]", get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else if (mavros_params_.count(_param_id)) {
        ROS_ERROR("Error in get param [%s] service calling, leaving current value [%f]", 
            get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else {
        mavros_params_[_param_id] = 0.0;
        ROS_ERROR("Error in get param [%s] service calling, initializing it to zero", 
            get_param_service.request.param_id.c_str());
    }
    return mavros_params_[_param_id];
}

void BackendMavrosFW::setParam(const std::string& _param_id, const int& _param_value) {
    mavros_msgs::ParamSet set_param_service;
    set_param_service.request.param_id = _param_id;
    set_param_service.request.value.integer = _param_value;     // FIX FOR FLOAT
    set_param_service.request.value.real = 0.0;

    while (updateParam(_param_id) != _param_value && ros::ok()) {
        if (!set_param_client_.call(set_param_service)) {
            ROS_ERROR("Error in set param [%s] service calling!", _param_id.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
            set_param_service.response.success ? "true" : "false");
#else
        // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
        //     set_param_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("Trying to set [%s] param to [%10d]", _param_id.c_str(), _param_value);
    }
}

void BackendMavrosFW::pushMission(const mavros_msgs::WaypointList& _wp_list) {
    mavros_msgs::WaypointPush push_waypoint_service;
    push_waypoint_service.request.start_index = 0;
    push_waypoint_service.request.waypoints = _wp_list.waypoints;

    if (!push_mission_client_.call(push_waypoint_service)) {
        ROS_ERROR("Error in push mission service calling!");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
    ROS_INFO("Push mission response.success = %s", set_param_service.response.success ? "true" : "false");
#else
    // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
    //     set_param_service.response.mode_sent ? "true" : "false");
#endif
    ROS_INFO("Trying to push mission");
}

void BackendMavrosFW::clearMission() {
    mavros_msgs::WaypointClear clear_mission_service;

    if (!clear_mission_client_.call(clear_mission_service)) {
        ROS_ERROR("Error in clear mission service calling!");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
    ROS_INFO("Clear mission response.success = %s", clear_mission_service.response.success ? "true" : "false");
#else
    // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
    //     set_param_service.response.mode_sent ? "true" : "false");
#endif
    ROS_INFO("Trying to clear mission");
}


void BackendMavrosFW::addTakeOffWp(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::WaypointSet& _waypoint_set, const int& wp_set_index) {

    // std::cout<<"PoseStamped:: x: "<<_pose_list.pose.position.x<<" y: "<<_pose_list.pose.position.y<<" z: "<<_pose_list.pose.position.z<<std::endl;

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_set.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_set.params[i].name,_waypoint_set.params[i].value) );    }

    mavros_msgs::Waypoint wp;

    if (_waypoint_set.type == WaypointSetType::TAKEOFF_POSE) {
        
        if (_waypoint_set.posestamped_list.size() != 1){ ROS_ERROR("Error in [%d]-th waypoint set, posestamped list lenght is not 1!", wp_set_index); }
        
        wp = poseStampedtoGlobalWaypoint(_waypoint_set.posestamped_list[0]);
    }

    else if (_waypoint_set.type == WaypointSetType::TAKEOFF_AUX) {
        std::vector<std::string> required_aux_params { "aux_distance","aux_angle","aux_height" };
        checkParams(params_map, required_aux_params, wp_set_index);

        geometry_msgs::PoseStamped aux_pose = pose();
        aux_pose.pose.position.x += params_map["aux_distance"] * cos(params_map["aux_angle"]);
        aux_pose.pose.position.y += params_map["aux_distance"] * sin(params_map["aux_angle"]);
        aux_pose.pose.position.z += params_map["aux_height"];

        wp = poseStampedtoGlobalWaypoint(aux_pose);
    }

    wp.frame = 3;
    wp.command = 22;
    wp.is_current = true;
    wp.autocontinue = true;

    std::vector<std::string> required_params { "minimum_pitch","yaw_angle" };
    checkParams(params_map, required_params, wp_set_index);

    wp.param1 = params_map["minimum_pitch"];    // (if airspeed sensor present), desired pitch without sensor
    wp.param4 = params_map["yaw_angle"];        // (if magnetometer present), ignored without magnetometer. NaN for unchanged.

    _wp_list.waypoints.push_back(wp);
    takeoff_wps_on_mission_.push_back(_wp_list.waypoints.size()-1);

}

void BackendMavrosFW::addPassWpList(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::WaypointSet& _waypoint_set, const int& wp_set_index) {

    if (_waypoint_set.posestamped_list.size() == 0){ ROS_ERROR("Error in [%d]-th waypoint set, posestamped list is empty!", wp_set_index); }

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_set.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_set.params[i].name,_waypoint_set.params[i].value) );    }

    std::vector<std::string> required_params { "acceptance_radius","orbit_distance","yaw_angle" };
    checkParams(params_map, required_params, wp_set_index);

    for ( auto &_pose : _waypoint_set.posestamped_list ) {

        mavros_msgs::Waypoint wp;

        if (_waypoint_set.type == 2) { wp = poseStampedtoGlobalWaypoint(_pose); }

        wp.frame = 3;
        wp.command = 16;
        wp.is_current = false;
        wp.autocontinue = true;

        wp.param2 = params_map["acceptance_radius"];        // (if the sphere with this radius is hit, the waypoint counts as reached)
        wp.param3 = params_map["orbit_distance"];           // 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit,
                                                            // negative value for counter-clockwise orbit. Allows trajectory control.
        wp.param4 = params_map["yaw_angle"];                // Desired yaw angle at waypoint (rotary wing). NaN for unchanged.

        _wp_list.waypoints.push_back(wp);

    }
}

void BackendMavrosFW::addLoiterWpList(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::WaypointSet& _waypoint_set, const int& wp_set_index) {

    if (_waypoint_set.posestamped_list.size() == 0){ ROS_ERROR("Error in [%d]-th waypoint set, posestamped list is empty!", wp_set_index); }

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_set.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_set.params[i].name,_waypoint_set.params[i].value) );    }

    for ( auto &_pose : _waypoint_set.posestamped_list ) {

        mavros_msgs::Waypoint wp;
        wp = poseStampedtoGlobalWaypoint(_pose);
        wp.frame = 3;
        wp.is_current = false;
        wp.autocontinue = true;

        if (_waypoint_set.type == WaypointSetType::LOITER_UNLIMITED) {

            std::vector<std::string> required_params { "radius","yaw_angle" };
            checkParams(params_map, required_params, wp_set_index);

            wp.command = 17;
            wp.param3 = params_map["radius"];               // Radius around waypoint. If positive loiter clockwise, else counter-clockwise
            wp.param4 = params_map["yaw_angle"];            // NaN for unchanged.

        }
        else if (_waypoint_set.type == WaypointSetType::LOITER_TURNS) {

            std::vector<std::string> required_params { "turns","radius","forward_moving" };
            checkParams(params_map, required_params, wp_set_index);

            wp.command = 18;
            wp.param1 = params_map["turns"];                // Number of turns.
            wp.param3 = params_map["radius"];               // Radius around waypoint. If positive loiter clockwise, else counter-clockwise
            wp.param4 = params_map["forward_moving"];       // this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location.
                                                            // Else, this is desired yaw angle. NaN for unchanged.

        }
        else if (_waypoint_set.type == WaypointSetType::LOITER_TIME) {

            std::vector<std::string> required_params { "time","radius","forward_moving" };
            checkParams(params_map, required_params, wp_set_index);

            wp.command = 19;
            wp.param1 = params_map["time"];                 // 	Loiter time.
            wp.param3 = params_map["radius"];               // 	Radius around waypoint. If positive loiter clockwise, else counter-clockwise.
            wp.param4 = params_map["forward_moving"];       // this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location.
                                                            // Else, this is desired yaw angle. NaN for unchanged.

        }
        else if (_waypoint_set.type == WaypointSetType::LOITER_HEIGHT) {

            wp.command = 31;

            std::vector<std::string> required_params { "heading","radius","forward_moving" };
            checkParams(params_map, required_params, wp_set_index);

            wp.param1 = params_map["heading"];              // Heading Required (0 = False)
            wp.param2 = params_map["radius"];               // If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
            wp.param4 = params_map["forward_moving"];       // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location

        }

        _wp_list.waypoints.push_back(wp);
    }

}

void BackendMavrosFW::addLandWpList(mavros_msgs::WaypointList& _wp_list, const uav_abstraction_layer::WaypointSet& _waypoint_set, const int& wp_set_index) {

    std::map<std::string, float> params_map;
    for(std::vector<int>::size_type i = 0; i != _waypoint_set.params.size(); i++) {
        params_map.insert( std::pair<std::string,float>(_waypoint_set.params[i].name,_waypoint_set.params[i].value) );    }

    mavros_msgs::Waypoint wp1;
    wp1.frame = 2;
    wp1.command = 189;
    wp1.is_current = false;
    wp1.autocontinue = true;

    _wp_list.waypoints.push_back(wp1);
    land_wps_on_mission_.push_back(_wp_list.waypoints.size() -1 );

    mavros_msgs::Waypoint wp2;

    if (_waypoint_set.type == WaypointSetType::LAND_POSE) {

        if (_waypoint_set.posestamped_list.size() != 2){ ROS_ERROR("Error in [%d]-th waypoint, posestamped list length is not 2!", wp_set_index); }
        std::cout<<"PoseStamped:: x: "<<_waypoint_set.posestamped_list[1].pose.position.x<<" y: "<<_waypoint_set.posestamped_list[1].pose.position.y<<" z: "<<_waypoint_set.posestamped_list[1].pose.position.z<<std::endl;

        wp2 = poseStampedtoGlobalWaypoint(_waypoint_set.posestamped_list[1]);
        
    }

    else if (_waypoint_set.type == WaypointSetType::LAND_AUX) {

        if (_waypoint_set.posestamped_list.size() != 1){ ROS_ERROR("Error in [%d]-th waypoint, posestamped list length is not 2!", wp_set_index); }

        std::vector<std::string> required_aux_params { "aux_distance","aux_angle","aux_height" };
        checkParams(params_map, required_aux_params, wp_set_index);

        geometry_msgs::PoseStamped aux_pose = _waypoint_set.posestamped_list[0];
        aux_pose.pose.position.x += params_map["aux_distance"] * cos(params_map["aux_angle"]) + local_start_pos_[0];
        aux_pose.pose.position.y += params_map["aux_distance"] * sin(params_map["aux_angle"]) + local_start_pos_[1];
        aux_pose.pose.position.z += params_map["aux_height"];
        std::cout<<"PoseStamped:: x: "<<aux_pose.pose.position.x<<" y: "<<aux_pose.pose.position.y<<" z: "<<aux_pose.pose.position.z<<std::endl;

        wp2 = poseStampedtoGlobalWaypoint(aux_pose);

    }

    std::cout<<"PoseStamped:: x: "<<_waypoint_set.posestamped_list[0].pose.position.x<<" y: "<<_waypoint_set.posestamped_list[0].pose.position.y<<" z: "<<_waypoint_set.posestamped_list[0].pose.position.z<<std::endl;

    wp2.frame = 3;
    wp2.command = 31;
    wp2.is_current = false;
    wp2.autocontinue = true;
    wp2.param1 = params_map["loit_heading"];            	// Heading Required (0 = False)
    wp2.param2 = params_map["loit_radius"];                 // If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
    wp2.param4 = params_map["loit_forward_moving"];         // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location

    _wp_list.waypoints.push_back(wp2);
    land_wps_on_mission_.push_back(_wp_list.waypoints.size() -1 );

    mavros_msgs::Waypoint wp3;
    wp3 = poseStampedtoGlobalWaypoint(_waypoint_set.posestamped_list[0]);
    wp3.frame = 3;
    wp3.command = 21;
    wp3.is_current = false;
    wp3.autocontinue = true;

    std::vector<std::string> required_params { "abort_alt","precision_mode","yaw_angle" };
    checkParams(params_map, required_params, wp_set_index);

    wp3.param1 = params_map["abort_alt"];                   // Minimum target altitude if landing is aborted (0 = undefined/use system default).
    wp3.param2 = params_map["precision_mode"];              // Precision land mode.
    wp3.param4 = params_map["yaw_angle"];                   // NaN for unchanged.

    _wp_list.waypoints.push_back(wp3);
    land_wps_on_mission_.push_back(_wp_list.waypoints.size() -1 );

}

mavros_msgs::Waypoint BackendMavrosFW::poseStampedtoGlobalWaypoint(const geometry_msgs::PoseStamped& _actual_cartesian) {

    geometry_msgs::Point32 geo_point;
    geo_point.x = _actual_cartesian.pose.position.x;
    geo_point.y = _actual_cartesian.pose.position.y;
    geo_point.z = _actual_cartesian.pose.position.z;

    geographic_msgs::GeoPoint actual_geo = cartesian_to_geographic(geo_point, origin_geo_);

    mavros_msgs::Waypoint waypoint;
    waypoint.x_lat = actual_geo.latitude;
    waypoint.y_long = actual_geo.longitude;
    waypoint.z_alt = actual_geo.altitude;

    return waypoint;

}

void BackendMavrosFW::checkParams(const std::map<std::string, float>& existing_params_map, const std::vector<std::string>& required_params, const int& wp_set_index){
    for ( auto &_param :  required_params){
        if (existing_params_map.count(_param) == 0){
            ROS_ERROR("Warn in [%d]-th waypoint set, [%s] param not provided!", wp_set_index, _param.c_str());
        }
    }
}

}}	// namespace grvc::ual