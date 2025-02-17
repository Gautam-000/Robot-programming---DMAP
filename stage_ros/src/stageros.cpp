/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>

// libstage
#include <stage.hh>

// ROS
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

#define USAGE "stageros <worldfile>"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"
#define POSE "cmd_pose"

class StageNode
{
private:
    ros::NodeHandle n_;
    boost::mutex msg_lock;
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;

    struct StageRobot
    {
        Stg::ModelPosition* positionmodel;
        std::vector<Stg::ModelRanger *> lasermodels;
        ros::Publisher odom_pub;
        ros::Publisher ground_truth_pub;
        std::vector<ros::Publisher> laser_pubs;
        ros::Subscriber cmdvel_sub;
        ros::Subscriber pose_sub;
    };

    std::vector<StageRobot const *> robotmodels_;
    std::vector<Stg::Pose> initial_poses;
    ros::ServiceServer reset_srv_;
    ros::Publisher clock_pub_;
    bool use_model_names;
    tf::TransformBroadcaster tf;
    ros::Time sim_time;
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;
    std::vector<Stg::Pose> base_last_globalpos;
    ros::Time base_last_globalpos_time;

    static void ghfunc(Stg::Model* mod, StageNode* node)
    {
        if (dynamic_cast<Stg::ModelRanger *>(mod))
            node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
        if (auto p = dynamic_cast<Stg::ModelPosition *>(mod)) {
            node->positionmodels.push_back(p);
            node->initial_poses.push_back(p->GetGlobalPose());
        }
    }

    static bool s_update(Stg::World* world, StageNode* node)
    {
        node->WorldCallback();
        return false;
    }

    // generate unique for multiple robot (e.g. robot_0, robot_1)
    const char *mapName(const char *name, size_t robotID, Stg::Model* mod) const
    {
        if ((positionmodels.size() > 1) || use_model_names) {
            static char buf[100];
            std::size_t found = std::string(mod->Token()).find(":");
            if ((found==std::string::npos) && use_model_names) {
                snprintf(buf, sizeof(buf), "/%s/%s", mod->Token(), name);
            } else {
                snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
            }
            return buf;
        }
        return name;
    }

    // generate unique for multiple sensor (e.g. base_scan_0, base_scan_1) 
    const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const
    {
        if ((positionmodels.size() > 1 ) || use_model_names) {
            static char buf[100];
            std::size_t found = std::string(mod->Token()).find(":");
            if ((found==std::string::npos) && use_model_names) {
                snprintf(buf, sizeof(buf), "/%s/%s_%u", mod->Token(), name, (unsigned int)deviceID);
            } else {
                snprintf(buf, sizeof(buf), "/robot_%u/%s_%u", (unsigned int)robotID, name, (unsigned int)deviceID);
            }
            return buf;
        }
        static char buf[100];
        snprintf(buf, sizeof(buf), "/%s_%u", name, (unsigned int)deviceID);
        return buf;
    }


public:
    StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names)
        : use_model_names(use_model_names), sim_time(0), base_last_cmd(0)
    {
        struct stat s;
        if(stat(fname, &s) != 0) {
            ROS_FATAL("World file %s not found", fname);
            ROS_BREAK();
        }

        double t;
        ros::NodeHandle local_nh("~");
        local_nh.param("base_watchdog_timeout", t, 0.2);
        base_watchdog_timeout.fromSec(t);

        Stg::Init(&argc, &argv);
        world = gui ? new Stg::WorldGui(600, 400, "Stage (ROS)") : new Stg::World();
        world->Load(fname);
        world->AddUpdateCallback((Stg::world_callback_t)s_update, this);
        world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
    }

    ~StageNode() {
        for (auto r : robotmodels_) delete r;
    }

    int SubscribeModels() {
        n_.setParam("/use_sim_time", true);
        for (size_t r = 0; r < positionmodels.size(); r++) {
            StageRobot* new_robot = new StageRobot();
            new_robot->positionmodel = positionmodels[r];
            new_robot->positionmodel->Subscribe();

            for (auto lasermodel : lasermodels) {
                if (lasermodel->Parent() == new_robot->positionmodel) {
                    new_robot->lasermodels.push_back(lasermodel);
                    lasermodel->Subscribe();
                }
            }

            new_robot->odom_pub = n_.advertise<nav_msgs::Odometry>(
                mapName(ODOM, r, new_robot->positionmodel), 10);
            new_robot->ground_truth_pub = n_.advertise<nav_msgs::Odometry>(
                mapName(BASE_POSE_GROUND_TRUTH, r, new_robot->positionmodel), 10);
            new_robot->cmdvel_sub = n_.subscribe<geometry_msgs::Twist>(
                mapName(CMD_VEL, r, new_robot->positionmodel), 10,
                boost::bind(&StageNode::cmdvelReceived, this, r, _1));
            new_robot->pose_sub = n_.subscribe<geometry_msgs::Pose>(
                POSE, 10, boost::bind(&StageNode::poseReceived, this, r, _1));

            for (size_t s = 0; s < new_robot->lasermodels.size(); ++s) {
                if (new_robot->lasermodels.size() == 1) {
                    new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(
                        mapName(BASE_SCAN, r, new_robot->positionmodel), 10));
                } else {
                    new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(
                        mapName(BASE_SCAN, r, s, new_robot->positionmodel), 10));
                }
            }
            robotmodels_.push_back(new_robot);
        }
        clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);
        reset_srv_ = n_.advertiseService("reset_positions", &StageNode::cb_reset_srv, this);
        return 0;
    }

    // command velocity
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg) {
        boost::mutex::scoped_lock lock(msg_lock);
        positionmodels[idx]->SetSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
        base_last_cmd = sim_time;
    }

    // set pose
    void poseReceived(int idx, const boost::shared_ptr<geometry_msgs::Pose const>& msg) {
        boost::mutex::scoped_lock lock(msg_lock);
        Stg::Pose pose;
        tf::Matrix3x3 m(tf::Quaternion(msg->orientation.x, msg->orientation.y, 
                                     msg->orientation.z, msg->orientation.w));
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose.x = msg->position.x;
        pose.y = msg->position.y;
        pose.z = msg->position.z;
        pose.a = yaw;
        positionmodels[idx]->SetPose(pose);
    }

    // ros service for reset simulation
    bool cb_reset_srv(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        for (size_t r = 0; r < positionmodels.size(); r++) {
            positionmodels[r]->SetPose(initial_poses[r]);
            positionmodels[r]->SetStall(false);
        }
        return true;
    }

    void WorldCallback() {
        boost::mutex::scoped_lock lock(msg_lock);
        sim_time.fromSec(world->SimTimeNow() / 1e6);

        //  Moved when button pressed
        if((base_watchdog_timeout.toSec() > 0.0) && 
            ((sim_time - base_last_cmd) >= base_watchdog_timeout))
        {
            for (size_t r = 0; r < positionmodels.size(); r++)
            positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
        }

        // Process laser data
        for (size_t r = 0; r < robotmodels_.size(); ++r) {
            StageRobot const * robot = robotmodels_[r];
            for (size_t s = 0; s < robot->lasermodels.size(); ++s) {
                auto lasermodel = robot->lasermodels[s];
                const auto& sensors = lasermodel->GetSensors();
                if(sensors.empty()) continue;

                sensor_msgs::LaserScan msg;
                const auto& sensor = sensors[0];
                msg.angle_min = -sensor.fov/2.0;
                msg.angle_max = +sensor.fov/2.0;
                msg.angle_increment = sensor.fov/(sensor.sample_count-1);
                msg.range_min = sensor.range.min;
                msg.range_max = sensor.range.max;
                msg.ranges.assign(sensor.ranges.begin(), sensor.ranges.end());
                msg.header.frame_id = robot->lasermodels.size() > 1 ? 
                    mapName("base_laser_link", r, s, robot->positionmodel) :
                    mapName("base_laser_link", r, robot->positionmodel);
                msg.header.stamp = sim_time;
                robot->laser_pubs[s].publish(msg);
            }

            // Odometry and ground truth
            nav_msgs::Odometry odom_msg;
            auto posemodel = robot->positionmodel;
            odom_msg.pose.pose.position.x = posemodel->est_pose.x;
            odom_msg.pose.pose.position.y = posemodel->est_pose.y;
            odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(posemodel->est_pose.a);
            auto vel = posemodel->GetVelocity();
            odom_msg.twist.twist.linear.x = vel.x;
            odom_msg.twist.twist.linear.y = vel.y;
            odom_msg.twist.twist.angular.z = vel.a;
            odom_msg.header.frame_id = mapName("odom", r, posemodel);
            odom_msg.header.stamp = sim_time;
            robot->odom_pub.publish(odom_msg);

            // Ground truth
            nav_msgs::Odometry gt_msg;
            Stg::Pose gpose = posemodel->GetGlobalPose();
            gt_msg.pose.pose.position.x = gpose.x;
            gt_msg.pose.pose.position.y = gpose.y;
            gt_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(gpose.a);
            gt_msg.header = odom_msg.header;
            robot->ground_truth_pub.publish(gt_msg);

            // TF transforms
            tf::Transform odom_tf(tf::createQuaternionFromYaw(posemodel->est_pose.a),
                                 tf::Vector3(posemodel->est_pose.x, posemodel->est_pose.y, 0));
            tf.sendTransform(tf::StampedTransform(odom_tf, sim_time,
                mapName("odom", r, posemodel), mapName("base_footprint", r, posemodel)));
        }

        // Publish clock
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = sim_time;
        clock_pub_.publish(clock_msg);
    }

    Stg::World* world;
};

int main(int argc, char** argv)
{
    if(argc < 2) { puts(USAGE); exit(-1); }
    ros::init(argc, argv, "stageros");
    
    bool gui = true, use_model_names = false;
    for(int i=0; i<argc-1; i++) {
        if(!strcmp(argv[i], "-g")) gui = false;
        if(!strcmp(argv[i], "-u")) use_model_names = true;
    }

    StageNode sn(argc-1, argv, gui, argv[argc-1], use_model_names);
    if(sn.SubscribeModels() != 0) exit(-1);
    
    boost::thread t(boost::bind(&ros::spin));
    sn.world->Start();
    
    ros::WallRate r(10.0);
    while(ros::ok() && !sn.world->TestQuit()) {
        if(gui) Fl::wait(r.expectedCycleTime().toSec());
        else { sn.world->UpdateAll(); r.sleep(); }
    }
    t.join();
    return 0;
}