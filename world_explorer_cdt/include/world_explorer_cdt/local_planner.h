#pragma once

#include <string>

// ROS stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// // ROS messages
#include <grid_map_msgs/GridMap.h>
#include <cdt_msgs/Graph.h>
#include <cdt_msgs/GraphNode.h>
#include <cdt_msgs/Frontiers.h>

// Eigen library
#include <Eigen/Dense>
// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

class LocalPlanner
{
    struct FrontierCost{
        double x_;
        double y_;
        double closest_visited_point_dist_;
        double cost_;
    };

private:
    // Members
    // TF listener
    tf::TransformListener tf_listener_;

    // Time to run RRT
    double planning_time_;

    // // Internals
    grid_map::GridMap traversability_;
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > sampled_points_;
    std::string traversability_layer_;

    // Planner
    std::shared_ptr<ompl::geometric::RRTstar> rrt_star_planner_;
    std::shared_ptr<ompl::base::ProblemDefinition> problem_definition_;

    // Stuff for selecting frontiers
    std::vector<geometry_msgs::Pose> visited_poses_;
    int num_visited_poses_;
    double FRONTIER_REPULSION_COEFF = 20.0;
    double FRONTIER_ORIENTATION_PENALTY = 5.0;

    // Utils
    bool processPlannerOutput(const ompl::base::PlannerStatus& solved, 
                                std::vector<Eigen::Vector2d>& route);
    bool isStateValid(const ompl::base::State *state);
    bool isPoseValid(const Eigen::Isometry3d& pose);
    double wrapAngle(double x);

public:
    // Constructor
    LocalPlanner();

    // Interfaces to set data
    void setMap(const grid_map::GridMap& map);
    void setPlanningTime(double planning_time){ planning_time_ = planning_time; };
    void setVisitedPositions(const std::vector<geometry_msgs::Pose>& nodes);

    // Main methods
    std::vector<Eigen::Vector2d> searchFrontiers(cdt_msgs::Frontiers frontiers, 
                                                 const double& robot_x, const double&  robot_y, const double&  robot_theta);
    
    bool planPath(const double& robot_x, const double& robot_y , const double& robot_theta,
                const Eigen::Vector2d& pose_goals,
                std::vector<Eigen::Vector2d>& route);


};