#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <directed_exploration/mapprocessing/ogmap.h>
#include <directed_exploration/util.h>
#include <techx_msgs/LocalNavStatus.h>
#include <compass/compass.h>
#include <techx_msgs/NextWayPoint.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief Structure used to store frontier points together with their profit values
 *
 * This structure is used to store frontier points. Frontier point's location in the global fixed frame
 * is stored. The associated cost/profit value for the point is also stored. This cost/profit value is 
 * used to compare frontier points in deciding the next target point
 */
struct FrontierPoint   
{
	::mapprocessing::Vector2d p; /**< the position of the frontier point in global fixed frame */
	double profit;               /**< the profit of the frontier point */
};

/**
 * @brief The class that encapsulates all the data structures and associated algorithms that implement the 
 * the directed exploration strategy
 *
 * This class contains all the data structures and associated algorithms that implement the directed exploration strategy to move towards a goal avoiding
 */
class DE_Planner
{
public:
    DE_Planner();                                                    /**< The public constructor */
    virtual ~DE_Planner();                                           /**< The destructor */

private:    
    ros::NodeHandle* n;                                              /**< The public node handle th*/
    ros::NodeHandle* private_n;                                      
	
    tf::TransformListener* listener;                                 /**< listener used to get robot's position */

    ros::Subscriber map_sub;
    ros::Subscriber dobs_sub;
	ros::Subscriber costmap_sub;
	ros::Subscriber compas_sub;
	ros::Subscriber waypoint_sub;
	ros::Subscriber stopsignal_sub; 

	ros::Publisher donesignal_pub; /**< de_node publishes an empty message to notify nodes that it reached current goal */
    ros::Publisher vis_pub;
    ros::Publisher target_vis_pub;
    ros::Publisher goal_vis_pub;
	ros::Publisher navstatus_pub;

    ros::Timer timer;                               // timer to check robot status every 0.5 secs
    ros::Time mapStamp;
    ros::Time lastActiveTime;	
    ros::Time mapUpdateTime;
    ros::Rate* pollRate;

	nav_msgs::GridCells local_costmap;        /**< local cost map data from costmap_2d */

    mapprocessing::OgMap* map;

    MoveBaseClient* ac;	
    mapprocessing::Vector2d mapRobotPos;
    mapprocessing::Vector2d robotStartingPos;
    mapprocessing::Vector2d robotCurrPos;           // robot's last active position, curr is a misleading term, but keeping for now, TODO : change this during code cleaning
    mapprocessing::Vector2d pathHalfPoint;

    std::string robot_frame;
    std::string fixed_frame;
    std::string map_topic;
    std::string move_base_node;
    std::string make_plan_service;
		
    double range, bearing;  // bearing is given in angles
	volatile bool gotRobotOrientation;
	double robots_bearing_wrt_north;
	bool bearing_wrt_north;
	
    double goalX;
    double goalY;
    double currentTargetX;
    double currentTargetY;	
	bool goingToFinalGoal;
	volatile bool isStuck;
	double stuck_timeout_threshold;
	volatile bool goingToSafePlace;
    bool goalReached;
    volatile bool running_;
    volatile bool targetActive;                     // targetActive is true when ac->sendGoal is called	
	bool useOldMap;
	volatile bool waitingForWayPoint;
	
    double rangeMax;
    double obs_inflation_dis;                      // the inflation distance used by inhouse planner to calculate paths
    double frontier_clearance_dis;                 // the minimum clearance distance a frontier point should have from an obstacle
    double plan_goal_tolerance;                    // the tolerance for the planner to relax constriants when original goal is obstructed
	double max_distance_between_waypoints;
	double target_reach_tolerance;
	double intermediate_target_reach_tolerance;
	double max_distance_of_plan;
	double min_distance_to_frontier;
	double max_distance_to_frontier;

	techx_msgs::LocalNavStatus nav_status; /**< stores the status of the de_planner */

    std::vector< FrontierPoint >* frontierList;   
    boost::shared_ptr< boost::circular_buffer< mapprocessing::Vector2d> > assignedTargets;
	
    boost::shared_ptr< boost::thread > controllerThread;
	boost::shared_ptr< boost::thread > statusPubThread;
    boost::mutex decision_mutex;
	boost::mutex costmap_mutex;
	boost::mutex navstatus_mutex;
	
    // callbacks
	void ogmap_cb(const nav_msgs::OccupancyGrid::ConstPtr & ogmap);
    //void dobs_cb(const directed_exploration::DynamicObstacleCommandData::ConstPtr & cmdData);
	void costmap_cb(const nav_msgs::GridCells::ConstPtr & costmap);
	void compas_cb(const compass::compass::ConstPtr & compas_data);
	void next_wp_cb(const techx_msgs::NextWayPoint::ConstPtr & wp_data);
	void stop_signal_cb(const std_msgs::Empty::ConstPtr & msg);
	
    // utility functions
    bool setMoveBaseGoal(move_base_msgs::MoveBaseGoal & goal, geometry_msgs::Pose & start, 
						 mapprocessing::Vector2d & tgt, mapprocessing::Vector2d & prevPt, double planner_tolerance);   // prevPt is not used
    void updateMap(const nav_msgs::OccupancyGrid & ogmap);      
    void rotate360();	
    void checkRobotStatus();
    void robotPosCheck(const ros::TimerEvent& event);
    bool targetAlreadyAssigned(::mapprocessing::Vector2d & tp);		
    bool veryCloseToRobot(::mapprocessing::Vector2d & tgt, ::mapprocessing::Vector2d & robotPos);
    visualization_msgs::Marker generateVisMarker(std::string & frameID, std::string & ns, int mID, int type, int action, 
						 mapprocessing::Vector2d pos, double scale, std_msgs::ColorRGBA & color, bool persistent = true);
    geometry_msgs::Pose getRobotCurrentPose();
    bool getNextFrontierPoint(FrontierPoint & fp, mapprocessing::Vector2d & robotPos);
    bool findATraversablePointTowardsGoal(FrontierPoint & fp, move_base_msgs::MoveBaseGoal & goal, mapprocessing::Vector2d & robotPos, geometry_msgs::Pose & robot_curr_pose);
	/* returns the distance of the path, if the distance reaches the threshold, then store the index of the path that reached the threshold  */
    double getPathDistance(nav_msgs::Path & plan, double thresh, unsigned int & tgtIndex);  
	void waitForNextWayPoint();
	bool isGoalInMappedRegion(const CvPoint & goalP);
	bool isRobotStuck();
	void setStuck(bool stuck);
	void pubStatus();
	void setNavStatus(uint8_t status);
};
