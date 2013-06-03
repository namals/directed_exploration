#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <stdexcept>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <rotate_recovery/rotate_recovery.h>

#include <directed_exploration/de_planner.h>
#include <directed_exploration/mapprocessing/mapimage.h>
#include <directed_exploration/mapprocessing/ogmap.h>
#include <compass/compass.h>

#define EUCLIDEAN_DIST(X1,Y1, X2,Y2) sqrt( ((X1)-(X2))*((X1)-(X2)) + ((Y1)-(Y2))*((Y1)-(Y2)) )

#define PLAN_TOLERANCE_FOR_GOAL 0.1

bool compareFrontierPoints(FrontierPoint a, FrontierPoint b)
{
    return a.profit > b.profit;
}

DE_Planner::DE_Planner() : assignedTargets(new boost::circular_buffer<mapprocessing::Vector2d>(50))
{
    n = new ros::NodeHandle();

	// set the parameters
	ros::NodeHandle private_n("~");
	
	pollRate = new ros::Rate(10.0);

	if( !private_n.getParam("obs_inflation_dis", obs_inflation_dis) )
		obs_inflation_dis = 0.25;        // conservative estimates for P3AT
	if( !private_n.getParam("frontier_clearance_dis", frontier_clearance_dis) )
		frontier_clearance_dis = 0.35;   // conservative estimate for P3AT
	if( !private_n.getParam("plan_goal_tolerance", plan_goal_tolerance) )
		plan_goal_tolerance = 0.5;       // planner goal tolerance of 0.5 m in default setting
	if( !private_n.getParam("fixed_frame", fixed_frame) )
		fixed_frame = "/map";
	if( !private_n.getParam("robot_frame", robot_frame) )
		robot_frame = "/base_link";
	if( !private_n.getParam("move_base_node", move_base_node) )
		move_base_node = "/move_base";
	make_plan_service = move_base_node;
	make_plan_service.append("/make_plan");

	gotRobotOrientation = false;
	if( !private_n.getParam("bearing_wrt_north", bearing_wrt_north) )
		bearing_wrt_north = false;
	if( !private_n.getParam("max_distance_between_waypoints", max_distance_between_waypoints) )
		max_distance_between_waypoints = 10.0;
	if( !private_n.getParam("target_reach_tolerance", target_reach_tolerance) )
		target_reach_tolerance = 0.3;
	if( !private_n.getParam("max_distance_of_plan", max_distance_of_plan) )
		max_distance_of_plan = 3.0;   // set the maximum distance of a plan to 3 m
	std::string compass_topic;
	if( !private_n.getParam("compass_topic", compass_topic) )
		compass_topic = "/drrobot_compass_info";
	if( !private_n.getParam("min_distance_to_frontier", min_distance_to_frontier) )
		min_distance_to_frontier = 1.0;
	if( !private_n.getParam("max_distance_to_frontier", max_distance_to_frontier) )
		max_distance_to_frontier = 15.0;

	std::string local_cost_map_topic = move_base_node;
	local_cost_map_topic.append("/local_costmap/inflated_obstacles");

	// read goal information
	
	bool result;
	result = private_n.getParam("max_sensor_range", rangeMax);
	if(!result)
	{ ROS_ERROR("Max. Sensor range is not set, exiting..."); exit(0); }
	if( !private_n.getParam("stuck_timeout_threshold", stuck_timeout_threshold) )
	{
		stuck_timeout_threshold = 10.0;  // default is 10 seconds
	}

    listener = new tf::TransformListener();   
	do{
		costmap_sub = n->subscribe<nav_msgs::GridCells>(local_cost_map_topic, 1, &DE_Planner::costmap_cb, this);
		pollRate->sleep();
	}while(!costmap_sub);
	ROS_INFO("Subscribing to : %s", local_cost_map_topic.c_str());

	do
	{
		waypoint_sub = n->subscribe<techx_msgs::NextWayPoint>("next_wp", 1, &DE_Planner::next_wp_cb, this);
	}while( !waypoint_sub );
	
	do
	{
		stopsignal_sub = n->subscribe<std_msgs::Empty>("reached_gps_goal", 1, &DE_Planner::stop_signal_cb, this);
	}while( !stopsignal_sub );   

	donesignal_pub = n->advertise<std_msgs::Empty>("reached_map_goal", 1);
	navstatus_pub = n->advertise<techx_msgs::LocalNavStatus>("local_nav_status", 20);

	// visualizing goal
	vis_pub = n->advertise<visualization_msgs::MarkerArray>("frontier_markers", 0);
	target_vis_pub = n->advertise<visualization_msgs::Marker>("target_marker", 0);
	goal_vis_pub = n->advertise<visualization_msgs::Marker>("goal_marker", 0);		
	
	ROS_INFO("Waiting for planning service to come up");
	if( ros::service::waitForService(make_plan_service) ) // blocks
	{		
		ROS_INFO("Path planning service is up...");
	}	
	else
	{
		ROS_INFO("Path planning service could not be contacted");
		exit(0);
	}
	
	ac = new MoveBaseClient("move_base", true);
 	while( !(ac->waitForServer(ros::Duration(5.0))) )
	{
	 	ROS_INFO("Waiting for the move_base action server to come up");
	}
	
/******************************************************************************************************************
*******************************************************************************************************************/
// CODE FOR COMPAS INTEGRATION

	robots_bearing_wrt_north = 0.0;    // initialize robot's bearing to zero in case compas data is not used
	if(bearing_wrt_north)
	{
		do{
			compas_sub = n->subscribe<compass::compass>(compass_topic, 1000, &DE_Planner::compas_cb, this);	
		}while(!compas_sub);
		ROS_INFO("subscribed to compas");
	}
	else
		gotRobotOrientation = true;		
/******************************************************************************************************************
*******************************************************************************************************************/
	timer = n->createTimer(ros::Duration(0.05), &DE_Planner::robotPosCheck, this);  // check the position at 20Hz frequency

	frontierList = NULL;
	goalReached = false;   			
	currentTargetX = -100.0;
	currentTargetY = -100.0;
	map = NULL;
	mapRobotPos.x = 0.0;
	mapRobotPos.y = 0.0;
	mapStamp = ros::Time(0);

	robotCurrPos.x = 0.0;
	robotCurrPos.y = 0.0;
	lastActiveTime = ros::Time::now();

	nav_status.status = techx_msgs::LocalNavStatus::IDLE;

	// initialize the robot status thread	
	controllerThread = boost::shared_ptr<boost::thread>( new boost::thread(boost::bind(&DE_Planner::checkRobotStatus, this)) );	
	goingToFinalGoal = false;
	isStuck = false;
	goingToSafePlace = false;
	targetActive = false;

	// initialize the status thread
	statusPubThread = boost::shared_ptr<boost::thread>( new boost::thread(boost::bind(&DE_Planner::pubStatus, this)) );
}

DE_Planner::~DE_Planner()
{
    delete n;
    delete private_n;
    delete listener;
	delete frontierList;
	delete ac;
	running_ = false;	
}

void 
DE_Planner::ogmap_cb(const nav_msgs::OccupancyGrid::ConstPtr & ogmap)    // call back is doing nothing
{
}

void
DE_Planner::costmap_cb(const nav_msgs::GridCells::ConstPtr & costmap)
{
	costmap_mutex.lock();	
	local_costmap = *costmap;   // update the costmap info	
	costmap_mutex.unlock();
}

void
DE_Planner::compas_cb(const compass::compass::ConstPtr & compas_data)
{
	robots_bearing_wrt_north = compas_data->Yval/10.0;
	gotRobotOrientation = true;
	ROS_INFO("angle : %f", robots_bearing_wrt_north);
}

void
DE_Planner::updateMap(const nav_msgs::OccupancyGrid & ogmap)
{
	try
    {
		tf::StampedTransform transform;
     	listener->lookupTransform( fixed_frame, robot_frame, ros::Time(0), transform );

		mapStamp = ros::Time::now();
		mapRobotPos.x = transform.getOrigin().x();
		mapRobotPos.y = transform.getOrigin().y();

		// ---------------- get frontiers ------------------------
		int mapWidth = (ogmap).info.width;  // in x direction
		int mapHeight = (ogmap).info.height; // in y direction
		
		if(mapWidth == 0 || mapHeight == 0)
		{
			ROS_WARN("mapwidth : %d, mapheight : %d", mapWidth, mapHeight);
			if( frontierList != NULL )
			{
				delete frontierList;
				frontierList = NULL;
			}			
		}

		double mapresolution = (ogmap).info.resolution;
		int data_len = mapWidth*mapHeight;
		unsigned char* map_data = new unsigned char[data_len];
		for(int i = 0; i < data_len; i++)
		{
			if( (ogmap).data[i] >= 0 && (ogmap).data[i] < 50 )
				map_data[i] = 0;
			else if( (ogmap).data[i] >= 50 && (ogmap).data[i] <= 100 )
				map_data[i] = 254;
			else
				map_data[i] = 127;
		}	   
		if(map != NULL)
			delete map;

		map = new mapprocessing::OgMap( (ogmap).info.origin.position.x,
										(ogmap).info.origin.position.y,
										mapWidth, mapHeight, mapresolution,
										map_data, obs_inflation_dis, frontier_clearance_dis );		

		mapprocessing::Vector2d pos; pos.x = transform.getOrigin().x(); pos.y = transform.getOrigin().y();
		CvPoint rp = map->getPositionInPixelPoints(pos);
		unsigned int limit = static_cast<unsigned int>(ceil(10.0/map->getMetersPerCellX()));   // limit 10.0 is arbitrary
		map->getMapImage()->fillSmallHolesInFreeSpaceAroundRobot(rp,limit);
		ROS_INFO("filled small holes");
		
		ROS_INFO("adding inflation layer");
		costmap_mutex.lock();
		std::vector<mapprocessing::Vector2d> inflated_points;
		std::vector<geometry_msgs::Point>::iterator pit;
		for(pit = (local_costmap).cells.begin(); pit != (local_costmap).cells.end(); pit++)
		{
			mapprocessing::Vector2d v;
			v.x = (*pit).x; v.y = (*pit).y;
			inflated_points.push_back(v);
		}
		costmap_mutex.unlock();
		map->addInflatedObsLayer(inflated_points);				
		
		std::vector<CvPoint> frontiers;
		map->getMapImage()->getFrontiers(frontiers, 
										 static_cast<int>( ceil( frontier_clearance_dis/mapresolution) ) );		
		map->calculateShortestDistances(pos);

		if(frontierList != NULL)
			delete frontierList;
		frontierList = new std::vector<FrontierPoint>();

		visualization_msgs::MarkerArray marray;
		marray.markers.resize(frontiers.size());
		double bestProfit = -DBL_MAX;
		mapprocessing::Vector2d best_tp;
		double res = map->getMetersPerCellX();
		for(unsigned int i = 0; i < frontiers.size(); i++)
		{
		 	CvPoint dest = frontiers[i];
		 	double dis = map->getShortestDistanceToInPixels(dest);
		 	mapprocessing::Vector2d v = map->getPositionInMeters(dest.x, dest.y);
			if( dis == DBL_MAX )
		 	 	continue;
			double distance = dis*res;
			// filter out too close and too long frontiers from consideration
			if(distance <= min_distance_to_frontier || distance > max_distance_to_frontier ) 
			 	continue;			
			if( v.y < 1.0 ) // if the frontier is behind the robot, (y < 1.0 is behind as we reset map repeatedly)
				continue;

			// ---------- Display Markers ---------------------------
			std::string frame_id(fixed_frame); std::string ns("frontier_ns");
			std_msgs::ColorRGBA color; color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0; 
			visualization_msgs::Marker marker = generateVisMarker( frame_id, ns, i, 
									       int(visualization_msgs::Marker::SPHERE), 
									       int(visualization_msgs::Marker::ADD),
									       v, 0.05, color, false);			
			marray.markers[i] = marker;
			// ------------------------------------------------
			FrontierPoint fp;
			fp.p = v;
			double disToTarget = sqrt( (v.x-goalX)*(v.x-goalX) + (v.y-goalY)*(v.y-goalY) );			
			fp.profit = -(distance + disToTarget);			
			if( fp.profit > bestProfit )
			{ bestProfit = fp.profit; best_tp = v; }
			frontierList->push_back(fp);
			 
		}
		ROS_INFO("publishing frontier markers");
		vis_pub.publish(marray);

		// sort the frontiers in decreasing profit order
		sort(frontierList->begin(), frontierList->end(), compareFrontierPoints);		
		mapUpdateTime = ros::Time::now();		
    }
    catch(tf::TransformException ex)
    {
     	ROS_ERROR( "%s", ex.what() );
    }    
}

void
DE_Planner::robotPosCheck(const ros::TimerEvent& event)
{
	try
	{
		tf::StampedTransform transform;
		listener->lookupTransform( fixed_frame, robot_frame, ros::Time(0), transform );	  // get robot position

		if( EUCLIDEAN_DIST( robotCurrPos.x, robotCurrPos.y, transform.getOrigin().x(), transform.getOrigin().y() ) <= 0.1 )  // if the position has not changed much
		{
			ros::Duration d = ros::Time::now() - lastActiveTime;     // what is the time since the last active movement
			if( targetActive && (d.toSec() >= stuck_timeout_threshold ) )                // if this duration is over 8 seconds
			{
				// cancell all goals
				ROS_INFO("cancelling the target because robot has remained idle for over %f seconds", (float)stuck_timeout_threshold);
				setStuck(true);
				ac->cancelAllGoals();
			}
		}
		else // the robot has moved 
		{
			lastActiveTime = ros::Time::now();              // reset the time
			robotCurrPos.x = transform.getOrigin().x();     // update robot's active current Position
			robotCurrPos.y = transform.getOrigin().y();
			ROS_INFO("pos : [%f, %f]", robotCurrPos.x, robotCurrPos.y);

			double disToTgt = EUCLIDEAN_DIST( currentTargetX, currentTargetY, robotCurrPos.x, robotCurrPos.y );
			//ROS_INFO("Current Target : %f, %f", )
			//if( targetActive &&  disToTgt <= 0.2 && currentTargetX == goalX && currentTargetY == goalY && !goingToSafePlace )
			if( targetActive && disToTgt <= target_reach_tolerance && goingToFinalGoal )
			{
				ROS_INFO("cancelling the target because it has reached required target reach tolerence for GOAL");
				ac->cancelAllGoals();
			}
			else if(targetActive && disToTgt <= target_reach_tolerance && !goingToSafePlace)
			{
			    ROS_INFO("reached tolerence for intermediate target");
			    ac->cancelAllGoals();
			}			
			// since the robot has moved, it is not stuck
			setStuck(false);
		}
	}
	catch(tf::TransformException & ex)
	{
	    ROS_ERROR("%s", ex.what() );
	}	
}

double
DE_Planner::getPathDistance(nav_msgs::Path & plan, double thresh, unsigned int & tgtIndex)
{
	double dis = 0.0;
	unsigned int path_len = plan.poses.size();
	if( path_len <= 0 )
		return dis;
	for(unsigned int i = 0; i < (path_len-1); i++)
	{
		geometry_msgs::Point p1, p2;
		try
		{
			p1 = plan.poses.at(i).pose.position;
			p2 = plan.poses.at(i+1).pose.position;
		}
		catch(std::out_of_range & ex) { continue; }
		dis += EUCLIDEAN_DIST( p1.x, p1.y, p2.x, p2.y );
		if( dis >= thresh )
		{
			tgtIndex = i;
			break;
		}
	}
	return dis;
}

bool
DE_Planner::setMoveBaseGoal(move_base_msgs::MoveBaseGoal & goal, geometry_msgs::Pose & start, mapprocessing::Vector2d & tgt, mapprocessing::Vector2d & prevPt, double planner_tolerance)  
{		
	goal.target_pose.pose.position.x = tgt.x;
	goal.target_pose.pose.position.y = tgt.y;
	double yaw;
	// call service client to ask for a plan with generated goal	
	try
	{		
		if( !ros::service::exists(make_plan_service, true) )
			return false;
		
		ROS_INFO("make plan service exists");
		ros::ServiceClient planClient = n->serviceClient<nav_msgs::GetPlan>(make_plan_service);
		nav_msgs::GetPlan srv;
	
		// create the srv request
		srv.request.start.header.frame_id = "map";
		srv.request.start.header.seq = 1;
		srv.request.start.header.stamp = ros::Time::now();
		srv.request.start.pose = start;
		srv.request.goal.header.frame_id = "map";
		srv.request.goal.header.seq = 1;
		srv.request.goal.header.stamp = ros::Time::now();
		srv.request.goal.pose = goal.target_pose.pose;
		//srv.request.tolerance = planner_tolerance;
		srv.request.tolerance = plan_goal_tolerance;

		// should call planning service only if it is in an inactive state
		while( true )
		{
			actionlib::SimpleClientGoalState gs = ac->getState();		
			if( gs == actionlib::SimpleClientGoalState::PREEMPTED || 
				gs == actionlib::SimpleClientGoalState::SUCCEEDED || 
				gs == actionlib::SimpleClientGoalState::ABORTED || 
				gs == actionlib::SimpleClientGoalState::REJECTED || 
				gs == actionlib::SimpleClientGoalState::LOST )
				break;
			else
			{
				ROS_INFO("move base is : %s", gs.toString().c_str());
				ac->cancelAllGoals();   // cancell all goals so that move_base becomes available
				pollRate->sleep();
			}
		}
		ROS_INFO("calling planning service");
		if( planClient.call(srv) )
		{
			// if it was successfull
			ROS_INFO("Plan found");			
			unsigned int path_len = srv.response.plan.poses.size();
			ROS_INFO("plan length : %d", path_len);
			unsigned int point_index = 0;
			double path_dis = getPathDistance( srv.response.plan, max_distance_of_plan, point_index);
			ROS_INFO("path distance : %f", path_dis);			
			if( path_dis <= 0 )
				return false;
			if( path_dis < max_distance_of_plan || planner_tolerance <= PLAN_TOLERANCE_FOR_GOAL )
			{
				point_index = path_len-1;
			}
			if( point_index < 1 || point_index >= path_len )
				return false;

			ROS_INFO("Plan is valid");
			goal.target_pose.pose = srv.response.plan.poses.at( point_index ).pose;			
			geometry_msgs::Point gp, pbgp;  // goal point, point before goal point
			gp = goal.target_pose.pose.position; 
			int k = (point_index > 10 ) ? 10 : point_index/2;
			pbgp = srv.response.plan.poses.at( point_index - k).pose.position;
			yaw = atan2( (gp.y - pbgp.y), (gp.x - pbgp.x) );
			goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);			
			return true; 
		}
		else
		{				
			ROS_INFO("Plan generation failed...");
			return false;   // valid plan could not be found, so goal could not be set
		}
	}
	catch(std::out_of_range & e)
	{
		ROS_ERROR("%s", e.what());
		return false;
	}
	catch(ros::InvalidNameException & e)
	{
		ROS_ERROR("%s", e.what() );
		return false;
	}
	catch(ros::Exception & e)
	{
		ROS_ERROR("%s", e.what() );
		return false;
	}
	return false;  // default return value;
}

inline visualization_msgs::Marker 
DE_Planner::generateVisMarker(std::string & frameID, std::string & ns, int mID, int type, int action, 
			      mapprocessing::Vector2d pos, double scale, std_msgs::ColorRGBA & color, bool persistent)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frameID;
	if( persistent )
	  marker.header.stamp = ros::Time(0);   // persistant marker
	else
	  marker.header.stamp = ros::Time::now();  // non-persistent
	marker.ns = ns;
	marker.id = mID;
	marker.type = type;
	marker.action = action;
	marker.pose.position.x = pos.x;
	marker.pose.position.y = pos.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = scale;
	marker.scale.y = scale;
	marker.scale.z = scale;
	marker.color = color;	
	return marker;
}

geometry_msgs::Pose 
DE_Planner::getRobotCurrentPose()
{
	// get current position
	tf::StampedTransform transform;
	listener->lookupTransform( fixed_frame, robot_frame, ros::Time(0), transform );	
	
	geometry_msgs::Pose robot_curr_pose;
	robot_curr_pose.position.x = transform.getOrigin().x();
	robot_curr_pose.position.y = transform.getOrigin().y();
	robot_curr_pose.position.z = 0.0;
	robot_curr_pose.orientation = tf::createQuaternionMsgFromYaw( tf::getYaw( transform.getRotation() ) );

	return robot_curr_pose;
}

bool
DE_Planner::getNextFrontierPoint(FrontierPoint & fp, mapprocessing::Vector2d & robotPos)
{
	bool is_empty = false;

	if( frontierList == NULL )
	{
		ROS_WARN("Frontier List is not initialized yet..");
		is_empty = true;
	}
					
	if(frontierList->size() == 0 )
	{
		ROS_ERROR("Frontier List is empty!");
		is_empty = true;
	}

	do
	{
		if( frontierList->size() == 0)
		{
			is_empty = true;
			break;
		}
		fp = frontierList->front();
		frontierList->erase(frontierList->begin());
	}
	while( targetAlreadyAssigned(fp.p) || veryCloseToRobot(fp.p, robotPos) );

	return is_empty;
}

bool
DE_Planner::findATraversablePointTowardsGoal(FrontierPoint & fp, move_base_msgs::MoveBaseGoal & goal, mapprocessing::Vector2d & robotPos, geometry_msgs::Pose & robot_curr_pose)
{
	// move to a point in the traversable area which gets the robot closer to the goal
	ROS_INFO("Entering recovery behavior");	
	double r = std::min(5.0, EUCLIDEAN_DIST( goalX, goalY, robotPos.x, robotPos.y ) ); // if the distance to goal is greater than 5.0, use 5.0m
	double max_r;
	double theta;
	int theta_var;
	double dis_thresh;
	double backup_dis;
	if( isRobotStuck() )   // if the robot is stuck, use robot's current orientation
	{
		max_r = 5.0;
		theta = tf::getYaw( robot_curr_pose.orientation );
		theta_var = 180; // if robot stuck, use full 360 for checking
		backup_dis = 0.1;
	}
	else  // if not stuck, probably the robot could easily move towards goal direction
	{
		theta = atan2( (goalY - robotPos.y), (goalX - robotPos.x) );
		theta_var = 15;    // use only 30 degree check
		backup_dis= 0.2;
		max_r = r < 1.0 ? 1.0 : r;
	}
	dis_thresh = 1.0;
	mapprocessing::Vector2d tempTgt;
	bool foundTarget = false;
	
	// populate the theta update
	std::vector<double> th_deltas;
	std::vector<double>::iterator it;
	th_deltas.push_back( 0 );
	int incr = 2;
	for(int i = incr; i < theta_var; i = i+incr)
	{
		th_deltas.push_back( i*3.14159265359/180.0 );
		th_deltas.push_back( -i*3.14159265359/180.0 );
	}
	
	for( it = th_deltas.begin(); it != th_deltas.end(); it++ )
	{
		double th = theta + *it; 
		double l = max_r;
		while( !foundTarget && l >= dis_thresh ) 
		{
			tempTgt.x = robotPos.x + l*cos(th); tempTgt.y = robotPos.y + l*sin(th);
			double disToPos = map->getShortestDistanceToInMeters(tempTgt);
			if(disToPos != DBL_MAX)   // we have a temporary winner!
			{
				// if actual planner returns a valid plan, YAY!
				// get planner's final pose as MoveBaseGoal's goal pose
				ROS_INFO("recovery point distance: %f", disToPos);
				std::vector<mapprocessing::Vector2d> path;
				if( setMoveBaseGoal( goal, robot_curr_pose, tempTgt, tempTgt, PLAN_TOLERANCE_FOR_GOAL ) )  // second tempTgt is of no use
				{
					foundTarget = true;
					fp.p = tempTgt;			   
				}
			}
			l = l - backup_dis;     // decrease l
		}		
		if( foundTarget )
			break;		
	}
	if( foundTarget )
	{
		//always make sure the robot faces towards the goal
		double yaw = atan2( (goalY - goal.target_pose.pose.position.y), (goalX - goal.target_pose.pose.position.x) );
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		goingToSafePlace = true;
		ROS_INFO("Done with recovery behavior");
	}	
	return foundTarget;
}

void
DE_Planner::checkRobotStatus()
{
	//running_ = false;  // for testing only
	decision_mutex.lock();
	waitingForWayPoint = true;
	decision_mutex.unlock();

	while(!gotRobotOrientation)
	{
		ROS_INFO("Waiting for robot's orientation information from compass...");
		pollRate->sleep();
	}
	compas_sub.shutdown(); // unsubscribe from compass data
	// at the end of this -> robots_bearing_wrt_north is set according to compas data
	
	while(true)   // thread's loop
	{		
	waitForNextWayPoint();            
	goingToFinalGoal = false;	
	//rotate360();                    // this is temporary hack to avoid robot selecting nearby frontiers, will be removed
	                                  // when actual distance to goal through unknown space is also considered for utility
	decision_mutex.lock();
	double rad_bearing;
	rad_bearing = bearing;    // bearing is in the clockwise direction w.r.t. robot's heading, in radians
	goalX = range*cos(rad_bearing);
	goalY = -range*sin(rad_bearing);
	decision_mutex.unlock();
//--------------------------------------------------------------------------------------------------------------	
	ROS_INFO("GOAL : (%f, %f)", goalX, goalY);
	std_msgs::ColorRGBA color; color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = 1.0;
	mapprocessing::Vector2d goalPos; goalPos.x = goalX; goalPos.y = goalY;
	std::string ns("frontier_ns");
	visualization_msgs::Marker marker = generateVisMarker( fixed_frame, ns, 1, 
														   int(visualization_msgs::Marker::CUBE), 
														   int(visualization_msgs::Marker::ADD), 
														   goalPos, 0.3, color );
	goal_vis_pub.publish(marker);		
//--------------------------------------------------------------------------------------------------------------
	
	int frontiers_checked = 0;
	int recovery_failure_count = 0;
	useOldMap = false; goalReached = false;	double running;
	decision_mutex.lock();
	running_ = true;
	running = running_;
	decision_mutex.unlock();	
	while(running)
	{
		geometry_msgs::Pose robot_curr_pose; 
		try
	    {
			robot_curr_pose = getRobotCurrentPose();
	    }
		catch(tf::TransformException & ex)
	    { continue; }
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = fixed_frame;
		goal.target_pose.header.stamp = ros::Time::now();

		bool foundTarget = false;		
		mapprocessing::Vector2d robotPos; 
		robotPos.x = robot_curr_pose.position.x;
		robotPos.y = robot_curr_pose.position.y;
		ros::ServiceClient mapClient = n->serviceClient<nav_msgs::GetMap>("dynamic_map");
		nav_msgs::GetMap srv;
		ros::Duration d = goal.target_pose.header.stamp - mapUpdateTime;
		if( !useOldMap && d.toSec() > 0.5  )  // last map update was done 0.75 second ago, then update the map 
		{
			if( mapClient.call(srv) )
			{
				ROS_INFO("map returned");
				updateMap(srv.response.map);
			}
			else
			{
				ROS_INFO("Failed to call map service");
				usleep(100000);
				continue;
			}				
		}				

		ROS_INFO("Getting next target");
		FrontierPoint fp;		
		mapprocessing::Vector2d goalv; goalv.x = goalX; goalv.y = goalY;
		double disToGoal = map->getShortestDistanceToInMeters(goalv);
		CvPoint goalp = map->getPositionInPixelPoints(goalv);
		unsigned int goal_map_val = static_cast<unsigned int>(map->getMapVal(goalp.y, goalp.x));
		bool goal_in_mapped_region = isGoalInMappedRegion(goalp);
		if( isRobotStuck() )
		{
			// go to recovery behavior and selct a safe point to move
			ROS_INFO("Robot is stuck...., going to recovery mode");
			foundTarget = findATraversablePointTowardsGoal(fp, goal, robotPos, robot_curr_pose);
			if( !foundTarget )
			{
				ROS_INFO("Well... this is embarrassing... ONE SHOULD BE ABLE TO FIND A SAFE POINT");
				usleep(100000);
				continue;
			}
			else
			{
				setStuck(false);			
				setNavStatus(techx_msgs::LocalNavStatus::RECOVERY);
			}
		}
		else if(disToGoal != DBL_MAX || goingToFinalGoal)
		{
			ROS_INFO("Goal is visible");			
			std::vector<mapprocessing::Vector2d> path;
			mapprocessing::Vector2d tgt; tgt.x = goalX; tgt.y = goalY;
			map->getShortestPath(path, tgt);
			fp.p.x = goalX; fp.p.y = goalY;

			ROS_INFO("Travelling directly to goal");				
			if( !setMoveBaseGoal( goal, robot_curr_pose, tgt, path[path.size()-2], PLAN_TOLERANCE_FOR_GOAL ) )
			{
				ROS_ERROR("Cannot find a proper path to go to goal, interesting...");
				foundTarget = findATraversablePointTowardsGoal(fp, goal, robotPos, robot_curr_pose);								
				if( !foundTarget )
				{
					setStuck(true); // force full recovery
					usleep(100000); 
					continue;
				}								
				setNavStatus(techx_msgs::LocalNavStatus::RECOVERY);
			}	
			else
			{	
				goalX = goal.target_pose.pose.position.x; goalY = goal.target_pose.pose.position.y;   // update goalX and goalY
				foundTarget = true;
				setNavStatus(techx_msgs::LocalNavStatus::NAVIGATING);
				goingToFinalGoal = true;
			}
			fp.p.x = goal.target_pose.pose.position.x; fp.p.y = goal.target_pose.pose.position.y;
		}   // ~ end if goal reachable
		else if( goal_map_val < 100 || goal_map_val > 150 || goal_in_mapped_region ) 
		{   // specified goal position is inside mapped region, and collision free path cannot be planned
			// but cannot use frontiers as the goal is not beyond frontiers
			ROS_INFO("GOAL is within the mapped region, but is unreachable, so moving to a traversable place .......");
			foundTarget = findATraversablePointTowardsGoal(fp, goal, robotPos, robot_curr_pose);   // uses a restricted version of recovery
			if( !foundTarget )
			{
				usleep(100000);
				setStuck(true);   // restricted recovery failed, so force full recovery by setting stuck flag
				continue;
			}
			setNavStatus(techx_msgs::LocalNavStatus::NAVIGATING);   // this is not a stuck situation, as the robot can "see" the goal
		}
		else  // else : goal is not reachable yet, so moving to an intermediate position
		{
			std::cout << "using frontiers" << std::endl;
			bool empty = getNextFrontierPoint(fp, robotPos);     // get the next possible frontier point				
			if( empty || frontiers_checked >= 3 )   // if there are no frontiers available, but have not reached goal and goal is not visible
			{
				ROS_INFO("No traversable frontier present");
				foundTarget = findATraversablePointTowardsGoal(fp, goal, robotPos, robot_curr_pose);
				if( !foundTarget )
				{
					ROS_ERROR("There are no possible targets for robot to move, which is impossible...");	
					setStuck(true); // forcing full recovery
					usleep(100000); // sleep for sometime before making another decision
					continue;
				}
				setNavStatus(techx_msgs::LocalNavStatus::RECOVERY);
			} // ~ end if frontier list is empty
			else   // a frontier is selected, frontier list is not empty
			{
				ROS_INFO("Checking frontier point");
				std::vector<mapprocessing::Vector2d> path;
				map->getShortestPath(path, fp.p);				
				ROS_INFO("got path");
				if( path.size() > 2 )
				{
					if( !setMoveBaseGoal( goal, robot_curr_pose, fp.p, path[ path.size() - 2 ], plan_goal_tolerance ) )
					{
						ROS_INFO("plan couldnot be generated");
						assignedTargets->push_back(fp.p);
						frontiers_checked++;
						useOldMap = true;
						continue;
					}
					else
					{
						foundTarget = true;
						setNavStatus(techx_msgs::LocalNavStatus::NAVIGATING);
					}
				}											
				else
				{
					ROS_INFO("path too short");
					assignedTargets->push_back(fp.p);
					frontiers_checked++;
					useOldMap = true;
					continue;
				}				
			} // ~ end if frontiers are available
		}  // end : normal frontier target, goal unreachable yet					
		assignedTargets->push_back(fp.p); // assign target
		if( !foundTarget ){
			continue;
		}

		frontiers_checked = 0; // reset the counter
		useOldMap = false;
		currentTargetX = goal.target_pose.pose.position.x;
		currentTargetY = goal.target_pose.pose.position.y;
			
		//--------------- publish marker visualization ------------------------------------------------------------------
		mapprocessing::Vector2d currentTgt; currentTgt.x = currentTargetX; currentTgt.y = currentTargetY;
		std_msgs::ColorRGBA color; color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 1.0; 
		std::string ns("frontier_ns");
		visualization_msgs::Marker marker = generateVisMarker( fixed_frame, ns, 1, 
															   int(visualization_msgs::Marker::CUBE), int(visualization_msgs::Marker::ADD),
															   currentTgt, 0.3, color);						
		target_vis_pub.publish(marker);
		//----------------------------------------------------------------------------------------------------------------
		   
		ac->sendGoal(goal);					   
		setStuck(false);                                // when a goal is sent, stuck flag is set to false
		targetActive = true;
		lastActiveTime = ros::Time::now();              // reset the time
		ROS_INFO("Sent action command : (%f, %f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		mapprocessing::Vector2d targetP; targetP.x = goal.target_pose.pose.position.x; targetP.y = goal.target_pose.pose.position.y;
		double disToTgt = map->getShortestDistanceToInMeters( targetP );			
		if( disToTgt == DBL_MAX )
		{
			disToTgt = 1.0;
		}
		bool done = ac->waitForResult(ros::Duration( 2*(disToTgt/0.1) ));   // calculate duration based on the robot's speed and distance to target		
		if( done && ( ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) )
		{
			if( EUCLIDEAN_DIST( robotCurrPos.x, robotCurrPos.y, goalX, goalY ) <= target_reach_tolerance )
			{
				ROS_INFO("Reached goal");		
				goalReached = true;
				decision_mutex.lock();
				running_ = false;				
				waitingForWayPoint = true;
				decision_mutex.unlock();
			}
			else
				ROS_INFO("Reached temporary target according to de_planner");
		}			
		else
		{			
			if( EUCLIDEAN_DIST( robotCurrPos.x, robotCurrPos.y, goalX, goalY ) <= target_reach_tolerance )
			{
				ROS_INFO("Reached goal");		
				goalReached = true;
				decision_mutex.lock();
				running_ = false;				
				waitingForWayPoint = true;
				decision_mutex.unlock();
			}
			else
				ROS_INFO("Didn't reach target, target timedout or target cancelled");				
		}
		ROS_INFO("Current pos : [%f, %f]", robotCurrPos.x, robotCurrPos.y);
		ROS_INFO("Current Target : [%f, %f]", currentTargetX, currentTargetY);
		if( goalReached )
		{
			std_msgs::Empty done_msg;
			donesignal_pub.publish(done_msg);
		}
		setNavStatus(techx_msgs::LocalNavStatus::IDLE);
		targetActive = false;	
		goingToSafePlace = false;
		goingToFinalGoal = false;
		decision_mutex.lock();
		running = running_;
		decision_mutex.unlock();
	} // end of thread loop, running_ = false	
	} // end of while(true) loop	
	controllerThread->join();
}

bool 
DE_Planner::targetAlreadyAssigned(::mapprocessing::Vector2d & tp)
{
 	boost::circular_buffer<mapprocessing::Vector2d>::iterator it;
 	for(it = assignedTargets->begin(); it != assignedTargets->end(); it++)
 	{
 		mapprocessing::Vector2d ap = *it;
 		double d = sqrt( (tp.x-ap.x)*(tp.x-ap.x) + (tp.y-ap.y)*(tp.y-ap.y) );
 		if(d <= 0.1 )
 			return true;
 	}
 	return false;
}

void
DE_Planner::rotate360()
{
	//return;  //testing
	ros::Rate r(10.0);
	ros::NodeHandle n;
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/drrobot_cmd_vel", 10);

	tf::StampedTransform global_pose, temp_pose;
	bool done = false;
	while(!done)
	  {	    
	    try{	      
	      listener->lookupTransform( fixed_frame, robot_frame, ros::Time(0), global_pose );
	      done = true;
	    }
	    catch(tf::TransformException & ex){}
	 
	  }
	
	// initialize robotStartingPos;
	robotStartingPos.x = global_pose.getOrigin().x();
	robotStartingPos.y = global_pose.getOrigin().y();

	geometry_msgs::Twist cmd_vel_lin;
	cmd_vel_lin.linear.y = 0.0;
	cmd_vel_lin.angular.z = 0.0;
	cmd_vel_lin.linear.x = 0.4;
	done = false;
	while( n.ok() )
	{
	    try
		{
			listener->lookupTransform(fixed_frame, robot_frame, ros::Time(0), global_pose);
			if( global_pose.getOrigin().x() >= 0.95 )
			{
				cmd_vel_lin.linear.x = 0.0;
				done = true;
			}
			vel_pub.publish(cmd_vel_lin);
			r.sleep();
			if( done )
				break;
		}
	    catch(tf::TransformException & ex){ continue; }
	}
		
	cmd_vel_lin.linear.x = 0.0;
	vel_pub.publish(cmd_vel_lin);
	
	usleep(100000);
	return;
	
	double current_angle = -1.0 * M_PI;

	bool got_180 = false;
	bool close_to_180 = false;
	double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
	while(n.ok()){
		listener->lookupTransform( fixed_frame, robot_frame, ros::Time(0), global_pose );
		double norm_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
		current_angle = angles::normalize_angle(norm_angle + start_offset);
		ROS_INFO("angles : %f, %f, %f", current_angle, norm_angle, start_offset);

		double dist_left = M_PI - current_angle;

		double vel = sqrt(2 * 3.2 * dist_left);

		vel = std::min(std::max(vel, 0.4), 0.6 );   // changed from 1.0 to 0.6
		
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.angular.z = vel;
		
		vel_pub.publish(cmd_vel);

		if( current_angle > 3.0 )
		  close_to_180 = true;

		if(current_angle < 0.0 && close_to_180 )
			got_180 = true;

		if(got_180 && current_angle >= (0.0 - 0.1))
		{
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
		
			vel_pub.publish(cmd_vel);
			vel_pub.publish(cmd_vel);
			vel_pub.publish(cmd_vel);
			vel_pub.publish(cmd_vel);
			usleep(250000);
			return;
		}
		r.sleep();
	}
}

bool
DE_Planner::veryCloseToRobot(::mapprocessing::Vector2d & tgt, ::mapprocessing::Vector2d & robotPos)
{
	if( map->getShortestDistanceToInMeters(tgt) <= 1.0 )
		return true;
	else 
		return false;
}

/*
 * When next way point is sent by test_nav, set the range and bearing, and also 
 * reset the flag that waits for way point to false
 */
void
DE_Planner::next_wp_cb(const techx_msgs::NextWayPoint::ConstPtr & wp_data)
{
	ROS_INFO("Next waypoint received");
	decision_mutex.lock();
	range = wp_data->range;
	bearing = wp_data->bearing;
	waitingForWayPoint = false;
	decision_mutex.unlock();
}

/**
 * This function is called in response to the stop signal sent by test_nav
 * This means the robot must get ready to go to the next way point supplied by test_nav
 * Hence, it should stop its current navigation loop, hence setting running_ to false
 * and setting waitingForWayPoint to true
 * in addition, cancel all active navigation goals submitted to move_base
 */
void DE_Planner::stop_signal_cb(const std_msgs::Empty::ConstPtr & msg)
{
	ROS_INFO("Stop signal received from navigator, robot reached target according to GPS measurement");
	if( targetActive )
		ac->cancelAllGoals();		// stop signal received, so cancelling all goals
	decision_mutex.lock();
	waitingForWayPoint = true;
	running_ = false;               // stop the inner loop that works on current goal navigation
	decision_mutex.unlock();
}

void DE_Planner::waitForNextWayPoint()
{
	ROS_INFO("Waiting for next waypoint");
	ros::Rate r(20);
	while( true )
	{		
		r.sleep();
		decision_mutex.lock();
		if( !waitingForWayPoint )
		{
			decision_mutex.unlock();
			break;
		}
		decision_mutex.unlock();
	}
}

bool DE_Planner::isGoalInMappedRegion(const CvPoint & goalP)
{
	ROS_INFO("is the goal in mapped region");
	int SIZE = 24;
	CvPoint offset[SIZE];
	int k = 0;	
	for(int i = -2; i <=2; i++)
	{
		for(int j = -2; j <= 2; j++)
		{
			if( i == 0 && j == 0 ) continue;
			offset[k].x = i;
			offset[k++].y = j;
		}
	}
	int mappedCount = 0;
	int width = map->getWidth();
	int height = map->getHeight();
	for(int i = 0; i < 24; i++)
	{
		CvPoint newP; newP.x = goalP.x + offset[i].x; newP.y = goalP.y + offset[k].y;
		if( newP.x < 0 || newP.x > width-1 || newP.y < 0 || newP.y > height-1 )
			continue;
		if( !IS_UNKNOWN(static_cast<unsigned int>( map->getMapVal(newP.y, newP.x) ) ) )
			mappedCount++;
	}
	//bool rval = (mappedCount >= (double)SIZE/2.0 )?true:false;
	bool rval = (mappedCount > 0)?true:false;  // if the goal point has a very close mapped pixel, goal is in the mapped region or very close to it
	ROS_INFO("checked! : %d", rval);
	return rval;
}

bool DE_Planner::isRobotStuck()
{
	bool rtype;
	decision_mutex.lock();
	rtype = isStuck;
	decision_mutex.unlock();
}

void DE_Planner::setStuck(bool stuck)
{
	decision_mutex.lock();
	isStuck = stuck;
	decision_mutex.unlock();
}

void DE_Planner::pubStatus()
{
	ros::Rate r(10);
	while( ros::ok() )
	{
		navstatus_mutex.lock();
		navstatus_pub.publish(nav_status);
		navstatus_mutex.unlock();
		r.sleep();
	}

	statusPubThread->join();
}

void DE_Planner::setNavStatus(uint8_t status)
{
	navstatus_mutex.lock();
	nav_status.status = status;
	navstatus_mutex.unlock();
}
