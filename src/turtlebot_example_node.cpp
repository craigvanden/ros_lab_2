//particle filter algorithm - p90 (78) Thrun
//mapping algorithm - p238 (226) & p242 (230) Thrun
//laser scanning model Chapter 6 - p141 (129) Thrun

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <algorithm> // std::min and std::max
#include <sensor_msgs/LaserScan.h>

#define MAP_SIZE_X 100
#define MAP_SIZE_Y 100
#define OBSTACLE_THICKNESS
#define SCANNER_RESOLUTION 
#define SCANNER_WIDTH
#define SCANNER_MAX_RANGE 200

#define PROB_IN_RAY 0.4
#define INITIAL_PROB 0.5
#define PROB_AT_END 0.6

#define SIMULATION_MODE

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

bool origin_init = false;
double origin_x;
double origin_y;
double origin_yaw;

std::vector<float> scan_ranges;
int scan_pts;
float scan_angle;
float scan_angle_inc;

double belief_map[MAP_SIZE_X][MAP_SIZE_Y];
double log_map_init[MAP_SIZE_X][MAP_SIZE_Y];
double log_map[MAP_SIZE_X][MAP_SIZE_Y];

short sgn(int x) { return x >= 0 ? 1 : -1; }
double logit(double val) { return log(val / (1 - val)); }
double log_to_prob(double val) { return exp(val / (1 + val)); }

void init_origin()
{
    origin_x = ips_x;
    origin_y = ips_y;
    origin_yaw = ips_yaw;
    origin_init = true;
}

#ifdef SIMULATION_MODE

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{
    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

    if (!origin_init)
    {
    	init_origin();
    }
}

#endif

#ifndef SIMULATION_MODE

//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}

#endif

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
}

void scanner_callback(const sensor_msgs::LaserScan& msg)
{
    // get number of measurements in laser scan
    scan_pts = msg.ranges.size();
    scan_ranges.resize(scan_pts);
    for (int i = 0; i < scan_pts; i++)
    {
	   scan_ranges[i] = msg.ranges[i];
    }
    scan_angle = msg.angle_min;
    scan_angle_inc = msg.angle_increment;
}

// Bresenham Line Algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

void map_update()
{
	// cache position global variables in case of unintentional update
	double ips_x_cache = ips_x;
	double ips_y_cache = ips_y;
	double ips_yaw_cache = ips_yaw;

    int scan_pts_cache = scan_pts;
    float scan_angle_cache;
    float scan_angle_inc_cache;
    std::vector<float> scan_ranges_cache = scan_ranges;

    for (int i = 0; i < scan_pts_cache; i++)
    {
       scan_ranges_cache[i] = scan_ranges[i];
    }

	// current robot grid position (origin for measurement)
	int grid_pos_x = std::max(1, std::min( MAP_SIZE_X, int(MAP_SIZE_X / 2 + ips_x_cache - origin_x)));
	int grid_pos_y = std::max(1, std::min( MAP_SIZE_Y, int(MAP_SIZE_Y / 2 + ips_y_cache - origin_y)));

	std::vector<int> bresenham_vector_x;
	std::vector<int> bresenham_vector_y;

	// loop through all measurements from the scan
	for (int j = 1; j < scan_pts_cache; j++)
	{
		// calculate measurement yaw
		double meas_yaw = scan_angle_cache + scan_angle_inc_cache * j;
		double ray_angle = meas_yaw + ips_yaw_cache - origin_yaw;

		// calculate measurement endpoint
		int endpt_x = std::max(1, std::min( MAP_SIZE_X, int(scan_ranges_cache[j] * cos(ray_angle) - ips_x_cache - origin_x)));
		int endpt_y = std::max(1, std::min( MAP_SIZE_Y, int(scan_ranges_cache[j] * sin(ray_angle) - ips_y_cache - origin_y)));

		// send values to bresenham algorithm
		bresenham(grid_pos_x, grid_pos_y, endpt_x, endpt_y, bresenham_vector_x, bresenham_vector_y);

        int k;
		for (k = 0; k < std::max( bresenham_vector_x.size(), bresenham_vector_y.size() ); k++)
		{
			log_map[bresenham_vector_x[k]][bresenham_vector_y[k]] = log_map[bresenham_vector_x[k]][bresenham_vector_y[k]] + logit(PROB_IN_RAY) - logit(INITIAL_PROB);
		}

		if ( scan_ranges_cache[j] < SCANNER_MAX_RANGE )
		{
			log_map[bresenham_vector_x[k]][bresenham_vector_y[k]] = log_map[bresenham_vector_x[k]][bresenham_vector_y[k]] + logit(PROB_AT_END) - logit(INITIAL_PROB);
		}
	}
}

void init_maps()
{
	for (int i = 0; i < MAP_SIZE_X; i++)
	{
		for (int j = 0; j < MAP_SIZE_Y; j++)
		{
			belief_map[i][j] = INITIAL_PROB;
			log_map[i][j] = logit(belief_map[i][j]);
		}
	}
}

void calc_probs()
{
	for (int i = 0; i < MAP_SIZE_X; i++)
	{
		for (int j = 0; j < MAP_SIZE_Y; j++)
		{
			belief_map[i][j] = log_to_prob(log_map[i][j]);
		}
	}
}

int main(int argc, char **argv)
{
	// Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    // Initialize Maps
    init_maps();

    // Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
	ros::Subscriber scanner_sub = n.subscribe("/scan", 1, scanner_callback);

    // Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    // Set the loop rate
    ros::Rate loop_rate(20);    // 20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); // Maintain the loop rate
    	ros::spinOnce();   // Check for new messages

        // Velocity control variable
        geometry_msgs::Twist vel;
        vel.angular.z = 0.3; // set angular speed
        velocity_publisher.publish(vel); // Publish the command velocity

    	map_update();
    	calc_probs();
    }

    return 0;
}