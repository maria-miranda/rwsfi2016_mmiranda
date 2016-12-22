/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>
#include <rwsfi2016_msgs/GameQuery.h>
#include <math.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PointT;
/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */
using namespace std;
using namespace ros;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public rwsfi2016_libs::Player
{
  public: 

    ros::Publisher publisher;
    ros::Subscriber subscriber;
    visualization_msgs::Marker bocas_msg;
    ros::ServiceServer service;

    pcl::PointCloud<PointT> pointcloud;

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){

        publisher = node.advertise<visualization_msgs::Marker>("/bocas", 1);

        subscriber = node.subscribe("/object_point_cloud", 1, &MyPlayer::pointcloudCallback, this);


        bocas_msg.header.frame_id = name;
        bocas_msg.ns = name;
        bocas_msg.id = 0;
        bocas_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        bocas_msg.action = visualization_msgs::Marker::ADD;
        bocas_msg.scale.z = 0.4;
        bocas_msg.pose.position.y = 0.3;
        bocas_msg.color.a = 1.0; // Don't forget to set the alpha!
        bocas_msg.color.r = 0.0;
        bocas_msg.color.g = 0.0;
        bocas_msg.color.b = 0.0;

        service = node.advertiseService("/mmiranda/game_query", &MyPlayer::queryCallback, this);
    };

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
       pcl::fromROSMsg(*msg, pointcloud);

       //std::cout << "###############PCL################" << pointcloud.points.size() << std::endl;
    }

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
        //Custom play behaviour. Now I will win the game
        bocas_msg.header.stamp = ros::Time();

        double dist_min1=999, dist_min2=999, dist_hmin=999;
        int i_min1 = 0, i_min2 = 0, h_min = 0;

        for(int i = 0; i < msg.red_alive.size(); i++){
            double dist = getDistanceToPlayer(msg.red_alive[i]);
            if(dist <= dist_min1){
                dist_min1 = dist;
                i_min1 = i;
            }
        }

        int i_min;

        i_min = i_min1;


        for(int i=0; i< msg.green_alive.size(); i++){
            double dist_h = getDistanceToPlayer(msg.green_alive[i]);
            if(dist_h <= dist_hmin){
                dist_hmin = dist_h;
                h_min = i;
            }

        }

        double angleMove;
        double distance_to_arena = getDistanceToArena();

        float green_angle = getAngleToPLayer(msg.green_alive[h_min]);


        string arena = "/map";
        double angle_to_arena = getAngleToPLayer(arena);

        if (distance_to_arena >7.5) //behaviour move to the center of arena
        {
            move(msg.max_displacement*(7.8-distance_to_arena), angle_to_arena);
            bocas_msg.text = "Estive ao pe do precipicio e dei um passo em frente";
        }
        else{
            //Behaviour follow the closest prey
            if((dist_hmin <= dist_min1) && (dist_hmin < 3.0)){
                float tdist = 5.0;
                if (distance_to_arena < tdist)
                {
                    angleMove = - green_angle;
                    move(msg.max_displacement, angleMove);
                }
                else
                {
                    angleMove = -1*(1 - (distance_to_arena - tdist)/(8.0-tdist))* green_angle+((distance_to_arena - tdist)/(8.0-tdist))*angle_to_arena ;
                    move(msg.max_displacement, angleMove);
                }

                bocas_msg.text = "Damn! Xau " + msg.green_alive[h_min] +" vou dar o fora!";
            }
            else{
                if(msg.red_alive.size()>=1)
                {
                    if(getDistanceToPlayer(msg.red_alive[i_min]) <= 0.3 * getDistanceToPlayer(msg.red_alive[0]))
                    {
                        angleMove = getAngleToPLayer(msg.red_alive[i_min]);
                        bocas_msg.text = "Poe te a toques " +msg.red_alive[i_min] ;
                    }
                    else
                    {
                        angleMove = getAngleToPLayer(msg.red_alive[0]);
                        bocas_msg.text = "Poe te a toques " +msg.red_alive[0] ;
                    }

                    move(msg.max_displacement, angleMove);
                }
                else
                {
                    angleMove = getAngleToPLayer(msg.red_alive[i_min]);
                    move(msg.max_displacement, angleMove);
                    bocas_msg.text = "Poe te a toques " +msg.red_alive[i_min] ;
                }


            }
        }

        //move(msg.max_displacement, M_PI);

        publisher.publish(bocas_msg);
    }


    bool queryCallback(rwsfi2016_msgs::GameQuery::Request &req, rwsfi2016_msgs::GameQuery::Response &res){


        float R=0, G=0, B=0;

        for(int i=0; i <pointcloud.points.size(); i++){
            R += pointcloud.points[i].r;
            G += pointcloud.points[i].g;
            B += pointcloud.points[i].b;
        }

        R /= pointcloud.points.size();
        G /= pointcloud.points.size();
        B /= pointcloud.points.size();


    std::cout << "R G B: " << R << " " << G << " " << B << std::endl;

        if(R > 59 && G >63 && B>88)
            res.resposta = "soda_can";
        else if(R > 61 && G >35 && B>47)
            res.resposta = "onion";
        else if(R > 150 && G >141 && B>44)
            res.resposta = "banana";
        else
            res.resposta = "tomato";

        return true;
    }
};


/**
 * @brief The main function. All you need to do here is enter your name and your pets name
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 * @return result
 */
int main(int argc, char** argv)
{
  // ------------------------
  //Replace this with your name
  // ------------------------
  string my_name = "mmiranda";
  string my_pet = "/cat";

  //initialize ROS stuff
  ros::init(argc, argv, my_name);

  //Creating an instance of class MyPlayer
  MyPlayer my_player(my_name, my_pet);

  //Infinite spinning (until ctrl-c)
  ros::spin();
}
