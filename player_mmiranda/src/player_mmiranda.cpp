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
    visualization_msgs::Marker bocas_msg;
    ros::ServiceServer service;

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){

        publisher = node.advertise<visualization_msgs::Marker>("/bocas", 1);
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

        service = node.advertiseService("mmiranda/game_query", &MyPlayer::queryCallback, this);
    };

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
      //Custom play behaviour. Now I will win the game
        bocas_msg.header.stamp = ros::Time();

        double dist_min1=999, dist_min2=999, dist_hmin=999;
        int i_min1 = 0, i_min2 = 0, h_min = 0;

        if(msg.red_alive.size() >= 1 ){
            dist_min1 = getDistanceToPlayer(msg.red_alive[0]);
            i_min1 = 0;
        }
        else{

            for(int i = 0; i < msg.red_alive.size(); i++){
                double dist = getDistanceToPlayer(msg.red_alive[i]);
                if(dist == dist){
                    if(dist <= dist_min1){
                        double aux = dist_min1;
                        dist_min1 = dist;
                        dist_min2 = aux;
                        i_min1 = i;
                        i_min2 = i_min1;
                    }
                }
            }

            int i_min;
            if(getAngleToPLayer(msg.red_alive[i_min2]) < getAngleToPLayer(msg.red_alive[i_min1]))
                i_min = i_min2;
            else
                 i_min = i_min1;
        }



        for(int i=0; i< msg.green_alive.size(); i++){
            double dist_h = getDistanceToPlayer(msg.green_alive[i]);
            if(dist_h == dist_h){
                if(dist_h <= dist_hmin){
                    dist_hmin = dist_h;
                    h_min = i;
                }
            }
        }

        double angleMove;
        double distance_to_arena = getDistanceToArena();
        if (distance_to_arena > 7.5) //behaviour move to the center of arena
        {
            string arena = "/map";
            move(msg.max_displacement/2.0, getAngleToPLayer(arena)/2);
            bocas_msg.text = "Going to center";
        }
        else{


          //Behaviour follow the closest prey
            if(dist_hmin <= dist_min1 && dist_hmin <= 3){
                angleMove = - getAngleToPLayer(msg.green_alive[h_min]);
                move(msg.max_displacement, angleMove);
                bocas_msg.text = "Running";
            }
            else{
                angleMove = getAngleToPLayer(msg.red_alive[i_min]);
                move(msg.max_displacement, angleMove);
                bocas_msg.text = "Catching";
            }
        }

      //move(msg.max_displacement, M_PI);

        publisher.publish(bocas_msg);
    }

    bool queryCallback(rwsfi2016_msgs::GameQuery::Request &req, rwsfi2016_msgs::GameQuery::Response &res){
        res.resposta = "Hello World";
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
