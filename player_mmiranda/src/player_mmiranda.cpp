/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>
#include <math.h>

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

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){};

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
      //Custom play behaviour. Now I will win the game
        double dist, dist_min=999, dist_hmin=999;
        int i_min = 0, h_min = 0;
        for(int i = 0; i < msg.red_alive.size(); i++){
            dist = getDistanceToPlayer(msg.red_alive[i]);
            if(dist <= dist_min){
                dist_min = dist;
                i_min = i;
            }
        }

        for(int i=0; i< msg.green_alive.size(); i++){
            double dist_h = getDistanceToPlayer(msg.green_alive[i]);
            if(dist_h <= dist_hmin){
                dist_hmin = dist_h;
                h_min = i;
            }
        }

        double angleMove;
      //Behaviour follow the closest prey
        if(dist_hmin < dist_min)
            angleMove = - getAngleToPLayer(msg.green_alive[h_min]);
        else
            angleMove = getAngleToPLayer(msg.red_alive[i_min]);

      move(msg.max_displacement, angleMove);
      //move(msg.max_displacement, M_PI);


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
