#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

using namespace std;

class Movement{
  private:
    ros::NodeHandle nH; //interaccion con sistema ROS y manejar comunicacion
    ros::Publisher cmd_vel_pub; //publicar mensajes de tipo twist (velocidad)

  public:
    Movement(ros::NodeHandle &nh){
      this->nH = nH;
      this->cmd_vel_pub = nH.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    void move(){
      cout << "Intrucciones: "
          "\n\tW:     IR HACIA ADELANTE"
          "\n\tA:     GIRAR IZQUIERDA"
          "\n\tD:     GIRAR DERECHA"
          "\n\tS:     IR HACIA ATRAS"
          "\n\tZ:     STOP"
          "\n\tX:     SALIR\n";

      geometry_msgs::Twist base_cmd; //twist es un tipo de mensaje que representa velocidad lineal y angular

      char cmd;
      bool stop = false;

      while(nH.ok() && !stop){
        cin >> cmd;
        cmd = tolower(cmd);
      
        switch(cmd){
          case 'w':
            base_cmd.linear.x = 0.2;
            base_cmd.angular.z = 0;
            break;

          case 'a':
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 0.1;
            break;

          case 'd':
            base_cmd.linear.x = 0;
            base_cmd.angular.z = -0.1;
            break;

          case 's':
            base_cmd.linear.x = -0.2;
            base_cmd.angular.z = 0;
            break;

          case 'z':
            base_cmd.angular.z = 0;
            base_cmd.linear.x = 0;
            break;
          case 'x':
            base_cmd.angular.z = 0;
            base_cmd.linear.x = 0;
            stop = true;
            break;

          default:
            cout << "Comando desconocido: " << cmd << "\n";
            continue;
        }

        ros::spinOnce();
        cmd_vel_pub.publish(base_cmd);
      }
    }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "movement");
  ros::NodeHandle nh;

  Movement mov(nh);
  mov.move();
  ros::shutdown();
}