#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tek4030_visual_servoing_msgs/ImageFeaturePoints.h>
#include <Eigen/Eigen>

ros::Publisher* pub_twist;
ros::Publisher* pub_set;
ros::Publisher* pub_err;

using namespace tek4030_visual_servoing_msgs;
void callback(tek4030_visual_servoing_msgs::ImageFeaturePoints s_msg)
{
    double z_c = 1.0;
    double x = 0.15;
    double y = -x;
    Eigen::Vector8d s_d(x,x,y,x,y,y,x,y);
    Eigen::Vector8d s(
            s.p[0].x,s.p[0].y,
            s.p[1].x,s.p[1].y,
            s.p[2].x,s.p[2].y,
            s.p[3].x,s.p[3].y);
    Eigen::Vector8d e_s = s_d - s;

    Eigen::Matrix<float, 8, 6 > L;
    L << 
    -1/z_c,     0, p1.x/z_c,   p1.x * p1.y, -(1 + p1.x*p1.x), p1.y,
         0,-1/z_c, p1.y/z_c, 1 + p1.y*p1.y,     -p1.x * p1.y, -p1.x,
    -1/z_c,     0, p2.x/z_c,   p2.x * p2.y, -(1 + p2.x*p2.x), p2.y,
         0,-1/z_c, p2.y/z_c, 1 + p2.y*p2.y,     -p2.x * p2.y, -p2.x,
    -1/z_c,     0, p3.x/z_c,   p3.x * p3.y, -(1 + p3.x*p3.x), p3.y,
         0,-1/z_c, p3.y/z_c, 1 + p3.y*p3.y,     -p3.x * p3.y, -p3.x,
    -1/z_c,     0, p4.x/z_c,   p4.x * p4.y, -(1 + p4.x*p4.x), p4.y,
         0,-1/z_c, p4.y/z_c, 1 + p4.y*p4.y,     -p4.x * p4.y, -p4.x;
    Eigen::Matrix<float, 6,8> L_t = L.transpose();
    Eigen::Matrix<float, 6,8> L_inv = (L_t * L).invert() *L_t;
    Eigen::Matrix<float, 6,6> G;
    G <<
        -1, 0, 0,  0,-wz, wy,
         0,-1, 0, wz,  0,-wx,
         0, 0,-1,-wy, wx,  0,
         O, O, O,-1, 0, 0,
         O, O, O, 0,-1, 0,
         O, O, O, 0, 0,-1;

    Eigen::Matrix<float, 8, 8 > K;
        K << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        v_c_r = L_inv * K * e_s;
    ImageFeaturePoints s_d_msg;
    ImageFeaturePoints e_s_msg;
    for (int i = 0; i < 4; i++) {
        geometry_msgs::Point p(2*i,2*i+1); s_d_msg.p.push_back(p);
        geometry_msgs::Point e(2*i,2*i+1); e_s_msg.p.push_back(e);
    }
    geometry_msgs::Twist v_c_msg;
    v_c_msg.linear.x = v_c(0);
    v_c_msg.linear.y = v_c(1);
    v_c_msg.linear.z = v_c(2);
    v_c_msg.angular.x = v_c(3);
    v_c_msg.angular.y = v_c(4);
    v_c_msg.angular.z = v_c(5);
    
    pub_twist->publish(v_c_msg);
    pub_set->publish(s_d_msg);
    pub_err->publish(e_s_msg);
}
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh; 

    ros::Subscriber sub = nh.subscribe
        ("/imgproc/points_normalized", 1000, callback);
    ros::Publisher ctw = nh.advertise
        <geometry_msgs::Twist>("camera_twist", 1000);
    ros::Publisher set = nh.advertise
        <ImageFeaturePoints>("/imgproc/points_setpoint", 1000);
    ros::Publisher err = nh.advertise
        <ImageFeaturePoints>("/imgproc/points_error", 1000);

    pub_twist = &ctw;
    pub_set = &ctw;
    pub_err = &ctw;

    ros::spin();
 
    return 0;
 }
