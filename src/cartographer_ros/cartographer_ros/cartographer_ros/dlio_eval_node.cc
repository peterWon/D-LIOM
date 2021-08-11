#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <fstream>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(topic, "",
              "topic to listen.");
DEFINE_string(save_filename, "", "full file path to save trajectory poses.");

class EvalWriter{
public:
  EvalWriter(ros::NodeHandle nh, std::string topic, std::string filename){
    nh_ = nh;
    sub_ = nh_.subscribe(topic, 1000, &EvalWriter::poseCallback, this);
    ofs_ = std::ofstream(filename);
    if(!ofs_.is_open()){
      LOG(WARNING)<<"Open file [%s] failed!", filename;
    }else{
        ofs_ << "\%time,field.header.seq,field.header.stamp,field.pose.position.x,field.pose.position.y,field.pose.position.z,field.pose.orientation.x,field.pose.orientation.y,field.pose.orientation.z,field.pose.orientation.w\n";
    }
  }
  ~EvalWriter(){
    if(ofs_.is_open()) 
      ofs_.close();
  }
private:
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(!ofs_.is_open()){
      return;
    }else{
      ofs_ << msg->header.stamp.toNSec() <<"," << seq_++ 
           << "," << msg->header.stamp.toNSec() << ","
           << msg->pose.position.x << ","
           << msg->pose.position.y << ","
           << msg->pose.position.z << ","
           << msg->pose.orientation.x << ","
           << msg->pose.orientation.y << ","
           << msg->pose.orientation.z << ","
           << msg->pose.orientation.w << "\n";
      ofs_.flush();
    }
  };
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  int seq_ = 0;
  std::ofstream ofs_;
};

int main(int argc, char**argv){
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_topic.empty()) << "-topic is missing.";
  CHECK(!FLAGS_save_filename.empty()) << "-save_filename is missing.";

  ::ros::init(argc, argv, "dlio_eval_node");
  ::ros::start();
  ::ros::NodeHandle n;
  EvalWriter evaluator(n, FLAGS_topic, FLAGS_save_filename);
  ::ros::spin();
  ::ros::shutdown();
  return 0;
}