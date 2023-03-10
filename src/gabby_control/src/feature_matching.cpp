#include <ros/ros.h>

#include <gabby_msgs/Features.h>
#include <gabby_msgs/MatchList.h>
#include <geometry_msgs/Point.h>

class FeatureMatcher {
    public:
        FeatureMatcher(ros::NodeHandle& nh) {
            nh_ = nh;
            feature_sub_ = nh_.subscribe("/features", 1, &FeatureMatcher::featureCallback, this);
            matches_pub_ = nh_.advertise<gabby_msgs::MatchList>("/matches", 1);
            inner_threshold_ = 0.2; // To be redefined, no idea what the resolution is
            outer_threshold_ = 0.5; // To be redefined, no idea what the resolution is
        }
        float calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
            return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
        }
        void featureCallback(const gabby_msgs::Features::ConstPtr& new_features) {
            gabby_msgs::MatchList matches;
            
            ROS_INFO("Got %d features", new_features->points.size());
            for (int i = 0; i < new_features->points.size(); i++) {
                for (int j = 0; j < features_.size(); j++) {
                    geometry_msgs::Point p1 = new_features->points[i];
                    geometry_msgs::Point p2 = features_[j];
                    float distance = calculateDistance(p1, p2);

                    if (distance <= inner_threshold_) {
                        ROS_INFO("Feature %d matches feature %d", i, j);
                        matches.detected_points.push_back(p1);
                        matches.matched_points.push_back(p2);
                    } else if (distance >= outer_threshold_) {
                        ROS_INFO("New feature added to list");
                        features_.push_back(p1);
                    }
                }       
            }

            matches_pub_.publish(matches);
        }
    private:
        ros::NodeHandle nh_;
        ros::Subscriber feature_sub_;
        ros::Publisher matches_pub_;

        std::vector<geometry_msgs::Point> features_;
        float inner_threshold_;
        float outer_threshold_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_matcher");
    ros::NodeHandle nh;

    FeatureMatcher fm(nh);
    
    ros::spin();
    return 0;
}