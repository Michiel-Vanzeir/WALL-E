#include <ros/ros.h>
#include "fl/Headers.h"

#include "my_robot_msgs/Inputvars.h"
#include "my_robot_msgs/Throttle.h"

ros::Publisher throttlepub;

class FuzzyEngine {
    private:
        fl::Engine* engine = new fl::Engine();
        fl::InputVariable* error = new fl::InputVariable();
        fl::InputVariable* angle = new fl::InputVariable();
        fl::OutputVariable* left_throttle = new fl::OutputVariable();
        fl::OutputVariable* right_throttle = new fl::OutputVariable();
        fl::RuleBlock* rules = new fl::RuleBlock();
     public:
        FuzzyEngine () {
            angle->setName("angle");
            angle->setEnabled(true);
            angle->setRange(-90.0, 90.0);
            angle->setLockValueInRange(false);
            angle->addTerm(new fl::Trapezoid("nlarge", -90.0, -90.0, -45.0, -30.0));
            angle->addTerm(new fl::Triangle("nmedium", -30.0, -20.0, -10.0));
            angle->addTerm(new fl::Trapezoid("small", -15.0, -10.0, 10.0, 15.0));
            angle->addTerm(new fl::Triangle("medium", 10.0, 20.0, 30.0));
            angle->addTerm(new fl::Trapezoid("large", 30.0, 45.0, 90.0, 90.0));
            engine->addInputVariable(angle);

            error->setName("error");
            error->setEnabled(true);
            error->setRange(-160.0, 160.0);
            error->setLockValueInRange(false);
            error->addTerm(new fl::Trapezoid("nlarge", -160.0, -160.0, -145.0, -120.0));
            error->addTerm(new fl::Triangle("nmedium", -135.0, -85.0, -35.0));
            error->addTerm(new fl::Triangle("small", -50.0, 0.0, 50.0));            
            error->addTerm(new fl::Triangle("medium", 35.0, 85.0, 135.0));
            error->addTerm(new fl::Trapezoid("large", 120.0, 145.0, 160.0, 160.0));
            engine->addInputVariable(error);
    
            right_throttle->setName("rthrottle");
            right_throttle->setEnabled(true);
            right_throttle->setRange(-0.5, 0.6);
            right_throttle->setLockValueInRange(false);
            right_throttle->setAggregation(new fl::Maximum());
            right_throttle->setDefuzzifier(new fl::Centroid(100));
            right_throttle->setDefaultValue(0.0);
            right_throttle->addTerm(new fl::Triangle("nlow", -0.35, -0.25, -0.25));
            right_throttle->addTerm(new fl::Triangle("low", 0.2, 0.2, 0.24));
            right_throttle->addTerm(new fl::Triangle("average", 0.32, 0.37, 0.47));
            right_throttle->addTerm(new fl::Trapezoid("high", 0.37, 0.5, 0.55, 0.55));
            engine->addOutputVariable(right_throttle);

            left_throttle->setName("lthrottle");
            left_throttle->setEnabled(true);
            left_throttle->setRange(-0.5, 0.6);
            left_throttle->setLockValueInRange(false);
            left_throttle->setAggregation(new fl::Maximum());
            left_throttle->setDefuzzifier(new fl::Centroid(100));
            left_throttle->setDefaultValue(0.0);
            left_throttle->addTerm(new fl::Triangle("nlow", -0.35, -0.25, -0.25));
            left_throttle->addTerm(new fl::Triangle("low", 0.2, 0.2, 0.24));
            left_throttle->addTerm(new fl::Triangle("average", 0.32, 0.37, 0.47));
            left_throttle->addTerm(new fl::Trapezoid("high", 0.37, 0.5, 0.55, 0.55));
            engine->addOutputVariable(left_throttle);

            rules->setName("rules");
            rules->setEnabled(true);
            rules->setConjunction(new fl::Minimum());
            rules->setDisjunction(new fl::Maximum());
            rules->setImplication(new fl::AlgebraicProduct());
            rules->setActivation(new fl::General());
            rules->addRule(fl::Rule::parse("if error is nlarge and angle is nlarge then lthrottle is nlow and rthrottle is average", engine));   
            rules->addRule(fl::Rule::parse("if error is nlarge or angle is nlarge then lthrottle is low and rthrottle is high", engine));         
            rules->addRule(fl::Rule::parse("if error is nmedium or angle is nmedium then lthrottle is low and rthrottle is average", engine));
            rules->addRule(fl::Rule::parse("if error is small and angle is small then lthrottle is average and rthrottle is average", engine));
            rules->addRule(fl::Rule::parse("if error is medium or angle is medium then lthrottle is average and rthrottle is low", engine));
            rules->addRule(fl::Rule::parse("if error is large or angle is large then lthrottle is high and rthrottle is low", engine));
            rules->addRule(fl::Rule::parse("if error is large and angle is large then lthrottle is average and rthrottle is nlow", engine));
            engine->addRuleBlock(rules);
        }

        bool ready() {
            std::string status;
            if (engine->isReady(&status)) {
                return true;
            } return false;
        }

        std::tuple<float, float> calculateOutput(float error_input, float angle_input) {
            error->setValue(error_input);
            angle->setValue(angle_input);
            engine->process();
            return {left_throttle->getValue(), right_throttle->getValue()};
        }
};

void calculate_throttle(const my_robot_msgs::Inputvars::ConstPtr& msg) {
    FuzzyEngine engine;
    if (engine.ready()) {
        auto [left_throttle, right_throttle] = engine.calculateOutput((float)msg->error, (float)msg->angle);

        my_robot_msgs::Throttle throttle_msg;
        throttle_msg.left_throttle = left_throttle;
        throttle_msg.right_throttle = right_throttle;
        throttlepub.publish(throttle_msg);
        ros::spinOnce();
    } else {
        ROS_ERROR("Fuzzy engine not ready");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fuzzy_controller");
    ros::NodeHandle nh;
    
    throttlepub = nh.advertise<my_robot_msgs::Throttle>("throttlefeed", 1); 
    ros::Subscriber sub = nh.subscribe("inputfeed", 1, calculate_throttle);

    ros::spin();
    return 0;
}
