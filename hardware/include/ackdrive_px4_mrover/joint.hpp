#ifndef DIFFDRIVE_ARDUINO_JOINT_HPP
#define DIFFDRIVE_ARDUINO_JOINT_HPP

#include <string>
#include <cmath>


class Joint
{
    public:

    std::string name = "";
    float enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Joint() = default;

    Joint(const std::string &joint_name, int counts_per_rev)
    {
      setup(joint_name, counts_per_rev);
    }

    
    void setup(const std::string &joint_name, int counts_per_rev)
    {
      name = joint_name;
    //   rads_per_count = (2*M_PI)/counts_per_rev;
      rads_per_count = counts_per_rev;
    }

    float calc_enc_angle()
    {
      return enc;
    }



};


#endif // DIFFDRIVE_ARDUINO_JOINT_HPP
