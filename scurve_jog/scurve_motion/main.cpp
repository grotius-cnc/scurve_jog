#include "scurve_motion.h"

int main(int argc, char *argv[])
{
    scurve_status s;

    s.maxacc=10;
    s.jerk_max=10;
    s.maxvel=25;
    s.cycletime=0.01;

    scurve_motion().button_forward_cycle(s);

    while(s.vr<s.maxvel){
         scurve_motion().update_cycle(s);
         std::cout<<"v:"<<s.vr<<std::endl;
    }

    scurve_motion().button_release_cycle(s);

    while(s.vr>0){
         scurve_motion().update_cycle(s);
         std::cout<<"v:"<<s.vr<<std::endl;
    }

    return 0;
}
