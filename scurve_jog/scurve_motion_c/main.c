#include "scurve_motion.h"

int main(int argc, char *argv[]) {
    struct scurve_status s;

    s.maxacc = 10;
    s.jerk_max = 10;
    s.maxvel = 25;
    s.cycletime = 0.01;

    button_forward_cycle(&s);

    while (s.vr < s.maxvel) {
        update_cycle(&s);
        printf("v: %f \n", s.vr);
    }

    button_release_cycle(&s);

    while (s.vr > 0) {
        update_cycle(&s);
        printf("v: %f \n", s.vr);
    }

    return 0;
}
