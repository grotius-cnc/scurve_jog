#ifndef SCURVE_MOTION_H
#define SCURVE_MOTION_H

#include <stdbool.h>
#include <stdio.h>
#include <stdbool.h>

struct scurve_status {
    double cycletime;
    double jerk_max;
    double curtime;
    double curvel;
    double curpos;
    double curacc;
    double tarpos;
    double maxacc;
    double maxvel;
    double vr;
    double sr;
    double ar;
    double vo;
    double stot;
    double s_cycle;
    bool pause;
    bool jog_fwd;
    bool jog_rev;
    bool do_dcc;
    bool do_acc;
    bool fwd_stop_trigger;
    bool rev_stop_trigger;
    bool btn_trigger_on;
    bool btn_trigger_off;
};

void button_forward_cycle(struct scurve_status *s);
void button_reverse_cycle(struct scurve_status *s);
void button_release_cycle(struct scurve_status *s);
void update_cycle(struct scurve_status *s);

bool jog_fwd(double jm, double ival, double *ct,
             double a, double vm,
             double vo,
             double *vr, double *sr, double *ar, double *s_cycle);

bool jog_fwd_stop(double jm, double ival, double *ct,
                  double a, double vo,
                  double *vr, double *sr, double *ar, double *s_cycle);

// Curve may have begin velocity or acceleration.
// Curve flow up to velocity max, then steady up to tarpos.
bool jog_pid_acc_fwd(double cycletime,
                     double jm,
                     double curvel,
                     double curpos,
                     double curacc,
                     double maxacc,
                     double maxvel,
                     double tarpos,
                     double *vr, double *sr, double *ar, double *s_cycle);

// Curve may have begin velocity or acceleration.
// Curve flow's to velocity zero.
bool jog_pid_acc_fwd_stop(double cycletime,
                          double jm, // Saved curve initial velocity at interupt.
                          double curvel,
                          double curpos,
                          double curacc,
                          double maxacc,
                          double *vr, double *sr, double *ar, double *s_cycle);

void pid_t1_acc_dcc_to_zero_fwd(double cycletime,
                                double jm, double curvel, double curacc,
                                double *vr, double *sr, double *ar, double *s_cycle);

// If a interupt is at positive acceleration, we need to go to accelereration zero first.
// This is for a forward curve.
void pid_t3_acc_dcc_to_zero_fwd(double cycletime,
                                double jm, double curvel, double curacc,
                                double *vr, double *sr, double *ar, double *s_cycle);

bool pid_t3_fits(bool fwd, double jm, double curvel, double maxvel, double curacc);

// Get periods t1,t2,t3 given a, jm, vo, ve.
void get_acc_dcc_time(double *a, double *jm, double vo, double ve,
                      double *t1, double *t2, double *t3, double *as);

// Get velocity's, displacements at periods t1,t2,t3.
void get_acc_dcc_velocity_displacement(double as, double jm, double vo,
                                       double t1, double t2,  double t3,
                                       double *v1, double *s1, double *v2, double *s2,
                                       double *v3, double *s3, double *ttot, double *stot);

// Cycle results for periods : t1,t2,t3,t4.
void pid_t1_acc_fwd_rev(double cycletime, double as, double jm, double curacc, double curvel,
                        double *vr, double *sr, double *ar, double *s_cycle);
void pid_t2_acc_fwd_rev(double cycletime, double as, double curpos, double curvel,
                        double *vr, double *sr, double *ar, double *s_cycle);
void pid_t3_acc_fwd_rev(double cycletime, double curvel, double as, double jm, double curacc,
                        double *vr, double *sr, double *ar, double *s_cycle);
void pid_t4_acc_fwd_rev(double cycletime, double curpos, double curvel,
                        double *vr, double *sr, double *ar, double *s_cycle);


// Forumula functions.
double vr_t1(double vo, double jm, double t);
double sr_t1(double vo, double jm, double t);
double ar_t1(double jm, double t);
double vr_t2(double vo, double as, double t);
double sr_t2(double ve, double vo, double as);
double ar_t2(double as);
double vr_t3(double vh, double jm, double as, double t);
double sr_t3(double vh, double jm, double as, double t);
double ar_t3(double as, double jm, double t);
void vr_sr_ar_t1(double vo, double jm, double t, double *vr, double *sr, double *ar);
void vr_sr_ar_t2(double v1, double s1, double as, double t, double *vr, double *sr, double *ar);
void vr_sr_ar_t3(double v2, double s1, double s2, double as, double jm, double t, double *vr, double *sr, double *ar);
void vr_sr_ar_t4(double v1, double s1, double t, double *sr);

#endif




















