#include "scurve_motion.h"
#include "math.h"
#include <stdio.h>

void button_forward_cycle(struct scurve_status *s) {
    if (s->btn_trigger_on) { // Prevent autorepeat button press.
        return;
    }
    s->btn_trigger_on = 1;
    s->btn_trigger_off = 0;

    if (s->vr >= 0.0) {
        s->jog_fwd = 1;
        s->jog_rev = 0;
        s->vo = s->vr;
        s->curtime = 0;

        if (s->curvel > 0 && s->curacc > 0) {
            s->do_acc = 0;
            s->do_dcc = 1;
        }

        if (s->curvel > 0 && s->curacc < 0) {
            s->do_acc = 1;
            s->do_dcc = 0;
        }
    }
}

void button_reverse_cycle(struct scurve_status *s) {
    if (s->btn_trigger_on) {
        return;
    }
    s->btn_trigger_on = 1;
    s->btn_trigger_off = 0;

    if (s->vr <= 0.0) {
        s->jog_fwd = 0;
        s->jog_rev = 1;
        s->vo = s->vr;
        s->curtime = 0;

        if (s->curvel < 0 && s->curacc < 0) {
            s->do_acc = 1;
            s->do_dcc = 0;
        }

        if (s->curvel < 0 && s->curacc > 0) {
            s->do_acc = 0;
            s->do_dcc = 1;
        }
    }
}

void button_release_cycle(struct scurve_status *s) {
    if (s->btn_trigger_off) {
        return;
    }
    s->btn_trigger_on = 0;
    s->btn_trigger_off = 1;

    if (s->vr <= 0) { // Jog reverse release.
        s->jog_fwd = 0;
        s->jog_rev = 0;
        s->vo = s->curvel;
        s->curtime = 0;

        if (s->ar != 0) {
            s->rev_stop_trigger = 1;
        } else {
            s->rev_stop_trigger = 0;
        }
    }

    if (s->vr >= 0) { // Jog forward release.
        s->jog_fwd = 0;
        s->jog_rev = 0;
        s->vo = s->curvel;
        s->curtime = 0;

        if (s->ar != 0) {
            s->fwd_stop_trigger = 1;
        } else {
            s->fwd_stop_trigger = 0;
        }
    }
}

// T1 acceleration.
inline double vr_t1(double vo, double jm, double t){
    return vo + jm*(t*t)/2;
}

inline double sr_t1(double vo, double jm, double t){
    return vo*t + jm*(t*t*t)/6;
}

inline double ar_t1(double jm, double t){
    return jm*t;
}

// T2 - steady.
inline double vr_t2(double vo, double as, double t){
    return vo+as*t; //! Linear acc.
}

inline double sr_t2(double ve, double vo, double as){
    return (ve*ve-vo*vo)/(2*as);
}

inline double ar_t2(double as){
    return as;
}

// T3 - deceleration.
inline double vr_t3(double vh, double jm, double as, double t){
    return vh + as*t - jm*(t*t)/2;
}

inline double sr_t3(double vh, double jm, double as, double t){
    return vh*t + as*(t*t)/2 - jm*(t*t*t)/6;
}

inline double ar_t3(double as, double jm, double t){
    return as-jm*t;
}

inline void vr_sr_ar_t1(double vo, double jm, double t, double *vr, double *sr, double *ar) {
    *vr = vr_t1(vo, jm, t); // vo + jm * (t * t) / 2;
    *sr = sr_t1(vo, jm, t); // vo * t + jm * (t * t * t) / 6;
    *ar = ar_t1(jm, t);      // jm * t;
}

inline void vr_sr_ar_t2(double v1, double s1, double as, double t, double *vr, double *sr, double *ar) {
    *vr = vr_t2(v1, as, t);  // v1 + as * t; //! Linear acc.
    *sr = sr_t2(*vr, v1, as); // (*vr * *vr - v1 * v1) / (2 * as);
    *sr += s1;                // add displacement s1.
    *ar = ar_t2(as);          // as;
}

inline void vr_sr_ar_t3(double v2, double s1, double s2, double as, double jm, double t, double *vr, double *sr, double *ar) {
    *vr = vr_t3(v2, jm, as, t);   // v2 + as * t - jm * (t * t) / 2;
    *sr = sr_t3(v2, jm, as, t);   // v2 * t + as * (t * t) / 2 - jm * (t * t * t) / 6;
    *sr += s1 + s2;               // add displacement s1 & s2.
    *ar = ar_t3(as, jm, t);       // as - jm * t;
}

//! If "v1" current velocity input is negative, "sr" displacement output is also negative.
inline void vr_sr_ar_t4(double v1, double s1, double t, double *sr) {
    *sr = v1 * t; // s = v * t
    *sr += s1;    // add displacement s1.
}

inline void pid_t1_acc_fwd_rev(double cycletime, double as, double jm,
                               double curacc, double curvel,
                               double *vr, double *sr, double *ar, double *s_cycle) {
    if (jm > 0 && curacc < 0) { // Motion forward.
        curacc = 0;
    }
    if (jm > 0 && curacc > as) {
        curacc = as;
    }

    if (jm < 0 && curacc > 0) { // Motion reverse.
        curacc = 0;
    }
    if (jm < 0 && curacc < as) {
        curacc = as;
    }

    double ts = curacc / jm; // Convex curve start time, from maxacc as to zero.

    double tm = as / jm; // Max time.
    double te = ts + cycletime;
    if (te > tm) {
        te = tm;
    }

    // Calculate the vo given the start time ts.
    double vo = curvel - (jm * ts * ts) / 2.0;

    double vrp, srp, arp;
    vr_sr_ar_t1(vo, jm, ts, &vrp, &srp, &arp); // Curve at ts.
    vr_sr_ar_t1(vo, jm, te, vr, sr, ar);      // Curve at ts+interval.

    *s_cycle = *sr - srp;
}

inline void pid_t2_acc_fwd_rev(double cycletime, double as, double curpos, double curvel,
                               double *vr, double *sr, double *ar, double *s_cycle) {
    vr_sr_ar_t2(curvel, curpos, as, cycletime, vr, sr, ar);
    *s_cycle = *sr - curpos;
}

inline void pid_t3_acc_fwd_rev(double cycletime, double curvel, double as, double jm, double curacc,
                               double *vr, double *sr, double *ar, double *s_cycle) {
    if (jm > 0 && curacc < 0) { // Motion forward. Limit acceleration from 0 to as.
        curacc = 0;
    }
    if (jm > 0 && curacc > as) {
        curacc = as;
    }

    if (jm < 0 && curacc > 0) { // Motion reverse.
        curacc = 0;
    }
    if (jm < 0 && curacc < as) {
        curacc = as;
    }

    double ts = (as - curacc) / jm; // Convex curve start time, from maxacc as to zero.

    double vo = curvel - as * ts + 0.5 * jm * (ts * ts); // Derived from v=vh+as*t-jm*(t*t)/2;

    double so = 0;
    double vrp, srp, arp;
    vr_sr_ar_t3(vo, so, 0, as, jm, ts, &vrp, &srp, &arp); // Curve at ts.

    double tm = as / jm; // Time max.
    double te = ts + cycletime; // Time at cycletime.
    if (te > tm) { // Limit time to time max.
        te = tm;
    }

    vr_sr_ar_t3(vo, so, 0, as, jm, te, vr, sr, ar);

    *s_cycle = *sr - srp;
}

inline void pid_t4_acc_fwd_rev(double cycletime, double curpos, double curvel,
                               double *vr, double *sr, double *ar, double *s_cycle) {
    vr_sr_ar_t4(curvel, curpos, cycletime, sr);
    *vr = curvel;
    *ar = 0;
    *s_cycle = *sr - curpos;
}

inline void get_acc_dcc_time(double *a, double *jm, double vo, double ve,
                              double *t1, double *t2, double *t3, double *as) {

    *a = fabs(*a);
    *jm = fabs(*jm);

    if (vo < ve) {
        // curve is acc.
    }
    if (vo == ve) {
        // curve is steady.
    }
    if (vo > ve) {
        *a = -*a;
        *jm = -*jm;
        // curve is dcc.
    }

    *as = 2 * (*a); // "as" Max acceleration at inflection point. as=2*A.
    double dvt = 2 * (*as) / (*jm); // "dvt" Delta velocity time,
    // time for an acceleration or deceleration period. Derived from: jm=2*as/t1.
    // This period includes the period-stages t1,t2,t3.
    double dv = fabs((dvt * (*as)) / 2); // "dv" Delta velocity, Dv=vo-ve, derived from: t1=2*(ve-vo)/as;

    double v_diff = fabs(vo - ve);

    *t1 = 0, *t2 = 0, *t3 = 0;

    if (v_diff == dv) { // Curve is size of dv.
        // Curve has a full t1 and t3 period, curve has no t2 period.
        // Concave period.
        *t1 = 0.5 * dvt;
        *t2 = 0;
        *t3 = *t1;
    }
    if (v_diff < dv) { // Curve < dv, curve t1 has ace, curve t3 has acs.
        // Curve has only a t1 & t2 period, where acceleration at inflection point < as."
        double vh = (vo + ve) / 2;
        // t1, from vo to vh. From acc 0 to ace ?..
        *t1 = sqrt(2 * (vh - vo) / (*jm)); // Derived from: vh=vo+jm*(t*t)/2
        *t2 = 0;
        *t3 = *t1;

        // Set new as value, if curve ve-vo>=dv the as=2*a.
        // If curve ve-vo<dv, the curve acceleration at the inflection point is calculated by:
        *as = *jm * (*t1);
    }
    if (v_diff > dv) { // Curve > dv
        // Curve has a full t1 and t3 period, curve has a linear acc period t2 to fit.
        double dv_t2 = v_diff - dv;
        *t1 = 0.5 * dvt;
        *t2 = dv_t2 / (*as); // t = dv / as, derived from linear acc-dcc, t = ve - vo / a
        *t3 = *t1;
    }

    *t1 = fabs(*t1);
    *t2 = fabs(*t2);
    *t3 = fabs(*t3);
}

void inline get_acc_dcc_velocity_displacement(double as, double jm, double vo, double t1, double t2, double t3,
                                       double *v1, double *s1, double *v2, double *s2,
                                       double *v3, double *s3, double *ttot, double *stot) {

    *v1 = vr_t1(vo, jm, t1);   // vo + jm * (t1 * t1) / 2; //! Ve period t1.
    *s1 = sr_t1(vo, jm, t1);   // vo * t1 + jm * (t1 * t1 * t1) / 6;

    *v2 = vr_t2(*v1, as, t2);   // v1 + as * t2; //! Linear acc.
    *s2 = sr_t2(*v2, *v1, as);  // (*v2 * *v2 - *v1 * *v1) / (2 * as);

    *v3 = vr_t3(*v2, jm, as, t3);  // vh + as * t - jm * (t * t) / 2;
    *s3 = sr_t3(*v2, jm, as, t3);  // vh * t + as * (t * t) / 2 - jm * (t * t * t) / 6;

    *ttot = t1 + t2 + t3;
    *stot = *s1 + *s2 + *s3;
}

inline void scurve_motion_get_acc_dcc_velocity_displacement(double as, double jm, double vo, double t1, double t2, double t3,
                                                             double *v1, double *s1, double *v2, double *s2,
                                                             double *v3, double *s3, double *ttot, double *stot) {

    *v1 = vr_t1(vo, jm, t1);   // vo + jm * (t1 * t1) / 2; //! Ve period t1.
    *s1 = sr_t1(vo, jm, t1);   // vo * t1 + jm * (t1 * t1 * t1) / 6;

    *v2 = vr_t2(*v1, as, t2);  // *v1 + as * t2; //! Linear acc.
    *s2 = sr_t2(*v2, *v1, as);  // (*v2 * *v2 - *v1 * *v1) / (2 * as);

    *v3 = vr_t3(*v2, jm, as, t3);  // vh + as * t3 - jm * (t3 * t3) / 2;
    *s3 = sr_t3(*v2, jm, as, t3);  // vh * t3 + as * (t3 * t3) / 2 - jm * (t3 * t3 * t3) / 6;

    *ttot = t1 + t2 + t3;
    *stot = *s1 + *s2 + *s3;
}

inline void pid_t1_acc_dcc_to_zero_fwd(double cycletime, double jm, double curvel, double curacc,
                                        double *vr, double *sr, double *ar, double *s_cycle) {

    double as = curacc;
    double v = curvel;
    double ts = curacc / jm;

    double vo = v - (jm * ts * ts) / 2.0;

    double te = ts + cycletime; // Time at cycletime.

    *vr = vo + jm * (te * te) / 2;
    double srp = vo * ts + jm * (ts * ts * ts) / 6;
    *sr = vo * te + jm * (te * te * te) / 6;

    *s_cycle = *sr - srp;

    *ar = jm * te;
}

inline void pid_t3_acc_dcc_to_zero_fwd(double cycletime, double jm, double curvel, double curacc,
                                       double *vr, double *sr, double *ar, double *s_cycle) {

    double as = curacc;
    double v = curvel;
    double ts = (as - curacc) / jm;
    double vo = v - as * ts + 0.5 * jm * (ts * ts); // Derived from v=vh+as*t-jm*(t*t)/2;

    double tm = as / jm; // Time max.
    double te = ts + cycletime; // Time at cycletime.
    if (te > tm) { // Limit time to time max.
        te = tm;
    }

    *vr = vo + as * te - jm * (te * te) / 2;

    double srp = vo * ts + as * (ts * ts) / 2 - jm * (ts * ts * ts) / 6;
    *sr = vo * te + as * (te * te) / 2 - jm * (te * te * te) / 6;

    *s_cycle = *sr - srp;

    *ar = as - jm * te;
}

inline bool pid_t3_fits(bool fwd, double jm, double curvel, double maxvel, double curacc) {

    double as = curacc;

    double v = curvel;
    double ts = (as - curacc) / jm;
    double vo = v - as * ts + 0.5 * jm * (ts * ts); // Derived from v=vh+as*t-jm*(t*t)/2;

    // v=vo+as*ts-jm*(ts*ts)/2;
    // v check.

    double tm = as / jm; // Time max.

    double vr = vo + as * tm - jm * (tm * tm) / 2;

    if (fwd && curvel + (vr - v) < maxvel) {
        return 1;
    }

    if (!fwd && curvel - (v - vr) > 0) {
        return 1;
    }
    return 0;
}

void update_cycle(struct scurve_status *s) {
    if (s->jog_rev && s->curvel <= 0) { // Jog reverse.
        if (s->do_dcc) {
            pid_t3_acc_dcc_to_zero_fwd(s->cycletime, s->jerk_max, s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->curacc <= 0) {
                s->do_dcc = 0;
            }
        }
        if (s->do_acc) {
            pid_t1_acc_dcc_to_zero_fwd(s->cycletime, s->jerk_max, s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->curacc >= 0) {
                s->do_acc = 0;
            }
        }
        if (!(s->do_acc) && !(s->do_dcc)) {
            jog_fwd(s->jerk_max, s->cycletime, &(s->curtime), s->maxacc, -s->maxvel, s->vo, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
        }
    }

    if (!(s->jog_rev) && s->curvel < 0) { // Jog reverse stop.
        if (s->rev_stop_trigger && s->curacc < 0) {
            pid_t3_acc_dcc_to_zero_fwd(s->cycletime, -(s->jerk_max), s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->ar >= 0) {
                s->rev_stop_trigger = 0;
            }
        }
        if (s->rev_stop_trigger && s->curacc > 0) {
            pid_t1_acc_dcc_to_zero_fwd(s->cycletime, -(s->jerk_max), s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->ar <= 0) {
                s->rev_stop_trigger = 0;
            }
        }
        if (!(s->rev_stop_trigger)) {
            jog_fwd_stop(s->jerk_max, s->cycletime, &(s->curtime), s->maxacc, s->vo, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
        }
    }

    if (s->jog_fwd && s->curvel >= 0) { // Jog fwd.
        if (s->do_dcc) {
            pid_t3_acc_dcc_to_zero_fwd(s->cycletime, s->jerk_max, s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->curacc <= 0) {
                s->do_dcc = 0;
            }
        }
        if (s->do_acc) {
            pid_t1_acc_dcc_to_zero_fwd(s->cycletime, s->jerk_max, s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->curacc >= 0) {
                s->do_acc = 0;
            }
        }
        if (!(s->do_acc) && !(s->do_dcc)) {
            jog_fwd(s->jerk_max, s->cycletime, &(s->curtime), s->maxacc, s->maxvel, s->vo, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
        }
    }

    if (!(s->jog_fwd) && s->curvel > 0) { // Jog fwd stop.
        if (s->fwd_stop_trigger && s->curacc > 0) {
            pid_t3_acc_dcc_to_zero_fwd(s->cycletime, s->jerk_max, s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->ar <= 0) {
                s->fwd_stop_trigger = 0;
            }
        }
        if (s->fwd_stop_trigger && s->curacc < 0) {
            pid_t1_acc_dcc_to_zero_fwd(s->cycletime, s->jerk_max, s->curvel, s->curacc, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
            s->curtime = 0;
            s->vo = s->vr;

            if (s->ar >= 0) {
                s->fwd_stop_trigger = 0;
            }
        }
        if (!(s->fwd_stop_trigger)) {
            jog_fwd_stop(s->jerk_max, s->cycletime, &(s->curtime), s->maxacc, s->vo, &(s->vr), &(s->sr), &(s->ar), &(s->s_cycle));
        }
    }

    if (!(s->jog_fwd) && !(s->jog_rev) && s->curvel < 0.000001 && s->curvel > -0.000001) {
        s->curtime = 0;
        s->vo = 0;
        s->s_cycle = 0;
    }

    if (!isnan(s->s_cycle)) {
        s->stot += s->s_cycle;
    }

    s->curacc = s->ar;
    s->curvel = s->vr;
    s->curpos = s->sr;
}

bool jog_fwd(double jm, double ival, double *ct,
             double a, double vm,
             double vo,
             double *vr, double *sr, double *ar, double *s_cycle) {
    double as = 2 * a;
    double t1 = 0, t2 = 0, t3 = 0;
    get_acc_dcc_time(&a, &jm, vo, vm, &t1, &t2, &t3, &as);
    double v1, v2, v3, s1, s2, s3, ttot, stot;
    get_acc_dcc_velocity_displacement(as, jm, vo, t1, t2, t3, &v1, &s1, &v2, &s2, &v3, &s3, &ttot, &stot);

    int cycles = (int)(ttot / ival);
    ival = ttot / cycles;

    if (*ct > ttot) {
        // *ct = ttot;
    }

    double t = *ct;
    double vrp = 0, srp = 0, arp = 0;
    if (t < t1) { // Concave period T1.
        vr_sr_ar_t1(vo, jm, fmax((t - ival), 0), &vrp, &srp, &arp);
        vr_sr_ar_t1(vo, jm, t, vr, sr, ar);
        // T1
    }
    if (t >= t1 && t <= t1 + t2) { // Steady period T2.
        t = t - t1;
        vr_sr_ar_t2(v1, s1, as, fmax((t - ival), 0), &vrp, &srp, &arp);
        vr_sr_ar_t2(v1, s1, as, t, vr, sr, ar);
        t = t + t1;
        // T2
    }
    if (t > t1 + t2 && t <= t1 + t2 + t3) { // Convex period T3.
        t = t - t1 - t2;
        vr_sr_ar_t3(v2, s1, s2, as, jm, fmax((t - ival), 0), &vrp, &srp, &arp);
        vr_sr_ar_t3(v2, s1, s2, as, jm, t, vr, sr, ar);
        t = t + t1 + t2;
        // T3
    }
    if (t > t1 + t2 + t3) {
        t = t - t1 - t2 - t3;
        vr_sr_ar_t4(v3, stot, fmax((t - ival), 0), &srp);
        vr_sr_ar_t4(v3, stot, t, sr);
        *ar = 0;
        *vr = v3;
        // t=t+t1+t2+t3;
        // T4
    }

    *s_cycle = *sr - srp;

    *ct += ival;

    return 0;
}

bool jog_fwd_stop(double jm, double ival, double *ct,
                  double a, double vo,
                  double *vr, double *sr, double *ar, double *s_cycle) {
    double as = 2 * a;
    double t1 = 0, t2 = 0, t3 = 0;
    get_acc_dcc_time(&a, &jm, vo, 0, &t1, &t2, &t3, &as);
    double v1, v2, v3, s1, s2, s3, ttot, stot;
    get_acc_dcc_velocity_displacement(as, jm, vo, t1, t2, t3, &v1, &s1, &v2, &s2, &v3, &s3, &ttot, &stot);

    int cycles = (int)(ttot / ival);
    ival = ttot / cycles;

    if (*ct > ttot) {
        *ct = ttot;
    }

    double t = *ct;
    double vrp, srp, arp;
    if (t < t1) { // Concave period T1.
        vr_sr_ar_t1(vo, jm, fmax((t - ival), 0), &vrp, &srp, &arp);
        vr_sr_ar_t1(vo, jm, t, vr, sr, ar);
        // T1
    }
    if (t >= t1 && t <= t1 + t2) { // Steady period T2.
        t = t - t1;
        vr_sr_ar_t2(v1, s1, as, fmax((t - ival), 0), &vrp, &srp, &arp);
        vr_sr_ar_t2(v1, s1, as, t, vr, sr, ar);
        t = t + t1;
        // T2
    }
    if (t > t1 + t2) { // Convex period T3.
        t = t - t1 - t2;
        vr_sr_ar_t3(v2, s1, s2, as, jm, fmax((t - ival), 0), &vrp, &srp, &arp);
        vr_sr_ar_t3(v2, s1, s2, as, jm, t, vr, sr, ar);
        t = t + t1 + t2;
        // T3
    }

    *s_cycle = *sr - srp;

    *ct += ival;

    return 0;
}

bool jog_pid_acc_fwd_stop(double cycletime,
                           double jm,
                           double curvel,
                           double curpos,
                           double curacc,
                           double maxacc,
                           double *vr,
                           double *sr,
                           double *ar,
                           double *s_cycle) {
    double as = 2 * maxacc;
    as = -as;
    jm = -jm;

    if (curacc > 0) {
        pid_t3_acc_dcc_to_zero_fwd(cycletime, -jm, curvel, curacc, vr, sr, ar, s_cycle);
        return 0;
    }

    bool ok = pid_t3_fits(false, jm, curvel, 0, curacc);

    if (!ok) { // Add t3.
        pid_t3_acc_fwd_rev(cycletime, curvel, as, jm, curacc, vr, sr, ar, s_cycle);
    }

    if (ok && curacc == as) { // Add t2.
        pid_t2_acc_fwd_rev(cycletime, as, curpos, curvel, vr, sr, ar, s_cycle);
    }

    if (ok && curacc > as) { // Add t1.
        pid_t1_acc_fwd_rev(cycletime, as, jm, curacc, curvel, vr, sr, ar, s_cycle);
    }
    return 0;
}

bool jog_pid_acc_fwd(double cycletime,
                     double jm,
                     double curvel,
                     double curpos,
                     double curacc,
                     double maxacc,
                     double maxvel,
                     double tarpos,
                     double *vr,
                     double *sr,
                     double *ar,
                     double *s_cycle) {
    double as = 2 * maxacc;

    if (curacc < 0) {
        pid_t3_acc_dcc_to_zero_fwd(cycletime, -jm, curvel, curacc, vr, sr, ar, s_cycle);
        return 0;
    }

    bool ok = pid_t3_fits(true, jm, curvel, maxvel, curacc);

    if (!ok) { // Add t3.
        pid_t3_acc_fwd_rev(cycletime, curvel, as, jm, curacc, vr, sr, ar, s_cycle);
    }

    if (ok && curacc == as) { // Add t2.
        pid_t2_acc_fwd_rev(cycletime, as, curpos, curvel, vr, sr, ar, s_cycle);
    }

    if (ok && curacc < as) { // Add t1.
        pid_t1_acc_fwd_rev(cycletime, as, jm, curacc, curvel, vr, sr, ar, s_cycle);
    }

    if (curvel == maxvel && curacc == 0) {
        pid_t4_acc_fwd_rev(cycletime, curpos, curvel, vr, sr, ar, s_cycle);
    }

    if (*sr >= tarpos) {
        return 1;
    }
    return 0;
}
