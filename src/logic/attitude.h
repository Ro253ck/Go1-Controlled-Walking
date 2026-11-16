#ifndef ATTITUDE_H
#define ATTITUDE_H
#include <stdint.h>

typedef struct {
    double   q[4];        /* body to world */
    double   kp;          /* correction gain, 1/s; time constant is 1/kp */
    double   gate;        /* above the |‖f‖ - g| threshold the accelerometer is ignored*/
    int      seeded;
    
    uint64_t n_gated;     /* Number of times the accelerometer has been ignored */
    uint64_t n_updates;
} filter_t;

/*These functions compute, respectively, the quaternion product, the conjugate and the normalisation*/
void   quaternion_mul(double r[4], const double a[4], const double b[4]);
void   quaternion_conj(double r[4], const double q[4]);
void   quaternion_normalize(double q[4]);

/* v_world = q * v_body * q^-1, and the inverse rotation. */
void   quaternion_rotate(double out[3], const double q[4], const double v[3]);
void   quaternion_rotate_inv(double out[3], const double q[4], const double v[3]);

/*Takes a quaternion and returns the pitch, roll and yaw values*/
void   quaternion_to_rpy(const double q[4], double *roll, double *pitch, double *yaw);

void filter_init(filter_t *f, double kp, double gate);

/* Aligns the estimate with the measured gravity direction, leaving the yaw
   at zero. Called once at startup, so the filter does not have to converge
   starting from an arbitrary attitude. */
void filter_seed(filter_t *f, const double acc[3]);

/* One step: integrates the gyroscope, slowly correcting towards the direction
   the accelerometer indicates as up. */
void filter_update(filter_t *f, const double gyro[3], const double acc[3], double dt);

#endif /* ATTITUDE_H */
