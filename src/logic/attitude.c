#include "attitude.h"
#include "config.h"

#include <math.h>


/* Hamilton product: composes two rotations. We obtain another quaternion. */
void quaternion_mul(double r[4], const double a[4], const double b[4])
{
    const double w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];

    const double x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    const double y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    const double z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];

    r[0] = w; r[1] = x; r[2] = y; r[3] = z;
}

/*The conjugate of a quaternion is simply the same quaternion with the "vector" components sign-flipped*/
void quaternion_conj(double r[4], const double q[4])
{
    r[0] =  q[0];
    r[1] = -q[1];
    r[2] = -q[2];
    r[3] = -q[3];
}

/*Normalises the quaternion*/
void quaternion_normalize(double q[4])
{
    double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); /*Computation of the norm*/
    if (n < 1e-12) { q[0] = 1.0; q[1] = q[2] = q[3] = 0.0; return; } 
    for (int i = 0; i < 4; i++){
        q[i] /= n;
    }
}

/*Rotates the vector. From body frame to world frame*/
void quaternion_rotate(double out[3], const double q[4], const double v[3])
{
    const double v_q[4]={0,v[0],v[1],v[2]};
    double q_c[4],tmp[4];

    quaternion_conj(q_c,q);

    quaternion_mul(tmp,q,v_q);
    quaternion_mul(tmp,tmp,q_c);

    out[0]=tmp[1];
    out[1]=tmp[2];
    out[2]=tmp[3];
}

/*Performs the inverse rotation. From world frame to body frame*/
void quaternion_rotate_inv(double out[3], const double q[4], const double v[3])
{
    double qc[4];
    quaternion_conj(qc, q);
    quaternion_rotate(out, qc, v);
}

/*Returns the roll, pitch and yaw values*/
void quaternion_to_rpy(const double q[4], double *roll, double *pitch, double *yaw)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];

    double sin_pitch = 2.0 * (w*y - z*x); /*Formula derived from the rotation matrices*/
    if (sin_pitch >  1.0) sin_pitch =  1.0;
    if (sin_pitch < -1.0) sin_pitch = -1.0;

    /*Formulas derived from the rotation matrices*/
    if(roll) *roll  = atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));
    if(pitch) *pitch = asin(sin_pitch);
    if(yaw) *yaw   = atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

void filter_init(filter_t *f, double kp, double gate)
{
    f->q[0] = 1.0; f->q[1] = f->q[2] = f->q[3] = 0.0; /*No rotation*/
    f->kp        = kp;
    f->gate      = gate;
    f->seeded    = 0;
    f->n_gated   = 0;
    f->n_updates = 0;
}

void filter_seed(filter_t *f, const double accelerometer[3])
{
    /*accelerometer gives us where the force points, in body coordinates*/
    const double n = sqrt(accelerometer[0]*accelerometer[0] + accelerometer[1]*accelerometer[1] + accelerometer[2]*accelerometer[2]);

    /*Check that I am still and that I am not in free fall*/
    if (n < 1e-6 || fabs(n - ATT_GRAVITY) > f->gate) return; 
    
    const double ax = accelerometer[0]/n, ay = accelerometer[1]/n, az = accelerometer[2]/n; /*Unit vector*/
    const double s = sqrt(ax*ax + ay*ay);

    const double cx = ay, cy = -ax, cz = 0.0;     /* (ax,ay,az) x (0,0,1) */

    if (s < 1e-9) {                 
        f->q[0] = (az > 0.0) ? 1.0 : 0.0;
        f->q[1] = (az > 0.0) ? 0.0 : 1.0;
        f->q[2] = 0.0;
        f->q[3] = 0.0;
    } else {
        const double angle = atan2(s, az); /*sin/cos*/
        const double half  = 0.5 * angle;
        const double k     = sin(half) / s;
        f->q[0] = cos(half);
        f->q[1] = cx * k;
        f->q[2] = cy * k;
        f->q[3] = cz * k;
    }
    quaternion_normalize(f->q);
    f->seeded = 1;
}


void filter_update(filter_t *f, const double gyro[3],const double accelerometer[3], double dt)
{
    double w[3] = { gyro[0], gyro[1], gyro[2] };
    const double a_norm = sqrt(accelerometer[0]*accelerometer[0] + accelerometer[1]*accelerometer[1] + accelerometer[2]*accelerometer[2]);

    /*Check again whether I am still or in free fall. This time, if so, I integrate the gyroscope only, with no accelerometer correction*/
    if (fabs(a_norm - ATT_GRAVITY) <= f->gate && a_norm > 1e-6) {
        double up_meas[3] = { accelerometer[0]/a_norm, accelerometer[1]/a_norm, accelerometer[2]/a_norm };
        double up_pred[3];
        const double ez[3] = { 0.0, 0.0, 1.0 };

        
        quaternion_rotate_inv(up_pred, f->q, ez); /*I bring the world "up" into body coordinates according to the current rotation */

        /*I add a correction proportional to the difference between the accelerometer measurement and the prediction*/
        w[0] += f->kp * (up_meas[1]*up_pred[2] - up_meas[2]*up_pred[1]);
        w[1] += f->kp * (up_meas[2]*up_pred[0] - up_meas[0]*up_pred[2]);
        w[2] += f->kp * (up_meas[0]*up_pred[1] - up_meas[1]*up_pred[0]);
    } else {
        f->n_gated++;
    }

    {
        const double w_norm = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]); /*How fast I am rotating*/
        double dq[4], qn[4];    

        /*Integration of the updated gyroscope*/
        if (w_norm * dt > 1e-9) {
            const double half = 0.5 * w_norm * dt;
            const double k    = sin(half) / w_norm;
            dq[0] = cos(half);
            dq[1] = w[0] * k;
            dq[2] = w[1] * k;
            dq[3] = w[2] * k;
        } else {
            
            dq[0] = 1.0;
            dq[1] = 0.5 * w[0] * dt;
            dq[2] = 0.5 * w[1] * dt;
            dq[3] = 0.5 * w[2] * dt;
        }

        quaternion_mul(qn, f->q, dq);
        for (int i = 0; i < 4; i++) f->q[i] = qn[i];
        quaternion_normalize(f->q);
    }
    f->n_updates++;
}
