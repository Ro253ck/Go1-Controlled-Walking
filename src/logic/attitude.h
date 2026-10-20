#ifndef ATTITUDE_H
#define ATTITUDE_H
#include <stdint.h>

typedef struct {
    double   q[4];        /* body to world */
    double   kp;          /* correction gain, 1/s; time constant is 1/kp */
    double   gate;        /* sopra la soglia |‖f‖ - g| l'accelerometro è ignorato*/
    int      seeded;
    
    uint64_t n_gated;     /* Numero di volte in cui l'accelerometro è statp ignorato */
    uint64_t n_updates;
} filter_t;

/*Queste funzioni calcolano rispettivamente la moltiplicazione tra quaternioni, il coniugato e la normalizzazione*/
void   quaternion_mul(double r[4], const double a[4], const double b[4]);
void   quaternion_conj(double r[4], const double q[4]);
void   quaternion_normalize(double q[4]);

/* v_world = q * v_body * q^-1, e la rotazione inversa. */
void   quaternion_rotate(double out[3], const double q[4], const double v[3]);
void   quaternion_rotate_inv(double out[3], const double q[4], const double v[3]);

/*Prende un quaternione e ritorna i valori di pitch,roll e yaw*/
void   quaternion_to_rpy(const double q[4], double *roll, double *pitch, double *yaw);

void filter_init(filter_t *f, double kp, double gate);

/* Allinea la stima con la direzione della gravità misurata, lasciando lo yaw
   a zero. Chiamata una volta all'avvio, così il filtro non deve convergere
   partendo da un assetto arbitrario. */
void filter_seed(filter_t *f, const double acc[3]);

/* Un passo: integra il giroscopio, correggendo lentamente verso la direzione
   che l'accelerometro indica come alto. */
void filter_update(filter_t *f, const double gyro[3], const double acc[3], double dt);

#endif /* ATTITUDE_H */
