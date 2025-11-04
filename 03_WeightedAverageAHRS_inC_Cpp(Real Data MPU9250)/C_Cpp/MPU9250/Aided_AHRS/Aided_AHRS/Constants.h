
#include "math.h"

// Constants needed

//#define dt_5ms			(5);
#define dt				(0.005);
#define pi				(3.14159265358979);
#define r2d				(57.2957795130823)		// 180/pi
#define d2r				(0.0174532925199433)	// pi/180

#define AlignTime			(1.0)  // 1 second  may be increased accordingly 
#define	percentage_weight	 (10)

// Offline Computed from 6 axis positions data for accelerometer calibration
#define offset_xup      (0.8399);
#define offset_xdown    (-0.5530);
#define offset_yup      (1.1986);
#define offset_ydown    (-1.1106);
#define offset_zup      ( 1.5446);
#define offset_zdown    (-1.4022);


// Offline Computed for Magnetometer calibration
#define mgx_scale ( 2.87695360183716)
#define mgy_scale ( 2.89569568634033)
#define mgz_scale ( 2.78324174880981)

#define mgx_bias ( 9.88952779769897)
#define mgy_bias ( 26.7851877212524)
#define mgz_bias (8.34972524642944)