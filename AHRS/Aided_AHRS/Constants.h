
#include "math.h"

// Constants needed

//#define dt_5ms			(5);
#define dt				(0.005);
#define pi				(3.14159265358979);
#define r2d				(57.2957795130823)		// 180/pi
#define d2r				(0.0174532925199433)	// pi/180

#define AlignTime			(5.0)  // 1 second  may be increased accordingly 
#define	percentage_weight	 (10)

// Offline Computed from 6 axis positions data for accelerometer calibration
#define offset_xup      (0.8399);
#define offset_xdown    (-0.5530);
#define offset_yup      (1.1986);
#define offset_ydown    (-1.1106);
#define offset_zup      ( 1.5446);
#define offset_zdown    (-1.4022);


// Offline Computed for Magnetometer calibration

#define mgx_scale (1.01570050084345)	//(1.0)//( 2.87695360183716)
#define mgy_scale (0.984777500877184)	//(1.0)//( 2.89569568634033)
#define mgz_scale (1.0)					//( 2.78324174880981)

#define mgx_bias (-5.86063445999441)	//( 9.88952779769897)
#define mgy_bias (29.8989579820804)		//( 26.7851877212524)
#define mgz_bias (10.9179233862754)		//(8.34972524642944)

#define wbx_bias ( -0.000285502895686776)
#define wby_bias ( 0.000193062871998542)
#define wbz_bias ( 0.000163157297291633)