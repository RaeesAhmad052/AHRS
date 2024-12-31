#include <stdio.h>
#include <conio.h>
#include <iostream>
#include "Constants.h"
#include "math.h"
using namespace std;


int mian(void);
void Align_IMU(void);
void AHRS_9DOF(void);
void get_imu_data(FILE *fp);
FILE *Input, *fp, *Output; 

double Time1, abx, aby, abz, wbx, wby, wbz, mgx, mgy, mgz, temp1, temp2,temp3,total_a;
double abx1=0, aby1=0, abz1=0, mgx1=0, mgy1=0, mgz1=0;
double XAccelOffset, YAccelOffset ,ZAccelOffset, mag_x_level , mag_y_level;
double pitch=0.0, roll=0.0, yaw=0.0, yaw_out=0.0, dummy;
double pitch_q, roll_q, yaw_q;
double pitch_a, roll_a, yaw_m;
double q0=0,q1=0,q2=0,q3=0, q0_p=0,q1_p=0,q2_p=0,q3_p=0;
double Time = 0;
unsigned int average_cnt = 0, weightCount, IMU_MODE;
double wtc , wtp, countAvg=0;
int eof;
double yaw_p = 0.0, pitch_p = 0.0, roll_p = 0.0;
double yaw_pp = 0.0, pitch_pp = 0.0, roll_pp = 0.0;

int main() {


	char filename[30] = "imu_data9dof.dat";
	errno_t err;
	int dispcnt = 0;

	//printf("Enter the name of file with extension =  ");
	//	scanf("%s = ", filename);
	//scanf_s("%30s", filename, (unsigned)_countof(filename)); 

	//	if( (err =(fopen_s( &Input, "imu_ascii.dat", "r")) ==0 ) ){
	if( !(err =(fopen_s( &Input, filename, "r") ) ) ){
		printf("Input File is opened ...\n"); 	
	}
	else {
		printf("Input File is not opened !\n"); 
	}


	if( (err =(fopen_s( &Output, "out.dat", "w")) ==0 ) ){
		printf("out.dat File is opened ...\n"); 	
	}
	else {
		printf("out.dat File is not opened !\n");
	}

	// Initialization of Filter and IMU Mode
	weightCount = (100/percentage_weight);
	IMU_MODE = 1;

	while(eof != EOF){

		get_imu_data(Input);

		Time += dt;

		if (IMU_MODE==1)
			Align_IMU();
		else if(IMU_MODE==2)
		{
			AHRS_9DOF();
		}


		// file writing
		if(yaw < 0.0) {
			yaw_out = yaw + 2*pi;
		}
		else {
			yaw_out = yaw;
		}
		fprintf(Output, "%20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f \t\t %20.8f\n", Time, yaw_out, pitch, roll,yaw_m, pitch_a, roll_a,Time1,temp1,temp2,temp3);

		//Display Time in minutes
		dispcnt++;
		if (dispcnt == 12000) {
			dispcnt = 0;
			printf("\b\b\b\b\b\b\b\b\b %d min",((int)(Time+0.5)/60));
		}
	}

	fclose(Output);
	printf("\n\nPresss Any key to Exit ... ");

	_getch();
	return 0;
}

void get_imu_data(FILE *fp){

	//	eof = fscanf(fp," %lf %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf",&Time1,&abx,&aby,&abz,&wbx,&wby,&wbz,&mgx,&mgy,&mgz);
	eof = fscanf_s(fp," %lf %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf",&Time1,&abx,&aby,&abz,&wbx,&wby,&wbz,&mgx,&mgy,&mgz,&temp1,&temp2,&temp3);


	/////////// Accelerometers Bias Compansation /////////////////////////
	if (abx > 0){	XAccelOffset = offset_xup;		}
	else{			XAccelOffset = -offset_xdown;	}
	if (aby > 0){	YAccelOffset = offset_yup;		}
	else{			YAccelOffset = -offset_ydown;	}
	if(abz > 0){	ZAccelOffset = offset_zup;		}
	else{			ZAccelOffset = -offset_zdown;	}

	abx -= XAccelOffset;
	aby -= YAccelOffset;
	abz -= ZAccelOffset;

	wbx -= wbx_bias;
	wby -= wby_bias;
	wbz -= wbz_bias; 

	total_a = sqrt(abx*abx + aby*aby + abz*abz);

	// Magnetometer calibration
	mgx = mgx*mgx_scale - mgx_bias;
	mgy = mgy*mgy_scale - mgy_bias;
	mgz = mgz*mgz_scale - mgz_bias;

}


void Align_IMU(){


	// Averaging data for initialization
	average_cnt++;
	abx1 = ((average_cnt-1)*abx1 + abx) /average_cnt;
	aby1 = ((average_cnt-1)*aby1 + aby) /average_cnt;
	abz1 = ((average_cnt-1)*abz1 + abz) /average_cnt;

	mgx1 = ((average_cnt-1)*mgx1 + mgx) /average_cnt;
	mgy1 = ((average_cnt-1)*mgy1 + mgy) /average_cnt;
	mgz1 = ((average_cnt-1)*mgz1 + mgz) /average_cnt;

	if(  Time > AlignTime){
		average_cnt = 0;

		// Compute initial Attitude angles
		pitch = atan2(abx1 , sqrt(aby1*aby1 + abz1*abz1) );
		roll  = atan2(-aby1 , sqrt(abx1*abx1 +  abz1*abz1) );

		mag_x_level = mgx1*cos(pitch) + mgy1*sin(roll)*sin(pitch) + mgz1*cos(roll)*sin(pitch);
		mag_y_level = mgy1*cos(roll) - mgz1 * sin(roll);

		yaw =  atan2(-mag_y_level,mag_x_level);

		//		if(yaw < 0.0) yaw += 2*pi;
		// Initialization Quaternions
		q0 = cos(yaw/2.0)*cos(pitch/2.0)*cos(roll/2.0) + sin(yaw/2.0)*sin(pitch/2.0)*sin(roll/2.0);
		q1 = cos(yaw/2.0)*cos(pitch/2.0)*sin(roll/2.0) - sin(yaw/2.0)*sin(pitch/2.0)*cos(roll/2.0);
		q2 = cos(yaw/2.0)*sin(pitch/2.0)*cos(roll/2.0) + sin(yaw/2.0)*cos(pitch/2.0)*sin(roll/2.0);
		q3 = sin(yaw/2.0)*cos(pitch/2.0)*cos(roll/2.0) - cos(yaw/2.0)*sin(pitch/2.0)*sin(roll/2.0);

		q0_p  = q0;
		q1_p  = q1;
		q2_p  = q2;
		q3_p  = q3;
		abx1=0, aby1=0, abz1=0, mgx1=0, mgy1=0, mgz1=0;

		IMU_MODE = 2;
	}
}


void AHRS_9DOF(){

	// Quaternions Updating @0.005 s
	q0 = q0_p - 0.5*(q1_p*wbx + q2_p*wby + q3_p*wbz)*dt;
	q1 = q1_p + 0.5*(q0_p*wbx + q2_p*wbz - q3_p*wby)*dt;
	q2 = q2_p + 0.5*(q0_p*wby - q1_p*wbz + q3_p*wbx)*dt;
	q3 = q3_p + 0.5*(q0_p*wbz + q1_p*wby - q2_p*wbx)*dt;

	// Attitude Update
	yaw_q = atan2( 2.0*(q1*q2 + q0*q3),  (q0*q0 + q1*q1 - q2*q2 - q3*q3) ); 
	dummy = -2*(q1*q3-q0*q2);
	pitch_q	= asin(dummy); // atan2( dummy , (sqrt(1- dummy*dummy)));		
	roll_q	= atan2(2*(q2*q3+q0*q1) ,   (q0*q0 - q1*q1 - q2*q2 + q3*q3) );


	// Moving Averagingsdfd
	average_cnt++;
	abx1 = ((average_cnt-1)*abx1 + abx) /average_cnt;
	aby1 = ((average_cnt-1)*aby1 + aby) /average_cnt;
	abz1 = ((average_cnt-1)*abz1 + abz) /average_cnt;

	mgx1 = ((average_cnt-1)*mgx1 + mgx) /average_cnt;
	mgy1 = ((average_cnt-1)*mgy1 + mgy) /average_cnt;
	mgz1 = ((average_cnt-1)*mgz1 + mgz) /average_cnt;

	if(average_cnt == 20 ){  // (20*0.005 =0.100 sec) 100 milli second Measurement Update rate 

		average_cnt = 0;

		pitch_a = atan2(abx1 , sqrt(aby1*aby1 + abz1*abz1) );
		roll_a  = atan2(-aby1 , sqrt(abx1*abx1 +  abz1*abz1) );

		mag_x_level = mgx1*cos(pitch_a) + mgy1*sin(roll_a)*sin(pitch_a) + mgz1*cos(roll_a)*sin(pitch_a);
		mag_y_level = mgy1 * cos(roll_a) - mgz1 * sin(roll_a);

		yaw_m =  atan2(-mag_y_level,mag_x_level);

		// Aiding Filter for smoth output 
		if(countAvg < weightCount){
			countAvg++;
			wtc = (double)(1/countAvg);
			wtp = (double)(1 -wtc);
		}


		if (yaw_m < 0.0 && yaw_q > 0.0){
			yaw_m += 2*pi;
		}
		else if(yaw_m > 0.0 && yaw_q < 0.0){
			yaw_m -= 2*pi;
		}

		yaw    = (wtp)*yaw_q    + (wtc)*yaw_m  ;
		pitch  = (wtp)*pitch_q  + (wtc)*pitch_a;
		roll   = (wtp)*roll_q   + (wtc)*roll_a ;

		abx1=0, aby1=0, abz1=0, mgx1=0, mgy1=0, mgz1=0;
	}
	else {
		yaw    = yaw_q  ;
		pitch  = pitch_q;
		roll   = roll_q ;
	}


	q0 = cos(yaw/2.0)*cos(pitch/2.0)*cos(roll/2.0) + sin(yaw/2.0)*sin(pitch/2.0)*sin(roll/2.0);
	q1 = cos(yaw/2.0)*cos(pitch/2.0)*sin(roll/2.0) - sin(yaw/2.0)*sin(pitch/2.0)*cos(roll/2.0);
	q2 = cos(yaw/2.0)*sin(pitch/2.0)*cos(roll/2.0) + sin(yaw/2.0)*cos(pitch/2.0)*sin(roll/2.0);
	q3 = sin(yaw/2.0)*cos(pitch/2.0)*cos(roll/2.0) - cos(yaw/2.0)*sin(pitch/2.0)*sin(roll/2.0);

	q0_p  = q0;
	q1_p  = q1;
	q2_p  = q2;
	q3_p  = q3;

	// smoothing filter
	pitch = (0.7*pitch + 0.2*pitch_p + 0.1*pitch_pp);
	roll = (0.7*roll + 0.2*roll_p + 0.1*roll_pp);
	pitch_pp = pitch_p;
	roll_pp = roll_p;
	pitch_p = pitch;
	roll_p = roll;
}