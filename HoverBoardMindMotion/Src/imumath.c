//IMU mounting and angle math (imumath.h)
#include "imumath.h"

uint8_t ImuAxes(uint16_t mount, int8_t* ax){
	if(mount==0||mount>36) return 0;
	int8_t up = (int8_t)((mount-1)/6), fwd = (int8_t)((mount-1)%6);
	int8_t i = (int8_t)(up%3), j = (int8_t)(fwd%3);
	if(i==j) return 0;
	//Y = Z x X: e_i x e_j = +-e_k, + for the cyclic order (x y z); the signs of the two multiply in
	int8_t k = (int8_t)(3-i-j), neg = (int8_t)((up>=3)^(fwd>=3)^((j-i+3)%3!=1));
	ax[0] = fwd;
	ax[1] = (int8_t)(k+(neg ? 3 : 0));
	ax[2] = up;
	return 1;
}

void ImuToBoard(const int8_t* ax, const int16_t* chip, float* board){
	for(uint8_t n=0;n<3;n++){
		float v = (float)chip[ax[n]%3];
		board[n] = ax[n]>=3 ? -v : v;
	}
}

//atan on 0..1 as z * (45 + (1 - z) * (14.02 + 3.80 z)) degrees (pi/4 z - z (z - 1)(0.2447 + 0.0663 z),
//max error 0.0015 rad), then the octant
float Atan2Deg(float y, float x){
	float ax = x<0 ? -x : x, ay = y<0 ? -y : y;
	if(ax==0.0f&&ay==0.0f) return 0.0f;
	float z = ax>ay ? ay/ax : ax/ay;
	float a = z*(45.0f+(1.0f-z)*(14.02f+3.80f*z));
	if(ay>ax) a = 90.0f-a;
	if(x<0) a = 180.0f-a;
	return y<0 ? -a : a;
}
