// sixense_wrapper.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "sixense_wrapper.h"
#include "sixense.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "windows.h"
#include "process.h"

LARGE_INTEGER Frequency;
LARGE_INTEGER start_time;



long long getTime(){
	LARGE_INTEGER timestamp;
	QueryPerformanceCounter(&timestamp);
	LARGE_INTEGER from_start;
	from_start.QuadPart = timestamp.QuadPart - start_time.QuadPart;
	from_start.QuadPart *= 10000;
	from_start.QuadPart /= Frequency.QuadPart;
	if (from_start.QuadPart % 10 < 5){
		return from_start.QuadPart / 10;
	}
	else{
		return (from_start.QuadPart / 10) + 1;
	}
}

#define ELEMS 2

typedef struct _dataHistory {
	sixenseControllerData data[ELEMS];
	long long times[ELEMS];
} dataHistory;

dataHistory histories[2];


void printControllerData(sixenseControllerData* data){
	printf("controller=%d (%.2f, %.2f, %.2f) hand=%d\n", data->controller_index, data->pos[0], data->pos[1], data->pos[2], data->packet_type, data->which_hand);
}

void printHistory(){
	/*
	printf("data history\n");
	printf("%lld ", histories[0].times[0]);
	printControllerData(&(histories[0].data[0]));
	printf("%lld ", histories[0].times[1]);
	printControllerData(&(histories[0].data[1]));
	printf("%lld ", histories[1].times[0]);
	printControllerData(&(histories[1].data[0]));
	printf("%lld ", histories[1].times[1]);
	printControllerData(&(histories[1].data[1]));
	*/
}



void pushToHistory(sixenseControllerData* pdata, long long time){
	int c = pdata->controller_index;
	memcpy(&(histories[c].data[0]), &(histories[c].data[1]), sizeof sixenseControllerData);
	histories[c].times[0] = histories[c].times[1];
	memcpy(&(histories[c].data[1]), pdata, sizeof sixenseControllerData);
	histories[c].times[1] = time;
}

CRITICAL_SECTION CriticalSection;

#define EPS 0.01f
bool isEqual(sixenseControllerData* data1, sixenseControllerData* data2){
	return (data1->pos[0] - data2->pos[0]) < EPS &&
		(data1->pos[1] - data2->pos[1]) < EPS &&
		(data1->pos[2] - data2->pos[2]) < EPS;
}

volatile bool stop = false;

void update_loop(void* params){
	sixenseControllerData current_controller;
	while (!stop){
		for (int c = 0; c < 2; c++){
			sixenseGetNewestData(c, &current_controller);
			long long time = getTime();
			if (!isEqual(&current_controller,&(histories[c].data[1])) ||
				(time - histories[c].times[1]) >= 5){
					EnterCriticalSection(&CriticalSection);
				//	printf("got data\n   ");
				//	printControllerData(&current_controller);
					pushToHistory(&current_controller,time);
				//	printHistory();
					LeaveCriticalSection(&CriticalSection);
			}
		}
	}
}

void predictPosition(int controller, sixenseControllerData* data, long long forward_time_interval, long long dt){
	for (int i = 0; i < 3; i++){
		//predicted position = x_1 + v*t
		//v = (x_1 - x_0)/dt
		float v = (histories[controller].data[1].pos[i] - histories[controller].data[0].pos[i]) / dt;
		data->pos[i] = v*forward_time_interval + histories[controller].data[1].pos[i];
	}
}

void invertQuaternion(float* quat){
	float norm = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3];
	quat[0] *= -1.0f / norm;
	quat[1] *= -1.0f / norm;
	quat[2] *= -1.0f / norm;
	quat[2] *= 1.0f / norm;
}

void quatToAxisAngle(float* quat,float* axis, float* angle){
	*angle = (float)(2 * acos(quat[3]));
	float s = sqrt(1 - (quat[3] * quat[3])); // assuming quaternion normalised then w is less than 1, so term always positive.
	if (s < 0.001) {
		memcpy(axis, quat, 3 * (sizeof(float)));
	} else {
		axis[0] = quat[0] / s;
		axis[1] = quat[1] / s;
		axis[2] = quat[2] / s;
	}
}

void normalizeVector3(float* v,float* result){
	float norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (norm > 0){
		result[0] = v[0] / norm;
		result[1] = v[1] / norm;
		result[2] = v[2] / norm;
	}
}

void normalizeQuat(float* q, float* result){
	float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3]*q[3]);
	result[0] = q[0] / norm;
	result[1] = q[1] / norm;
	result[2] = q[2] / norm;
	result[3] = q[3] / norm;
}

void axisAngleToQuat(float* quat, float* axis, float angle){
	float normalizedAxis[3];
	normalizeVector3(axis, normalizedAxis);

	float c = cos(angle / 2.0f);
	float s = sin(angle / 2.0f);
	quat[0] = s*normalizedAxis[0];
	quat[1] = s*normalizedAxis[1];
	quat[2] = s*normalizedAxis[2];
	quat[3] = c;
	normalizeQuat(quat, quat);
}

void mult_quaternions(float* a, float*b, float* ab){
	ab[0] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]; //x
	ab[1] = -a[0] * b[2] + a[1] * b[3] + a[2] * b[0] + a[3] * b[1]; //y
	ab[2] = a[0] * b[1] - a[1] * b[0] + a[2] * b[3] + a[3] * b[2]; //z
	ab[3] = -a[0] * b[0] - a[1] * b[1] - a[2] * b[2] + a[3] * b[3]; //w

}

float identity_quat[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

void predictRotation(int controller, sixenseControllerData* data, long long forward_time){
	float q1[4] = { histories[controller].data[0].rot_quat[0],
		histories[controller].data[0].rot_quat[1],
		histories[controller].data[0].rot_quat[2],
		histories[controller].data[0].rot_quat[3] };
	float q2[4] = { histories[controller].data[1].rot_quat[0],
		histories[controller].data[1].rot_quat[1],
		histories[controller].data[1].rot_quat[2],
		histories[controller].data[1].rot_quat[3] };
	float t1 = histories[controller].times[0];
	float t2 = histories[controller].times[1];
	float t3 = forward_time;
	if (t1 == 0 || t2 == 0 || t3 == 0){
		memcpy(data->rot_quat,identity_quat,4*(sizeof(float)));
		return;
	}

	float q1i[4] = { q1[0], q1[1], q1[2], q1[3] };
	float rot[4];
	mult_quaternions(q2, q1i,rot); // rot is the rotation from t1 to t2

	float dt = (t3 - t1) / (t2 - t1); // dt = extrapolation factor
	float ang;
	float axis[3];
	quatToAxisAngle(rot, axis, &ang);// find axis-angle representation
	if (ang > M_PI) ang -= 2*M_PI;  // assume the shortest path
	ang = ang*dt; // multiply angle by the factor
	while (ang > 2 * M_PI){
		ang -= 2 * M_PI;
	}
	while (ang <- 2 * M_PI){
		ang += 2 * M_PI;
	}
	float q4[4];
	axisAngleToQuat(q4, axis, ang); //create quat from axis and angle
	mult_quaternions(q4, q1, data->rot_quat);// combine with first rotation, save to the output
}



void getReading(int controller, sixenseControllerData* data, long long predictMillis){
	long long forward_time = getTime() + predictMillis;
	printf("getReading controller(%d,__,predictMillis=%d)\n",controller,predictMillis);
	EnterCriticalSection(&CriticalSection);
	printHistory();
	memcpy(data, &(histories[controller].data[1]), sizeof sixenseControllerData);
	printControllerData(data);
	long long forward_time_interval = forward_time - histories[controller].times[1];
	long long dt = histories[controller].times[1] - histories[controller].times[0];
	predictPosition(controller, data, forward_time_interval,dt);
	predictRotation(controller,data,forward_time);
	LeaveCriticalSection(&CriticalSection);
	
}


void exit(){
	printf("wait until thread is dead\n");
	stop = true;
	Sleep(100);
	printf("delete critical section\n");
	DeleteCriticalSection(&CriticalSection);
	printf("exit sixense\n");
	sixenseExit();
}

void init(){
	QueryPerformanceFrequency(&Frequency);
	printf("Init sixense\n");
	sixenseInit();
	printf("Init critical section\n");
	QueryPerformanceCounter(&start_time);
	if (!InitializeCriticalSectionAndSpinCount(&CriticalSection,
		0x00000400))
		return;
	printf("start update loop\n");
	_beginthread(update_loop, 0, (void*)12);
}
