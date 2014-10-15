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

#define MIN_PPREDICT_TIME 1
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
			EnterCriticalSection(&CriticalSection);
				if (stop){
					LeaveCriticalSection(&CriticalSection);
					return;
				}
				if (!isEqual(&current_controller, &(histories[c].data[1])) ||
					(time - histories[c].times[1]) >= 5){
					pushToHistory(&current_controller, time);
				}
			LeaveCriticalSection(&CriticalSection);
		}
	}
}

float identityPos[3] = { 0.0f, 0.0f, 0.0f };

void predictPosition(sixenseControllerData* prev_data,
					sixenseControllerData* current_data,
					sixenseControllerData* data,
					long long prev_time,
					long long current_time,
					long long forward_time){
	if (current_time == 0 ||
		forward_time <= 0 ||
		current_data->enabled == 0 ||
		current_data->is_docked == 1){
		//return identity quaternion
		memcpy(data->pos, identityPos, 3 * (sizeof(float)));
		return;
	}
	if (current_time - prev_time > -MIN_PPREDICT_TIME &&
		current_time - prev_time < MIN_PPREDICT_TIME){
		//return last position
		memcpy(data->pos, current_data->pos, 3 * sizeof(float));
	}
	else{
		for (int i = 0; i < 3; i++){
			float velocity = (current_data->pos[i] - prev_data->pos[i]) / (current_time - prev_time);
			data->pos[i] = current_data->pos[i] + velocity*(forward_time - current_time);
		}
	}
}

void normalizeVector3(float* v, float* result){
	float norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (norm > 0){
		result[0] = v[0] / norm;
		result[1] = v[1] / norm;
		result[2] = v[2] / norm;
	}
}

void invertQuaternion(float* quat, float* result){
	float norm2 = (quat[0] * quat[0]) + (quat[1] * quat[1]) + (quat[2] * quat[2]) + (quat[3] * quat[3]);
	result[0] = -quat[0] / norm2;
	result[1] = -quat[1] / norm2;
	result[2] = -quat[2] / norm2;
	result[3] = quat[3] / norm2;
}

void normalizeQuaternion(float* quat, float* result){
	float norm = sqrt((quat[0] * quat[0]) + (quat[1] * quat[1]) + (quat[2] * quat[2]) + (quat[3] * quat[3]));
	result[0] = quat[0] / norm;
	result[1] = quat[1] / norm;
	result[2] = quat[2] / norm;
	result[3] = quat[3] / norm;
}

void quatToAxisAngle(float* quat,float* axis, float* angle){
	float normalized[4];
	normalizeQuaternion(quat,normalized);
	*angle = (float)(2 * acos(normalized[3]));
	float s = sqrt(1 - (normalized[3] * normalized[3])); // assuming quaternion normalised then w is less than 1, so term always positive.
	if (s < 0.001f) {
		axis[0] = 1;
		axis[1] = 0;
		axis[2] = 0;
	} else {
		axis[0] = normalized[0] / s;
		axis[1] = normalized[1] / s;
		axis[2] = normalized[2] / s;
	}
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
	normalizeQuaternion(quat, quat);
}

void mult_quaternions(float* a, float*b, float* ab){
	ab[0] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]; //x
	ab[1] = -a[0] * b[2] + a[1] * b[3] + a[2] * b[0] + a[3] * b[1]; //y
	ab[2] = a[0] * b[1] - a[1] * b[0] + a[2] * b[3] + a[3] * b[2]; //z
	ab[3] = -a[0] * b[0] - a[1] * b[1] - a[2] * b[2] + a[3] * b[3]; //w
}

float identity_quat[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

void predictRotation(sixenseControllerData* prev_data,
					sixenseControllerData* current_data,
					sixenseControllerData* data,
					long long prev_time,
					long long current_time,
					long long forward_time){
	
	if (current_time == 0 ||
		forward_time <= 0 ||
		current_data->enabled == 0 ||
		current_data->is_docked == 1){
		//return identity quaternion
		memcpy(data->rot_quat, identity_quat, 4 * (sizeof(float)));
		return;
	}
	if (current_time - prev_time > -MIN_PPREDICT_TIME &&
		current_time - prev_time < MIN_PPREDICT_TIME){
		//return last position
		memcpy(data->rot_quat,current_data->rot_quat, 4 * sizeof(float));
	}
	else{
		float* q1 = prev_data->rot_quat;
		float* q2 = current_data->rot_quat;


		float t1 = prev_time;
		float t2 = current_time;
		float t3 = forward_time;

		float q1i[4];
		invertQuaternion(q1, q1i);
		float rot[4];
		mult_quaternions(q2, q1i, rot); // rot is the rotation from t1 to t2

		float dt = (t3 - t1) / (t2 - t1); // dt = extrapolation factor
		float ang;
		float axis[3];
		quatToAxisAngle(rot, axis, &ang);// find axis-angle representation
		if (ang > M_PI) ang -= 2 * M_PI;  // assume the shortest path
		ang = ang*dt; // multiply angle by the factor
		while (ang > 2 * M_PI){
			ang -= 2 * M_PI;
		}
		while (ang < -2 * M_PI){
			ang += 2 * M_PI;
		}
		float q4[4];
		axisAngleToQuat(q4, axis, ang); //create quat from axis and angle
		mult_quaternions(q4, q1, data->rot_quat);// combine with first rotation, save to the output
	}
}

void printControllerData(sixenseControllerData* data){
	printf("controller=%d (%.2f, %.2f, %.2f) hand=%c\n", data->controller_index, data->pos[0], data->pos[1], data->pos[2], data->packet_type, data->which_hand);
}

//thread unsafe, call only inside critical section
void printHistory(){
	printf("data history\n");
	printf("%lld ",histories[0].times[0]);
	printControllerData(&(histories[0].data[0]));
	printf("%lld ", histories[0].times[1]);
	printControllerData(&(histories[0].data[1]));
	printf("%lld ", histories[1].times[0]);
	printControllerData(&(histories[1].data[0]));
	printf("%lld ", histories[1].times[1]);
	printControllerData(&(histories[1].data[1]));
}

SIXENSE_WRAPPER_API int getReading(int controller, sixenseControllerData* data, long predictMillis){
	long long forward_time = getTime() + predictMillis;
	sixenseControllerData prev_data,current_data;
	long long prev_time, current_time;

	EnterCriticalSection(&CriticalSection);
		prev_time = histories[controller].times[0];
		current_time = histories[controller].times[1];
		memcpy(&current_data, &(histories[controller].data[1]), sizeof sixenseControllerData);
		memcpy(&prev_data, &(histories[controller].data[0]), sizeof sixenseControllerData);
	LeaveCriticalSection(&CriticalSection);

	memcpy(data, &(current_data), sizeof sixenseControllerData);
	predictPosition(&prev_data, &current_data, data, prev_time, current_time, forward_time);
	predictRotation(&prev_data, &current_data, data, prev_time, current_time, forward_time);
	return 1;
}


SIXENSE_WRAPPER_API int exit_wrapper(){
	printf("wait until thread is dead\n");
	EnterCriticalSection(&CriticalSection);
	stop = true;
	LeaveCriticalSection(&CriticalSection);
	printf("delete critical section\n");
	DeleteCriticalSection(&CriticalSection);
	printf("exit sixense\n");
	sixenseExit();
	return 2;
}

SIXENSE_WRAPPER_API int init_wrapper(){
	QueryPerformanceFrequency(&Frequency);
	printf("Init sixense\n");
	//sixenseInit();
	//turn off filtering to reduce delay
	//sixenseSetFilterEnabled(0);
	printf("Init critical section\n");
	QueryPerformanceCounter(&start_time);
	if (!InitializeCriticalSectionAndSpinCount(&CriticalSection,
		4000))
		return 0;
	printf("start update loop\n");
	_beginthread(update_loop, 0, (void*)12);
	return 3;
}