// SixenceUseTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include "sixense.h"
#include <windows.h>
#include <process.h>
#include <fstream>
#include <iostream>
#include <random>
#include "structdef.h"
#include "sixense_wrapper.h"

void printData(sixenseControllerData* data){
	printf("c=%d  (%.2f, %.2f, %.2f) packet_type=%u hand=%d\n",data->controller_index, data->pos[0], data->pos[1], data->pos[2], data->packet_type, data->which_hand);
}

void polling_loop(){
	sixenseControllerData current_controller;
	for (int i = 0; i < 10000; i++){
		printf("step %d || ",i);
		getReading(0, &current_controller, 4);
		Sleep(rand() % 10);
		getReading(1, &current_controller, 4);
		printData(&current_controller);
		Sleep(rand() % 10);
		//getReading(1, &current_controller, 4);
		//printData(&current_controller);
	}
}


int _tmain(int argc, _TCHAR* argv[])
{
	sixenseControllerData newData;
	printf("Let some time for driver to init\n");
	sixenseInit();
	Sleep(1000);
	printf("Init wrapper\n");
	init_wrapper();
	Sleep(1000);
	polling_loop();
	exit_wrapper();
	return 0;
}