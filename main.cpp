// gcc -c -Wall -Wno-format-truncation -ffunction-sections -fdata-sections -fpack-struct -lm libs/RPI-serial/RPIserial.c -o RPIserial.o
// gcc -c -Wall -Wno-format-truncation -ffunction-sections -fdata-sections -fpack-struct -lm libs/Lightware_SF40-c/lightwareSF40.c -o lightware.o
// g++ -c main.cpp -Wall -Wno-format-truncation -std=c++11 $(pkg-config --cflags opencv4) -o main.o
// g++ -o program main.o RPIserial.o lightware.o $(pkg-config --libs opencv4) -lm -pthread -ffunction-sections -fdata-sections



#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <vector>


extern "C" {
	#include "libs/Lightware_SF40-c/lightwareSF40.h"
}

#define RANGE 500		// Measuring range in centimeters
#define RESOLUTION 5	// Amount of centimeter per pixel
#define SIZE 3			// Amount of pixels 

#define CANVAS_SIZE ((RANGE * 2) / 5)

#define LIDAR_LOCATION (CANVAS_SIZE / 2)

pthread_mutex_t imageMutex;
cv::Mat outputImg = cv::Mat::ones(CANVAS_SIZE, CANVAS_SIZE, CV_8UC1) * 255;
bool newImage = false;

void InitLidar(){
    setupLidar("/dev/ttyS0", LIDAR_921K6);

	while(true){
        char name[16];
		getName(name);
		printf("Received lidar ID: %s.\n\r", name);
		if (strcmp(name, MODEL_NUMBER) == 0) break;
		usleep(100000);
	}

    // Wait for the motor to enter the opperation mode
	while(true){
		motorState_t state = getMotorState();
		usleep(100000);
		printf("Motor status: %d.\n\r", state);
		if(state == 0 || state == MOTOR_PRE_STARTUP || state == MOTOR_WAIT_ON_REVS) continue;
		else if(state == MOTOR_ERROR){
			printf("Motor has failed to communicate.\n\r");
			restartLidar(getToken());
		}
		else break;
	}

	// Turn on the laser
	while(true){
		enableLaser(true);
		bool laserState = checkLaser();
		printf("Laser status: %d.\n\r", laserState);
		if(laserState) break;
		usleep(100000);
	}

	 // Get lidar voltage
    printf("Lidar voltage: %.3f.\n\r", getVoltage());

    // Get motor voltage
    printf("Motor voltage: %.3f.\n\r", getMotorVoltage());

	setOutputRate(LIDAR_20010_PPS);

	printf("Lidar setup finished.\n\r");
}

void* processStream(void* arg){
	streamOutput_t output;
	static uint16_t previousIndex = 0;

	cv::Mat img(CANVAS_SIZE, CANVAS_SIZE, CV_8UC1, cv::Scalar(255));

    while (1){
		if(getStream(&output) != 0) continue;

		if(previousIndex != output.revolutionIndex){
			previousIndex = output.revolutionIndex;
			
			pthread_mutex_lock(&imageMutex);   
			outputImg = img.clone();
			newImage = true;
			pthread_mutex_unlock(&imageMutex); 

        	 img.setTo(cv::Scalar(255));
		}
		for (int i = 0; i<output.pointCount; i++){
			float angle = 2.0f * M_PI * (float)(output.pointStartIndex + i) / (float)output.pointTotal;
			int px = LIDAR_LOCATION + (int)(output.pointDistances[i] / RESOLUTION * cosf(angle));
			int py = LIDAR_LOCATION - (int)(output.pointDistances[i] / RESOLUTION * sinf(angle));

			if (px>=0 && px < CANVAS_SIZE && py >= 0 && py < CANVAS_SIZE)
                img.at<uchar>(py, px) = 0; 
		}
	} 
	return nullptr;
}

int main(int argc, char *argv[]){
    InitLidar();
	
	while(getStreamState() != 3){
		enableStream(true);
		usleep(100000);
	}

	// Initialize the mutex
    if (pthread_mutex_init(&imageMutex, NULL) != 0) {
        fprintf(stderr, "Mutex init failed\n");
        return 1;
    }
	pthread_t streamThread;
	pthread_create(&streamThread, NULL, processStream, NULL);


    while (true){
		cv::Mat display;

		pthread_mutex_lock(&imageMutex);   
		if(!newImage){
			pthread_mutex_unlock(&imageMutex); 
			continue;
		}

		display = outputImg.clone();
		newImage = false;
		pthread_mutex_unlock(&imageMutex); 

		if (!display.empty())
            cv::imwrite("lidar_frame.png", display);

        if (cv::waitKey(1) == 27) break; // exit on ESC
	}
	
	pthread_cancel(streamThread);
    pthread_join(streamThread, nullptr);
    pthread_mutex_destroy(&imageMutex);

    return 0;
}