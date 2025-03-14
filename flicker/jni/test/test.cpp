/********************************************************************************
Copyright (c) 2025, STMicroelectronics - All Rights Reserved
This file is licensed under open source license ST SLA0103
********************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <inttypes.h>
#include <pthread.h>

#include "vd628x_interface.h"

#include <dlfcn.h>

#define LOG printf

pthread_t PollThread;
SpectralSensorInterface * pIO;
uint8_t PollThreadIsRunning = 0;
uint8_t ExitMainLoop = 0;

void sighandler(int signal)
{
	LOG("SIGNAL Handler called, signal = %d\n", signal);
	ExitMainLoop = 1;
}

static void *PollThreadRoutine(void * )
{
#define NB_SAMPLES 1
	int n;
	NCSDataMultiSpectralSensor SensorData[NB_SAMPLES];
	NCSDataMultiSpectralSensor * pSD;
	// struct ConfigureParameters CfgParams;

	//int poll_count = 0;
	int wait_in_ms = 500;

	// wait first before polling the first time
	usleep(100000);

	while (PollThreadIsRunning) {

		n = pIO->PollSensorData(NB_SAMPLES, (void *)&SensorData);
		pSD = SensorData;

		// after 1 second lets configure dynamically a new sampling frequency
		//if (poll_count * wait_in_ms > 60000) {
		//	CfgParams.configType = SamplingFrequency;
		//	CfgParams.configPayload.samplingFrequency = 600; // driver will take first greater value of the table (640)
		//	pIO->Configure(&CfgParams, 1);
		//}
		//else
		//	poll_count++;

		if (n > 0) {
			LOG("\n\n============================ Poll sensor Data ================================\n");
			for (uint8_t i=0; i<NB_SAMPLES; i++) {
				LOG("===== flickerInfo ===== \n");
				LOG("Valid: %d\n", pSD->flickerInfo.isValid);
				if (pSD->flickerInfo.isValid) {
					LOG("Primary Channel: ");
					switch (pSD->flickerInfo.channel) {
						case RedChannel:
							LOG("Red\n");
							break;
						case GreenChannel:
							LOG ("Green\n");
							break;
						case BlueChannel:
							LOG ("Blue\n");
							break;
						case ClearChannel1:
							LOG ("Clear1\n");
							break;
						case ClearChannel2:
							LOG ("Clear2\n");
							break;
						case InfraredChannel:
							LOG ("InfraRed\n");
							break;
						default:
							LOG ("Invalid !\n");
							break;
					}
					//LOG("samplingTime: %lu\n", pSD->flickerInfo.samplingTime);
					LOG("FLICKER: SampFreq: %d, 1st Peak: %f, %f, 2nd Peak: %f, %f, \n",
						pSD->flickerInfo.configuredSamplingFlickerFreq,
						pSD->flickerInfo.firstMaximaPeak.frequency,
						pSD->flickerInfo.firstMaximaPeak.amplitude,
						pSD->flickerInfo.secondMaximaPeak.frequency,
						pSD->flickerInfo.secondMaximaPeak.amplitude);
					LOG("avgRawFlickerData: %d maxRawFlickerData: %d minRawFlickerData: %d\n",
						pSD->flickerInfo.avgRawFlickerData,
						pSD->flickerInfo.maxRawFlickerData,
						pSD->flickerInfo.minRawFlickerData);
					LOG("avgFlickerFreqAmplitude: %d.    exposureGainofFlickerChannel: %f\n",
						pSD->flickerInfo.avgFlickerFreqAmplitude,
						pSD->flickerInfo.expGainOfFlickerChannel);
				}
				pSD++;
			}
		}
		else if (n == 0)
			LOG("PollSensorData - no new samples\n");
		else
			LOG("PollSensorData failed.\n");

		// wait between each poll
		usleep(wait_in_ms*1000);

	}
	return NULL;
}


int main(int, char const **)
{
	// open the so file
	void *sensor_lib = NULL;
	void *retval;
	int err;
	uint32_t count = 0;

	sensor_lib = dlopen("libvd628x_flicker.so", RTLD_LAZY);
	if (NULL == sensor_lib)
	{
		LOG("Failing to open sensor library! Error: %s\n", dlerror());
		return -1;
	}
	dlerror();

	// get the interface object
	typedef void (*GetVD628xInterface_t)(SpectralSensorInterface** ppInterfaceObject);
	GetVD628xInterface_t GetVD628xInterface = (GetVD628xInterface_t)dlsym(sensor_lib, "GetSpectralSensorInterface");
	if (GetVD628xInterface == NULL) {
		LOG("GetVD628xInterface is NULL. Aborting.\n");
		return -1;
	}
	GetVD628xInterface(&pIO);

	// Query Driver info
	QueryInfo SensorInfo;
	struct DriverInformation * pDI;
	SensorInfo.queryType = DriverInfo;
	pIO->QuerySensorInfo(&SensorInfo);
	pDI = (struct DriverInformation *)SensorInfo.pData;

	LOG ("============= Query Driver info ==================\n");
	LOG ("Name of the sensor: %s\n", pDI->name);
	LOG ("Vendor: %s\n", pDI->vendor);
	LOG ("Hardware version: %s\n", pDI->hardwareVersion);
	LOG ("Driver version: %d\n", pDI->driverVersion);

	// Query sensor Attributes
	SensorInfo.queryType = SensorAttributes;
	struct SensorAttribute * pSA;
	struct Attribute * pA;
	pIO->QuerySensorInfo(&SensorInfo);
	pSA = (struct SensorAttribute *)SensorInfo.pData;
	pA = (struct Attribute *)pSA->pAttributes;
	LOG ("============= Query Sensor Attributes  ==================\n");
	for (uint8_t i=0; i<pSA->length; i++) {
		LOG("Attribute: %d\n", i);
		LOG("Attribute name: %s\n", pA->attributeName);
		LOG("Min: %f, Max: %f\n", pA->data.range.min, pA->data.range.max);
		pA++;
	}


	// Open sensor
	LOG ("============= Open Sensor  ==================\n");
	pIO->OpenSensor();

	// Configure parameters: change
	struct ConfigureParameters CfgParams;

	LOG ("============= Configure Samping Frequency  ==================\n");
	CfgParams.configType = SamplingFrequency;
	CfgParams.configPayload.samplingFrequency = 2000;  // driver will take first greater value of the table (1024)
	pIO->Configure(&CfgParams, 1);

	// wait 1000 ms
	usleep(1000000);

	// start a thread that gets the ALS values and calculated lux and CCT
	PollThreadIsRunning = 1;
	err = pthread_create(&PollThread, NULL, PollThreadRoutine, NULL);
	if (err) {
		LOG("could not start poll thread\n");
		return -1;
	}

	// Start sensor
	LOG ("========================\n");
	LOG ("===== Start Sensor =====\n");
	LOG ("========================\n");
	count = 0;
	err = 0;
	do {
		err = pIO->StartSensor();
		if (err == 0)
			break;
		usleep(50000); // 50ms
		count++;
	} while (err != 0);
	if (count == 100) {
		LOG("FATAL error: could not start the device");
		return -1;
	};


	// when signal is received: stop and wait before closing the poll thread
	signal(SIGINT, sighandler);

	do {
		// wait 100 ms
		usleep(100000);
	} while (ExitMainLoop == 0);


	LOG ("================================\n");
	LOG ("===== Stopping poll thread =====\n");
	LOG ("================================\n");

	// ensure the ending of the read
	// to be added: mutex to protect alsThreadRuns
	PollThreadIsRunning = 0;
	// wait for the thread completion
	pthread_join(PollThread, &retval);

	// Stop sensor
	// Stop sensor must be called once PollSensor is ensure to be no called
	// Otherwise PollSensor can be stucked in waiting for a signal to happen
	// This is in line with specifications
	//
	LOG ("===========================\n");
	LOG ("===== Stopping Sensor =====\n");
	LOG ("===========================\n");
	pIO->StopSensor();

	// wait 100 ms
	//usleep(100000);

	// Close sensor
	LOG ("============================\n");
	LOG ("===== Closing Sensor =======\n");
	LOG ("============================\n");
	pIO->CloseSensor();

	// close
	dlclose(sensor_lib);
}
