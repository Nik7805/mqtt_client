#include <iostream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <exception>
#include <map>
#include <functional>
#include <thread>
#include <chrono>

#include "gpio.h"
#include "i2c.h"
#include "HMC5883L.hpp"
#include "mosquitto.h"

using namespace std;
static HMC5883L* magnetSensor;
static const char* BrokerIP = "192.168.0.250";
static const char* MQTT_Username = "Nik";
static const char* MQTT_Password = "1234";
static int sleep_ms = 1000;

const map<string, function<void(string)>> paramHandle =
{
    {"-c", [](string p){magnetSensor->Calibrate();}},
    {"-g", [](string p){magnetSensor->SetGain((uint8_t)stoi(p));}}
};

const map<string, function<void(char*)>> mqttMessageParser =
{
    {"Home/magnet/sleepTime", [](char* m){sleep_ms = stoi(m);}}  
};

void on_connect(struct mosquitto *mosq, void *obj, int reason_code)
{
	printf("on_connect: %s\n", mosquitto_connack_string(reason_code));
	if(reason_code != 0)
    {
		mosquitto_disconnect(mosq);
	}
    else
    {
        mosquitto_subscribe(mosq, NULL, "Home/magnet/sleepTime", 0);
    }
}

void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{
    std::string topic(msg->topic);
    if(auto pair = mqttMessageParser.find(topic); pair != mqttMessageParser.end())
    {
        try
        {
            pair->second((char*)msg->payload);
        }
        catch(...)
        {
            cerr << "Can't parse topic payload.";
        }
    }
}

void ProcessParams(int numParams, char** params)
{
    for(int i = 1; i < numParams; i++)
    {
        string p = params[i];
        string arg = i + 1 == numParams ? "" : params[i + 1];
        
        if(auto pair = paramHandle.find(p); pair != paramHandle.end())
            pair->second(arg);
    }
}

int main(int numParams, char** params) 
{
    struct mosquitto *mosq;
	int rc;

    std::cout << "Configuring mosquitto lib\n";
	mosquitto_lib_init();

    std::cout << "Creating new mqtt cient\n";
	mosq = mosquitto_new(NULL, true, NULL);
	if(mosq == NULL){
		fprintf(stderr, "Error: Out of memory.\n");
		return 1;
	}

	mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    std::cout << "Configuring mqtt client.\nUsername: " << MQTT_Username << " password: " << MQTT_Password << std::endl;
    if(int pwSetRes = mosquitto_username_pw_set(mosq, MQTT_Username, MQTT_Password); pwSetRes != MOSQ_ERR_SUCCESS)
    {
        std::cout << "Mosquitto creditionals setting error " << pwSetRes << std::endl;
        exit(1);
    }

    std::cout << "Connecting to broker " << BrokerIP << std::endl;
    rc = mosquitto_connect(mosq, BrokerIP, 1883, 60);
	if(rc != MOSQ_ERR_SUCCESS){
		mosquitto_destroy(mosq);
		fprintf(stderr, "Error: %s\n", mosquitto_strerror(rc));
		exit(1);
	}

    rc = mosquitto_loop_start(mosq);
	if(rc != MOSQ_ERR_SUCCESS){
		mosquitto_destroy(mosq);
		fprintf(stderr, "Error: %s\n", mosquitto_strerror(rc));
		exit(1);
	}

    try
    {
        magnetSensor = new HMC5883L("/dev/i2c-2");

        ProcessParams(numParams, params);
        std::string str = "";
        while(true)
        {
            if(magnetSensor->Measure())
            {
                HMC5883L::PlainData_t plainData = magnetSensor->GetPlainData(HMC5883L::Axes::X, HMC5883L::Axes::Z);
                str = "X:" + std::to_string(magnetSensor->GetMagnitude(HMC5883L::Axes::X)) + 
                     " Y:" + std::to_string(magnetSensor->GetMagnitude(HMC5883L::Axes::Y)) + 
                     " Z:" + std::to_string(magnetSensor->GetMagnitude(HMC5883L::Axes::Z)) +
                     "\n" +
                     "XZ mag:" + std::to_string(plainData.absValue) + "; angle:" + std::to_string(plainData.degrees);
            }
            else
            {
                str = "Sensor overload";
            }

            if(int rc = mosquitto_publish(mosq, NULL, "Home/magnet/value", str.length(), str.c_str(), 2, false); rc != MOSQ_ERR_SUCCESS)
            {
                fprintf(stderr, "Error publishing: %s\n", mosquitto_strerror(rc));
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }
    catch(exception ex)
    {
        cout << "Unhandeled exception:" << ex.what() << endl;
    }
    catch(...)
    {
        cout << "Unhandeled exception\n";
    }

    mosquitto_lib_cleanup();
	return 0;
}
