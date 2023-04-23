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
#include "bmp280_app.hpp"

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

template<class T>
void PublishData(mosquitto* mosq, std::string topic, T data, bool retain)
{
    std::string sData = to_string(data);
    mosquitto_publish(mosq, NULL, topic.c_str(), sData.length(), sData.c_str(), 0, retain);
}


int main(int numParams, char** params) 
{
    struct mosquitto *mosq;
	int rc;

    std::cout << "Configuring mosquitto lib\n";
	mosquitto_lib_init();

    std::cout << "Creating new mqtt cient\n";
	mosq = mosquitto_new(NULL, true, NULL);

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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(int8_t i = BMP280_Init("/dev/i2c-2"); i != 0)
        {
            fprintf(stderr, "BMP280 init error: %d\n", i);
            exit(1);
        }

        ProcessParams(numParams, params);
        std::string sMagnetData  = "";
        std::string sTemperature = "";
        std::string sPressure    = "";
        while(true)
        {
            if(magnetSensor->Measure())
            {
                HMC5883L::PlainData_t plainData = magnetSensor->GetPlainData(HMC5883L::Axes::X, HMC5883L::Axes::Z);
                
                PublishData(mosq, "Home/magnet/X",  magnetSensor->GetMagnitude(HMC5883L::Axes::X), false);
                PublishData(mosq, "Home/magnet/Y",  magnetSensor->GetMagnitude(HMC5883L::Axes::Y), false);
                PublishData(mosq, "Home/magnet/Z",  magnetSensor->GetMagnitude(HMC5883L::Axes::Z), false);
                PublishData(mosq, "Home/magnet/XZ_abs", plainData.absValue, false);
                PublishData(mosq, "Home/magnet/XZ_ang", plainData.degrees,  false);
            }
            else
            {
                fprintf(stderr, "HMC5883L overload\n");
            }

            if(BMP280_Measure())
            {
                sTemperature = to_string(BMP280_GetTemperature());
                sPressure    = to_string(BMP280_GetPressure());
            }
            else
            {
                fprintf(stderr, "BMP280 measure error\n");
                sTemperature = "NaN";
                sPressure    = "NaN";
            }
            mosquitto_publish(mosq, NULL, "Home/temperature", sTemperature.length(), sTemperature.c_str(), 2, false);
            mosquitto_publish(mosq, NULL, "Home/pressure",    sPressure.length(),    sPressure.c_str(),    2, false);
            
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
