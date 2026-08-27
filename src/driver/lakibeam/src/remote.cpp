#include <stdio.h>
#include <ros/ros.h> 
#include <curl/curl.h>
#include "../thirdparty/rapidjson/document.h"
#include "../thirdparty/rapidjson/prettywriter.h"
#include "../include/remote.h"

using namespace rapidjson;

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

static size_t dummy_callback(void *buffer, size_t size, size_t nmemb, void *userp)
{
   return size * nmemb;
}

int sensor_config(std::string sensor_ipaddr, std::string parameter, std::string value)
{
    long http_code;

    CURL *curl = curl_easy_init();
    std::string URL_RESTFUL_API = "http://" + sensor_ipaddr + parameter;

    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);

    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, URL_RESTFUL_API.c_str());
    	curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, value.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, dummy_callback);

        if(curl_easy_perform(curl) == CURLE_OK){
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
        }
    }

    curl_easy_cleanup(curl);
    curl_global_cleanup();

    return 0;
}

int get_telemetry_data(std::string sensor_ipaddr)
{
    CURL *curl;
    CURLcode res;

    std::string readBuffer;
    std::string URL_API_FIRMWARE = "http://" + sensor_ipaddr + "/api/v1/system/firmware";
    std::string URL_API_MONITOR = "http://" + sensor_ipaddr + "/api/v1/system/monitor";
    std::string URL_API_OVERVIEW = "http://" + sensor_ipaddr + "/api/v1/sensor/overview";

    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);

    if(curl) {
        readBuffer = "";
        curl_easy_setopt(curl, CURLOPT_URL, URL_API_FIRMWARE.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        const char* json = const_cast<char*>(readBuffer.c_str());
        Document jsondoc;
        jsondoc.Parse(json);
        assert(jsondoc.IsObject());

    }

    curl = curl_easy_init();

    if(curl) {
        readBuffer = "";
        curl_easy_setopt(curl, CURLOPT_URL, URL_API_MONITOR.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        const char* json = const_cast<char*>(readBuffer.c_str());
        Document jsondoc;
        jsondoc.Parse(json);
        assert(jsondoc.IsObject());
    }

    curl = curl_easy_init();

    if(curl) {
        readBuffer = "";
        curl_easy_setopt(curl, CURLOPT_URL, URL_API_OVERVIEW.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        const char* json = const_cast<char*>(readBuffer.c_str());
        Document jsondoc;
        jsondoc.Parse(json);
        assert(jsondoc.IsObject());
    }

    return 0;
}
