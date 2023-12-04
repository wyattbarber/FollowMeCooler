#pragma once
#include "pico/stdlib.h"
#include "hardware/uart.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <tuple>


#define UI_CODE_POS 'C'
#define UI_CODE_MODE 'M'
#define UI_MODE_HOLD "H"
#define UI_MODE_FOLLOW "F"
#define UI_MODE_TEST_DRIVE "TD"
#define UI_MODE_TEST_OA "TOA"


class UIParser
{
public:
    UIParser()
    {
        hasfix = false;
        newdata = false;
        latitude = 0;
        longitude = 0;
        msg = {""};
    }

    long userLatitude(){return latitude;}
    long userLongitude(){return longitude;}
    bool userFix(){return hasfix;}
    bool holdMode(){return hold;}
    bool driveMode(){return drive;}
    bool testDriveMode(){return test_drive;}
    bool testOAMode(){return test_oa;}
    bool newData()
    {
        if(newdata)
        {
            newdata = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    void newChar(char c)
    {
        if (c == '\n')
        {
            // Find start of message to handle alignment errors
            auto idx = msg.find(UI_CODE_POS);
            if(idx == std::string::npos)
            {
                idx = msg.find(UI_CODE_MODE);
            }
            if(idx != std::string::npos)
            {
                // Decode input message
                decode_msg(msg[idx], msg.substr(idx+1));
            }
            msg = {""};
        }
        else
        {
            msg.append(1, c);
        }
    }

protected:
    bool drive, hold, test_drive, test_oa;
    long latitude, longitude;
    std::string msg;
    bool hasfix, newdata;

    void decode_msg(char cmd, std::string data)
    {
        printf("Decoding BLE input: %c : %s\n", cmd, data.c_str());
        newdata = true;
        switch (cmd)
        {
        case UI_CODE_POS:
        {
            hasfix = true;
            size_t split = data.find(':');
            latitude = std::stol(data);
            longitude = std::stol(data.substr(split + 1));
            break;
        }

        case UI_CODE_MODE:
        {
            if (data.compare(UI_MODE_HOLD) == 0)
            {
                drive = false;
                hold = true; 
                test_drive = false;
                test_oa = false;
            }
            else if (data.compare(UI_MODE_FOLLOW) == 0)
            {
                drive = true;
                hold = false; 
                test_drive = false;
                test_oa = false;
            }
            else if (data.compare(UI_MODE_TEST_DRIVE) == 0)
            {
                drive = false;
                hold = false; 
                test_drive = true;
                test_oa = false;
            }
            else if (data.compare(UI_MODE_TEST_OA) == 0)
            {
                drive = false;
                hold = false; 
                test_drive = false;
                test_oa = true;
            }
            else
            {
                printf("Invalid mode %s\n", data.c_str());
            }
            break;
        }
        default:
            break;
        }
    }
};


