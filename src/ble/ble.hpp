#pragma once
#include "pico/stdlib.h"
#include "hardware/uart.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>

std::string ui_msg = {""};
#define UI_CODE_POS 'C'
#define UI_CODE_MODE 'M'
#define UI_MODE_HOLD "H"
#define UI_MODE_FOLLOW "F"
#define UI_MODE_TEST_DRIVE "TD"
#define UI_MODE_TEST_OA "TOA"

typedef enum
{
    HOLD,
    FOLLOW,
    DRIVE_TEST,
    OA_TEST
} OpMode;

class UIParser
{
public:
    UIParser()
    {
        hasfix = false;
        newdata = false;
        latitude = 0;
        longitude = 0;
        mode = HOLD;
        msg = {""};
    }

    OpMode currentMode(){return mode;}
    long userLatitude(){return latitude;}
    long userLongitude(){return longitude;}
    bool userFix(){return hasfix;}
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
            ui_msg = {""};
        }
        else
        {
            msg.append(1, c);
        }
    }

protected:
    long latitude, longitude;
    OpMode mode;
    std::string msg;
    bool hasfix, newdata;

    void decode_msg(char cmd, std::string data)
    {
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
                mode = HOLD;
            else if (data.compare(UI_MODE_FOLLOW) == 0)
                mode = FOLLOW;
            else if (data.compare(UI_MODE_TEST_DRIVE) == 0)
                mode = DRIVE_TEST;
            else if (data.compare(UI_MODE_TEST_OA) == 0)
                mode = OA_TEST;
            break;
        }
        default:
            break;
        }
    }
};


