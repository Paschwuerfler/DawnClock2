
#include <Arduino.h>

//The RTC module uses standard I2C pins 

//CHANGE THEESE VALUES IF YOU USE THE CODE 
#define LightPin D8         //PIN FOR PWM dimmed LED 
#define AlarmSwitch A0      //PIN FOR A SWITCH TO ACTIVATE THE ALARM (has to be pulled up for activation)
#define maxPWM 70           //I use a pwm-controlled-current led driver so this is important 
#define WAKESPEED 1.2       //Formula: pwmvalue = pow(secondssincealarm / 50, WAKESPEED); (1.2 for about 10 minutes of wakeup)) 

#define dayStart 6                //For dimming end 
#define dayEnd 22                 //For dimming start 
#define DisplayBrightnessDay 255  //Brightness of the display during day
#define DisplayBrightnessNight 50 //Brightness of the display during night


//For TM1637
const byte PIN_CLK = D3;  // define CLK pin (any digital pin)
const byte PIN_DIO = D0;  // define DIO pin (any digital pin)

long lastCursor = 0; 
byte cursorState = 1; 
const int BLINK_RATE = 500; 


enum MODE {
    DTIME,
    DSETALARMHOURS,  //Both used by getTime
    DSETALARMMINUTES,
    DSETTIME,  //Handeled differently to alarm time
    DVAL       //Used by getVal
} currentMode;

enum LIGHT {
    LIGHT_OFF,
    LIGHT_ON,
    LIGHT_ALARM
} lightState;

enum ALARM {
    ALARM_DISABLED,
    
    ALARM_CHANGE_ON,   //Used to switch modes
    ALARM_CHANGE_OFF,  //Used to switch modes
    ALARM_OFF,
    ALARM_LIGHT,
    ALARM_SOUND,

} alarmState;

unsigned char lightvalue = 0;

int timeSinceAlarm = 0;


char lastSwitchState = 0;

/*
IRAM_ATTR void alarmOn() {
    Serial.println("Alarm on");
    alarmState = ALARM_CHANGE_ON;
}

IRAM_ATTR void alarmOff() {
    Serial.println("Alarm off");
    alarmState = ALARM_CHANGE_OFF;
}
*/

/////////////////////////////ENCODER SETUP////////////////////////////////////
#include <RotaryEncoder.h>  //https://github.com/mathertel/RotaryEncoder

//Encoder Pins
#define PIN_IN1 D5
#define PIN_IN2 D6

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

//Interrupt routine
IRAM_ATTR void checkPosition() {
    encoder->tick();  // just call tick() to check the state.
    if (currentMode == DTIME) {
        if (lightState == LIGHT_ON) {
            int dir = (int)(encoder->getDirection());
            if (dir == 1 && lightvalue < maxPWM) {
                lightvalue += 1;
            } else if (dir == -1 && lightvalue > 1) {
                lightvalue -= 1;
            }
        }
    }
}

/////////////////////////////DISPLAY SETUP////////////////////////////////////
#include "SevenSegmentTM1637.h"  //https://github.com/bremme/arduino-tm1637


SevenSegmentTM1637 display(PIN_CLK, PIN_DIO);

void PrintTime(int hours, int minutes) {
    uint8_t buffer[4];

    buffer[0] = display.encode(int16_t(hours / 10));
    buffer[1] = display.encode(int16_t(hours % 10));
    buffer[2] = display.encode(int16_t(minutes / 10));
    buffer[3] = display.encode(int16_t(minutes % 10));

    display.printRaw(buffer, 4, 0);
    display.setColonOn(cursorState); 
}

///////////////////////DS3231 SETUP/////////////////////////////////////////
#include <DS3231.h>  //https://github.com/NorthernWidget/DS3231
#include <Wire.h>

DS3231 myclock;

/////////////////////////////BUTTON SETUP////////////////////////////////////
#include "OneButton.h"  //https://github.com/mathertel/OneButton

#define BUTTON_INPUT D4
OneButton button(BUTTON_INPUT, true);
unsigned long pressStartTime;

ICACHE_RAM_ATTR void checkTicks() {
    // include all buttons here to be checked
    button.tick();  // just call tick() to check the state.
}

//#include"buttons.cpp"

// this function will be called when the button was pressed 1 time only.
void singleClick() {
    Serial.println("sClick");

    switch (currentMode) {
        case DTIME:
            if (lightState == LIGHT_OFF) {
                lightState = LIGHT_ON;
            } else if (lightState == LIGHT_ON) {
                lightState = LIGHT_OFF;
            } else if (lightState >= LIGHT_ALARM) {
                lightState = LIGHT_OFF;
                alarmState = ALARM_OFF;
            }

            break;
        case DSETALARMHOURS:
            currentMode = DSETALARMMINUTES;
            break;
        case DSETALARMMINUTES:
            //Shw final time, then switch back
            currentMode = DTIME;
            break;
        case DVAL:
            currentMode = DTIME;

            break;
    }

}  // singleClickleClick

// this function will be called when the button was pressed 2 times in a short timeframe.
void doubleClick() {
    Serial.println("doubleClick() detected.");

}  // doubleClick

// this function will be called when the button was pressed multiple times in a short timeframe.
void multiClick() {
    Serial.print("multiClick(");
    Serial.print(button.getNumberClicks());
    Serial.println(") detected.");

    if (button.getNumberClicks() == 3 && currentMode == DTIME) {
        currentMode = DSETTIME;
    }

}  // multiClick

// this function will be called when the button was held down for 1 second or more.
void pressStart() {
    Serial.println("pressStart()");
    switch (currentMode) {
        case DTIME:
            currentMode = DSETALARMHOURS;
            break;
        case DSETALARMHOURS:

            break;
        case DSETALARMMINUTES:

            break;
        case DVAL:

            break;
    }
}  // pressStart()

// this function will be called when the button was released after a long hold.
void pressStop() {
    Serial.print("pressStop(");
    Serial.print(millis() - pressStartTime);
    Serial.println(") detected.");
}  // pressStop()

//////////////////TIME SETUP///////////////////////////////////////////////

int alarmSeconds = 0;
long timeSeconds = 0;

/////////////////////////////SETUP//////////////////////////////////////////

int utoHours(int timeSeconds) {
    return timeSeconds / 3600;
}

int utoMinutes(int timeSeconds) {
    return (timeSeconds % 3600) / 60;
}

int timeToUtc(int hours, int minutes) {
    return hours * 3600 + minutes * 60;
}

#define Debug_GetVal 0

int getVal(int min, int max) {
    min *= 2;  //for more accuracy when turning
    max *= 2;
    currentMode = DVAL;
    encoder->setPosition(min);

    int pos = 0;

    while (currentMode == DVAL) {  //Short Button Press Handles Mode Change
        //Serial.print("en");

        ESP.wdtFeed();

        encoder->tick();  // just call tick() to check the state.
        button.tick();

        int newPos = encoder->getPosition();

        if (pos != newPos) {
            if (Debug_GetVal) {
                Serial.print("pos:");
                Serial.print(newPos);
                Serial.print(" dir:");
                Serial.println((int)(encoder->getDirection()));
            }

            if (pos < min) {
                encoder->setPosition(min);
                newPos = min;
            }
            if (pos > max) {
                encoder->setPosition(max);
                newPos = max;
            }

            pos = newPos;
            display.clear();
            display.print(pos / 2);
            delay(2);
        }
    }

    display.clear();
    display.print(pos / 2);
    display.blink();

    return encoder->getPosition() / 2;
}

long getTime(int hours, int minutes) {
    currentMode = DSETALARMHOURS;

    hours *= 2;
    minutes *= 2;

    encoder->setPosition(hours);

    int pos = 0;

    PrintTime(hours / 2, minutes / 2);

    while (currentMode == DSETALARMHOURS) {  //Short Button Press Handles Mode Change

        ESP.wdtFeed();
        //Serial.print("en");

        encoder->tick();  // just call tick() to check the state.
        button.tick();

        int newPos = encoder->getPosition();

        if (newPos != pos) {
            if (newPos < 0) {
                encoder->setPosition(47);
                newPos = 47;
            }
            if (newPos > 47) {
                encoder->setPosition(0);
                newPos = 0;
            }

            pos = newPos;
            display.clear();
            PrintTime(pos / 2, minutes / 2);

            Serial.print(pos / 2);
            Serial.print(":");
            Serial.println(minutes / 2);

            delay(2);
        }
    }

    hours = pos / 2;
    pos = 0;
    encoder->setPosition(minutes);

    while (currentMode == DSETALARMMINUTES) {  //Short Button Press Handles Mode Change

        ESP.wdtFeed();
        //Serial.print("en");
        encoder->tick();  // just call tick() to check the state.
        button.tick();

        int newPos = encoder->getPosition();

        if (newPos != pos) {
            if (newPos < 0) {
                encoder->setPosition(119);
                newPos = 119;
                if (hours > 0) hours--;
            }
            if (newPos > 119) {
                encoder->setPosition(0);
                newPos = 0;
                if (hours < 47) hours++;
            }

            pos = newPos;
            display.clear();
            PrintTime(hours, pos / 2);

            Serial.print(hours);
            Serial.print(":");
            Serial.println(pos / 2);

            delay(2);
        }
    }

    minutes = pos / 2;

    display.clear();
    PrintTime(hours, minutes);
    display.blink();

    return hours * 3600 + minutes * 60;
}

void timeSet() {
    int year = getVal(2020, 2099);
    myclock.setYear(year - 2000);
    int month = getVal(1, 12);
    myclock.setMonth(month);
    int day = getVal(1, 31);
    myclock.setDate(day);
    int time = getTime(0, 0);
    myclock.setHour(time / 3600);
    myclock.setMinute(time % 3600 / 60);
}

void timeDump() {
    bool null;
    ////This isnt working during setup for some reason

    display.blink(myclock.getYear() + 2000);
    Serial.print("Year: ");
    Serial.println(myclock.getYear() + 2000);

    display.blink(myclock.getMonth(null));
    Serial.print("Month: ");
    Serial.println(myclock.getMonth(null));

    display.blink(myclock.getDate());
    Serial.print("Date: ");
    Serial.println(myclock.getDate());

    display.blink(myclock.getHour(null, null));
    Serial.print("Hour: ");
    Serial.println(myclock.getHour(null, null));

    display.blink(myclock.getMinute());
    Serial.print("Minute: ");
    Serial.println(myclock.getMinute());
}


void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    


    pinMode(AlarmSwitch, INPUT_PULLUP);

    /*
    attachInterrupt(digitalPinToInterrupt(AlarmSwitch), alarmOn, FALLING);
    attachInterrupt(digitalPinToInterrupt(AlarmSwitch), alarmOff, RISING);
    */

    ///////////////////////////////ENCODER SETUP////////////////////////////////////
    Serial.println("INFO: Setting up encoder");

    encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

    // register interrupt routine
    attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

    /////////////////////////////DISPLAY SETUP////////////////////////////////////
    Serial.println("INFO: Setting up display");

    display.begin();            // initializes the display
    display.setBacklight(100);  // set the brightness to 100 %
    display.print("INIT");      // display INIT on the display

    display.setColonOn(true); 

    delay(1000);

    Serial.println("Button");

    /////////////////////////////BUTTON SETUP////////////////////////////////////
    Serial.println("INFO: Setting Up Buttons");

    attachInterrupt(digitalPinToInterrupt(BUTTON_INPUT), checkTicks, CHANGE);

    // link the xxxclick functions to be called on xxxclick event.
    button.attachClick(singleClick);
    button.attachDoubleClick(doubleClick);
    button.attachMultiClick(multiClick);

    button.setPressTicks(1000);  // that is the time when LongPressStart is called
    button.attachLongPressStart(pressStart);
    button.attachLongPressStop(pressStop);

    Serial.println("DS3231");

    /////////////////////////////DS3231 SETUP////////////////////////////////////
    Serial.println("INFO: Setting up DS3231");

    Wire.begin();
    myclock.setClockMode(false);  // set to 24h mode

    while ((myclock.getTemperature() < -20)) {
        Serial.println("ERROR: DS3231 not found");

        Wire.clearWriteError();
        Wire.flush();
        Wire.begin();

        display.print("ERR");
        display.blink();
    }

    if (myclock.getYear() < 1) {
        Serial.println("DS3231 has not been set. Setting now.");
        Serial.println(myclock.getYear());

        timeSet();
    }

    timeDump();

    currentMode = DTIME;
    lightState = LIGHT_OFF;
    alarmState = ALARM_OFF;
}

void loop() {

    static int pos = 0;
    static int oldmillis = 0;

    //Print cylce time
    if ((millis() - oldmillis) > 52) {
        Serial.print("time: ");
        Serial.println(millis() - oldmillis);
    }

    //Ticks:

    button.tick();
    encoder->tick();  // just call tick() to check the state.
    
    //////////////////////DISPLAY TIME when not setting stuff/////////////////////////////
    if (currentMode == DTIME) {
        display.clear();
        bool h12Flag, pmFlag;
        PrintTime(myclock.getHour(h12Flag, pmFlag), myclock.getMinute());
    }

    int switchVal = analogRead(AlarmSwitch);
    Serial.println(switchVal);

    //////////////////////ALARM SWITCH/////////////////////////////

    //This is the result of A0 beiing incapable of interrupts 
    if(switchVal < 200 && alarmState == ALARM_DISABLED) {
        alarmState = ALARM_CHANGE_ON;
    }

    if(switchVal > 800 && alarmState >= ALARM_OFF ) {
        alarmState = ALARM_CHANGE_OFF;
    }

    if (alarmState == ALARM_CHANGE_ON) {
        alarmState = ALARM_OFF;
        Serial.println("Switch pressed");

        byte alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits;
        bool alarmDy, alarmH12Flag, alarmPmFlag;

        myclock.getA1Time(alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits, alarmDy, alarmH12Flag, alarmPmFlag);

        PrintTime(alarmHour, alarmMinute);
        display.blink();

    } else if (alarmState == ALARM_CHANGE_OFF) {

        alarmState = ALARM_DISABLED;
        Serial.println("Switch released");

        if (lightState == LIGHT_ALARM) {
            lightState = LIGHT_OFF;
        }

        display.print("OFF ");
        display.blink();
    }

    /////////////////SET ALARM TIME ///////////////////////////////
    if (currentMode == DSETALARMHOURS) {
        Serial.println("DSETALARMHOURS");

        byte alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits;
        bool alarmDy, alarmH12Flag, alarmPmFlag;

        myclock.getA1Time(alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits, alarmDy, alarmH12Flag, alarmPmFlag);

        int alarmtime = getTime(alarmHour, alarmMinute);

        myclock.setA1Time(alarmDay, alarmtime / 3600, alarmtime % 3600 / 60, alarmSecond, alarmBits, alarmDy, false, false);
        Serial.print("Alarm Time: ");
        Serial.print(alarmtime / 3600);
        Serial.print(":");
        Serial.print(alarmtime % 3600 / 60);
    }

    ///////////////SET ALL RTC VARS AGAIN/////////////////////////////
    if (currentMode == DSETTIME) {
        Serial.println("DSETTIME");
        timeSet();
        timeDump();
    }

    //////////////ALRAM GOING OFF? ///////////////////////////
    if (currentMode == DTIME && alarmState == ALARM_OFF) {
        byte alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits;
        bool alarmDy, alarmH12Flag, alarmPmFlag;

        myclock.getA1Time(alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits, alarmDy, alarmH12Flag, alarmPmFlag);

        if (alarmHour == myclock.getHour(alarmH12Flag, alarmPmFlag) && alarmMinute == myclock.getMinute()) {
            alarmState = ALARM_LIGHT;
            timeSinceAlarm = millis();
            Serial.println("Alarm");
        }
    }

    ///////////////LIGHT ALARM/////////////
    if (alarmState >= ALARM_LIGHT) {
        lightState = LIGHT_ALARM;

        int diff = millis() - timeSinceAlarm;
        ////Here ould be a buzzer implementation
        float seconds = diff / 1000.0;
        float value = pow(seconds / 50, WAKESPEED);

        Serial.print("Value: ");
        Serial.println(value);

        if (value > maxPWM) {
            value = maxPWM;
        }
        analogWrite(LightPin, value);
    }


    ///////////////DESK LAMP/////////////
    if (lightState == LIGHT_OFF) {
        analogWrite(LightPin, 0);
    } else if (lightState == LIGHT_ON) {
        analogWrite(LightPin, lightvalue);
        Serial.println(lightvalue);
    }

    /////////////Blink the Cursor////////////////
    if((millis() - lastCursor) > BLINK_RATE) {
        if(cursorState == 0) {
            cursorState = 1; 
        } else {
            cursorState = 0; 
        }
        lastCursor = millis();
    }

    oldmillis = millis();
    delay(50);


    /////DISPAY DIMMING at night/////////////////////
    if (lightState == LIGHT_OFF) {
        bool alarmDy, alarmH12Flag, alarmPmFlag;
        if (myclock.getHour(alarmH12Flag, alarmPmFlag) > dayEnd || myclock.getHour(alarmH12Flag, alarmPmFlag) < dayStart) {
            display.setBacklight(DisplayBrightnessNight);
        } else {
            display.setBacklight(DisplayBrightnessDay);
        }
    }




    /* ///Encoder Stuff
  position: encoder->getPosition();
  direction: encoder->getDirection();
  */
}
