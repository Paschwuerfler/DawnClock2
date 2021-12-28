# DawnClock2 - An Open-Source Light Alarm CLock
This is a reimanging of my first user-ready project ever, a light alarm clock. 

I wanted to implement some more features, but making an user friendly interface took an surprisiing amount of time. 

Originally, i wanted to use NTP to get the time and skip the RTC, but realized that, because im not sure the clock would be powered on all the time (the outlet in question is also used for the vacuum cleaner) i would have to back up the alarm times, along other stuff to NVS which would probably wear out within a few years, so i went with an RTC (DS3231)

I also thought about a buzzer as a second state for the alarm, the current hardware has one unused pin, so i might implemment it in the future. 

This is my biggest microcontroller coding project ever, interfacing with a lot of hardware is surprisingly hard.

I tried to use Interupts to set flags that were then processed in the main code as a general coding scheme, which makes for extremely responsive code with little nesting, i would assume that it also wouldnt disrupt wifi to badly

