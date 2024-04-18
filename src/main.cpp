/**
    ESP32 Motorcover
    Aurthor: Jason Chen, 2022
    Licence: TODO

    An ESP32-based low-powered wireless motor controller that works with bipolar stepper motors.
    It is a closed-loop system so it is capable of keeping track of its position while the motor is
    off.

    You can use it to motorize and automate the opening & closing of blinds/shades/windows etc.
**/
#include <Arduino.h>
#include "logger.h"
#include "system_task.h"
#include "motor_task.h"
#include "wireless_task.h"


static WirelessTask wireless_task(0);  // Running on core0
static SystemTask system_task(0);      // Running on core0
static MotorTask motor_task(1);        // Running on core1


void setup() {
    // Initializing serial output if compiled
    LOG_INIT(115200, LogLevel::INFO);

    // Initializing LED
    pinMode(LED_PIN, OUTPUT);

    // The system task performs coordination between all tasks
    system_task.init();
    system_task.addMotorTaskQueue(motor_task.getQueue());

    wireless_task.init();
    wireless_task.addSystemTaskQueue(system_task.getQueue());
    wireless_task.addSystemSleepTimer(system_task.getSystemSleepTimer());
    wireless_task.addMotorTaskQueue(motor_task.getQueue());
    wireless_task.addMotorStandbySemaphore(motor_task.getMotorStandbySemaphore());

    // The motor task runs the motor and checks the rotary encoder to keep track of the motor's
    // position. TODO not start motor task on wake to reduce boot time
    motor_task.init();
    motor_task.addWirelessTaskQueue(wireless_task.getQueue());
    motor_task.addSystemSleepTimer(system_task.getSystemSleepTimer());

    // Delete setup/loop task
    vTaskDelete(NULL);
}


void loop() {}