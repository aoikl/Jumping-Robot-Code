/*[------------------------------------------------]

    ESP32 position motion control example with magnetic sensor

[------------------------------------------------]*/
#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);  // motor1

/* magnetic sensor instance - SPI(AS5048A)
AS5048A(SPI):
white  - CSn(SS)(I_C)
red    - CLK0(SCL0)
yellow - MISO(SDA0)
black  - MOSI(SDA1)
green  - VDD5V
purple - GND

Deng2.0 Driver:
SDA0 - MISO 接AS5048A的 MISO
SDA1 - MOSI 接AS5048A的 MOSI
SCL0 - SCK(CLK)
I0& I1 - SS*/
MagneticSensorSPI sensor1 = MagneticSensorSPI(15, 14, 0x3FFF);  //(ss,bit resolution,Register address) motor1

Commander command = Commander(Serial);
void doMotor1(char* cmd) {
    command.motor(&motor1, cmd);
}
/*[------------------------------------------------]

    FOC

[------------------------------------------------]*/
void FOC() {
    while (1) {
        //  main FOC algorithm function
        motor1.loopFOC();
        motor1.move();
        command.run();
        if (abs(motor1.shaft_angle - motor1.shaft_angle_sp) > 0.1) {
            motor1.monitor();
        }
    }
}

/*========================================

    Setup & Loop

========================================*/

void setup() {
    Serial.begin(115200);
    //  initialize encoder sensor hardware
    sensor1.init();
    // link the motor to the sensor
    motor1.linkSensor(&sensor1);

    // driver config
    driver1.voltage_power_supply = 12;
    driver1.init();
    motor1.linkDriver(&driver1);

    // choose FOC modulation (optional)
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // set motion control loop to be used
    motor1.controller = MotionControlType::angle;

    // controller configuration
    // default parameters in defaults.h

    // controller configuration based on the control type
    // velocity PID controller parameters
    // default P=0.5 I = 10 D =0
    motor1.PID_velocity.P = 0.2;
    motor1.PID_velocity.I = 20;
    motor1.PID_velocity.D = 0.001;
    // jerk control using voltage voltage ramp
    // default value is 300 volts per sec  ~ 0.3V per millisecond
    // motor1.PID_velocity.output_ramp = 1000;

    // velocity low pass filtering
    // default 5ms - try different values to see what is the best.
    // the lower the less filtered
    motor1.LPF_velocity.Tf = 0.01;

    // angle P controller -  default P=20
    motor1.P_angle.P = 20;

    //  maximal velocity of the position control
    // default 20
    motor1.velocity_limit = 50;  //(rad/s)
    // default voltage_power_supply
    motor1.voltage_limit = 20;

    // initialize motor
    motor1.init();
    // align encoder and start FOC
    motor1.initFOC(/*0.93, Direction::CW*/);  // motor.initFOC(Zero elec. angle:offset, direction) ex:(2.33,Direction::CCW)

    command.add('M', doMotor1, "motor1");
    // comment out if not needed
    motor1.useMonitoring(Serial);
    motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
    // 下采样
    motor1.monitor_downsample = 5;
    Serial.println("Motor ready.");
    _delay(1000);
}

void loop() {
    FOC();
}