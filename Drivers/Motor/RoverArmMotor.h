// #include <PID_v1.h>
#include "pid.h" // This is the file we're working on
#include <movingAvg.h>
// #include <Servo.h>
// #include "stm32l4xx_hal.h"
#include "main.h"



// #define MOTOR_DC_DIR_1
struct Pin {
  GPIO_TypeDef* port;
  uint16_t pin;
  bool valid;

  Pin() : port(nullptr), pin(0), valid(false) {}
  Pin(GPIO_TypeDef* p, uint16_t pn) : port(p), pin(pn), valid(true) {}
};


class RoverArmMotor{

    public:

        // Our motors use two different ESC's with different control schemes
        #define CYTRON 0
        #define BLUE_ROBOTICS 1

        // ADC values representing 359 and 0 degrees respectively
        #define MAX_ADC_VALUE 4095 //3850 
        #define MIN_ADC_VALUE 0     //200

        #define FWD 1
        #define REV -1

        // RoverArmMotor(int pwm_pin, int encoder_pin, int esc_type, double minimum_angle, 
        //               double maximum_angle, int dir_pin, int brake_pin);
        RoverArmMotor(SPI_HandleTypeDef* spi_handle, Pin pwm_pin, Pin dir_pin, Pin encoder_pin, 
        int esc_type, double minimum_angle, double maximum_angle, Pin brake_pin = Pin());

        // Setters for various tunable parameters of our motors
        void setAggressiveCoefficients(double P, double I, double D);
        void setRegularCoefficients(double P, double I, double D);
        void setRetuningGapLimit(int gap);
        void setAngleLimits(double lowest_angle, double highest_angle);
        bool setMultiplierBool(bool mult, double ratio); 
        bool newSetpoint(double angle);

        void setPIDOutputLimits(double lower_end, double upper_end);
        void setMovingAverageWindowSize(int size);
        void disengageBrake();
        void engageBrake();

        double getCurrentAngle();
        double getSetpoint();
        double getCurrentOutput();
        int getDirection();
        double getRatio(); 
        // void setGearRatio(double ratio);

        void begin(double aggP, double aggI, double aggD, double regP, double regI, double regD);
        void tick();

    private:
    public: // TESTING only
        // Default to open loop, will need to enter the coefficients to begin
        PID internalPIDInstance;
        movingAvg internalAveragerInstance;

        //TODO: Add support for other ESC types
        // Servo internalServoInstance;

        double aggressiveKp, aggressiveKi, aggressiveKd, regularKp, regularKi, regularKd;
        //PINS
        Pin pwm, dir, encoder, brake;
        SPI_HandleTypeDef* spi;

        int movingAverageWindowSize;
        double lowestAngle, highestAngle;
        int escType;
        int adcResult;
        double currentAngle, lastAngle;
        bool wrist_waist; 
        // int multiplier;
        double input;
        double output;
        double setpoint;
        int actuationState;
        double gearRatio;

        enum ActuationStates
        {
            FIRST_ROTATION_REGION,
            SECOND_ROTATION_REGION,
            RATIO_IS_ONE
        };

        double mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

        void WatchdogISR();
};

