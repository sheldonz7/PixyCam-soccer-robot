#include <Pixy2.h>
#include <math.h>

// the size of the ball when it's 15 cm away
#define REAL_SIZE = 1260
#define REAL_DISTANCE = 15
#define SHOOT_GOAL_DISTANCE 60
#define M_PI 3.14159265358979323846

// distance between two front wheels
#define AXLE_BASE 30

// speed used while turning
#define TURNING_SPEED 60

// speed used while moving towards/away from the ball
#define SHOOTING_SPEED 120

// This is the main Pixy object 
Pixy2 pixy;

// -------------------------------------------------------
// Global Variables
// -------------------------------------------------------
int rightMotorLogicPin1 = 2;    
int rightMotorLogicPin2 = 4;  
int leftMotorLogicPin1 = 7;    
int leftMotorLogicPin2 = 8; 
int leftMotorSpeedControl = 5;
int rightMotorSpeedControl = 6;

// -------------------------------------------------------
// Subroutine: Initialize Motor Pins
// -------------------------------------------------------
void initializeMotorPins(){
    // Configuration the motor pins
    pinMode(rightMotorLogicPin1, OUTPUT);   
    pinMode(rightMotorLogicPin2, OUTPUT);   
    pinMode(leftMotorLogicPin1, OUTPUT);   
    pinMode(leftMotorLogicPin2, OUTPUT);   
  
    // Print the motor pin configuration for wiring
    Serial.print("Right Motor Pin 1 = ");
    Serial.println(rightMotorLogicPin1);
    Serial.print("Right Motor Pin 2 = ");
    Serial.println(rightMotorLogicPin2);
    Serial.print("Left Motor Pin 1 = ");
    Serial.println(leftMotorLogicPin1);
    Serial.print("Left Motor Pin 2 = ");
    Serial.println(leftMotorLogicPin2);
}


// -------------------------------------------------------
// Subroutine: Set Motor Pins to enable the droid to move or change its motion
// -------------------------------------------------------
void setMotorPins(int currentDirection, int movingSpeed)
{
    // Set the motor pins appropriately
    // Use the motor logic from lectures
    
    // Pin1  Pin2  Motor
    //  0     0    Idle
    //  0     5v   Forward
    //  5v    0    Reverse
    //  5v    5v   Idle
    
    // ReftMotor  LeftMotor  Direction
    //    For        For      Forward
    //    For        Rev      Turn Left
    //    Rev        For      Turn Right
    //    Rev        Rev      Backwards

    if (currentDirection == 'f') 
    {
        digitalWrite(rightMotorLogicPin1, LOW);   
        digitalWrite(rightMotorLogicPin2, HIGH);   
        digitalWrite(leftMotorLogicPin1, LOW);   
        digitalWrite(leftMotorLogicPin2, HIGH);   
        analogWrite(leftMotorSpeedControl, movingSpeed); 
        analogWrite(rightMotorSpeedControl, movingSpeed); 
    }
    if (currentDirection == 'b') 
    {
        digitalWrite(rightMotorLogicPin1, HIGH);   
        digitalWrite(rightMotorLogicPin2, LOW);   
        digitalWrite(leftMotorLogicPin1, HIGH);   
        digitalWrite(leftMotorLogicPin2, LOW);   
        analogWrite(leftMotorSpeedControl, movingSpeed); 
        analogWrite(rightMotorSpeedControl, movingSpeed); 
    }
    if (currentDirection == 'l') 
    {
        digitalWrite(rightMotorLogicPin1, LOW);   
        digitalWrite(rightMotorLogicPin2, HIGH);   
        digitalWrite(leftMotorLogicPin1, HIGH);   
        digitalWrite(leftMotorLogicPin2, LOW);   
        analogWrite(leftMotorSpeedControl, movingSpeed); 
        analogWrite(rightMotorSpeedControl, movingSpeed); 
    }
    if (currentDirection == 'r') 
    {
        digitalWrite(rightMotorLogicPin1, HIGH);   
        digitalWrite(rightMotorLogicPin2, LOW);   
        digitalWrite(leftMotorLogicPin1, LOW);   
        digitalWrite(leftMotorLogicPin2, HIGH);  
        analogWrite(leftMotorSpeedControl, movingSpeed); 
        analogWrite(rightMotorSpeedControl, movingSpeed/*pwmDutyCycle*/);  
    }
    if (currentDirection == 's')
    {
        digitalWrite(rightMotorLogicPin1, LOW);   
        digitalWrite(rightMotorLogicPin2, LOW);   
        digitalWrite(leftMotorLogicPin1, LOW);   
        digitalWrite(leftMotorLogicPin2, LOW);   
        analogWrite(leftMotorSpeedControl, movingSpeed); 
        analogWrite(rightMotorSpeedControl, movingSpeed); 
    }
}

// Function that utilizes PixyCam to locate the ball
double Navigation(int signature)
{
    int area = 0;
    double distance = 0;

    // detect if the ball is already within the vision
    pixy.ccc.getBlocks();

    // whether there is any block detected
    if (pixy.ccc.numBlocks)
    {
        Serial.print("Detected ");
        Serial.println(pixy.ccc.numBlocks);
        for (int i=0; i<pixy.ccc.numBlocks; i++)
        {
            pixy.ccc.blocks[i].print();    

            // if it's the ball, is it in the middle of vision
            if (pixy.ccc.blocks[i].m_signature == signature && pixy.ccc.blocks[i].m_x > 140 && pixy.ccc.blocks[i].m_x < 176) 
            {
                area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
                distance = pow(6800.0/area, 0.5) * 20.0;
                Serial.print("distance: ");
                Serial.println(distance);
                return distance;
            }
            else
            {
                // rotate until it finds the ball in the middle
                setMotorPins('r', TURNING_SPEED);
                while(true)
                {
                    pixy.ccc.getBlocks();
                    if (pixy.ccc.numBlocks)
                    {
                        Serial.print("Detected ");
                        Serial.println(pixy.ccc.numBlocks);
                        for (int i=0; i<pixy.ccc.numBlocks; i++)
                        {
                            pixy.ccc.blocks[i].print();        
                            if (pixy.ccc.blocks[i].m_signature == signature && pixy.ccc.blocks[i].m_x > 140 && pixy.ccc.blocks[i].m_x < 176) 
                            {
                                setMotorPins('s', 0);
                                area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
                                distance = pow(6800.0/area, 0.5) * 20.0;
                                Serial.print("distance: ");
                                Serial.println(distance);
                                return distance;
                            }
                            else
                            {
                                Serial.println("break");
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    else
    {
        
    // Rotate until the ball is found in the centre of vision
    setMotorPins('r', TURNING_SPEED);
     
        while(true)
        {
            pixy.ccc.getBlocks();
            if (pixy.ccc.numBlocks)
            {
                Serial.print("Detected ");
                Serial.println(pixy.ccc.numBlocks);
                for (int i=0; i<pixy.ccc.numBlocks; i++)
                {
                    pixy.ccc.blocks[i].print();        
                    if (pixy.ccc.blocks[i].m_signature == signature && pixy.ccc.blocks[i].m_x > 130 && pixy.ccc.blocks[i].m_x < 186) 
                    {
                        setMotorPins('s', 0);
                        area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
                        distance = pow(6800.0/area, 0.5) * 20.0;
                        Serial.print("distance: ");
                        Serial.println(distance);
                        return distance;
                    }
                    else
                    {
                        Serial.println("break");
                        break;
                    }
                }
            }
        }
    }
}


// Function that drives droid directly towards the ball
void MovingTowardsBall(double distance)
{
    // leave roughly 2cm of space between droid and the ball
    int timeInterval = (distance - 2) / SHOOTING_SPEED;
    setMotorPins('f', SHOOTING_SPEED);
    delay(timeInterval);
    setMotorPins('s', 0);
}

void MovingAwayFromBall(double distance)
{
    int timeInterval = (distance - 2) / SHOOTING_SPEED;
    setMotorPins('b', SHOOTING_SPEED);
    delay(timeInterval);
    setMotorPins('s', 0);
}


void Shooting()
{
    // detect the angle between goal and ball
    double ballDistance = Navigation(1);
    unsigned long time1 = millis();
    double goalDistance = Navigation(2);
    unsigned long time2 = millis();
    unsigned long timeDifference = time2 - time1;

    // calculate the angle required to be turned
    double turningAngle = (0.5 * M_PI) - acos((3600 + pow(goalDistance, 2) - pow(ballDistance, 2)) / (120 * goalDistance)) - acos((pow(goalDistance, 2) + pow(ballDistance, 2) -3600) / (2 * ballDistance * goalDistance));

    // angular distance (arc length)
    double turningDistance = AXLE_BASE * turningAngle;

    // time required to finish the turning
    int turningTime = turningDistance / TURNING_SPEED;

    // whether goal is in the LHS or RHS of the ball
    if(timeDifference > 40)
    {
        setMotorPins('r', TURNING_SPEED);
    }
    else
    {
        setMotorPins('l', TURNING_SPEED);
    }
    delay(turningTime);
    
    // move towards the mid-line
    setMotorPins('f', SHOOTING_SPEED);
    int movingTime = (cos(turningAngle) * ballDistance) / SHOOTING_SPEED;
    delay(movingTime);

    // find straight distance from the ball
    double distance = Navigation(1);
   
    
    // move towards the ball and hit it (flapper version)
    MovingTowardsBall(distance);
}

// function of defending
void DefendGoal()
{
    double ballDistance = Navigation(0);
    if(ballDistance < 30)
    {
          MovingTowardsBall(ballDistance);
          MovingAwayFromBall(ballDistance);
    }
}

// function of chasing a moving ball
void FollowMovingBall()
{

    // moving towards ball
    MovingTowardsBall(Navigation(0));
}

// -------------------------------------------------------
// The setup() method runs once, when the sketch starts
// -------------------------------------------------------
void setup()
{ 
    // Initialize the serial communications
    Serial.begin(115200);
  
    // Print the program details
    Serial.println("-------------------------------------");
    Serial.println("Program: Motor Controller"); 
    Serial.println("Initializing ...");
    
    // Call a subroutine to initialize the motor pins
    initializeMotorPins();
  
    // Initialization completed successfully
    Serial.println("Initialization complete");

    // Initialization of PixyCam
    pixy.init();
}

// -------------------------------------------------------
// The loop() method runs over and over again
// -------------------------------------------------------
void loop()
{   
    // find the ball 
    double distance = Navigation(1);

    // kick the ball
    MovingTowardsBall(Navigation(0));

    // Shoot for goal
    Shooting();

    // defend the ball
    DefendGoal();
    
    // follow a moving ball
    FollowMovingBall();       
}
