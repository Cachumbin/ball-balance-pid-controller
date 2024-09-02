#include <Servo.h>

Servo servopid;

const int trigPin = 12;
const int echoPin = 11;

float Kp = 1.5;
float Ki = 1.4;
float Kd = 3.0;

float setPoint = 30.0;
float input, output;
float previousError = 0.0;
float integral = 0.0;

const float integralMax = 100.0;
const float integralMin = -100.0;

void setup()
{
    Serial.begin(9600);
    servopid.attach(3);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    servopid.write(55);
}

void loop()
{
    input = readDistance();

    float error = input - setPoint;

    integral += error;

    integral = constrain(integral, integralMin, integralMax);

    float derivative = error - previousError;

    output = Kp * error + Ki * integral + Kd * derivative;

    updateServo(output);

    Serial.println(output);

    previousError = error;

    delay(50);
}

float readDistance()
{
    long duration;
    float distance;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration * 0.034) / 2;

    return distance;
}

void updateServo(float pidOutput)
{
    int servoPosition = map(pidOutput, -70, 48, 30, 70);

    servoPosition = constrain(servoPosition, 30, 70);

    servopid.write(servoPosition);

    Serial.print("Servo Position: ");
    Serial.println(servoPosition);
}
