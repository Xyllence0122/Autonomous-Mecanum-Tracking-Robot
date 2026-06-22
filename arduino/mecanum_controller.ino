// ======================================
// Back
// ======================================

// LR
#define LR_IN1 30
#define LR_IN2 31

// RR
#define RR_IN1 32
#define RR_IN2 33

// PWM
#define LR_PWM 2
#define RR_PWM 3

// ======================================
// Front
// ======================================

// LF
#define LF_IN1 34
#define LF_IN2 35

// RF
#define RF_IN1 36
#define RF_IN2 37

// PWM
#define LF_PWM 4
#define RF_PWM 5

// ======================================
// Setup
// ======================================

void setup()
{
    Serial.begin(115200);

    pinMode(LR_IN1, OUTPUT);
    pinMode(LR_IN2, OUTPUT);

    pinMode(RR_IN1, OUTPUT);
    pinMode(RR_IN2, OUTPUT);

    pinMode(LF_IN1, OUTPUT);
    pinMode(LF_IN2, OUTPUT);

    pinMode(RF_IN1, OUTPUT);
    pinMode(RF_IN2, OUTPUT);

    pinMode(LR_PWM, OUTPUT);
    pinMode(RR_PWM, OUTPUT);

    pinMode(LF_PWM, OUTPUT);
    pinMode(RF_PWM, OUTPUT);

    stopAll();

    Serial.println("Mecanum Controller Ready");
}

// ======================================
// Loop
// ======================================

void loop()
{
    if (Serial.available())
    {
        String msg = Serial.readStringUntil('\n');

        int v1, v2, v3, v4;

        int result = sscanf(
            msg.c_str(),
            "%d,%d,%d,%d",
            &v1,
            &v2,
            &v3,
            &v4
        );

        if (result == 4)
        {
            setMotorLF(v1);
            setMotorRF(v2);
            setMotorLR(v3);
            setMotorRR(v4);
        }
    }
}

// ======================================
// LF
// ======================================

void setMotorLF(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(LF_IN1, LOW);
        digitalWrite(LF_IN2, HIGH);
    }
    else if (speed < 0)
    {
        digitalWrite(LF_IN1, HIGH);
        digitalWrite(LF_IN2, LOW);
    }
    else
    {
        digitalWrite(LF_IN1, LOW);
        digitalWrite(LF_IN2, LOW);
    }

    analogWrite(LF_PWM, abs(speed));
}

// ======================================
// RF
// ======================================

void setMotorRF(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(RF_IN1, HIGH);
        digitalWrite(RF_IN2, LOW);
    }
    else if (speed < 0)
    {
        digitalWrite(RF_IN1, LOW);
        digitalWrite(RF_IN2, HIGH);
    }
    else
    {
        digitalWrite(RF_IN1, LOW);
        digitalWrite(RF_IN2, LOW);
    }

    analogWrite(RF_PWM, abs(speed));
}

// ======================================
// LR
// ======================================

void setMotorLR(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(LR_IN1, LOW);
        digitalWrite(LR_IN2, HIGH);
    }
    else if (speed < 0)
    {
        digitalWrite(LR_IN1, HIGH);
        digitalWrite(LR_IN2, LOW);
    }
    else
    {
        digitalWrite(LR_IN1, LOW);
        digitalWrite(LR_IN2, LOW);
    }

    analogWrite(LR_PWM, abs(speed));
}

// ======================================
// RR
// ======================================

void setMotorRR(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(RR_IN1, HIGH);
        digitalWrite(RR_IN2, LOW);
    }
    else if (speed < 0)
    {
        digitalWrite(RR_IN1, LOW);
        digitalWrite(RR_IN2, HIGH);
    }
    else
    {
        digitalWrite(RR_IN1, LOW);
        digitalWrite(RR_IN2, LOW);
    }

    analogWrite(RR_PWM, abs(speed));
}

// ======================================
// Stop
// ======================================

void stopAll()
{
    setMotorLF(0);
    setMotorRF(0);
    setMotorLR(0);
    setMotorRR(0);
}
