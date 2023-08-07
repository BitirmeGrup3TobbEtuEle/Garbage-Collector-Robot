#include <Servo.h>

// Durumlar
enum State {
  WAITING,
  DISTANCE_CHECK,
  CUP_PICKUP,
  CUP_DROP,
  SENSOR_CHECK
};

State currentState = WAITING;

// Servo motor nesnesini oluştur
Servo servoMotor;

// Servo pwm pini => 10. pin 
int servoPWMPin = 10;

// Servo motorun uygun şekilde açılma açısı
int servoOpenArm = 180;

const int triggerPin = 9;    // Mesafe sensörü trigger pin yesil kablo
const int echoPin = 8;    // Mesafe sensörü echo pin mor kablo

const int appropriateDistance = 12;  // Bardak için uygun mesafeye göre 1/0 yazma pini JETSON 15
const int takeCup = 11; // Bardak için servo tuttuysa 1/0 okuma pini JETSON 16
const int wasteTypeAnalog = A0; // Dedektör okuma analog pin
const int wasteTypeDigital = 2; // Dedektör yazma dijital pin JETSON 40

unsigned long previousMillis = 0;
const unsigned long interval = 100; // Durum geçişleri arasındaki minimum zaman aralığı
const int sensorCheckCount = 50; // SENSOR_CHECK adımının tekrar sayısı
int sensorCheckCounter = 0;
float DECIDER = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(appropriateDistance, OUTPUT);  // Bardak için uygun mesafeye göre 1/0 okuma pini => OUTPUT
  pinMode(servoPWMPin, OUTPUT); // Servo pwm pini => OUTPUT
  pinMode(triggerPin, OUTPUT);  // Ultrasonik sensör trigger pini => OUTPUT
  pinMode(wasteTypeDigital, OUTPUT); // Dedektör yazma dijital pin => OUTPUT
  pinMode(takeCup, INPUT); // Bardak için servo tuttuysa 1/0 yazma pini => OUTPUT
  pinMode(echoPin, INPUT);  // Ultrasonik sensör echo pini => INPUT
  pinMode(wasteTypeAnalog, INPUT); // Dedektör okuma analog pin => INPUT
  
  servoMotor.attach(servoPWMPin); // Servo motor için uygun pin bağlantısı
  servoMotor.write(servoOpenArm);

  currentState = WAITING;
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentState == WAITING)
  {
    //Serial.println("WAITING");
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      DECIDER = 0;
      currentState = DISTANCE_CHECK;
    }
  }
  else if (currentState == DISTANCE_CHECK)
  {
    //Serial.println("DISTANCE_CHECK");
    digitalWrite(triggerPin, HIGH);  // Ölçüm için trigger pini HIGH
    delayMicroseconds(10);          // 10 mikrosaniye bekle
    digitalWrite(triggerPin, LOW); // Ölçüm için trigger pini LOW

    long durationMicroSec = pulseIn(echoPin, HIGH);  // Echo pininden pulse süresini ölçüyor
    float distanceincm = durationMicroSec / 58.2;  // Cm cinsinden mesafe hesaplanır
    Serial.println(distanceincm);
    digitalWrite(wasteTypeDigital, LOW);
    if (distanceincm < 7) 
    {
      digitalWrite(appropriateDistance, HIGH);  // JETSON NANO'ya uygun mesafe bilgisi verir
      currentState = CUP_PICKUP;
    }
    else 
    {
      digitalWrite(appropriateDistance, LOW);  // Mesafe uygun değilse 0 yazar
      currentState = DISTANCE_CHECK;
    }
  }
  else if (currentState == CUP_PICKUP)
  {
    //Serial.println("CUP_PICKUP");
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      int IS_CUP_TAKEN = digitalRead(takeCup);
      if (IS_CUP_TAKEN) 
      {
        delay(1000);
        servoMotor.write(servoOpenArm-22); 
        delay(1000); 
        servoMotor.write(servoOpenArm-44);
        delay(1000);          
        servoMotor.write(servoOpenArm-66);  
        delay(1000);          
        servoMotor.write(servoOpenArm-88); 
        delay(1000);          
        servoMotor.write(servoOpenArm-110); 
        digitalWrite(wasteTypeDigital, LOW);
        currentState = SENSOR_CHECK;
        sensorCheckCounter = 0; // SENSOR_CHECK adımının tekrar sayacını sıfırla
      }
      else 
      {
        digitalWrite(wasteTypeDigital, LOW);
        currentState = CUP_PICKUP;
      }
    }
  }
  else if (currentState == SENSOR_CHECK)
  {
    //Serial.println("SENSOR_CHECK");
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      digitalWrite(appropriateDistance, LOW);
      int SENSOR_READ_VALUE = analogRead(wasteTypeAnalog);
      Serial.println(SENSOR_READ_VALUE);
      if (SENSOR_READ_VALUE < 55) 
      {
        DECIDER = DECIDER + 1;
      }
      sensorCheckCounter++; // SENSOR_CHECK adımının tekrar sayacını artır
      if (sensorCheckCounter >= sensorCheckCount)
      {
        digitalWrite(wasteTypeDigital, LOW);
        Serial.println(DECIDER/50);
        if ((DECIDER/50) > 0.4)
        {
          digitalWrite(wasteTypeDigital, HIGH); // Threshold değerinin üzerinde ise dijital pine 1 yazılır
          Serial.println("METAL BULDUM");
        }
        else
        {
          digitalWrite(wasteTypeDigital, LOW); // Threshold değerinin üzerinde ise dijital pine 1 yazılır
          Serial.println("CAM/PLASTIK BULDUM"); 
        } 
        
        currentState = CUP_DROP;
      }
      else {
        delay(10);
        currentState = SENSOR_CHECK;
      }
    }
  }
  else if (currentState == CUP_DROP)
  {
    //Serial.println("CUP_DROP");
    if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;
      int IS_CUP_TAKEN = digitalRead(takeCup);
      if (!IS_CUP_TAKEN) 
      {
        servoMotor.write(servoOpenArm-88); 
        delay(1000);
        servoMotor.write(servoOpenArm-66); 
        delay(1000);
        servoMotor.write(servoOpenArm-44); 
        delay(1000); 
        servoMotor.write(servoOpenArm-22); 
        delay(1000);          
        servoMotor.write(servoOpenArm); 
        delay(3000);
        currentState = WAITING;
      }
      else 
      {
        currentState = CUP_DROP;
      }
    }
  }
}
