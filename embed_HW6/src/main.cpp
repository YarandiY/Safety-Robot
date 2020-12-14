#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <timer.h>

#define DC_MOTOR_1_IN2 6
#define DC_MOTOR_1_IN1 7
#define DC_MOTOR_1_EN1 9
#define DC_MOTOR_2_IN4 8
#define DC_MOTOR_2_IN3 4
#define DC_MOTOR_2_EN2 10
#define MQ2 5
#define TRIGGER 13
#define ECHO 12
// #define GAS_SENSOR 0
int GAS_SENSOR = A0;


enum moves{forward, backward, stop};

void move(moves dir);
void change_fan_speed(int speed);
void timerCallback(void);
void movingTask(void *pvParameters);
void fanTask(void *pvParameters);
void readDistanceTask(void);
void readGasTask(void);


Timer timer;
QueueHandle_t gas_queue;
QueueHandle_t distance_queue;

void setup() {

  /*********************  sensors setup  *********************/
  pinMode(DC_MOTOR_1_IN2, OUTPUT);
  pinMode(DC_MOTOR_1_IN1, OUTPUT);
  pinMode(DC_MOTOR_1_EN1, OUTPUT);
  pinMode(DC_MOTOR_2_IN3, OUTPUT);
  pinMode(DC_MOTOR_2_IN4, OUTPUT);
  pinMode(DC_MOTOR_2_EN2, OUTPUT);

  Serial.begin(9600);

  while( !Serial )
  {
    ;  // wait for serial port to connect
  }

  /*********************  timer setup  *********************/
  timer.setInterval(1000);
  timer.setCallback(timerCallback);
  timer.start();


  /*********************  queues setup  *********************/
  // gas_queue = xQueueCreate(1, sizeof(float));
  gas_queue = xQueueCreate(1, sizeof(int));
  distance_queue = xQueueCreate(1, sizeof(long));


  /*********************  event triggered tasks setup  *********************/
  xTaskCreate(movingTask, "Moving", 128, NULL, 2, NULL);
  xTaskCreate(fanTask, "Fan", 128, NULL, 1, NULL);

}

void loop() {
  timer.update();
}

void timerCallback() {
  readGasTask();
  readDistanceTask();
}

/*********************  read from distance sensor *********************/
void readDistanceTask(){
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  long distance = pulseIn(ECHO, HIGH);
  distance = distance/58;
  Serial.print("Distance from the object = ");
  Serial.println(distance);
  xQueueSend(distance_queue, &distance, portMAX_DELAY);
}

/*********************  read from gas sensor  *********************/
void readGasTask(){
  int value = analogRead(GAS_SENSOR);
  // int value = digitalRead(MQ2);
  xQueueSend(gas_queue, &value, portMAX_DELAY);
  Serial.print("gas sensor output : ");
  Serial.println(analogRead(GAS_SENSOR));
}

void movingTask(void *pvParameters){
  long distance_queue_item = 0;
  long gas_queue_item = 0;
  while (true){
    if ( xQueueReceive(distance_queue, &distance_queue_item, portMAX_DELAY ) == pdPASS ) {
      if(distance_queue_item < 300){
       move(backward);
       change_fan_speed(0);
      } else if (distance_queue_item > 500){
        move(forward);
      } else {
        if(xQueueReceive(gas_queue, &gas_queue_item, portMAX_DELAY ) == pdPASS){
          int temp = (int)(127*(gas_queue_item)/1023 + 127);
         if(gas_queue_item > 600){
           change_fan_speed(0);
              move(backward);
          }else
          {
            change_fan_speed(temp);
            move(stop);
          }
        }else{
          move(stop);
        }
      }
    }
  }
}

void fanTask(void *pvParameters){
  int gas_queue_item = 0.0;
  while (true){
    if (xQueueReceive(gas_queue, &gas_queue_item, portMAX_DELAY) == pdPASS) {
      int temp = (int)(127*(gas_queue_item)/1023 + 127);
      Serial.print("**** ");
      Serial.println(temp);
      if(gas_queue_item > 600){
        change_fan_speed(0);
      }else {
      change_fan_speed(temp);
      }
      
    }
  }
}

void move(moves dir){
  if(dir == forward){
    analogWrite(DC_MOTOR_1_EN1,200);
    digitalWrite(DC_MOTOR_1_IN1, HIGH);
    digitalWrite(DC_MOTOR_1_IN2,LOW);    
  }
  if(dir == stop){
    digitalWrite(DC_MOTOR_1_IN1,LOW); 
    digitalWrite(DC_MOTOR_1_IN2,LOW);
  }
  if(dir == backward){ 
    analogWrite(DC_MOTOR_1_EN1,200);
    digitalWrite(DC_MOTOR_1_IN1,LOW);
    digitalWrite(DC_MOTOR_1_IN2,HIGH);
  }
}

void change_fan_speed(int speed){
  analogWrite(DC_MOTOR_2_EN2, speed);
  digitalWrite(DC_MOTOR_2_IN3, HIGH);
  digitalWrite(DC_MOTOR_2_IN4, LOW);
}