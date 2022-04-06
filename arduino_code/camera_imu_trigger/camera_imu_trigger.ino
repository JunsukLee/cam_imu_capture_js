// https://forum.arduino.cc/index.php?topic=52534.0
// https://www.arduino.cc/en/Hacking/PinMapping2560
#define TRIGGER_PIN_CAM 8//0x20//8
#define TRIGGER_PIN_IMU 9//0x40//9

#define TriggerSyncThreshold 5


#define Trigger_Delay 3
const int FrequencyIMU = 10000-(int)Trigger_Delay; // 1000 is 1 ms

int counter_IMU_data = 1;
int mFrequency_imu = FrequencyIMU - Trigger_Delay;

void trigger_IMU();
void trigger_IMU_CAM();

void setup() {
  // put your setup code here, to run once:
  //digitalWrite(TRIGGER_PIN_CAM, LOW);  //  drive it low without temporarily driving it high
  //digitalWrite(TRIGGER_PIN_IMU, LOW);  //  drive it low without temporarily driving it high
  //pinMode(TRIGGER_PIN_CAM, OUTPUT);
  //pinMode(TRIGGER_PIN_IMU, OUTPUT);
  DDRH = B11100000; // set PORTD (digital 7~0) to outputs  
}

void loop() {
  // put your main code here, to run repeatedly:
  delayMicroseconds(FrequencyIMU);
  if(counter_IMU_data == TriggerSyncThreshold)
  {
    trigger_IMU_CAM();
    counter_IMU_data = 1;
  }else
  {
    trigger_IMU();
    ++counter_IMU_data;
  }
}

void trigger_IMU() {
  PORTH = B01000000; // digital 9 HIGH
  delayMicroseconds(Trigger_Delay);
  PORTH = B00000000; // digital 9 LOW  
}

void trigger_IMU_CAM(){
  PORTH = B01100000; // digital 8~9 HIGH
  delayMicroseconds(Trigger_Delay);
  PORTH = B00000000; // digital 8~9 LOW  
}
