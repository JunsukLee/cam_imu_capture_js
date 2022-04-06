#define TRIGGER_PIN_CAM 8
#define TRIGGER_PIN_IMU 9

#define TriggerSyncThreshold 5

#define FrequencyIMU 10000 // 1000 is 1 ms
#define Trigger_Delay 100

int counter_IMU_data = 1;
int mFrequency_imu = FrequencyIMU - Trigger_Delay;

void trigger_IMU();
void trigger_IMU_CAM();

void setup() {
  // put your setup code here, to run once:
  digitalWrite(TRIGGER_PIN_CAM, LOW);  //  drive it low without temporarily driving it high
  digitalWrite(TRIGGER_PIN_IMU, LOW);  //  drive it low without temporarily driving it high
  pinMode(TRIGGER_PIN_CAM, OUTPUT);
  pinMode(TRIGGER_PIN_IMU, OUTPUT);
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
  digitalWrite(TRIGGER_PIN_IMU, HIGH);
  delayMicroseconds(Trigger_Delay);
  digitalWrite(TRIGGER_PIN_IMU, LOW);
}

void trigger_IMU_CAM(){
  digitalWrite(TRIGGER_PIN_IMU, HIGH);
  digitalWrite(TRIGGER_PIN_CAM, HIGH);
  delayMicroseconds(Trigger_Delay);
  digitalWrite(TRIGGER_PIN_IMU, LOW);
  digitalWrite(TRIGGER_PIN_CAM, LOW);
}
