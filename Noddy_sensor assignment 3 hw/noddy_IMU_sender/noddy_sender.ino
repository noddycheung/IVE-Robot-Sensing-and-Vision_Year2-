#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

const int IMU_ADDR = 0x68;  // I2C address for IMU. ADO=5V -> 0x69

int16_t accel_X, accel_Y, accel_Z, temp_raw, gyro_X, gyro_Y, gyro_Z;  // variables for raw sensor data
double aX, aY, aZ, theta_acc, phi_acc;                                //accel values and theta and phi calculated from accelerometer
double theta_acc_previous = 0, theta_acc_LP, phi_acc_previous = 0, phi_acc_LP;
double gX, gY, gZ, theta_gyro = 0, phi_gyro = 0, psi_gyro;
double theta_compli, phi_compli;  //theta and phi calculated from accelerometer and gyroscope
unsigned long previousTime, elapseTime;
double dt;

int n_forward, n_backward, n_left, n_right;

//insert mac address of receiver
//uint8_t broadcastAddress[] = {0xE8, 0x68, 0xE7, 0x22, 0xC3, 0xF0};
uint8_t broadcastAddress[] = { 0x3C, 0x61, 0x05, 0x03, 0x95, 0x58 };
// 3C:61:05:03:95:58

//structure of the message to send
//must be matched to the receiver structure
typedef struct struct_message {
  int forward;
  int backward;
  int left;
  int right;
} struct_message;


//create a new variable to store the value of "struct_message"
struct_message myData;


//create a new variable to store information about the peer
esp_now_peer_info_t peerInfo;


long serial_print_pmillis = 0;
bool can_print = 0;

//define a callback function that will be executed when a message is sent
//simpfly print if the message was successfully delivered or not
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //if (can_print) {
    Serial.println(WiFi.macAddress());
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Failed");
  //}
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //set the device as a WiFi station
  WiFi.mode(WIFI_STA);

  //initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //once ESP-NOW is successfully init
  //register a callback function that will be called when a message is sent
  esp_now_register_send_cb(OnDataSent);

  //pair with anoother ESP-NOW device to send data
  //register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  //setup for I2C communicate of mpu9250
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);  //communicate with IMU
  Wire.write(0x6B);                  //PWR_MGMT_1 register
  Wire.write(0x0);
  Wire.endTransmission(true);

  Wire.beginTransmission(IMU_ADDR);  //communicate with IMU
  Wire.write(0x1C);                  //ACCEL_CONFIG register
  Wire.write(0b00000000);            //Bit3 & Bit 4 -> 0,0 -> +/-2g
  Wire.endTransmission(true);

  Wire.beginTransmission(IMU_ADDR);  //communicate with IMU
  Wire.write(0x1B);                  //GYRO_CONFIG register
  Wire.write(0b00000000);            //Bit3 & Bit 4 -> 0,0 -> +/-250deg/s
  Wire.endTransmission(true);

  Wire.beginTransmission(IMU_ADDR);  //communicate with IMU
  Wire.write(0x37);                  //  IMU INT PIN CONFIG
  Wire.write(0x02);                  //  0x02 activate bypass in order to communicate with magnetometer
  Wire.endTransmission(true);
  delay(100);

  previousTime = millis();
}

void loop() {
  can_print = 0;//millis() - serial_print_pmillis > 500;
  if (millis() - serial_print_pmillis > 500) serial_print_pmillis = millis();
  // put your main code here, to run repeatedly:

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);  // start from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 14, true);  //request a total of 14 bytes

  // read registers from IMU
  accel_X = Wire.read() << 8 | Wire.read();   //read registers 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accel_Y = Wire.read() << 8 | Wire.read();   //read registers 0x3D & 0x3E
  accel_Z = Wire.read() << 8 | Wire.read();   //read registers 0x3F & 0x40
  temp_raw = Wire.read() << 8 | Wire.read();  //read registers 0x41 & 0x42
  gyro_X = Wire.read() << 8 | Wire.read();    //read registers 0x43  (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro_Y = Wire.read() << 8 | Wire.read();    //read registers 0x45 & 0x46
  gyro_Z = Wire.read() << 8 | Wire.read();    //read registers 0x47 & 0x48

  Wire.endTransmission(true);

  aX = accel_X / 16384.0;  // convert the raw data to g value. 1g = 9.8 m/s2
  aY = accel_Y / 16384.0;
  aZ = accel_Z / 16384.0;
  if (can_print) {
    Serial.print("ax:\t");
    Serial.print(aX);
    Serial.print(",ay:\t");
    Serial.print(aY);
    Serial.print(",az:\t");
    Serial.println(aZ);
  }

  //--------------Getting Roll(phi) and Pitch(theta) from accelerometer readings only-------------------//

  theta_acc = atan2(aX, aZ) * (180 / PI) - 44;  // approximation of theta (pitch). use either this or the eauation right below.
  // theta_acc = atan2(aX, sqrt(pow(aY,2)+pow(aZ,2)))*(180/PI); // another approximation on theta (pitch). Either use this or the equation right above.
  phi_acc = atan2(aY, aZ) * (180 / PI) + 44;  // approximation of phi (roll). use either this or the eauation right below.
  // phi_acc = atan2(aY, sqrt(pow(aX,2)+pow(aZ,2)))*(180/PI);// another approximation on phi (roll). Either use this or the equation right above.

  // simple Low Pass Filter (LPF) added for theta(pitch) and phi(roll)

  theta_acc_LP = 0.7 * theta_acc_previous + 0.3 * theta_acc;  // adjust the ratio of preivous theta value and the newly measured value
  phi_acc_LP = 0.7 * phi_acc_previous + 0.3 * phi_acc;        //gs only-----

  theta_acc_previous = theta_acc_LP;
  phi_acc_previous = phi_acc_LP;

  //------------end of Roll(phi) and Pitch(theta) from accelerometer readings---------//

  //--------------Getting Roll(phi) and Pitch(theta) from gyroscope readin--------------//

  gX = gyro_X / 131.0 - 5.8;  // convert the raw data to rad/s. "- 5.8" is the gyro error
  gY = gyro_Y / 131.0 - 6;
  gZ = gyro_Z / 131.0;

  elapseTime = millis() - previousTime;
  dt = elapseTime / 1000.0;
  previousTime = millis();

  theta_gyro = theta_gyro - gY * dt;  // pitch angle from gyro
  phi_gyro = phi_gyro + gX * dt;      // roll angle from gyro
  psi_gyro = psi_gyro + gZ * dt;      // yaw angle from gyro (not quite useful, only incremental value)

  //-----------end of Roll(phi) and Pitch(theta) from gyroscope readings --------------------//

  // ------- Complimentary filter from accelerometer and gyroscope data ---------//
  // Complimentary filter uses a mix of two sensor data with
  //a specific ratio which depend on which one you trust better

  theta_compli = 0.95 * (theta_compli - gY * dt) + 0.05 * (theta_acc);
  phi_compli = 0.95 * (phi_compli + gX * dt) + 0.05 * (phi_acc);

  // ------- end of Complimentary filter from accelerometer and gyroscope data -----//


  //-------conditional limition of reading value (lowerLimit - larger than && upperLimit - smaller than)-------//

  if (theta_acc < -8 && theta_acc > -50 && phi_acc < -2 && phi_acc > -40) { n_forward = 1; }  //statement of forward
  else {
    n_forward = 0;
  }

  if (theta_acc > 10 && theta_acc < 50 && phi_acc < -2 && phi_acc > -40) { n_backward = 1; }  //statement of backward
  else {
    n_backward = 0;
  }

  if (theta_acc > 2 && theta_acc < 60 && phi_acc < -12 && phi_acc > -50) { n_left = 1; }  //statement of left
  else {
    n_left = 0;
  }

  if (theta_acc > 3 && theta_acc < 50 && phi_acc > 10 && phi_acc < 120) { n_right = 1; }  //statement of right
  else {
    n_right = 0;
  }

  //------------------------------End of limition-------------------------------//

  //set the variables values to send
  if (can_print) Serial.print("255*aY:\t");
  if (can_print) Serial.println(255 * aY);
  myData.forward = abs(255 * aY * n_forward);
  myData.backward = 255 * aY * !(n_forward || n_left || n_right);
  myData.left = constrain(map(aX, 1, -1, 255, -255),-255,255);
  myData.right = constrain(map(aX, 1, -1, -255, 255),-255,255);
  /*
  //set the variables values to send
  myData.forward = n_forward;
  myData.backward = n_backward;
  myData.left = n_left;
  myData.right = n_right;
  */

  if (can_print) Serial.print("theta_acc :");
  if (can_print) Serial.println(theta_acc);
  if (can_print) Serial.print("phi_acc :");
  if (can_print) Serial.println(phi_acc);

  //  Serial.print("theta_acc_LP :"); Serial.println(theta_acc_LP);
  //  Serial.print("phi_acc_LP :"); Serial.println(phi_acc_LP);

  //  Serial.print("theta_gyro :"); Serial.println(theta_gyro);
  //  Serial.print("phi_gyro :"); Serial.println(phi_gyro);
  //  Serial.print("psi_gyro :"); Serial.println(psi_gyro);

  //  Serial.print("theta_compli :"); Serial.println(theta_compli);
  //  Serial.print("phi_compli :"); Serial.println(phi_compli);

  if (can_print) {
    Serial.print("forward :");
    Serial.println(n_forward);
    Serial.print("backward :");
    Serial.println(n_backward);
    Serial.print("left :");
    Serial.println(n_left);
    Serial.print("right :");
    Serial.println(n_right);
  }

  //send the message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    /*if (can_print)*/ Serial.println("Sent with success");
  } else {
    /*if (can_print)*/ Serial.println("Error sending the data");
  }
  //delay(50);
}