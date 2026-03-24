#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// RF69
#define RF_CS   10
#define RF_RST  9
#define RF_IRQ  2
#define RF_FREQ 915.0
RH_RF69 rf69(RF_CS, RF_IRQ);


// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);


// JOYSTICKS
#define JOY_L_X   A0 // Left joystick X-axis (left track)
#define JOY_R_X   A1 // Right joystick X-axis (right track)
#define JOY_L_SW  4  // Left joystick button (mode toggle)
#define JOY_R_SW  5  // Right joystick button (IMU request)


// Mode tracking
bool lastModeSw = true;
uint8_t mode = 0; // 0 = RC, 1 = AUTO


// IMU orientation values
float roll = 0, pitch = 0, yaw = 0;


// Convert joystick analog value to signed speed command
int8_t joyToSpeed(int v) {
 int d = v - 512;
 if (abs(d) < 40) return 0;
 return map(d, -512, 512, -127, 127);
}


void setup() {
 Serial.begin(115200);


 // Configure joystick buttons with pull-ups
 pinMode(JOY_L_SW, INPUT_PULLUP);
 pinMode(JOY_R_SW, INPUT_PULLUP);


 // Initialize LCD
 lcd.init();
 lcd.backlight();
 lcd.clear();
 lcd.print("RC Tank Init");


 // Hardware reset of RF module
 pinMode(RF_RST, OUTPUT);
 digitalWrite(RF_RST, LOW);  delay(10);
 digitalWrite(RF_RST, HIGH); delay(10);
 digitalWrite(RF_RST, LOW);


 // Initialize RF69 radio
 rf69.init();
 rf69.setFrequency(RF_FREQ);
 rf69.setTxPower(20, true);


 // Indicate successful RF initialization
 lcd.setCursor(0, 1);
 lcd.print("RF OK");
 delay(1000);
}


void loop() {
 // Mode toggle (edge-detected button press)
 bool sw = digitalRead(JOY_L_SW);
 if (lastModeSw && !sw) mode ^= 1;
 lastModeSw = sw;


 // Read joysticks and compute speed
 int8_t left  = joyToSpeed(analogRead(JOY_L_X));
 int8_t right = joyToSpeed(analogRead(JOY_R_X));


 // Disable manual control in AUTO mode
 if (mode == 1) { left = 0; right = 0; }


 // Check if IMU data is being requested
 bool imuRequest = (digitalRead(JOY_R_SW) == LOW);


 // Transmit control packet
 // [left speed, right speed, mode, IMU request flag]
 uint8_t tx[4] = {
   (uint8_t)left,
   (uint8_t)right,
   mode,
   imuRequest ? 1 : 0
 };


 rf69.send(tx, 4);
 rf69.waitPacketSent();


 // Receive IMU data (if requested)
 if (imuRequest && rf69.waitAvailableTimeout(60)) {
   uint8_t buf[6];
   uint8_t len = sizeof(buf);


   if (rf69.recv(buf, &len) && len == 6) {
     // Decode signed roll, pitch, yaw (scaled by 100)
     int16_t r = (buf[0] << 8) | buf[1];
     int16_t p = (buf[2] << 8) | buf[3];
     int16_t y = (buf[4] << 8) | buf[5];


     roll  = r / 100.0;
     pitch = p / 100.0;
     yaw   = y / 100.0;
   }
 }


 // LCD display update
 lcd.clear();


 // Top row: speed and mode
 lcd.setCursor(0, 0);
 lcd.print("SPD:");
 lcd.print((abs(left) + abs(right)) / 2);


 lcd.setCursor(10, 0);
 lcd.print(mode == 0 ? "RC " : "AUTO");


 // Bottom row: IMU data or prompt
 lcd.setCursor(0, 1);
 if (imuRequest) {
   lcd.print("R"); lcd.print((int)roll);
   lcd.print(" P"); lcd.print((int)pitch);
   lcd.print(" Y"); lcd.print((int)yaw);
 } else {
   lcd.print("IMU: press R");
 }


 delay(50);
}
