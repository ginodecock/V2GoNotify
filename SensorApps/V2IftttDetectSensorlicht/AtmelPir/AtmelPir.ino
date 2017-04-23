//V2 IFTTT PIR people detect Sensorlicht
#define F_CPU 4000000 // clock frequency, set according to clock used!
#include <SoftwareSerial.h>
#include <espduino.h>
#include <rest.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define LED 13
#define Esp_IO0 17
#define Esp_RST 16
#define Sensor_power 15
#define Esp_power 10
#define Buzzer 9
#define Step_EN 7
#define Trigger_switch 3
#define Relay 18
#define Pir_sensor 2

#define IFTTT_trigger "/trigger/TrapPIR/with/key/<IFTTT trigger key>"

volatile long lastDebounceTime = 0;   // the last time the interrupt was triggered
#define debounceDelay 7    // the debounce time in ms; decrease if quick button presses are ignored, increase
//if you get noise (multipule button clicks detected by the code when you only pressed it once)

volatile boolean wifiConnected = false;
volatile boolean sendMessage = false;

volatile boolean startup = true;
volatile int currentPIR = 0;
volatile int previousPIR = 0;

volatile int lumen = 0;
SoftwareSerial debugPort(10, 11); // RX, TX

void wifiCb(void* response)
{
  uint32_t status1;
  RESPONSE res(response);
  debugPort.print(F("getArcWIFI CON STAT: "));
  debugPort.println(res.getArgc());

  if (res.getArgc() == 1) {
    res.popArgs((uint8_t*)&status1, 4);
    debugPort.print(F("WIFI CON STAT: "));
    debugPort.println(status1);

    if (status1 == STATION_GOT_IP) {
      debugPort.println(F("WIFI CONNECTED"));
      debugPort.println(status1);

      wifiConnected = true;
    } else {
      wifiConnected = false;
    }
  }
}
void triggerDetection()
{
  if (!startup){
    if ((millis() - lastDebounceTime) > debounceDelay) //if current time minus the last trigger time is greater than
    { //the delay (debounce) time, button is completley closed.
      lastDebounceTime = millis();
      digitalWrite(LED, HIGH);//LED
      sendMessage = true;
     }
  } else{
    startup = 0;
  }
}
void setup() {
  pinMode(Sensor_power, OUTPUT);
  digitalWrite(Sensor_power, LOW);
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, LOW);

  
  pinMode(Esp_IO0, OUTPUT);//esp IO0
  digitalWrite(Esp_IO0, HIGH);//esp IO0
  
  pinMode(Trigger_switch, OUTPUT); //PB switch power low for Low power
  digitalWrite(Trigger_switch, LOW);
  pinMode(Step_EN, OUTPUT);
  digitalWrite(Step_EN, HIGH); //3.3v on

  pinMode(LED, OUTPUT);//groene led
  pinMode(Buzzer, OUTPUT);//buzzer
  digitalWrite(Buzzer, LOW);//buzzer

  pinMode(Esp_power, OUTPUT);//PB Bridge
  digitalWrite(Esp_power, LOW);//PB Bridge
  pinMode(Esp_RST, OUTPUT);//esp reset
  digitalWrite(Esp_RST, HIGH);//esp reset

  pinMode(Pir_sensor, INPUT);// read pir sensor
  digitalRead(Pir_sensor);
  pinMode(A0, INPUT);// read pir sensor
  analogRead(A0);

  
  debugPort.begin(38400);
  debugPort.println("start v3.0");
  Serial.begin(38400);
  attachInterrupt(1, triggerDetection, CHANGE);
   delay(10000);
  digitalWrite(Buzzer,true);
  delay(100);
  digitalWrite(Buzzer,false);
  delay(100);
  digitalWrite(Buzzer,true);
  delay(100);
  digitalWrite(Buzzer,false);
  delay(100);
  digitalWrite(Buzzer,true);
  delay(100);
  digitalWrite(Buzzer,false);
 
}
void sendIFTTTMessage()
{
  int retrySend = 0;
  do
  {
    char data_buf[256];
    //reset wifi
    asm volatile ("cbi %0, %1 \n\t" :: "I" (_SFR_IO_ADDR(PORTC)), "I" (PORTC2));
 
    ESP esp(&Serial, &debugPort, 6);
    REST rest(&esp);

    wifiConnected = false;
    esp.enable();
    asm volatile ("sbi %0, %1 \n\t" :: "I" (_SFR_IO_ADDR(PORTC)), "I" (PORTC2));
    
    while (!esp.ready());
    if (!rest.begin("maker.ifttt.com", 443, true)) {
      debugPort.println("GN: failed setup rest client");
      while (1);
    }
    rest.setContentType("application/json");
    rest.setHeader("Authorization: GDC\r\n");
    debugPort.println("GN: setup wifi");
    esp.wifiCb.attach(&wifiCb);
    esp.wifiConnect("#SSID#", "saved");
    int timeout = 0;
    while (!wifiConnected)
    {
      esp.process();
      delay(10);
      timeout++;
      if (timeout > 3000) break;
    }
    if (!wifiConnected) {
      asm volatile ("cbi %0, %1 \n\t" :: "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB5));
      delay(50);              // wait for a second
      asm volatile ("sbi %0, %1 \n\t" :: "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB5));
      delay(50);
    }
    memset(data_buf, 0, sizeof(data_buf));
    sprintf(data_buf, "\n{\"value1\": \"%d\"}",lumen);
    
    debugPort.println(data_buf);
    rest.post(IFTTT_trigger, (const char*)data_buf);
    memset(data_buf, 0, sizeof(data_buf));
   if (rest.getResponse(data_buf, 256) == HTTP_STATUS_OK) {
      debugPort.println(data_buf);
      debugPort.println(F("GN: POST successful"));
      retrySend = 100;
    } else {
      debugPort.println(data_buf);
      debugPort.println(F("GN: POST error"));
      retrySend++;
    }
  } while (retrySend < 3);
  sendMessage = false;
}
void loop() {
  debugPort.println((String)digitalRead(Pir_sensor) + " " + (String)analogRead(A0));
  currentPIR = digitalRead(Pir_sensor);
  if (currentPIR && !previousPIR){
      sendMessage = true; 
      startup = 0;
  };
  previousPIR = currentPIR;
  
  if (sendMessage && !startup)
  {
    debugPort.println(F("GN: Send Message"));
    lumen = analogRead(A0);
    if (lumen < 20){
      digitalWrite(LED,HIGH);
      digitalWrite(Relay, LOW);
    }
    if (lumen > 80){
      digitalWrite(LED,HIGH);
      digitalWrite(Relay, LOW);
    }
    digitalWrite(Step_EN, HIGH); //3.3v on
    delay(100);
    digitalWrite(Esp_IO0, HIGH);//esp IO0
    digitalWrite(Esp_power, LOW);//PB Bridge
    delay(1000);
    sendIFTTTMessage();
    digitalWrite(Esp_power, HIGH);
    digitalWrite(Esp_RST, LOW);
    digitalWrite(Esp_IO0, LOW);//esp IO0
    digitalWrite(LED,LOW);
    delay(1000);
    digitalWrite(LED,HIGH);
    delay(5000);
    while (digitalRead(Pir_sensor) == HIGH) {
      digitalWrite(LED,LOW);
      delay(100);
      digitalWrite(LED,HIGH);
      delay(100);
    }
    digitalWrite(Relay, HIGH);
    digitalWrite(LED,LOW);
    delay(500);
    sendMessage = false;
  }
  delay(500); 
}
