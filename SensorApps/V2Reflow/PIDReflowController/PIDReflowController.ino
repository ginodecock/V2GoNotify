//Reflowcontroller
#define USE_MAX6675

#include <max6675.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
	REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS {
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef	enum SWITCH {
	SWITCH_NONE,
	SWITCH,	
}	switch_t;

typedef enum DEBOUNCE_STATE {
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50         // fan stops at this temperature on cool-down
#define TEMPERATURE_SOAK_MIN 135    // end of pre-heat, start of soak, drift to 150
#define TEMPERATURE_SOAK_MAX 185    // end of soak, start of reflow
#define TEMPERATURE_REFLOW_MAX 218  // buzzer beeps to open door, fan starts, cool,
                                    // temperature will drift to 225, max reflow
#define TEMPERATURE_COOL_MIN 100    // buzzer beeps, signaling remove PCB from oven
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50
#define PID_SAMPLE_TIME 1000

// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 40
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20

// ***** SOAKING STAGE *****
#define PID_KP_SOAK 0
#define PID_KI_SOAK 0
#define PID_KD_SOAK 0

// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 100  
#define PID_KI_REFLOW 0.025
#define PID_KD_REFLOW 25

// ***** WiFi thingspeak *****
#define SSID "<AP SSID>"
#define PASS "<AP Password>"
#define IP "184.106.153.149" // thingspeak.com
String GET = "GET /update?key=<Thingspeak API key>&field1=";
String DELETE= "DELETE /channnels/261313/feeds.xml?api_key=<Thingspeak API key>";

const char* lcdMsgReflowStatus[] = {
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
	"Wait,hot",
  "Error"
};


unsigned char degree[8]  = {140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
//#define fanPin 5
#define ledPin 13
#define ssrPin 14
#define buzzerPin 9
#define pb1Pin 3
#define tcCLKPin 18
#define tcCSPin 19
#define tcSOPin 2
#define Esp_IO0 17
#define Esp_RST 16
#define Esp_power 10
#define Sensor_power 15
#define Step_EN 7

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
unsigned long windowSize;       // for proportional time control
unsigned long windowStartTime;  // for proportional time control

// ***** MISC VARIABLES *****
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
long lastDebounceTime;
int timerSeconds;
int timeThingspeak = 0;

// Reflow oven controller state machine state variable
reflowState_t reflowState;

// Reflow oven controller status
reflowStatus_t reflowStatus;

// Switch debounce state machine state variable
debounceState_t debounceState;

// Switch press status
switch_t switchStatus;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Specify MAX6675 thermocouple interface
MAX6675 thermocouple(tcCLKPin, tcCSPin, tcSOPin);

void updateTemp(String tenmpF, String title){
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += IP;
  cmd += "\",80";
  Serial.println(cmd);
  //delay(2000);
  if(Serial.find("Error")){
    return;
  }
  cmd = GET;
  cmd += tenmpF;
  cmd += "&status=";
  cmd += title;
  cmd += "\r\n";
  Serial.print("AT+CIPSEND=");
  Serial.println(cmd.length());
  delay(100);
  if(Serial.find(">")){
    Serial.print(cmd);
  }else{
    Serial.println("AT+CIPCLOSE");
  }
}
void clearTemp(){
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += IP;
  cmd += "\",80";
  Serial.println(cmd);
  delay(200);
  if(Serial.find("Error")){
    return;
  }
  cmd = DELETE;
  cmd += "\r\n";
  Serial.print("AT+CIPSEND=");
  Serial.println(cmd.length());
  delay(1000);
  if(Serial.find(">")){
    Serial.print(cmd);
  }else{
    Serial.println("AT+CIPCLOSE");
  }
}
boolean connectWiFi(){
  Serial.println("AT+CWMODE=1");
  delay(2000);
  //Serial.println("AT+CWAUTOCONN=1");
  //Serial.println("AT+CWDHCP=1,1");
  String cmd="AT+CWJAP=\"";
  cmd+=SSID;
  cmd+="\",\"";
  cmd+=PASS;
  cmd+="\"";
  Serial.println(cmd);
  delay(5000);
  if(Serial.find("OK")){
    return true;
  }else{
    return false;
  }
}

void setup()
{
  pinMode(Esp_IO0, OUTPUT);//esp IO0
  digitalWrite(Esp_IO0, HIGH);//esp IO0
  pinMode(Sensor_power, OUTPUT);
  digitalWrite(Sensor_power, LOW);
  pinMode(Step_EN, OUTPUT);
  digitalWrite(Step_EN, LOW); //3.3v on
  pinMode(Esp_power, OUTPUT);//PB Bridge
  digitalWrite(Esp_power, LOW);//PB Bridge
  pinMode(Esp_RST, OUTPUT);//esp reset
  digitalWrite(Esp_RST, HIGH);//esp reset
  delay(2000);
  Serial.begin(38400);
  Serial.println("AT");
  delay(5000);
  if(Serial.find("OK")){
    connectWiFi();
  }
  clearTemp();
  // SSR pin initialization
  digitalWrite(ssrPin, HIGH);
  pinMode(ssrPin, OUTPUT);

  // pb1 pin initialization
  pinMode(pb1Pin, INPUT);

  // Buzzer pin initialization
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pin initialization
  digitalWrite(ledPin, LOW);
  pinMode(ledPin, OUTPUT);

  // Fan pin initialization
//  digitalWrite(fanPin, LOW);
//  pinMode(fanPin, OUTPUT);
  
  // Start-up splash
  digitalWrite(buzzerPin, HIGH);
  
  Serial.println("Arduino");
  Serial.println("PID Reflow");
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(2500);
  

  // Turn off LED
  digitalWrite(ledPin, HIGH);
  
  // Set window size
  windowSize = 2000;      // milliseconds

  // Initialize time keeping variable
  nextCheck = millis();
  
  // Initialize thermocouple reading variable
  nextRead = millis();
}

void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
		input = thermocouple.readCelsius();
		
    // If thermocouple problem detected
		if (isnan(input))
		{
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial 
     /* Serial.print(timerSeconds);
      Serial.print(" ");
      Serial.print(setpoint);
      Serial.print(" ");
      Serial.print(input);
      Serial.print(" ");
      Serial.println(output);*/
    }
    else
    {
      // Turn off LED
      digitalWrite(ledPin, HIGH);
    }

    // Print current system state
    //Serial.println(lcdMsgReflowStatus[reflowState]);
    // Move the cursor to line 2
//    lcd.setCursor(0, 1);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // No thermocouple wire connected
      Serial.println("TC Error!");
    }
    else
    {
      // Print current temperature
      //Serial.print(input);

			// Print degree Celsius symbol
			//Serial.print((uint8_t)0);
      //Serial.println("C ");
      if (reflowState != REFLOW_STATE_IDLE){
        if (timeThingspeak > 9){
        char buffer[10];
        String inputString = dtostrf(input, 4, 1, buffer);
        inputString.trim();
        updateTemp(inputString,(String)lcdMsgReflowStatus[reflowState]+ "-" +(String)timerSeconds);
        timeThingspeak = 0;
      }
      timeThingspeak++;
      //Serial.println(lcdMsgReflowStatus[reflowState]);   
      }
    }
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:
		// If oven temperature is still above room temperature
		if (input >= TEMPERATURE_ROOM)
		{
			reflowState = REFLOW_STATE_TOO_HOT;
//      digitalWrite(fanPin, HIGH);
		}
		else
		{
//			digitalWrite(fanPin, LOW);
			// If switch is pressed to start reflow process
			if (switchStatus == SWITCH)
			{
        // Send header for CSV file
        //Serial.println("Time Setpoint Input Output");
        // Intialize seconds timer for serial debug information
        timerSeconds = 0;
        // Initialize PID control window starting time
        windowStartTime = millis();
        // Ramp up to minimum soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN;
        // Tell the PID to range between 0 and the full window size
        reflowOvenPID.SetOutputLimits(0, windowSize);
        reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
        // Turn the PID on
        reflowOvenPID.SetMode(AUTOMATIC);
        // Proceed to preheat stage
        reflowState = REFLOW_STATE_PREHEAT;
      }
    }
    break;

  case REFLOW_STATE_PREHEAT:
    reflowStatus = REFLOW_STATUS_ON;
    // If minimum soak temperature is achieved       
    if (input >= TEMPERATURE_SOAK_MIN)
    {
      // Chop soaking period into smaller sub-period
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Set less agressive PID parameters for soaking ramp
      reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
      // Ramp up to first section of soaking temperature
      setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   
      // Proceed to soaking state
      reflowState = REFLOW_STATE_SOAK; 
    }
    break;

  case REFLOW_STATE_SOAK:     
    // If micro soak temperature is achieved       
    if (millis() > timerSoak)
    {
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Increment micro setpoint
      setpoint += SOAK_TEMPERATURE_STEP;
      if (setpoint > TEMPERATURE_SOAK_MAX)
      {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_REFLOW_MAX;   
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW; 
      }
    }
    break; 

  case REFLOW_STATE_REFLOW:
    // Adjustment for peak temperature management
    if (input >= (TEMPERATURE_REFLOW_MAX))
    {
//      digitalWrite(fanPin, HIGH);
      
      // buzz to indicate completion
      buzzerPeriod = millis() + 50;
      digitalWrite(buzzerPin, HIGH);

      // Set PID parameters for cooling ramp
      reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);

      // Ramp down to minimum cooling temperature
      setpoint = TEMPERATURE_COOL_MIN;   
      
      // Proceed to cooling state
      reflowState = REFLOW_STATE_COOL; 
    }
    break;   

  case REFLOW_STATE_COOL:
    if (millis() > buzzerPeriod) {
      digitalWrite(buzzerPin, LOW);
    }
    // If minimum cool temperature is achieved       
    if (input <= TEMPERATURE_COOL_MIN) {
//      digitalWrite(fanPin, HIGH);
      buzzerPeriod = millis() + 1000;
      // Turn on buzzer to indicate completion
      digitalWrite(buzzerPin, HIGH);
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;                
      // Proceed to reflow Completion state
      reflowState = REFLOW_STATE_COMPLETE; 
    }         
    break;    

  case REFLOW_STATE_COMPLETE:
    if (millis() > buzzerPeriod) {
      // Turn off buzzer
      digitalWrite(buzzerPin, LOW);
			// Reflow process ended
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;
	
	case REFLOW_STATE_TOO_HOT:
		// If oven temperature drops below room temperature
		if (input < TEMPERATURE_ROOM)
		{
			// Ready to reflow
			reflowState = REFLOW_STATE_IDLE;
		}
		break;
		
  case REFLOW_STATE_ERROR:
    // If thermocouple problem is still present
		if (isnan(input)) {
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR; 
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;    
  }    

  // If switch is pressed
  if (switchStatus == SWITCH)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  } 

  // Simple switch debounce state machine
  switch (debounceState)
  {
  case DEBOUNCE_STATE_IDLE:
    // No valid switch press
    switchStatus = SWITCH_NONE;
    // If switch is pressed
		if (digitalRead(pb1Pin) == 0)	{
				// Intialize debounce counter
				lastDebounceTime = millis();
				// Proceed to check validity of button press
				debounceState = DEBOUNCE_STATE_CHECK;
		}	
    break;

  case DEBOUNCE_STATE_CHECK:
		if (digitalRead(pb1Pin) == 0) {
				// If minimum debounce period is completed
				if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
				{
					// Proceed to wait for button release
					debounceState = DEBOUNCE_STATE_RELEASE;
				}
		}
			// False trigger
		else
			{
				// Reinitialize button debounce state machine
				debounceState = DEBOUNCE_STATE_IDLE; 
			}
    break;

  case DEBOUNCE_STATE_RELEASE:
		if (digitalRead(pb1Pin) == 1) {
      // Valid switch press
      switchStatus = SWITCH;
      // Reinitialize button debounce state machine
      debounceState = DEBOUNCE_STATE_IDLE; 
      clearTemp();
    }
    break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > windowSize)
    { 
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(output > (now - windowStartTime)) digitalWrite(ssrPin, LOW);
    else digitalWrite(ssrPin, HIGH);   
  }
  // Reflow oven process is off, ensure oven is off
  else 
  {
    digitalWrite(ssrPin, HIGH);
  }
}
