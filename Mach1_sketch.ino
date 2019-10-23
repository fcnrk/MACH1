//#include <dht.h>

/**********************************************************Sensors************************************************************/
#define DHT11_PIN                   22                                              // DHT11 pin
// #define MQ7_AOUT_PIN                78                                           // MQ7 CO sensor analog output pin
// #define MQ7_DOUT_PIN                25                                           // MQ7 CO sensor digital output pin
#define MQ135_AOUT_PIN              79                                              // MQ135 aq sensor analog output pin
#define MQ2_AOUT_PIN                80                                              // MQ2 combustible gas sensor analog output pin
#define CALIBARAION_SAMPLE          50                                              // number of samples collected for sensor calibration
#define CALIBRATION_SAMPLE_INTERVAL 500                                             // time interval between each sample during the calibration phase
#define MQ2_RL_VALUE                5                                               // load resistance on MQ2 sensor in kilo ohms
#define MQ135_RL_VALUE              20                                              // load resistance on MQ135 sensor in kilo ohms
#define MQ2_RO_CLEAN_AIR_FACTOR     9.83                                            // MQ2 sensor resistance in clean air. Documentation: https://www.pololu.com/file/0J309/MQ2.pdf
#define MQ135_RO_CLEAN_AIR_FACTOR   3.4                                             // MQ135 sensor resistance in clean air. Documentation: https://www.olimex.com/Products/Components/Sensors/Gas/SNS-MQ135/resources/SNS-MQ135.pdf
#define GAS_LPG                     0                                               // code for LPG (MQ2 sensor)
#define GAS_CO                      1                                               // code for CO (MQ2 sensor)
#define GAS_SMOKE                   2                                               // code for smoke (MQ2 sensor)

/***********************************************************WI-FI*************************************************************/
#define SSID                        "L-Wlan"                                        // Wifi SSID
#define PASS                        "L*G*db13baac4933629de285aba163"                // WiFi Password
#define DEST_HOST                   "test-iothub.logo-paas.com"                     // API site host
#define TIMEOUT                     5000                                            // mS
#define WIFI_RETRY_COUNT            5                                               // wifi connection retry count

/******************************************************Input Devices**********************************************************/
#define WHITE_BUTTON_PIN            23                                              // clock pin for white player
#define BLACK_BUTTON_PIN            24                                              // clock pin for black player
#define CHESS_SWITCH_PIN            25                                              // switch pin for chess mode
#define KEYPAD_R1                   26                                              // Keypad Row1 pin
#define KEYPAD_R2                   27                                              // Keypad Row2 pin
#define KEYPAD_R3                   28                                              // Keypad Row3 pin
#define KEYPAD_R4                   29                                              // Keypad Row4 pin
#define KEYPAD_C1                   30                                              // Keypad Column1 pin
#define KEYPAD_C2                   31                                              // Keypad Column2 pin
#define KEYPAD_C3                   32                                              // Keypad Column3 pin
#define CALIBRATE_FIRST             33                                              // Calibration switch pin

/******************************************************Output Devices*********************************************************/
//<MONOCHROME LCD DISPLAY PINS HERE>

/***********************************************************Globals***********************************************************/
#define CONTINUE                    false
#define HALT                        true
//curve values are extracted from https://www.pololu.com/file/0J309/MQ2.pdf
float LPGCurve[3] = {2.3,0.21,-0.47};               //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
//
float COCurve[3] = {2.3,0.72,-0.34};                //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)                                                     
//                                                    
float SmokeCurve[3] = {2.3,0.53,-0.44};             //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
//                                                    

float CO2Curve[2] = {116.6020682, -2.769034857};     //coefficients taken from: http://davidegironi.blogspot.com/2017/05/mq-gas-sensor-correlation-function.html#.XbALtUYzaHs
float MQ2_Ro = 10;                                  //MQ2_Ro is initialized to 10 kilo ohms
float MQ135_Ro = 20;                                //MQ135_Ro is initialized to 20 kilo ohms


/*******************************************************************************************************************************
* Purpose       : Sends a command to the ESP8266 and wait for acknowledgement
* Parameters    : String command - Command send to the ESP8266
*                 String ack - Expected acknowledgement string
*                 boolean halt_on_fail - Configures if we should halt on a failure
* Returns       : Returns true if ESP8266 successfully runs the command.
*******************************************************************************************************************************/
boolean runCommand(String command, String ack, boolean halt_on_fail)
{
  Serial.println(cmd);
  if (ack == "") // If no ack response specified, skip all available module output by iterating until 3 newlines are encountered
  {
      listenAndFindKeyword("\n");
      listenAndFindKeyword("\n");
      listenAndFindKeyword("\n");
  }                  
  else
    if (!listenAndFindKeyword(ack)) //timeout occurred while waiting for ack
      if (halt_on_fail)
        while(true){};
      else
        return false;
  return true;
}

/*******************************************************************************************************************************
* Purpose       : Listens to the response from the ESP8266 via Serial and finds a specific keyword in the response
* Parameters    : String keyword - the message part to be found within the response
* Returns       : Returns true if the keyword is found. Returns false if timed out
*******************************************************************************************************************************/
boolean listenAndFindKeyword(String keyword)
{
  byte index = 0;
  byte keyword_length = keyword.length();
  
  long deadline = millis() + TIMEOUT;              // Calculate timeout deadline
  while(millis() < deadline)                       // Try until deadline
  {
    if (Serial.available())                        // If characters are available
    {
      char ch = Serial.read();
      if (ch == keyword[index])
      {
        if (++index == keyword_length)
          return true;
      }
    }
  }
  return false;                                    // Timed out
}

/*******************************************************************************************************************************
* Purpose       : Connects to the WiFi
* Parameters    : String ssid - defines the WiFi SSID
*                 String password - defines the WiFi Password
* Returns       : Return TRUE - ESP8266 is connected to the defined WiFi
*                 Return FALSE - ESP8266 failed to connect
*******************************************************************************************************************************/
boolean connectWiFi(string ssid, String password)
{
  String cmd = "AT+CWJAP=\""; cmd += ssid; cmd += "\",\""; cmd += password; cmd += "\"";
  if (runCommand(cmd, "OK", CONTINUE))                                               
    return true;
  else
    return false;
}

/*******************************************************************************************************************************
* Purpose       : Builds an HTTP POST request to send telemetry to the IoT Hub and sends it
* Parameters    : N/A
* Returns       : HTTP POST request as string
*******************************************************************************************************************************/
bool sendTelemetry()
{
    String request = "POST ";
    request += DEST_HOST;
    request += "/<device key etc. here>";
    request += " HTTP/1.1\r\n\r\n";
    
    if (!runCommand("AT+CIPSEND=0,"+String(request.length()), ">", CONTINUE)) //prepare to send data - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPSEND
    {
        runCommand("AT+CIPCLOSE", "", CONTINUE); //close connection - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPCLOSE
        return false;
    }
    
    runCommand(request, "OK", CONTINUE);  // Send the raw HTTP request
    return listenAndFindKeyword("<ACKNOWLEDGEMENT HERE>");
}

/*******************************************************************************************************************************
* Purpose       : Assuming the MQ2 sensor is in clean air, this function calculates the sensor resistance in clean air and then
                  divides it with RO_CLEAN_AIR_FACTOR.
* Parameters    : N/A
* Returns       : Calibrated Ro (sensor resistence in ~1000ppm Hyrogen)
*******************************************************************************************************************************/
float MQ2Calibration()
{
    int i;
    float val=0;
    
    for (i=0; i<CALIBARAION_SAMPLE; i++)
    {
        float cgVal = analogRead(MQ2_AOUT_PIN);
        val += MQ2_RL_VALUE*(1023-cgVal)/cgVal; //Rs sum
        delay(CALIBRATION_SAMPLE_INTERVAL);
    }
    val = val/CALIBARAION_SAMPLE; //Avg Rs
    val = val/MQ2_RO_CLEAN_AIR_FACTOR; //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
    return val; 
}

/*******************************************************************************************************************************
* Purpose       : Assuming the MQ135 sensor is in clean air, this function calculates the sensor resistance in clean air and then
                  divides it with RO_CLEAN_AIR_FACTOR.
* Parameters    : N/A
* Returns       : Calibrated Ro (sensor resistence in ~1000ppm Hyrogen)
*******************************************************************************************************************************/
float MQ135Calibration()
{
    int i;  
    float val=0;
    
    for (i=0; i<CALIBARAION_SAMPLE; i++)
    {
        float cgVal = analogRead(MQ135_AOUT_PIN);
        val += MQ135_RL_VALUE*(1023-cgVal)/cgVal; //Rs sum
        delay(CALIBRATION_SAMPLE_INTERVAL);
    }
    val = val/CALIBARAION_SAMPLE; //Avg Rs
    val = val/MQ135_RO_CLEAN_AIR_FACTOR; //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
    return val; 
}

/*******************************************************************************************************************************
* Purpose       : Calculates the ppm of the target gas (LPG/Smoke)
* Parameters    : int gas_id - LPG/SMOKE
* Returns       : ppm of target gas
*******************************************************************************************************************************/
int MQ2GetGasPPM(int gas_id)
{
    float cgVal = analogRead(MQ2_AOUT_PIN);
    float rs = MQ2_RL_VALUE*(1023-cgVal)/cgVal;
    float rs_ro_ratio = rs / MQ2_Ro;
    if (gas_id == GAS_LPG)
        return (pow(10,( ((log(rs_ro_ratio)-LPGCurve[1])/LPGCurve[2]) + LPGCurve[0])));
    else if (gas_id == GAS_CO)
        return (pow(10,( ((log(rs_ro_ratio)-COCurve[1])/COCurve[2]) + COCurve[0])));
    else if (gas_id == GAS_SMOKE)
        return (pow(10,( ((log(rs_ro_ratio)-SmokeCurve[1])/SmokeCurve[2]) + SmokeCurve[0])));
    
    return 0;
}

/*******************************************************************************************************************************
* Purpose       : Calculates the co2 ppm according to the value read from MQ135 sensor analog pin
* Parameters    : N/A
* Returns       : Co2 ppm
*******************************************************************************************************************************/
int MQ135GetCO2PPM()
{
    float sensorVal = analogRead(MQ135_AOUT_PIN);
    float rs = MQ135_RL_VALUE*(1023-sensorVal)/sensorVal;
    float rs_ro_ratio = rs / MQ135_Ro;
    return CO2Curve[0] * pow(rs / MQ135_Ro, CO2Curve[1]);
}

/*******************************************************************************************************************************
* Purpose       : Arduino bootloader function - Sets up the MCU
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(TIMEOUT);
    
    if (digitalRead(CALIBRATE_FIRST))
    {
        MQ2_Ro = MQ2Calibration();
        MQ135_Ro = MQ135Calibration();
    }
    
    delay(2000);
    runCommand("AT+RST", "ready", HALT); //restart module - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+RST
    runCommand("AT+CWMODE=1", "", HALT); //set to client mode - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CWMODE
    runCommand("AT+CIPMUX=1", "", HALT); //enable multiple connections - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPMUX

    boolean connection_established = false;
    for (int i = 0; i < WIFI_RETRY_COUNT; i++)
    {
        if (connectWiFi(SSID, PASS))
        {
            connection_established = true;
            break;
        }
    }
    
    if(!connection_established)
        while(true){};
    
    delay(5000);
    //establish a connection to the IoTHub here
    String start_iot_connection = "AT+CIPSTART=0,\"TCP\",\""; start_iot_connection += "<IP HERE?>"; start_iot_connection += "\",<PORT HERE>";
    if (!runCommand(start_iot_connection, "OK", CONTINUE)) return;
    delay(2000);
    if (!runCommand("AT+CIPSTATUS", "OK", CONTINUE)) return; //get connection status - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPSTATUS
}

/*******************************************************************************************************************************
* Purpose       : Main method, runs forever on Arduino
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void loop()
{
    String temperature;
    String humidity;
    // int coValue = analogRead(MQ7_AOUT_PIN);
    // int coLimit = digitalRead(MQ7_DOUT_PIN);
    float co_ppm = MQ2GetGasPPM(GAS_CO);
    float co2_ppm = MQ135GetCO2PPM();
    float lpg_ppm = MQ2GetGasPPM(GAS_LPG);
    float smoke_ppm = MQ2GetGasPPM(GAS_SMOKE);
    bool dht11_success = false;
    int dht_chk = DHT.read11(DHT11_PIN);

    switch (dht_chk)
    {
    case DHTLIB_OK:
        temperature = DHT.temperature;
        humidity = DHT.humidity;
        dht11_success = true;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        break;
    case DHTLIB_ERROR_TIMEOUT:
        break;
    default:
        break;
    }


    
}