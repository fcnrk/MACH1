#include <Key.h>
#include <Keypad.h>
#include "DHT.h"
#include <U8g2lib.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

/*********************************************************Constants***********************************************************/
#define CONTINUE false
#define HALT true

/**********************************************************Sensors************************************************************/
// #define MQ7_AOUT_PIN 78              // MQ7 CO sensor analog output pin
// #define MQ7_DOUT_PIN 25              // MQ7 CO sensor digital output pin
#define DHTTYPE DHT11                   // DHT sensor type
#define DHT11_PIN 22                    // DHT11 pin
#define MQ135_AOUT_PIN 79               // MQ135 aq sensor analog output pin
#define MQ2_AOUT_PIN 80                 // MQ2 combustible gas sensor analog output pin
#define CALIBARAION_SAMPLE 50           // number of samples collected for sensor calibration
#define CALIBRATION_SAMPLE_INTERVAL 500 // time interval between each sample during the calibration phase
#define MQ2_RL_VALUE 5                  // load resistance on MQ2 sensor in kilo ohms
#define MQ135_RL_VALUE 20               // load resistance on MQ135 sensor in kilo ohms
#define MQ2_RO_CLEAN_AIR_FACTOR 9.83    // MQ2 sensor resistance in clean air. Documentation: https://www.pololu.com/file/0J309/MQ2.pdf
#define MQ135_RO_CLEAN_AIR_FACTOR 3.4   // MQ135 sensor resistance in clean air. Documentation: https://www.olimex.com/Products/Components/Sensors/Gas/SNS-MQ135/resources/SNS-MQ135.pdf
#define GAS_LPG 0                       // code for LPG (MQ2 sensor)
#define GAS_CO 1                        // code for CO (MQ2 sensor)
#define GAS_SMOKE 2                     // code for smoke (MQ2 sensor)

/***********************************************************WI-FI*************************************************************/
//#define SSID "L-Wlan"                         // Wifi SSID
//#define PASS "L*G*db13baac4933629de285aba163" // WiFi Password
#define SSID "erken"                          // Wifi SSID
#define PASS "1234qqqQ"                       // WiFi Password
#define DEST_HOST "test-iothub.logo-paas.com" // API site host
#define TIMEOUT 5000                          // mS
#define WIFI_RETRY_COUNT 5                    // wifi connection retry count

/******************************************************Input Devices**********************************************************/
#define WHITE_BUTTON_PIN 23 // clock pin for white player
#define BLACK_BUTTON_PIN 24 // clock pin for black player
#define CHESS_SWITCH_PIN 25 // switch pin for chess mode
#define KEYPAD_R1 26        // Keypad Row1 pin
#define KEYPAD_R2 27        // Keypad Row2 pin
#define KEYPAD_R3 28        // Keypad Row3 pin
#define KEYPAD_R4 29        // Keypad Row4 pin
#define KEYPAD_C1 30        // Keypad Column1 pin
#define KEYPAD_C2 31        // Keypad Column2 pin
#define KEYPAD_C3 32        // Keypad Column3 pin
#define CALIBRATE_FIRST 33  // Calibration switch pin

/******************************************************Output Devices*********************************************************/
#define BUZZER_PIN 34 // Buzzer pin
#define LED_PIN 35    // RGB Led
#define DISP_COM1 52  // KS0108 - PIN 7
#define DISP_COM2 50  // KS0108 - PIN 8
#define DISP_COM3 48  // KS0108 - PIN 9
#define DISP_COM4 46  // KS0108 - PIN 10
#define DISP_COM5 44  // KS0108 - PIN 11
#define DISP_COM6 42  // KS0108 - PIN 12
#define DISP_COM7 40  // KS0108 - PIN 13
#define DISP_COM8 38  // KS0108 - PIN 14
#define DISP_CS1 88   // KS0108 - PIN 15
#define DISP_CS2 89   // KS0108 - PIN 16
#define DISP_E 13     // KS0108 - PIN 6
#define DISP_DI 10    // KS0108 - PIN 4
#define DISP_RW 11    // KS0108 - PIN 5

/******************************************************Sensor Globals*********************************************************/
//curve values are extracted from https://www.pololu.com/file/0J309/MQ2.pdf
float LPGCurve[3] = {2.3, 0.21, -0.47}; //two points are taken from the curve.
                                        //with these two points, a line is formed which is "approximately equivalent"
                                        //to the original curve.
                                        //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
//
float COCurve[3] = {2.3, 0.72, -0.34}; //two points are taken from the curve.
                                       //with these two points, a line is formed which is "approximately equivalent"
                                       //to the original curve.
                                       //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
//
float SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
                                          //with these two points, a line is formed which is "approximately equivalent"
                                          //to the original curve.
                                          //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
//

float CO2Curve[2] = {116.6020682, -2.769034857}; //coefficients taken from: http://davidegironi.blogspot.com/2017/05/mq-gas-sensor-correlation-function.html#.XbALtUYzaHs
float MQ2_Ro = 10;                               //MQ2_Ro is initialized to 10 kilo ohms
float MQ135_Ro = 20;                             //MQ135_Ro is initialized to 20 kilo ohms
DHT dht(DHT11_PIN, DHTTYPE);

/*******************************************************Input Globals*********************************************************/
//4x3 membrane keypad
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

byte rowPins[ROWS] = {KEYPAD_R1, KEYPAD_R2, KEYPAD_R3, KEYPAD_R4};
byte colPins[COLS] = {KEYPAD_C1, KEYPAD_C2, KEYPAD_C3};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/******************************************************Output Globals*********************************************************/
//U8GLIB_KS0108_128 u8g(DISP_COM1, DISP_COM2, DISP_COM3, DISP_COM4, DISP_COM5, DISP_COM6, DISP_COM7, DISP_COM8, DISP_E, DISP_CS1, DISP_CS2, DISP_DI, DISP_RW);
//U8GLIB_ST7920_128X64 u8g(DISP_COM1, DISP_COM2, DISP_COM3, DISP_COM4, DISP_COM5, DISP_COM6, DISP_COM7, DISP_COM8, DISP_E, DISP_CS1, DISP_CS2, DISP_DI, DISP_RW);
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ DISP_E /* A4 */ , /* data=*/ DISP_RW /* A2 */, /* CS=*/ DISP_DI /* A3 */, /* reset=*/ U8X8_PIN_NONE);
/******************************************************Time Globals*********************************************************/
const int timeZone = 3;
WiFiUDP Udp;
unsigned int localPort = 8888;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov

/*******************************************************Game Globals**********************************************************/
bool isUsingMenu = false; // whether we are in the time-setting menu
bool isGameOver = false;
unsigned long previousMillis = 0;

class Player
{
public:
    int minutes;          // number of minutes allowed
    String menuText;      // text for the time-setting menu
    String playIndicator; // indicator for when active
    long secondsRun;      // seconds run while the player was active

    void SetMinutes(int inputMinute)
    {
        minutes = inputMinute;
        delay(250);
    }

    int SecondsRemaining()
    {
        // number of seconds remaining on the player's clock
        int minsAllowed = minutes;
        int secondsRunSoFar = secondsRun;
        return minsAllowed * 60 - secondsRunSoFar;
    }

    //sets the players secondsRun value to arduino's seconds run - cumulative seconds run, i.e. the difference
    void UpdateCounter(unsigned long rolloverAccountedMillis, Player p0, Player p1, Player p2)
    {
        secondsRun += rolloverAccountedMillis / 1000 - (p0.secondsRun + p1.secondsRun + p2.secondsRun); //https://www.arduino.cc/reference/en/language/functions/time/millis/
    }
};

Player p0 = {0, "", "  --  ", 0}; // dummy player, active when player clocks are not counting down
Player whitePlayer = {5, "White mins: ", "  WHITE  ", 0};
Player blackPlayer = {5, "Black mins: ", "  BLACK  ", 0};

// set a pointer to the first player to be shown in the time-setting menu
Player *menuPlayer = &whitePlayer;
// set the active player pointer to the player to start with
Player *activePlayer = &p0;

/******************************************************************************************************************************
/                                                          Methods
/******************************************************************************************************************************/

/*******************************************************************************************************************************
* Purpose       : Sends a command to the ESP8266 and wait for acknowledgement
* Parameters    : String command - Command send to the ESP8266
*                 String ack - Expected acknowledgement string
*                 boolean halt_on_fail - Configures if we should halt on a failure
* Returns       : Returns true if ESP8266 successfully runs the command.
*******************************************************************************************************************************/
boolean runCommand(String command, String ack, boolean halt_on_fail)
{
    Serial.println(command);
    if (ack == "") // If no ack response specified, skip all available module output by iterating until 3 newlines are encountered
    {
        listenAndFindKeyword("\n");
        listenAndFindKeyword("\n");
        listenAndFindKeyword("\n");
    }
    else if (!listenAndFindKeyword(ack)) //timeout occurred while waiting for ack
        updateErrorScreen("ESP8266 cmd failed ack...");
    if (halt_on_fail)
    {
        while (true)
        {
        };
    }
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
    long deadline = millis() + TIMEOUT; // Calculate timeout deadline. No need for rollover check here, that's why millis() is used directly.
    while (millis() < deadline)         // Try until deadline
    {
        if (Serial.available()) // If characters are available
        {
            char ch = Serial.read();
            if (ch == keyword[index])
            {
                if (++index == keyword_length)
                    return true;
            }
        }
    }
    return false; // Timed out
}

/*******************************************************************************************************************************
* Purpose       : Connects to the WiFi
* Parameters    : String ssid - defines the WiFi SSID
*                 String password - defines the WiFi Password
* Returns       : Return TRUE - ESP8266 is connected to the defined WiFi
*                 Return FALSE - ESP8266 failed to connect
*******************************************************************************************************************************/
boolean connectWiFi(String ssid, String password)
{
    
    String cmd = "AT+CWJAP=\"";
    cmd += ssid;
    cmd += "\",\"";
    cmd += password;
    cmd += "\"";
    if (runCommand(cmd, "OK", CONTINUE))
        return true;
    else
        return false;
    
    /* Old ESP8266 connect commands
    int remainingRetry = WIFI_RETRY_COUNT;
    WiFi.begin(ssid, password);

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        remainingRetry--;
        if (remainingRetry <= 0)
        {
            Serial.println();
            Serial.print("Connection timeout");
            return false;
        }
    }
    Serial.println();
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
    return true;
    */
}

/*******************************************************************************************************************************
* Purpose       : Builds an HTTP POST request to send telemetry to the IoT Hub and sends it
* Parameters    : float temperature, float humidity, float co_ppm, float co2_ppm, float lpg_ppm, float smoke_ppm
* Returns       : HTTP POST request as string
*******************************************************************************************************************************/
bool sendTelemetry(float temperature, float humidity, float co_ppm, float co2_ppm, float lpg_ppm, float smoke_ppm)
{
    String request = "POST ";
    request += DEST_HOST;
    request += "/<device key etc. here>"; //TODO - build request here
    request += " HTTP/1.1\r\n\r\n";

    if (!runCommand("AT+CIPSEND=0," + String(request.length()), ">", CONTINUE)) //prepare to send data - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPSEND
    {
        //there is an error, close connection.
        runCommand("AT+CIPCLOSE", "", CONTINUE); //close connection - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPCLOSE
        return false;
    }

    runCommand(request, "OK", CONTINUE);                   // Send the raw HTTP request
    return listenAndFindKeyword("<ACKNOWLEDGEMENT HERE>"); //TODO - look for ack here
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
    float val = 0;

    for (i = 0; i < CALIBARAION_SAMPLE; i++)
    {
        float cgVal = analogRead(MQ2_AOUT_PIN);
        val += MQ2_RL_VALUE * (1023 - cgVal) / cgVal; //Rs sum
        delay(CALIBRATION_SAMPLE_INTERVAL);
    }
    val = val / CALIBARAION_SAMPLE;      //Avg Rs
    val = val / MQ2_RO_CLEAN_AIR_FACTOR; //divided by RO_CLEAN_AIR_FACTOR yields the Ro
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
    float val = 0;

    for (i = 0; i < CALIBARAION_SAMPLE; i++)
    {
        float cgVal = analogRead(MQ135_AOUT_PIN);
        val += MQ135_RL_VALUE * (1023 - cgVal) / cgVal; //Rs sum
        delay(CALIBRATION_SAMPLE_INTERVAL);
    }
    val = val / CALIBARAION_SAMPLE;        //Avg Rs
    val = val / MQ135_RO_CLEAN_AIR_FACTOR; //divided by RO_CLEAN_AIR_FACTOR yields the Ro
    return val;
}

/*******************************************************************************************************************************
* Purpose       : Calculates the ppm of the target gas (LPG/Smoke)
* Parameters    : int gas_id - LPG/SMOKE/CO
* Returns       : ppm of target gas
*******************************************************************************************************************************/
int MQ2GetGasPPM(int gas_id)
{
    float cgVal = analogRead(MQ2_AOUT_PIN);
    float rs = MQ2_RL_VALUE * (1023 - cgVal) / cgVal;
    float rs_ro_ratio = rs / MQ2_Ro;
    if (gas_id == GAS_LPG)
        return (pow(10, (((log(rs_ro_ratio) - LPGCurve[1]) / LPGCurve[2]) + LPGCurve[0])));
    else if (gas_id == GAS_CO)
        return (pow(10, (((log(rs_ro_ratio) - COCurve[1]) / COCurve[2]) + COCurve[0])));
    else if (gas_id == GAS_SMOKE)
        return (pow(10, (((log(rs_ro_ratio) - SmokeCurve[1]) / SmokeCurve[2]) + SmokeCurve[0])));

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
    float rs = MQ135_RL_VALUE * (1023 - sensorVal) / sensorVal;
    float rs_ro_ratio = rs / MQ135_Ro;
    return CO2Curve[0] * pow(rs / MQ135_Ro, CO2Curve[1]);
}

/*******************************************************************************************************************************
* Purpose       : Updates the clock set screen - Chess clock setter
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void updateMenuScreen()
{
    int num = 0;
    char key = keypad.getKey();
    switch (key)
    {
    case NO_KEY:
        break;
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
        num = num * 10 + (key - '0');
        if (num >= 100)
            num = num % 100;
        break;
    case '*':
        menuPlayer->SetMinutes(num);
        switchMenuPlayer();
        break;
    case '#':
        isUsingMenu = false;
        break;
    }
    char whiteMin[2];
    char blackMin[2];
    sprintf(whiteMin, "%03d", whitePlayer.minutes);
    sprintf(blackMin, "%03d", blackPlayer.minutes);
    // u8g.firstPage();
    // do
    // {
    //     u8g.drawLine(64, 0, 64, 64);
    //     u8g.setFont(u8g_font_gdb30);
    //     u8g.drawStr(10, 48, whiteMin);
    //     u8g.drawStr(74, 48, blackMin);
    //     u8g.setFont(u8g_font_gdb12);
    //     u8g.drawStr(20, 16, "set white min");
    //     u8g.drawStr(82, 16, "set black min");

    // } while (u8g.nextPage());
}

/*******************************************************************************************************************************
* Purpose       : Switch menu player
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void switchMenuPlayer()
{
    if (menuPlayer == &whitePlayer)
        menuPlayer = &blackPlayer;
    else if (menuPlayer == &blackPlayer)
        menuPlayer = &whitePlayer;
}

/*******************************************************************************************************************************
* Purpose       : Update the game screen - Chess clock
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void updateGameScreen()
{
    if (activePlayer = &whitePlayer)
    {
        if (digitalRead(WHITE_BUTTON_PIN))
            activePlayer = &blackPlayer;
        else
            updateTimer();
    }
    else if (activePlayer = &blackPlayer)
    {
        if (digitalRead(BLACK_BUTTON_PIN))
            activePlayer = &whitePlayer;
        else
            updateTimer();
    }
}

/*******************************************************************************************************************************
* Purpose       : Update the player timer and the display
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void updateTimer()
{
    if (isGameOver)
    {
        updateGameOverScreen();
    }
    else
    {
        //game over but we don't know yet =)
        if (activePlayer->SecondsRemaining() <= 0)
        {
            isGameOver = true;
            //TODO - buzz
        }

        //game continues
        //update timer display here
        char whiteRemainingMin[2];
        char whiteRemainingSec[2];
        char blackRemainingMin[2];
        char blackRemainingSec[2];
        sprintf(whiteRemainingMin, "%03d", whitePlayer.SecondsRemaining() / 60);
        sprintf(whiteRemainingSec, "%03d", whitePlayer.SecondsRemaining() % 60);
        sprintf(blackRemainingMin, "%03d", blackPlayer.SecondsRemaining() / 60);
        sprintf(blackRemainingSec, "%03d", blackPlayer.SecondsRemaining() % 60);
        char whiteTime[5];
        char blackTime[5];
        strcpy(whiteTime, whiteRemainingMin);
        strcat(whiteTime, ":");
        strcat(whiteTime, whiteRemainingSec);
        strcpy(blackTime, blackRemainingMin);
        strcat(blackTime, ":");
        strcat(blackTime, blackRemainingSec);
        // u8g.firstPage();
        // do
        // {
        //     u8g.drawLine(64, 0, 64, 64);
        //     u8g.setFont(u8g_font_gdb30);
        //     u8g.drawStr(10, 48, whiteTime);
        //     u8g.drawStr(74, 48, blackTime);
        //     u8g.setFont(u8g_font_gdb12);
        //     u8g.drawStr(28, 16, "white");
        //     u8g.drawStr(90, 16, "black");

        // } while (u8g.nextPage());
    }
}

/*******************************************************************************************************************************
* Purpose       : Update the measurements display
* Parameters    : float temperature, float humidity, float co_ppm, float co2_ppm, float lpg_ppm, float smoke_ppm
* Returns       : N/A
*******************************************************************************************************************************/
void updateMeasurementsScreen(float temperature, float humidity, float co_ppm, float co2_ppm, float lpg_ppm, float smoke_ppm)
{
    //similar to -> https://create.arduino.cc/projecthub/mircemk/analog-digital-clock-and-thermometer-on-128x64-lcd-09ed02
      
    char coStr[8];
    char co2Str[8];
    char lpgStr[8];
    char smkStr[8];
    char tempStr[8];
    char humidStr[8];
    sprintf(coStr, "%.1f", co_ppm);
    sprintf(co2Str, "%.1f", co2_ppm);
    sprintf(lpgStr, "%.1f", lpg_ppm);
    sprintf(smkStr, "%.1f", smoke_ppm);
    sprintf(tempStr, "%.1f", temperature);
    sprintf(humidStr, "%.1f", humidity);
    delay(50);
    u8g2.clearBuffer();
    u8g2.drawRFrame(0, 0, 128, 64, 3);
    u8g2.drawRFrame(68, 4, 55, 56, 2);
    u8g2.drawRBox(3, 4, 62, 21,2);
    u8g2.setColorIndex(0);
    u8g2.setFont(u8g2_font_fub17_tf);
    u8g2.drawStr(29,21,":");
    //if (hour() < 10)
        //{
            u8g2.drawStr(3,23,"0");
          //  posicaoh = 16;
        //}
        //else posicaoh = 3;
        u8g2.setCursor(16, 23);
        u8g2.print(7);
        //Acerta a posicao do digito caso o minuto
        //seja menor do que 10
        //if (minute() < 10)
        // {
        //     u8g.drawStr(38,23,"0");
        //     posicao = 51;
        // }
        //else posicao = 38;
        u8g2.setCursor(38 ,23);
        u8g2.print(16);
        u8g2.setColorIndex(1);
    //measurements
    u8g2.setFont(u8g2_font_5x8_tf);
    // u8g2.setCursor(94,15);
    // u8g2.print(coStr);
    u8g2.drawStr(94, 15, coStr);
    u8g2.drawStr(94, 42, lpgStr);
    u8g2.drawStr(94, 55, smkStr);
    u8g2.drawStr(94, 28, co2Str);
    u8g2.drawStr(73, 15, "CO :");
    u8g2.drawStr(73, 28, "CO2:");
    u8g2.drawStr(73, 42, "LPG:");
    u8g2.drawStr(73, 55, "SMK:");
    
    u8g2.setFont(u8g2_font_t0_16b_tn);
    u8g2.drawStr(12, 38, tempStr); 
    u8g2.drawCircle(47, 28, 2);
    
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(18, 57, humidStr);
    u8g2.drawStr(9, 57, "%");
    u8g2.drawStr(53, 38, "C");
    u8g2.sendBuffer();
}

/*******************************************************************************************************************************
* Purpose       : Update the calibration waiting display
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void updateCalibrationScreen()
{
    //TODO - display calibration screen
}

/*******************************************************************************************************************************
* Purpose       : Update the game over display
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void updateGameOverScreen()
{
    //display winner
    char winner[11];
    if (whitePlayer.SecondsRemaining() <= 0)
        strcpy(winner, "black");
    if (blackPlayer.SecondsRemaining() <= 0)
        strcpy(winner, "white");
    strcat(winner, " wins!");
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_timR14_tr);
    u8g2.drawStr(18,36,winner);
    u8g2.sendBuffer();
}

/*******************************************************************************************************************************
* Purpose       : Update the display to show an error message
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void updateErrorScreen(char *errorMessage)
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.drawStr(4,10,errorMessage);
    u8g2.sendBuffer();
}

/*******************************************************************************************************************************
* Purpose       : Resets the player objects
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void resetGame()
{
    p0.secondsRun += (whitePlayer.secondsRun + blackPlayer.secondsRun); //required for cumulative calculations
    whitePlayer.secondsRun = 0;
    blackPlayer.secondsRun = 0;
    activePlayer = &p0;
    isGameOver = false;
    isUsingMenu = true;
}

/*******************************************************************************************************************************
* Purpose       : Retireves the time using NTP
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
time_t getNtpTime()
{
    while (Udp.parsePacket() > 0)
        ; // discard any previously received packets
    Serial.println("Transmit NTP Request");
    sendNTPpacket(timeServer);
    uint beginWait = millis();
    while (millis() - beginWait < 1500)
    {
        int size = Udp.parsePacket();
        if (size >= NTP_PACKET_SIZE)
        {
            Serial.println("Receive NTP Response");
            Udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
            unsigned long secsSince1900;
            // convert four bytes starting at location 40 to a long integer
            secsSince1900 = (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
        }
    }
    Serial.println("No NTP Response :-(");
    return 0; // return 0 if unable to get the time
}

/*******************************************************************************************************************************
* Purpose       : Send an NTP request over Udp to the time server at the given address
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
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
    // Serial.print("Start...");
    // Serial.println();
    /* TEST
    if (digitalRead(CALIBRATE_FIRST))
    {
        updateCalibrationScreen();
        MQ2_Ro = MQ2Calibration();
        MQ135_Ro = MQ135Calibration();
    }*/
    u8g2.begin();
    
    //delay(2000);
    /*Old ESP8266 reset commands
    runCommand("AT+RST", "ready", HALT); //restart module - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+RST
    runCommand("AT+CWMODE=1", "", HALT); //set to client mode - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CWMODE
    runCommand("AT+CIPMUX=1", "", HALT); //enable multiple connections - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPMUX
    

    boolean connection_established = false;
    for (int i = 0; i < WIFI_RETRY_COUNT; i++)
    {
        if (connectWiFi(SSID, PASS))
        {
            connection_established = true;
            updateErrorScreen("timeout...");
            break;
        }
    }

    if (!connection_established)
        updateErrorScreen("Connection failed to wlan...");
    else
        updateErrorScreen("Connected to wifi!");

    delay(5000);*/
    //Udp.begin(localPort);
    //setSyncProvider(getNtpTime);
    
    
    /* TEST
    //establish a connection to the IoTHub here
    String start_iot_connection = "AT+CIPSTART=0,\"TCP\",\"";
    start_iot_connection += "<IP HERE?>";
    start_iot_connection += "\",<PORT HERE>";
    if (!runCommand(start_iot_connection, "OK", CONTINUE))
        return;
    delay(2000);
    if (!runCommand("AT+CIPSTATUS", "OK", CONTINUE))
        return; //get connection status - https://room-15.github.io/blog/2015/03/26/esp8266-at-command-reference/#AT+CIPSTATUS
        */
}

/*******************************************************************************************************************************
* Purpose       : Main method, runs forever on Arduino
* Parameters    : N/A
* Returns       : N/A
*******************************************************************************************************************************/
void loop()
{
    /* TEST
    unsigned long currentMillis = millis();
    if (digitalRead(CHESS_SWITCH_PIN))
    {
        activePlayer->UpdateCounter(currentMillis - previousMillis, p0, whitePlayer, blackPlayer); //update secondsRun of the active player
        if (isUsingMenu)
            updateMenuScreen();
        else
            updateGameScreen();
    }
    else
        p0.UpdateCounter(currentMillis - previousMillis, p0, whitePlayer, blackPlayer); //if the chess switch is off, account for the p0

    previousMillis = currentMillis; //for millis() rollover
    */
     updateMeasurementsScreen(32, 41, 112, 106, 12, 33);
     delay(1000);
    
    /* TEST
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    // int coValue = analogRead(MQ7_AOUT_PIN);
    // int coLimit = digitalRead(MQ7_DOUT_PIN);
    float co_ppm = MQ2GetGasPPM(GAS_CO);
    float co2_ppm = MQ135GetCO2PPM();
    float lpg_ppm = MQ2GetGasPPM(GAS_LPG);
    float smoke_ppm = MQ2GetGasPPM(GAS_SMOKE);

    if (!sendTelemetry(temperature, humidity, co_ppm, co2_ppm, lpg_ppm, smoke_ppm))
        updateErrorScreen("Telemetry send failed...");

    if (!digitalRead(CHESS_SWITCH_PIN))
        updateMeasurementsScreen(temperature, humidity, co_ppm, co2_ppm, lpg_ppm, smoke_ppm);
        */
}