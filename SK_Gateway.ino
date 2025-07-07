/**********************  Read Me  *************************
  Step1 : Bluetooth through parametr Set Loop foe 3 min in Setup.
  Parameters Are : 1)WiFi Credentials.
               2)Set network connectivity:GSM,NBIOT,CATM1,GSM+NBIOT,GSM+CATM1,GSM+NBIOT+CATM1.
               3)SET PROTOCOL HTTP=1 or MQTT=0.
               4)Enable BLE.
               5)SET RTC.
               6)Enter Device Sleep Time.
               7)Enable Modbus:Enter modbus slave data.
  Step2 : As per the network select and Protocol select data can be received from Ble or Modbus and send to the server.
  Step3 : Asset Data received from ble which is stored in Spiffs memory.
  Step4 : If any network is connected then Data store in SD Card also if nework c onnected and data not able to server then also Data will store in SD Card.
  Step5 : If network get connected or data is able to send to server then current data send to sever and also Stored data from SD card send.
*********************************************************/

#include <SoftwareSerial.h>
#include<stdio.h>
#include<string.h>                  //string lib
#include "MAX17048.h"               //power optimization, giving battery life details           
#include"bq.h"                      //charging ic
#include "soc/soc.h"                //websocket
#include "soc/rtc_cntl_reg.h"
#include "esp_attr.h"
#include<stdlib.h>
#include<ArduinoJson.h>
#include<AWS_IOT.h>
#include<math.h>
extern MAX17048 pwr_mgmt;
extern String wtchdog_timer_f;

// 1 is battery attach , 0 means battery is not attached
extern byte bat_attach, power_wifi_disable, power_ble_disable;
extern String wtchdog_timer_f , dpm, reg8_0, reg8_1, power, port, bat_charg, bat_a, bat_temp_f,  bit_5_4_f, bat_status, bat_vtg_f;
extern byte dump, cnt, temp1, temp, temp2, temp3, temp4, temp5, reg_data8, reg_data9, batfet, bat_vtg;
/**************************** SD Card ************************************/
#include "SD.h"
#include "SPI.h"
char SD_File[] = "/000.txt";
char Read_pointer[] = "/Read_pointer.txt";
char Write_pointer[] = "/Write_pointer.txt";
char gID[] = "/gID.txt";
int SD_File_No_Write = 0, SD_File_No_Read = 0, SD_Read_enable = 0;

/**************************** O'LED Display ************************************/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/**************************** ESP RTC (Time Stamp) ************************************/
#include "time.h"
const char* ntpServer = "pool.ntp.org";

const long  gmtOffset_sec = 0;
const int   Network_daylightOffset_sec = 0; // 19800


int SEC = 0, MIN = 0, HR = 0, DAY = 0, MONTH = 0, YEAR = 0, w = 0, r = 0, loop_count = 0, local_time_flag = 0, mannual_rtc_flag = 0, Network_set_time_flag = 0, No_of_Days_in_Month = 0, GMT_HR = 5, GMT_MIN = 30;;
int hr_cel = 0, min_cel = 0, sec_cel = 0, day_cel = 0, month_cel = 0, year_cel = 0;
String cellular_rcv_time = "";
char Sec[10], Min[10], Hr[10], Day[10], Month[10], Year[10];
char time_stamp[40];
String  sec = "";
String  minit = "";
String  hr = "";
String  daY = "";
String  months = "";
String  years = "";
String Time_stamp = "";
int com_sec = 0, com_min = 0, com_hr = 0, wifi_try = 0, wifi_connected = 0;
struct tm timeinfo;
unsigned long total_tags_scan_time = 0, tags_scan_time = 0;
int RTC_TIME_RESET = 1; //in HR

/************************* SPIFFS variables **************************/
#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true
//void write_in_file(String u_data, int data_len);
char Parameter_set_File[] = "/Credentials.txt";
char Asset_File[] = "/Asset.txt";
char MODBUS_FILE[] = "/Modbus.txt";
char Network_set_time[] = "/Network_set_time_flag.txt";
char Mannual_rtc_enable_FILE[] = "/Mannual_rtc_flag.txt";
char ServerAddress[] = "/ServerAddress.txt";
char scan_time_file[] = "/scan_time_file.txt";
char TIME_STAMP[] = "/Time_Stamp.txt";
char WiFi_enable_file[] = "/WiFi_enable.txt";

int write_position = 1, read_position = 1, read_pointer = 0;
int fs_size = 0, inc = 1;
String read_g_str = "";
String read_g_str_1 = "", Data_buffer = "";
String SD_String_receive = "";
String slash = "/", quama = ",", ssid_M, password_M, arr_M, GMT_Time_M, ble_enable_M, config_device_M, HTTP_MQTT_M, add_mry_data, mry_data, Modbus_add_mry_data, modbus_enable_M, time_stamp_M, TIME_TO_SLEEP_M, bandmask_M; // Store Data in memory variables
int spiff_z, spiff_a, spiff_b, spiff_c, spiff_d, spiff_e, spiff_f, spiff_g, spiff_h, spiff_i, spiff_j, count; //  Split data from memory int variables
char time1[15], time2[15], time3[15], time4[15], time5[15], time6[15], time7[30], time8[15], time9[15], time10[15], GMT_Time_arr[15], GMT_MIN_arr[15], GMT_HR_arr[15];         // Split data from memory char Array variables
//char b[150];
int p, q , z, n = 0, end_flag = 0;                                   // int variables used in data Segrigation of asset tags.
char data1[150];
char bands[20];
char Band_b[4];
int Band_temp = 0, Band_temp2 = 0, Band_temp1 = 0, band_flag = 0, Band_j = 0, band_c = 0, bitmask = 0, space_flag = 0;

/*******************  Modbus Variables  ***I*************/
#include <ModbusRTU.h>
ModbusRTU mb;
#include <SoftwareSerial.h>
int DE_RE = 15; // for direction
int rx = 5;
int tx = 18;
SoftwareSerial S(rx, tx);//D6/D7  (RX , TX)
//uint16_t Mread[2];
uint16_t Holding_res_buf[100];
uint16_t Input_res_buf[00];
bool coils[20];
bool input_sts[20];
String P_coils = "", P_input_sts = "";
int mod_data_collection_flag = 0;
uint16_t Mread = 0;
int i = 0, j = 0, k = 0, m = 0, slave_counter = 0, mod_collection_flag = 0, mod = 0;
bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
  mod_collection_flag = 1;
  return true;
}
// used in decTohex fun
int  V1, V2, V3, V4, Modbus_count, exit_flag = 0;
char D1[15], D2[15], D3[15], D4[15];
char Slave_ID[4], Fun_Code[4], Reg_Addr[4];
int SlaveID[50], FunCode[50], RegAddr[50];
String Slave_ID_M = "", FunCode_M = "", RegAddr_M = "", App_command = "";
int  funCode = 0, slaveId = 0, regAddr = 0;
volatile int slave_no = 0;
String modbus_slave_data = "", ToTal_mod_slave_data = "", single_slave = "";
char mod_send_buffer[500];
int x = 0, y = 0, mod_entry_flag = 0;
/********************* ble param set variables and call back fun *********************/
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <HTTPClient.h>
// All below variables Are used for Bt Through configuration
BLEServer* pServer = NULL;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
int Ble_Available = 0;
String RCV_DATA;
char Recieved[100];
int i_2, j_2, k_2, l_2 = 0, Fragment_flag = 0, value_flag1 = 0, value_flag2 = 0, value_flag3 = 0, Mem_Write_count = 0;
char command1[100];
String command, command2;
char value[100];
char Bt_recieve[100];
char arr[5];
String connectivity = "";
String connectivity_type = "A", ssid = "prasad", password = "123456789";
int connect_type = 0, security_enable = 0, ble_connect_count = 0;
volatile int j_1 = 0, j_mod = 0, i_1 = 0, cellular_connectivity_enable = 0;

int Alert_count, Try = 0, valid_password_flag = 0, c_1 = 0, c_2 = 0, c_3 = 0, mem_write_flag = 0, credential_enable_flag = 0, flag_enable = 0, loop_flag = 0, flag_enable2 = 0, ble_enable = 1, WiFi_enable = 0, modbus_enable = 0, mod_flag_enable = 0;
unsigned long ble_data_time = 120000; //ble app through parameter set time
unsigned long Currentmillis = 0, Previoustmillis = 0, Startmillis_Password = 0;
char Rcv_Password[50];
String Password = "123456";
#define PASSKEY 654321

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      security_enable++;
      //      Serial.print("security_enable= ");
      //      Serial.println(security_enable);
      delay(100);
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      delay(100);
    }
};

class SecurityCallback : public BLESecurityCallbacks {

    uint32_t onPassKeyRequest() {
      return 000000;
    }

    void onPassKeyNotify(uint32_t pass_key) {}

    bool onConfirmPIN(uint32_t pass_key) {
      vTaskDelay(2000);

      return true;
    }

    bool onSecurityRequest() {
      return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
      if (cmpl.success) {
        //Serial.println("   - SecurityCallback - Authentication Success");
        //        BLEServer *pServer = BLEDevice::createServer();
        //        pServer->setCallbacks(new MyServerCallbacks());
        //        BLEService *pService = pServer->createService(SERVICE_UUID);
        //        security_enable++;
        //        Serial.print("security_enable= ");
        //        Serial.println(security_enable);

      } else {
        Serial.println("   - SecurityCallback - Authentication Failure*");
        pServer->removePeerDevice(pServer->getConnId(), true);
      }
      BLEDevice::startAdvertising();
    }
};


class MyCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
      char txString[30];
      dtostrf(txValue, 1, 2, txString);
      pCharacteristic->setValue(txString);
    }

    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        for (int i = 0, j = 0; i < value.length(); i++, j++)
          Recieved[j] = value[i];
        RCV_DATA = Recieved;
        Serial.println(RCV_DATA);
        //  Serial.println();
        if (RCV_DATA != "\0")
        {
          Ble_Available = 1;
        }
      }
    }
};

void ble_init_param_set_fun()
{
  BLEDevice::init("ETV1_8"); // Give it a name
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("WELCOME ");
  bleSecurity();
}

void bleSecurity() {
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = PASSKEY;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}
/************************* sleep mode variables **************************/
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
int TIME_TO_SLEEP = 15;        /* Time ESP32 will go to sleep (in seconds) */
int wakeup_timer = 0;
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); wakeup_timer = 1; mannual_rtc_flag = 1; break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); wakeup_timer = 0; mannual_rtc_flag = 0; break;
  }
}

/********************** WIFI Variables ***************************/
String WiFi_ssid = "";
String WiFi_password = "";
//const char* serverName = "http://34.231.193.123:8082/asset/tracking/get_tracking_details_of_tag_bulk"; // server name for http server through Wifi
//const char* serverName =   "http://103.235.105.138:8082/asset/tracking/get_tracking_details_of_tag_bulk"; // server name for http server through Wifi
String serverName;
String apnName;
int WiFi_try = 0;
String gmacAddress;
String gId;

/**************************** Publish/Subscribe Mqtt Variables ***************/
//char HOST_ADDRESS[] = "a1udxktg9dbtpj-ats.iot.ap-south-1.amazonaws.com";  //  Host Address for MQTT through Wifi
//char CLIENT_ID[] = "MQTT_FX_Client";                                      //  Client ID for MQTT through Wifi
//char TOPIC_NAME[] = "$aws/things/embel-iot/shadow";                       //  Topic name for MQTT through Wifi

String HOST_ADDRESS;                                                        //  Host Address for MQTT through Wifi
String CLIENT_ID;                                                           //  Client ID for MQTT through Wifi
String TOPIC_NAME;                                                          //  Topic name for MQTT through Wifi
char hostAddr[100];
char clientID[100];
char topic[100];

int status = WL_IDLE_STATUS; //wifi status
int aws_status = 0;
int MQTT_connect_flag = 0;
//-------------------------------------------------------------------
// How many times we should attempt to connect to AWS
//-------------------------------------------------------------------
#define AWS_MAX_RECONNECT_TRIES 5
char payload[500]; //aws cloud data
//String payload="";

String a = "[{\"gSrNo\":\"G123\",\"gMacId\":\"c4:4f:33:15:c9:21\",\"tMacId\":\"e6:76:52:90:4a:e1\",\"bSN\":\"20001d24\"},{\"gSrNo\":\"G125\",\"gMacId\":\"c4:4f:33:15:c9:21\",\"tMacId\":\"f6:84:e9:b3:30:f8\",\"bSN\":\"00003a24\"}]";
char b[500];
String c = "hello";
//String mry_data, d;
//int end_flag=0;
// variables for MQTT throgh Cellular
String Msg = "{\"company\":\"Embel\",\"Date\":\"30/6/2022\",\"Name\":\"Prasad\",\"Place\":\"pune\"}";
String Topic_name = "$aws/things/embel-iot/shadow";                        //  Topic name for MQTT through Cellular
String Pub_Topic_len = "";
String Pub_Topic_name = "";
String RL_data = "";
String Pub_RL = "";
String Pub_Msg = "";
String Data2 = "";
int Network_connected_flag = 0, current_data_flag = 0;
char receivedChars[100];
/****************** gateway hardware init ********************/
#define GSM Serial1
String resp_st = "";
int  HTTP_MQTT = 1;
#define vp        36
#define cap_p     37
#define cap_n     38
#define vn        39
#define bat_init  34
#define adc_1_ch  35
#define pwr_uc  25   // pinMode(25, OUTPUT); //POWER ON
#define dtr_uc  26   //   pinMode(26, OUTPUT);//DTR UC
#define rst_uc  27   //  pinMode(27, OUTPUT); //RESET
#define rts_uc  14   //  pinMode(14, OUTPUT); //RTS
///////////////////////
#define boot_gpio  0
///////////////////////////////////

#define en_ic     23
#define gpio_19   19
///////////////////////////////////////
#define cts0  5     //input pin
#define rts0  18    //output pin
/********************* Ble scanner function with variables  *********************/
int scanTime = 30;  //In seconds
BLEScan* pBLEScan;
String sending_data = "", assetTagName = "", tMacId = "", bSN = "", gMacId = "", gSrNo = "", Device_name = "", current_time = "", data_arr2 = "", data_arr3 = "", wifi_mac = "", storing_data = "", batStat = "";
int    counter_of_txt_creation = 1, counter_for_scanning_data = 0, counter_for_send_data_to_http = 1, BatStat = 0;
int    config_device = 0;
int    hexToDec(String hexString);
char send_buffer[500];
float tagDist = 0, Tx = -62.7, Rssi = 0;
int N = 2;
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      //  Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      if (advertisedDevice.haveRSSI()) {
        Rssi = (int)advertisedDevice.getRSSI();
      }
      else Serial.printf("\n");

      String man_data = advertisedDevice.toString().c_str();
      tags_scan_time = millis();
      // depending upon id this need to be switch..
      tMacId  = man_data.substring( (man_data.indexOf("Address: ") + 9) , (man_data.indexOf("Address: ") + 26) );
      assetTagName  = man_data.substring( (man_data.indexOf("manufacturer data:") + 27) , (man_data.indexOf("manufacturer data:") + 37) );
      batStat = man_data.substring( (man_data.indexOf("manufacturer data:") + 47) , (man_data.indexOf("manufacturer data:") + 49) );
      bSN = man_data.substring( (man_data.indexOf("manufacturer data:") + 53) , (man_data.indexOf("manufacturer data:") + 61) );

      BatStat = hexToDec(batStat);
      char ch = 0, char_hex = 0;
      String tp = "", st1 = assetTagName.substring(0, 2);
      int int_1 = hexToDec(st1);
      ch = (byte)int_1;
      tp = tp + ch;
      int_1 = 0;
      ch = 0;
      String st2 = assetTagName.substring(2, 4);
      int_1 = hexToDec(st2);
      ch = (byte)int_1;
      tp = tp + ch;
      int_1 = 0;
      ch = 0;
      String st3 = assetTagName.substring(4, 6);
      int_1 = hexToDec(st3);
      ch = (byte)int_1;
      tp = tp + ch;
      int_1 = 0;
      ch = 0;
      String st4 = assetTagName.substring(6, 8);
      int_1 = hexToDec(st4);
      ch = (byte)int_1;
      tp = tp + ch;
      int_1 = 0;
      ch = 0;
      String st5 = assetTagName.substring(8, 10);
      int_1 = hexToDec(st5);
      ch = (byte)int_1;
      tp = tp + ch;
      int_1 = 0;
      ch = 0;
      assetTagName = tp;
      if (assetTagName[0] == 'E' && assetTagName[1] == 'M' && assetTagName[2] == 'B')
      {
        //   Serial.print("man_data = ");
        //   Serial.println(man_data);
        ;
      }
      else
      {
        return;
      }

      counter_for_scanning_data++;
      Serial.print("counter_for_scanning_data=");
      Serial.println(counter_for_scanning_data);

      printLocalTime();
      //     2023-04-18 21:32:55
      mannual_Timestamp(Time_stamp);
      ADD_GMT_Time(HR, MIN, SEC, DAY, MONTH, YEAR);                      //  convert rcv network time to gmt time as per the regions (EX. 5:30)
      //    Time_stamp = (String)YEAR + "-" + (String)MONTH + "-" + (String)DAY + " " +  (String)HR + ":" + (String)MIN + ":" + (String)SEC;
      char s_time[30];
      sprintf(s_time, "%d-%02d-%02d %02d:%02d:%02d", YEAR, MONTH, DAY, HR, MIN, SEC);
      Time_stamp = String(s_time);
      Serial.print("Rssi");
      Serial.println(Rssi);
      tagDist = pow(10, ((Tx - Rssi) / (10 * N)));

      //  storing_data =  "{\"gSrNo\":\"G124\",\"gMacId\":\"c4:4f:33:4f:65:75\",\"tMacId\":\"" + tMacId + "\",\"bSN\":\"" + bSN + "\",\"batStat\":\"" + BatStat + "\",\"TS\":\"" + Time_stamp + "\",\"tagDist\":\"" + tagDist + "\"}";
      //   storing_data =  "{\"gSrNo\":\"G128\",\"gMacId\":\" 24:0a:c4:16:31:80\",\"tMacId\":\"" + tMacId + "\",\"bSN\":\"" + bSN + "\",\"batStat\":\"" + BatStat + "\",\"TS\":\"" + Time_stamp + "\",\"tagDist\":\"" + tagDist + "\"}";
      // storing_data =  "{\"gSrNo\":\"G123\",\"gMacId\":\"24:0a:c4:16:31:9c\",\"tMacId\":\"" + tMacId + "\",\"bSN\":\"" + bSN + "\"}";

      int entry_flag = 0;
      if (counter_for_scanning_data == 1)
      {
        entry_flag = 1;
      }

      memset(send_buffer, 0, sizeof(send_buffer));
      k = 0;
      j = 0;
      for (k = 0, j = 0; storing_data[k]; k++)
      {
        if (entry_flag == 1)
        {
          entry_flag = 0;
          send_buffer[0] = '[';
          j++;
        }
        if ( storing_data[k] == '}')
        {
          send_buffer[j] =  storing_data[k];
          j++;
          send_buffer[j] = ',';
          j++;
        }
        else
        {
          send_buffer[j] =  storing_data[k];
          j++;
        }
      }
      if (counter_for_scanning_data == config_device)       //change the value of config_device as per requirement
      {
        Serial.println("enter in last count ");
        j = j - 1;
        send_buffer[j] = ']';
      }
      storing_data = "";
      Serial.print("send_buffer= ");
      Serial.println(send_buffer);
      if (counter_for_scanning_data == 1)
      {
        writeFile(SPIFFS, Asset_File, send_buffer);
      }
      else
      {
        appendFile(SPIFFS, Asset_File, send_buffer);
        file_size(SPIFFS, Asset_File);
      }
      if (counter_for_scanning_data == config_device)   //change the value of config_device as per requirement
      {
        Serial.println("stop Advertising");
        advertisedDevice.getScan()->stop();
      }
    }
};
/************************************************************/
void setup() {
  Serial.begin(115200);
  Gateway_power_ON_SETUP();                                                    // BQ configuration, GPIO mode, GSM booting
  ble_init_param_set_fun();                                                    // BLE initialization
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
  {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  listDir(SPIFFS, "/", 0);                                                      // list all files under directory "/";
  readFile(SPIFFS, Parameter_set_File);                                         // read from file parameter set through bluetooth function

  read_g_str_1 = "";
  readFile1(SPIFFS, WiFi_enable_file);                                        // read mannual flag
  WiFi_enable  = read_g_str_1.toInt();                                        // manual flag for set rtc mannualy after wakeup from deep sleep mode
  GSM.begin(115200, SERIAL_8N1, 16, 17);                                      // gsm begin for cellular uart At command
  print_wakeup_reason();                                                      // getting wakeup reason


  // print date and time
  /*********** Aftet Hard Reset this loop will enable  *****************/
  if (wakeup_timer == 0)
  {
    com_sec = SEC;
    com_min = MIN;
    com_hr = HR;

    BLE_credential_Start();                                                     // parameter set through bluetooth function
    readFile(SPIFFS, Parameter_set_File);                                       // read from file parameter set through bluetooth function
    read_g_str_1 = "";
    readFile1(SPIFFS, WiFi_enable_file);                                        // read mannual flag
    WiFi_enable  = read_g_str_1.toInt();                                        // manual flag for set rtc mannualy after wakeup from deep sleep mode

    deleteFile(SPIFFS, Asset_File);                                             // delete asset file from spiffs
    writeFile(SPIFFS, scan_time_file, "0");                                     // reset scan time


    if (WiFi_enable == 1)
    {
      WiFi_Network_time();
    }
    if (cellular_connectivity_enable == 1)                     // enabling cellular connectivity Function
    {
      Network_connect_Gateway();
    }
    if (MQTT_connect_flag == 1)                               // enable cellular MQTT function
    {
      MQTT_cloud_connect();
    }

  }
  readDetails();                                             // Reading data in spiff

  WiFi.mode(WIFI_STA); // Set the WiFi mode to station mode
  gmacAddress = WiFi.macAddress();
  storing_data =  "{\"gSrNo\":\"" + gId + "\",\"gMacId\":\"" + gmacAddress + "\",\"tMacId\":\"" + tMacId + "\",\"bSN\":\"" + bSN + "\",\"batStat\":\"" + BatStat + "\",\"TS\":\"" + Time_stamp + "\",\"tagDist\":\"" + tagDist + "\"}";

  HOST_ADDRESS.toCharArray(hostAddr, sizeof(hostAddr));
  CLIENT_ID.toCharArray(clientID, sizeof(clientID));
  TOPIC_NAME.toCharArray(topic, sizeof(topic));

  /********** set RTC with gmt modes *****************/

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  /**************************************************************/

  S.begin(9600, SWSERIAL_8N1);        // modbus begin
  mb.begin(&S, DE_RE);                //Assing Software serial port to Modbus instance for MAX485 chip having DI,DE,RE,RO Pin at TTL side
  mb.master();                        //Assing Modbus function as master

  /**************************** sd card begin ****************************/
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  // deleteFile(SD, SD_File);

}

void loop() {
  /************* ble enable function *************/

  if (ble_enable == 1)                                     // enable ble
  {
    ble_init_Scanner_fun();                                        // BLE init function
    Serial.print("before scanning beacon = ");
    Serial.println(ESP.getFreeHeap());
    beacon_data();              // scanning beacon
    Serial.print("before ble deinit = ");
    Serial.println(ESP.getFreeHeap());
    BLEDevice::deinit();    // keep empty to clear memory and init ble again
    delay(100);
    Serial.print("after ble deinit = ");
    Serial.println(ESP.getFreeHeap());
  }
  /************* BQ(Battery percentage Fun) *************/
  Serial.println("");
  Serial.println("");
  reg8_reg9();
  Status_reg(reg_data8);
  fault( reg_data9);
  batt_attach();
  Serial.println("");
  Serial.println("");
  //  Serial.print("ble_enable=");
  //  Serial.println(ble_enable);
  Serial.print("counter reached to= "); Serial.println(counter_for_scanning_data);
  /************* nectwork connectivity function for ble (asset tracking)*************/
  if ((ble_enable == 1) && (counter_for_scanning_data == config_device))
  {
    Serial.println("================== BLE enable and device count matched =================");
    read_g_str_1 = "";                                                                       // clear String before read from file
    readFile1(SPIFFS, scan_time_file);                                                       // read 1 loop cycle time which is store
    total_tags_scan_time = read_g_str_1.toInt();                                             // convert String to int form
    deleteFile(SPIFFS, scan_time_file);                                                      // delete file for pevious tags Scan time which is store
    total_tags_scan_time = total_tags_scan_time + tags_scan_time + ( TIME_TO_SLEEP * 60000); // calculate total time of 1 loop cycle
    String scan_time_in_file = String(total_tags_scan_time);                                 // convert int to String to add in file
    writeFile(SPIFFS, scan_time_file, scan_time_in_file.c_str());                            // write 1 loop cycle time in file
    //    Serial.print("tags_scan_time:= ");
    //    Serial.println(tags_scan_time);
    //    Serial.print("total_tags_scan_time:= ");
    //    Serial.println(total_tags_scan_time);
    Serial.print("RTC_TIME_RESET:= ");
    Serial.println(RTC_TIME_RESET * 3600000);
    deleteFile(SPIFFS, TIME_STAMP);                                                           // delete old time
    printLocalTime();                                                                         // print date and time
    writeFile(SPIFFS, TIME_STAMP, Time_stamp.c_str());                                        // Write new time and date

    readFile1(SPIFFS, Asset_File);                                                            // read all Asset tags data stored in this file
    Data_buffer = "";
    Data_buffer = read_g_str_1;
    //    Serial.print("Data_buffer:= ");
    //    Serial.println(Data_buffer);

    if (WiFi_enable == 1)                                                                     // enabling Wifi function
    {
      if (HTTP_MQTT == 1)
      {
        WiFi_http();                                                                          // Wifi_Http Fun
      }
      else if (HTTP_MQTT == 0)
      {
        Wifi_MQTT();                                                                          // Wifi_Mqtt Fun
      }

      if (cellular_connectivity_enable == 0)
      {
        counter_for_scanning_data = 0;
      }
    }
    delay(10);
    if ((cellular_connectivity_enable == 1) && (counter_for_scanning_data == config_device)) // enabling Gsm Function
    {
      readFile1(SPIFFS, Asset_File);                                                         // Asset tag data stored in this file
      Data_buffer = "";
      Data_buffer = read_g_str_1;
      Serial.print("Data_buffer:= ");
      Serial.println(Data_buffer);
      //     Serial.println(ESP.getFreeHeap());
      connection_type();

      if (HTTP_MQTT == 1)
      {
        Cellular_HTTP_Post();                                                                // cellular http function
      }
      else if (HTTP_MQTT == 0)
      {
        Cellular_MQTT_Publish();                                                            // cellular mqtt function
      }
      read_g_str_1 = "";
      counter_for_scanning_data = 0;
    }  // Deep sleep function after
    //   Serial.println("last rtc time");
    //   printLocalTime();
    esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP * 60) * uS_TO_S_FACTOR);                   // wakeup function
    Serial.println("Gateway device wake_up after " + String(TIME_TO_SLEEP * 60) + " Seconds");
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
  }
  /************* nectwork connectivity function for Modbus *********************/
  else if (modbus_enable == 1)
  {
    Serial.println("Enter in modbus loop = ");
    readFile_2(SPIFFS, MODBUS_FILE);                                            // read Modbud slave data from file parameter set through bluetooth function
    modbus_master_call();
    if (WiFi_enable == 1)                              // enabling Wifi function
    {
      Serial.println(ESP.getFreeHeap());
      if (HTTP_MQTT == 1)
      {
        WiFi_http();                                  // Wifi_Http Fun
      }
      else if (HTTP_MQTT == 0)
      {
        Wifi_MQTT();                                  // Wifi_Mqtt Fun
      }
      counter_for_scanning_data = 0;
      //      Serial.print("cellular_connectivity_enable = ");
      //      Serial.println(cellular_connectivity_enable);
    }
    delay(10);
    if (cellular_connectivity_enable == 1)                     // enabling cellualar connectivity Function
    {
      Serial.println(ESP.getFreeHeap());
      connection_type();

      if (HTTP_MQTT == 1)
      {
        Cellular_HTTP_Post();                                 // cellular http function
      }
      else if (HTTP_MQTT == 0)
      {
        Cellular_MQTT_Publish();                              // cellular mqtt function
      }
      read_g_str_1 = "";
    }
    // print date and time
    esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP * 60) * uS_TO_S_FACTOR);                   // wakeup function
    Serial.println("Gateway device wake_up after " + String(TIME_TO_SLEEP * 60) + " Seconds");
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
  }
}

/************************ ble credential functions *****************************/
void BLE_credential_Start()
{
  Startmillis_Password = millis();
  Serial.println("The device started, now you can pair it with bluetooth!");
  while (millis() - Startmillis_Password <= ble_data_time)
  {
    if (c_1 == 0 && security_enable == 1)        // this loop for to print Enter password on Bt App
    {
      Ble_Available = 0;
      Serial.println("Enter Password");
      delay(2000);
      if (deviceConnected == true)
      {
        pCharacteristic->setValue("Enter Password:- \n");
        pCharacteristic->notify();
      }
      c_1++;
    }

    if (Ble_Available == 1)
    {
      Ble_credential_password();                                   //  password Recieved function
    }
    if (valid_password_flag == 1)                                 // if password is correct
    {
      if (Password == Rcv_Password)                               // compare is password is correct or not
      {
        pCharacteristic->setValue("Enter the Parameters\n");
        pCharacteristic->notify();
        pCharacteristic->setValue("Select WIFI(yes/no)\n");
        pCharacteristic->notify();
        Serial.println("Enter the Parameters");
        modbus_enable = 0;
        while (1)                                                 // Parameters Entry loop through Bt App
        {
          Currentmillis = millis();
          ble_credentials_data_split();
          if (flag_enable == 1)
          {
            Credentials();
            flag_enable = 0;
            mem_write_flag = 1;
            if (modbus_enable == 1)                                // after modbus enable
            {
              deleteFile(SPIFFS, MODBUS_FILE);                     // delete modbus file to enter new data in file
              pCharacteristic->setValue("Add New slave data (yes/no)\n");
              pCharacteristic->notify();
              while (1)                                            // modbus data entry loop
              {
                ModBus_Data_to_Mem();                              // enter modbus data 1 by 1 from bt app and store data in memory

                if (exit_flag == 1)                                // if all modbus data is enter then exit the loop
                {
                  break;
                }
              }
            }
            Previoustmillis = Currentmillis;                          // it gives 3 min time to recieved from Bt app for each entry
          }
          if (Currentmillis - Previoustmillis >= ble_data_time)       //exit 3 min loop for when data recieved from bt app.
          {
            Serial.println("Enter in 3 min exit loop");
            break;
          }
          else if (j_1 >= 23)                                         // if all parameter pass then terminate the loop
          {
            break;
          }
        }
        break;
      }
      else
      {
        Try++;
        pCharacteristic->setValue("Password Incorrect,try again\n");
        pCharacteristic->notify();
      }
    }
    if (Try == 3)                                                // if password enter 3 times incorrectly then terminate from loop
    {
      break;
    }
  }
  if (mem_write_flag == 1)
  {
    arr_M = arr;
    mem_write_flag = 0;
    deleteFile(SPIFFS, Parameter_set_File);                                           // delete file for add new data in same file

    Serial.println("before write");
    add_mry_data = slash + ssid_M + slash + password_M + slash + arr_M + slash + GMT_Time_M + slash + bandmask_M + slash + HTTP_MQTT_M + slash + ble_enable_M + slash + config_device_M + slash + time_stamp_M + slash + TIME_TO_SLEEP + slash + modbus_enable_M ; // add all data for to write in memoty
    Serial.print("add_mry_data=");
    Serial.println(add_mry_data);
    write_in_file(add_mry_data, add_mry_data.length());                        // Write data into memory
  }
  Serial.println("bt_Credential_task complete");
  pCharacteristic->setValue("data successfully Entered\n");
  pCharacteristic->notify();
  Serial.println(ESP.getFreeHeap());
  BLEDevice::deinit();    // keep empty to clear memory and init ble again
  Serial.println(" BT end");
  Serial.println(ESP.getFreeHeap());
}


void Ble_credential_password()                                             // this function is for password set in bluetooth app.
{
  int k_1 = 0;
  memset(Rcv_Password, 0, sizeof(Rcv_Password));
  if (Ble_Available == 1) {                                     // Serial data recieved loop From Bt app
    for (i_2 = 0, j_2 = 0, k_2 = 0; RCV_DATA[i_2]; i_2++, j_2++)         // split functon
    {
      Rcv_Password[j_2] = RCV_DATA[i_2];
    }
    Ble_Available = 0;
    valid_password_flag = 1;                                                // password enter flag
  }
}

void ble_credentials_data_split()                                         // segrigate the data when read from Bluetooth App
{
  int i_1 = 0;

  if (Ble_Available == 1)
  {

    if (Ble_Available == 1) {                                             // Serial data recieved loop From Bt app
      for (i_2 = 0, j_2 = 0, k_2 = 0; RCV_DATA[i_2]; i_2++, j_2++)         // split functon
      {
        Bt_recieve[j_2] = RCV_DATA[i_2];
      }
    }
    Ble_Available = 0;
    flag_enable = 1;                                                    // this flag is set for each entry fro bt app.
    j_1++;                                                              // depends on this varible data will enter 1 by 1.
  }
  for (i_2 = 0, j_2 = 0, k_2 = 0; Bt_recieve[i_2]; i_2++, j_2++)         // split functon
  {
    command1[j_2] = Bt_recieve[i_2];
  }
  memset(Bt_recieve, 0, sizeof(Bt_recieve)); // clear the array
}


void Credentials()                                            // to store the data in variables from Bluetooth app
{
  command = command1;
  if (j_1 == 1)                                               //   for Wifi set
  {
    if (command == "yes")                                    // entry command from bt app
    {
      Serial.println("Wifi is set");
      pCharacteristic->setValue("Enter WiFi ssid=\n");
      pCharacteristic->notify();
    }
    else if (command == "no")                                 // entry command from bt app
    {
      j_1 = j_1 + 2;
      pCharacteristic->setValue("Select GSM (yes/no)=\n");
      pCharacteristic->notify();
      writeFile(SPIFFS, WiFi_enable_file, "0");
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 2)                                                     // to enter wifi_ssid
  {
    ssid_M = command;
    pCharacteristic->setValue("Enter WiFi password=\n");
    pCharacteristic->notify();
    writeFile(SPIFFS, WiFi_enable_file, "1");
  }
  else if (j_1 == 3)                                                   // to enter WiFi_password
  {
    password_M = command;
    pCharacteristic->setValue("Select GSM (yes/no)=\n");
    pCharacteristic->notify();
  }
  else if (j_1 == 4)                                                   // Select GSM
  {
    if (command == "yes")                                             // entry command from bt app
    {
      pCharacteristic->setValue("Select NBIOT (yes/no)=\n");
      pCharacteristic->notify();
      arr[i_1] = 'A';
      i_1 = i_1 + 1;
    }
    else if (command == "no")                                        // entry command from bt app
    {
      pCharacteristic->setValue("Select NBIOT (yes/no)=\n");
      pCharacteristic->notify();
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 5)                                                  // Select NBIOT
  {
    //    Serial.print("command= ");
    //    Serial.println(command);
    if (command == "yes")                                            // entry command from bt app
    {
      pCharacteristic->setValue("Select CATM1 (yes/no)=\n");
      pCharacteristic->notify();
      arr[i_1] = 'B';
      i_1 = i_1 + 1;
    }
    else if (command == "no")                                        // entry command from bt app
    {
      pCharacteristic->setValue("Select CATM1 (yes/no)=\n");
      pCharacteristic->notify();
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 6)                                                  // Select CATM1
  {
    //    Serial.print("command= ");
    //    Serial.println(command);
    if (command == "yes")                                            // entry command from bt app
    {
      pCharacteristic->setValue("Enter GMT Time(yes/no)\n");
      pCharacteristic->notify();
      arr[i_1] = 'C';
      i_1 = i_1 + 1;
    }
    else if (command == "no")                                        // entry command from bt app
    {
      pCharacteristic->setValue("Enter GMT Time(yes/no)\n");
      pCharacteristic->notify();
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 7)                                                  // Select CATM1
  {
    if (command == "yes")                                            // entry command from bt app
    {
      pCharacteristic->setValue("Enter GMT TIME(Ex 5:30)\n");
      pCharacteristic->notify();
    }
    else if (command == "no")                                        // entry command from bt app
    {
      j_1 = j_1 + 1;
      pCharacteristic->setValue("Enter Bandmask(yes/no)\n");
      pCharacteristic->notify();
      Serial.println("Bandmask is not set");
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 8)                                                // Enable BLE
  {
    GMT_Time_M = command;
    pCharacteristic->setValue("Enter Bandmask(yes/no)\n");
    pCharacteristic->notify();
  }

  else if (j_1 == 9)                                                  // Select CATM1
  {
    if (command == "yes")                                            // entry command from bt app
    {
      pCharacteristic->setValue("Enter Bands= (Ex. 2,4,5,12)\n");
      pCharacteristic->notify();
    }
    else if (command == "no")                                        // entry command from bt app
    {
      j_1 = j_1 + 1;
      pCharacteristic->setValue("Enter HTTP=1 OR MQTT=0\n");
      pCharacteristic->notify();
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 10)                                                // Enable BLE
  {
    bandmask_M = command;
    pCharacteristic->setValue("Enter HTTP=1 OR MQTT=0..\n");
    pCharacteristic->notify();
  }

  else if (j_1 == 11)                                                // Enable HTTP_MQTT Flag HTTP=1,MQTT=0
  {
    HTTP_MQTT_M = command;
    if (HTTP_MQTT_M == "0" || HTTP_MQTT_M == "1")
    {
      if (HTTP_MQTT_M == "0")
      {
        pCharacteristic->setValue("Enter host address,clientId,topic name separated by coma(,)\n");
        pCharacteristic->notify();
        Serial.println(j_1);
      }
      else
      {
        j_1++;
      }
      //      if (j_1 == 12)
      //      {
      //        Serial.println("saving mqtt details");
      //        if (HTTP_MQTT_M == "0")
      //        {
      //          getMQTT();
      //        }
      //        j_1 = j_1 - 1;
      //        pCharacteristic->setValue("Enable BLE(yes/no)\n");
      //        pCharacteristic->notify();
      //        Serial.println(j_1);
      //      }
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter 0 for MQTT and 1 for HTTP \n");
      pCharacteristic->notify();
    }

  }
  if (j_1 == 12)                                                // Enable BLE
  {
    if (HTTP_MQTT_M == "0")
    {
      getMQTT();
    }
    pCharacteristic->setValue("Enable BLE(yes/no)\n");
    pCharacteristic->notify();
  }
  else if (j_1 == 13)                                                // Enable BLE
  {

    if (command == "yes")                                           // entry command from bt app
    {
      ble_enable_M = "1";                                          // ble enable
      pCharacteristic->setValue("Enter Ble Device(yes/no)\n");
      pCharacteristic->notify();
    }
    else if (command == "no")                                       // entry command from bt app
    {
      ble_enable_M = "0";                                          // ble disable
      j_1 = j_1 + 2;
      pCharacteristic->setValue("Set Time Stamp (yes/no)\n");
      pCharacteristic->notify();
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 14)                                               //  Enable BLE
  {
    //    Serial.print("command= ");
    //    Serial.println(command);
    if (command == "yes")                                          // entry command from bt app
    {
      pCharacteristic->setValue("Enter device Count (range 1 to 1000)\n");
      pCharacteristic->notify();
    }
    else if (command == "no")                                      // entry command from bt app
    {
      j_1 = j_1 + 1;

      pCharacteristic->setValue("Set Time Stamp (yes/no)\n");
      pCharacteristic->notify();
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 15)                                                     // to enter config device in numbers >0
  {
    int  device_config_int = command.toInt();                            // convert string to int for config device
    if (device_config_int >= 0 && device_config_int <= 1000)             // enter config device from 0 to 1000 in numbers only
    {
      config_device_M = command;                                        // store config device in memory variable to store data in memory
      pCharacteristic->setValue("Set Time Stamp (yes/no)\n");
      pCharacteristic->notify();
    }
    else                                                               // if data will not enter in numbers from 1 to 999 then it will ask for reenter the data
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter Count in numbers\n");
      pCharacteristic->notify();
      pCharacteristic->setValue("Enter device Count (range 1 to 1000)\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 16)                                              //  Enable BLE
  {
    //    Serial.print("command= ");
    //    Serial.println(command);
    if (command == "yes")                                          // entry command from bt app
    {
      pCharacteristic->setValue("Set Time Stamp as Follows\n");
      pCharacteristic->notify();
      pCharacteristic->setValue("sec,min,hr,day,month,year\n");
      pCharacteristic->notify();
    }
    else if (command == "no")                                     // entry command from bt app
    {
      j_1 = j_1 + 1;
      pCharacteristic->setValue("Enter device sleep time(range 1 to 120 min)\n");
      pCharacteristic->notify();
      // SerialBT.println("Enable Modbus(yes/no)");
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }
  else if (j_1 == 17)                                                // Server address information
  {
    time_stamp_M = command;
    pCharacteristic->setValue("Enter Server Address\n");
    pCharacteristic->notify();
  }
  else if (j_1 == 18)                                                // APN setting
  {
    serverName = command;
    writeFile(SPIFFS, ServerAddress, serverName.c_str());
    pCharacteristic->setValue("Enter Gateway Id\n");
    pCharacteristic->notify();
  }
  else if (j_1 == 19)                                                // Server address information
  {
    gId = command;
    writeFile(SPIFFS, gID, gId.c_str());
    pCharacteristic->setValue("Enter APN\n");
    pCharacteristic->notify();
  }
  else if (j_1 == 20)                                                // Enable BLE
  {
    apnName = command;
    pCharacteristic->setValue("Enter device sleep time(range 1 to 120 min)\n");
    pCharacteristic->notify();
  }

  else if (j_1 == 21)                                                // Enable HTTP_MQTT Flag HTTP=1,MQTT=0
  {
    TIME_TO_SLEEP = command.toInt();
    if (TIME_TO_SLEEP > 0 && TIME_TO_SLEEP <= 120)
    {
      TIME_TO_SLEEP_M = command;
      pCharacteristic->setValue("Enable Modbus(yes/no)\n");
      pCharacteristic->notify();
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please Enter device sleep time(range 1 to 120 min)\n");
      pCharacteristic->notify();
    }
  }

  else if (j_1 == 22)                                               //  Enable BLE
  {

    if (command == "yes")                                          // entry command from bt app
    {
      modbus_enable_M = '1';
      modbus_enable = 1;
      j_1 = j_1 + 1;

    }
    else if (command == "no")                                      // entry command from bt app
    {
      j_1 = j_1 + 2;
      modbus_enable_M = '0';

      pCharacteristic->setValue("all parameters are set\n");
      pCharacteristic->notify();
      writeDetails();
      // SerialBT.println("all parameters are set");                 // if all param set then it will print on bt app
    }
    else
    {
      j_1 = j_1 - 1;
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");
      pCharacteristic->notify();
    }
  }

  command = '0';
  command2 = '0';

  memset(command1, 0, sizeof(command1));                            //clear the array
  memset(value, 0, sizeof(value));                                  // clear the array
}

/********************** WIFI Function ***************************/
void WiFi_http()                                                             // WiFi Http Function
{
  WiFi.begin(WiFi_ssid.c_str(), WiFi_password.c_str());                      // wifi begin with ssid and password
  delay(100);
  wifi_try = 0;
  wifi_connected = 0;
  while (WiFi.status() != WL_CONNECTED)                                     // modify and put wifi connect condition
  {
    wifi_try++;
    delay(500);
    Serial.print("% ");
    if (wifi_try == 15)                                                    // after try of 15 times to connect wifi then it will reminate from the loop
      break;
  }
  if (WiFi.status() != WL_CONNECTED)                                        // if wifi is not connected
  {
    if (connectivity != "\0")                                             // if cellular connectivity is set then  cellular_connectivity_enable=1
    {
      cellular_connectivity_enable = 1;
    }
    else
    {
      counter_for_scanning_data = 0;
      SD_String_receive = "";
      SD_readFile(SD, Write_pointer);                                      // read current file pointer

      if (SD_String_receive != "\0")
      {
        SD_File_No_Write = SD_String_receive.toInt();                          // assign new file no. pointer
      }
      else
      {
        SD_File_No_Write = 0;
      }
      Make_file_name(SD_File_No_Write);                                 // make new file for current data store
      SD_File_No_Write++;                                               // file no. increament
      String SD_No_Write = String(SD_File_No_Write);
      writeFile(SD, Write_pointer, SD_No_Write.c_str());              // file no. pointer increament and write in file
      Serial.print("Buffer in wifi not connected:= ");
      Serial.println(Data_buffer);
      writeFile(SD, SD_File, Data_buffer.c_str());                     // write current asset data in new file
      file_size(SD, SD_File);                                           // file size
    }
    Serial.println("UNABLE TO CONNECT TO WIFI\n");                          //wifi_not connected
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("Battery = ");
    display.print(pwr_mgmt.percent());
    display.println("%");
    display.println("NW Fails...");
    display.display();
  }
  else    //wifi connected
  {

    /******************** set time by network ***********************/
    if (total_tags_scan_time > (RTC_TIME_RESET * 180000))               //3600000
    {
      writeFile(SPIFFS, scan_time_file, "0");
      Serial.println("RTC update by WiFi network..........................");
      configTime(gmtOffset_sec, Network_daylightOffset_sec, ntpServer); // set network time to rtc
      delay(100);
      //     printLocalTime();                                                 // print date and time
      //          configTime(gmtOffset_sec, Network_daylightOffset_sec, ntpServer); // set network time to rtc
      //          delay(1000);
      //          printLocalTime();                                                 // print date and time
      //      mannual_Timestamp(Time_stamp);
      //      ADD_GMT_Time(HR, MIN, SEC, DAY, MONTH, YEAR);             //  convert rcv network time to gmt time as per the regions (EX. 5:30)
      // printLocalTime();                                                 // print date and time
      //      setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);                       // sec min hr ,day/month/year
      //                printLocalTime();                                                 // print date and time
      //                delay(100);
      //                printLocalTime();                                                 // print date and time
      if (local_time_flag != 1)                                         // this flag will set when we ge network time
      {
        writeFile(SPIFFS, Mannual_rtc_enable_FILE, "0");                // if Mannual Rtc enable flag =0 then then network set time will continue
        Serial.println("RTC Set by network");
      }
      else
      {
        Serial.println("RTC not Set by network");
        writeFile(SPIFFS, Mannual_rtc_enable_FILE, "1");
        readFile1(SPIFFS, TIME_STAMP);                                  // Read timestamp Store in memory if time not set by network
        mannual_Timestamp(read_g_str_1);                                // set memory time
        setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);                     // sec min hr ,day/month/year
      }
    }

    /*******************************************/
    cellular_connectivity_enable = 0;                                               // if wifi connected then this celluar flag is disable
    Serial.println("CONNECT TO WIFI\n");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("Battery = ");
    display.print(pwr_mgmt.percent());
    display.println("%");
    display.println("NW Connected...");
    display.display();
    /*******************************************/
    Serial.print("inside http loop wifi connected");
    WiFiClient client;
    HTTPClient http;
    http.begin(client, serverName);                                       // http begin with server name
    Serial.println(serverName);
    Serial.print("HTTP Connected:----- ");
    Serial.println(http.connected());
    // Specify content-type header
    http.addHeader("Content-Type", "application/json");                 // content type is application json to send over http
    counter_for_scanning_data = 0;
    SD_String_receive = "";
    SD_readFile(SD, Write_pointer);                                      // read current file pointer
    if (SD_String_receive != "\0")
    {
      SD_File_No_Write = SD_String_receive.toInt();                          // assign new file no. pointer
    }
    else
    {
      SD_File_No_Write = 0;
    }

    if (SD_File_No_Write > 0)
    {
      SD_Read_enable = 1;
    }
    else
    {
      // Send HTTP POST request
      Serial.println(" Current Buffer in wifi connected and send to sever:= ");
      int httpResponseCode = http.POST(Data_buffer);                     // response code from http server
      delay(100);
      Serial.print("HTTP Response code:= ");
      Serial.println(httpResponseCode);
      Serial.println(http.getString());
      int Responsecode = 0;
      Responsecode = httpResponseCode;
      if ((Responsecode == 200) || (Responsecode == 202))          // check response code if not 200 or 202 then data write in file
      {
        ;
      }
      else
      {
        counter_for_scanning_data = 0;
        if (connectivity != "\0")                                 // if cellular connectivity is set then  cellular_connectivity_enable=1
        {
          cellular_connectivity_enable = 1;
        }
        else
        {
          SD_String_receive = "";
          SD_readFile(SD, Write_pointer);                                      // read current file pointer
          if (SD_String_receive != "\0")
          {
            SD_File_No_Write = SD_String_receive.toInt();                          // assign new file no. pointer
          }
          else
          {
            SD_File_No_Write = 0;
          }
          Make_file_name(SD_File_No_Write);                                 // make new file for current data store
          SD_File_No_Write++;                                               // file no. increament
          String SD_No_Write = String(SD_File_No_Write);
          writeFile(SD, Write_pointer, SD_No_Write.c_str());              // file no. pointer increament and write in file
          Serial.println("Buffer in wifi connected but response code is not 200 or 202 so store in sd file:= ");
          writeFile(SD, SD_File, Data_buffer.c_str());                     // write current asset data in new file
          file_size(SD, SD_File);                                           // file size
        }

      }
    }
    if (SD_Read_enable == 1)                                           // if this flag is enable then only file can be read from SD card
    {

      SD_Read_enable = 0;
      deleteFile(SPIFFS, Asset_File);

      SD_String_receive = "";
      SD_readFile(SD, Read_pointer);                                   // read current file pointer
      if (SD_String_receive != "\0")
      {
        SD_File_No_Read = SD_String_receive.toInt();                      // assign new file no. pointer
      }
      else
      {
        SD_File_No_Read = 0;
      }

      while (SD_File_No_Read < SD_File_No_Write)
      {
        Make_file_name(SD_File_No_Read);                                      // make new file for current data read
        SD_String_receive = "";
        SD_readFile(SD, SD_File);
        Serial.println("Buffer in wifi connected and send to sever read file from sd card 1 by 1 := ");
        WiFiClient client;
        HTTPClient http;
        http.begin(client, serverName);                                       // http begin with server name
        Serial.println(serverName);
        Serial.print("HTTP Connected:----- ");
        Serial.println(http.connected());
        // Specify content-type header
        http.addHeader("Content-Type", "application/json");                   // content type is application json to send over http
        int httpResponseCode = http.POST(SD_String_receive);                  // response code from http server
        delay(100);
        Serial.print("HTTP Response code:= ");
        Serial.println(httpResponseCode);
        Serial.println(http.getString());
        if ((httpResponseCode == 200) || (httpResponseCode == 202))
        {
          deleteFile(SD, SD_File);
          SD_File_No_Read++;                                               // file no. increament
          String SD_No_Read = String(SD_File_No_Read);
          writeFile(SD, Read_pointer, SD_No_Read.c_str());             // file no. pointer increament and write in file
        }
        else
        {
          Serial.println(" ");
          Serial.println(" ");
          Serial.println("break in sd file loop because response code is not 200 0r 202");
          Serial.println(" ");
          Serial.println(" ");
          break;
        }
        http.end();
        delay(1000);
      }
      if (SD_File_No_Read == SD_File_No_Write)
      {
        SD_File_No_Read = 0;
        SD_File_No_Write = 0;
        SD_Read_enable = 0;
        current_data_flag = 1;
        Serial.println("all files read then clear all pointers");
      }
      if (current_data_flag == 1)
      {
        current_data_flag = 0;
        WiFiClient client;
        HTTPClient http;
        http.begin(client, serverName);                                       // http begin with server name
        Serial.println(serverName);
        Serial.print("HTTP Connected:----- ");
        Serial.println(http.connected());
        // Specify content-type header
        http.addHeader("Content-Type", "application/json");                 // content type is application json to send over http
        Serial.println("current data Buffer in wifi connected and send to sever after all sd card data send := ");
        int httpResponseCode = http.POST(Data_buffer);                     // response code from http server
        delay(100);
        Serial.print("HTTP Response code:= ");
        Serial.println(httpResponseCode);
        Serial.println(http.getString());
      }
      else
      {
        Make_file_name(SD_File_No_Write);                                 // make new file for current data store
        SD_File_No_Write++;                                               // file no. increament
        String SD_No_Write = String(SD_File_No_Write);
        writeFile(SD, Write_pointer, SD_No_Write.c_str());              // file no. pointer increament and write in file
        Serial.println("Current buffer stored in sd file beacause all files of sd card not able to send to server := ");
        writeFile(SD, SD_File, Data_buffer.c_str());                     // write current asset data in new file
        file_size(SD, SD_File);                                           // file size
      }
    }
    delay(1000);
    http.end();                                                      // end http
    Serial.print("HEAP before hit http= ");
    Serial.println(ESP.getFreeHeap());
  }
  WiFi.disconnect();                                                // diconnect the wifi
  delay(100);
  while (WiFi.status() == WL_CONNECTED)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi is disconnectd");
      break;
    }
  }
  delay(50);
}



void Wifi_MQTT()                                                // Wifi Mqtt Function
{

  WiFi.begin(WiFi_ssid.c_str(), WiFi_password.c_str());         // begin Wifi
  wifi_try = 0;
  wifi_connected = 0;
  while (WiFi.status() != WL_CONNECTED)                   // wifi connect condition
  {
    wifi_try++;
    delay(500);
    Serial.print("% ");
    if (wifi_try == 15)                                  // disconnect wifi after try of 15
      break;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    if (connectivity != "\0")                                 // if cellular connectivity is set then  cellular_connectivity_enable=1
    {
      cellular_connectivity_enable = 1;
    }
    else
    {
      counter_for_scanning_data = 0;
      SD_Read_enable = 1;
      Make_file_name(SD_File_No_Write);
      SD_File_No_Write++;
      appendFile(SD, SD_File, read_g_str_1.c_str());
      file_size(SD, SD_File);
    }
    Serial.println("UNABLE TO CONNECT TO WIFI\n");
  }
  else    //wifi connected
  {
    cellular_connectivity_enable = 0;

    Serial.println("CONNECTED TO WIFI\n");
    /******************** set time by network ***********************/
    //    if (total_tags_scan_time > (RTC_TIME_RESET * 3600000))
    //    {
    //      writeFile(SPIFFS, scan_time_file, "0");
    //      // rtc.setTime(0, 0, 0, 0, 0, 0);                                    // reset rtc befor network time set to rtc
    //      configTime(gmtOffset_sec, Network_daylightOffset_sec, ntpServer); // set network time to rtc
    //      delay(100);
    //      configTime(gmtOffset_sec, Network_daylightOffset_sec, ntpServer); // set network time to rtc
    //      printLocalTime();                                                 // print date and time
    //
    //      if (local_time_flag != 1)                                         // this flag will set when we ge network time
    //      {
    //        writeFile(SPIFFS, Mannual_rtc_enable_FILE, "0");                // if Mannual Rtc enable flag =0 then then network set time will continue
    //        Serial.println("RTC Set by network");
    //      }
    //      else
    //      {
    //        Serial.println("RTC not Set by network");
    //        readFile1(SPIFFS, TIME_STAMP);                                 // Read timestamp Store in memory if time not set by network
    //        mannual_Timestamp(read_g_str_1);                                           // set memory time
    //      }
    //    }
    /***************************************************************/
    AWS_IOT* myiot = new AWS_IOT();                               // Aws iot fun store in myiot
    int retries = 1, exit_timeout = 0;
    Serial.println("Checking connection for AWS IOT");
    delay(50);
    exit_timeout = micros() + 10000000;
    while (retries <= 10)
    {
      if (micros() > exit_timeout)
      {
        Serial.println("\nCONNECTION TIMEOUT in aws()\n\n");
        break;
      }
      int rtn_vrb = myiot->connect(hostAddr, clientID);                        // connecting to AWS
      if (rtn_vrb == 5)
      {
        Serial.println("\nNetwork is low or not available try after some time again\n");
        //deinit and regain the memory again
        aws_status = 0;
        myiot->to_do = 2;
        if (myiot->connect(hostAddr, clientID) == 2)
        {
          Serial.println("");
          Serial.println("Client is disconneccted successfully");
          Serial.println("");
        }
        else
        {
          Serial.println("client return something else");
        }
        if (retries <= 9)
        {
          delete myiot;
          myiot = NULL;
          Serial.print("after client disconnected = ");
          Serial.println(ESP.getFreeHeap());
          myiot = new AWS_IOT();
        }
      }
      else if (rtn_vrb == 0)
      {
        Serial.println("\n\nAWS_cloud connected successfully\n\n");
        aws_status = 1;
        break;
      }
      else
      {
        aws_status = 0;
        if (connectivity != "\0")                                 // if cellular connectivity is set then  cellular_connectivity_enable=1
        {
          cellular_connectivity_enable = 1;
        }
        else
        {
          counter_for_scanning_data = 0;
          deleteFile(SPIFFS, Asset_File);
          SD_Read_enable = 1;
          Make_file_name(SD_File_No_Write);
          SD_File_No_Write++;
          appendFile(SD, SD_File, read_g_str_1.c_str());
          file_size(SD, SD_File);
        }
      }

      Serial.printf("\n\n****retries = %d ****\n\n ", retries);
      delay(1000);
      retries++;
    }
    if (aws_status == 1)
    {
      wifi_mqtt_send(myiot);           // fun to send data to cloud.// we can send it in a loop for x time or till full data is send.
    }
    aws_status = 0;
    Serial.println("/././././././././././././././././././././././././././.");
    Serial.print("before myiot delete = ");
    Serial.println(ESP.getFreeHeap());
    Serial.println("/././././././././././././././././././././././././././.");

    if (myiot != NULL)
    {
      myiot->to_do = 2;
      //      Serial.print("myiot->to_do = ");
      //      Serial.println(myiot->to_do);
      if (myiot->connect(hostAddr, clientID) == 2)
      {
        Serial.println("");
        Serial.println("Client is disconneccted successfully");
        Serial.println("");
      }
      else
      {
        Serial.println("client return something else");
      }
      Serial.print("after client disconnected = ");
      Serial.println(ESP.getFreeHeap());
      if (myiot != NULL)
      {
        delete myiot;
        myiot = NULL;
      }
      else
      {
        //myiot is null priviously
      }
    }
    delay(50);
    Serial.println("/././././././././././././././././././././././././././.");
    Serial.print("after pointer delete = ");
    Serial.println(ESP.getFreeHeap());
    Serial.println("/././././././././././././././././././././././././././.");

    WiFi.disconnect();
    while (WiFi.status() == WL_CONNECTED)
    {
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("WiFi is disconnectd");
        break;
      }
    }
    delay(50);
  }
}

void aws_data(AWS_IOT * myiot)
{
  if (myiot->publish(topic, payload) == 0)                                   // Publish the message(Temp and humidity)
  {
    Serial.println("\n------------------------------------------------");
    Serial.print("\nPublish Message :: ");
    Serial.println(payload);
    Serial.print("\nSize of monitor data packet after write :: ");
    Serial.print(sizeof(payload));
    Serial.println(" bytes");
    Serial.println("\n------------------------------------------------");
    Serial.println("Published monitor data packet to AWS_CLOUD successfully");
    Serial.println("------------------------------------------------");
    delay(50);
    memset(payload, 0, 500 *  sizeof(payload[0]));                                  //check is this clear.
    Serial.println("\n------------------------------------------------");
    Serial.print("Size of payload after free ::");
    Serial.print(sizeof(payload));
    Serial.println(" bytes");
    delay(50);

  }
}

void wifi_mqtt_send(AWS_IOT * myiot)                                // send data function
{
  Serial.print("TIME TAKEN TO SEND START = ");
  Serial.println(micros());
  Serial.println("\n----------------------------------------------------------------");
  Serial.print("wifi status :: ");
  Serial.println(status);
  Serial.println();
  Serial.print("aws_status = ");
  Serial.println(aws_status);
  Serial.println();
  //check if wifi connected or not.
  //if connected then execute if part otherwise it execute else part to connect with wifi and further process2
  if (WiFi.status() == WL_CONNECTED)
  {
    counter_for_scanning_data = 0;
    if (aws_status == 1)            //if connected then execute if part otherwise else part
    {
      // JSON_PACKET_RELAY1_ON();
      String_split_MQTT_WiFI(myiot);
      deleteFile(SPIFFS, Asset_File);
      counter_for_scanning_data = 0;
      if (SD_Read_enable == 1)
      {
        counter_for_scanning_data = 0;
        while (SD_File_No_Read <= SD_File_No_Write)
        {
          Make_file_name(SD_File_No_Read);
          readFile1(SD, SD_File);
          String_split_MQTT_WiFI(myiot);
          deleteFile(SD, SD_File);
          SD_File_No_Read++;
        }
        if (SD_File_No_Read > SD_File_No_Write)
        {
          SD_File_No_Read = 0;
          SD_File_No_Write = 0;
          SD_Read_enable = 0;
        }
      }
    }
    else                            // firstly create aws connection and then execute rest of like json packet and aws data
    {
      Serial.println("aws_status == 0 before JSON and Publish");
    }
  }
  delay(50);
}

void JSON_PACKET_RELAY1_ON()                                   // JSON Packet to send on mqtt
{
  // JSON memory allocation
  DynamicJsonDocument doc(500); //data packet buffer
  doc["NINA"] = "MODBUS";
  doc["RELAY1"] = "ON";
  //  doc["Location"] = "warje";
  //  doc["Servicing_Alert"] = "Alert";
  Serial.println("JSON DATA PACKET   :");
  serializeJsonPretty(doc, Serial);
  // serializeJson(doc, Serial); //show on serial
  serializeJson(doc, payload);//all obj stored from doc to playload
}

/***************** Asset Ble function ************/
void ble_init_Scanner_fun()                                          // ble init function
{
  BLEDevice::init("embel");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void beacon_data(void)                                     // beacon function
{
  Serial.print("HEAP in Beacon = ");
  Serial.println(ESP.getFreeHeap());
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
}

int hexToDec(String hexString)
{
  int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++)
  {
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}

/******************* network connection type ********************/
void connection_type()                                          // set connectivity through this fun
{
  connectivity_type = connectivity;
  if (connectivity_type == "A")                                // if connectivity type is "A" then connect type set to 1
  {
    connect_type = 1;
    Serial.println(connect_type);
  }
  else if (connectivity_type == "B")                           // if connectivity type is "B" then connect type set to 2
  {
    connect_type = 2;
    Serial.println(connect_type);
  }
  else if (connectivity_type == "C")                           // if connectivity type is "C" then connect type set to 3
  {
    connect_type = 3;
    Serial.println(connect_type);
  }
  else if (connectivity_type == "AB")                          // if connectivity type is "AB" then connect type set to 4
  {
    connect_type = 4;
    Serial.println(connect_type);
  }
  else if (connectivity_type == "AC")                         // if connectivity type is "AC" then connect type set to 5
  {
    connect_type = 5;
    Serial.println(connect_type);
  }
  else if (connectivity_type == "ABC")                        // if connectivity type is "ABC" then connect type set to 6
  {
    connect_type = 6;
    Serial.println(connect_type);
  }

  switch (connect_type)                                                  // to select te connectvity using connect_type
  {
    case 1 : Serial.println("GSM only");                                 // select GSM
      break;
    case 2 : Serial.println("NBIOT only");                              // select NB-Iot
      break;
    case 3 : Serial.println("CATM1 only");                              // select CAT M1
      break;
    case 4 : Serial.println("GSM + NBIOT");                            // select GSM + NB-Iot
      break;
    case 5 : Serial.println("GSM + CATM1");                            // select GSM + Cat M1
      break;
    case 6 : Serial.println("GSM + NBIOT + CATM1");                   // select GSM + NB-Iot + Cat M1
      break;
  }
}

/*********************** Network_connect_Gateway Function *****************************/
void Network_connect_Gateway()                // network connectivity functon of cellular module SARA R4 12M
{

  sendATCommand("AT+CPSMS?", 1);              // Power seving mode check enable or diesable.
  sendATCommand("AT+CMEE?", 2);               // Setting error reporting format
  sendATCommand("AT+UMNOPROF?", 2);          // Check Response if it is 3 set to 1
  sendATCommand("AT+USIMSTAT?", 2);          //Status of SIM CARD
  sendATCommand("AT+CGREG?", 2);             // GPRS network registration
  sendATCommand("AT+CGDCONT?", 2);                    //find what apn is set to this simcard

  if (connectivity == "A")
  {
    sendATCommand("AT+URAT=9", 2);                         // Set GSM  Band
  }
  else if (connectivity == "B")
  {
    sendATCommand("AT+URAT=8", 2);                         // Set NB-Iot  Band
  }
  else if (connectivity == "C")
  {
    sendATCommand("AT+URAT=7", 1);                        // Set CATM1  Band
    sendATCommand("AT+UMNOPROF?", 1); // Check Response if it is 3 set to 1
    sendATCommand("AT+UMNOPROF=1", 2); // SIM ICCID select which will select MNO profile
    sendATCommand("AT+CFUN=15", 120 ); // resetting to make setting effective(make 3 min if any issue)
    sendATCommand("AT+UMNOPROF?", 2); // Check whether setting is effective
    //    sendATCommand("AT+UBANDMASK=0,2074",20); // set for North America
    GSM.print("AT+UBANDMASK=0,");                   // Download the data in file
    GSM.print(bitmask);
    GSM.write("\r\n");
    printGSMToSerial(10);
    sendATCommand("AT+UBANDMASK?", 4); // Check whether setting is effective
  }
  else if (connectivity == "AB")
  {
    sendATCommand("AT+URAT=8,9", 2);                      // Set GSM + NB-Iot  Band
  }
  else if (connectivity == "AC")
  {
    sendATCommand("AT+URAT=7,9", 4);                      //  Set GSM + CATM1  Band
    sendATCommand("AT+URAT=7", 4);                        // Set CATM1  Band
    sendATCommand("AT+UMNOPROF?", 4); // Check Response if it is 3 set to 1
    sendATCommand("AT+UMNOPROF=1", 4); // SIM ICCID select which will select MNO profile
    sendATCommand("AT+CFUN=15", 120 ); // resetting to make setting effective(make 3 min if any issue)
    sendATCommand("AT+UMNOPROF?", 4); // Check whether setting is effective
    //sendATCommand("AT+UBANDMASK=0,2074", 20); // set for North America
    GSM.print("AT+UBANDMASK=0,");                   // Download the data in file
    GSM.print(bitmask);
    GSM.write("\r\n");
    printGSMToSerial(10);
    sendATCommand("AT+UBANDMASK?", 4); // Check whether setting is effective
  }
  else if (connectivity == "ABC")
  {
    sendATCommand("AT+URAT=7,8,9", 4);                    //  Set GSM + CATM1 + NB-Iot Band
    sendATCommand("AT+URAT=7", 4);                        // Set CATM1  Band
    sendATCommand("AT+UMNOPROF?", 4); // Check Response if it is 3 set to 1
    sendATCommand("AT+UMNOPROF=1", 4); // SIM ICCID select which will select MNO profile
    sendATCommand("AT+CFUN=15", 120 ); // resetting to make setting effective(make 3 min if any issue)
    sendATCommand("AT+UMNOPROF?", 4); // Check whether setting is effective
    //sendATCommand("AT+UBANDMASK=0,2074", 20); // set for North America
    GSM.print("AT+UBANDMASK=0,");                   // Download the data in file
    GSM.print(bitmask);
    GSM.write("\r\n");
    printGSMToSerial(10);
    sendATCommand("AT+UBANDMASK?", 4); // Check whether setting is effective
  }

  //  sendATCommand("AT+CFUN=15", 40);
  sendATCommand("AT+URAT?", 2);                          // To check whether modem is set to LTE CAT M1/2G/LTE/

  // sendATCommand("AT+CGDCONT=1,\"IP\",\"bsnlnet\"", 10);     // Setting of APN BSNL
  //  sendATCommand("AT+CGDCONT=1,\"IP\",\"www\"", 10);     // Setting of APN BSNL
  //airtelgprs.com
  //sendATCommand("AT+CGDCONT=1,\"IP\",\"Airtelgprs.com\"", 10);     // Setting of APN Airtel

  char ftpAPN[100];
  Serial.println(apnName);                                                              // Make sure this is large enough to hold the final command string
  sprintf(ftpAPN, "AT+CGDCONT=1,\"IP\",\"%s\"", apnName);
  Serial.println(ftpAPN);
  sendATCommand(ftpAPN, 1);

  //    sendATCommand("AT+CFUN=15",40);                         // resetting to make setting effective(make 3 min if any issue)
  sendATCommand("AT+CSQ", 2);                             //Signal power, quality
  sendATCommand("AT+CGACT=1,1", 10);                      // for apn and its ip address
  sendATCommand("AT+CGATT?", 1);                       // 1: Attached to network; 0:detached)
  sendATCommand("AT+CGACT?", 1);   //120                  // refer 8.5.1 of application note; expected 1,1;
  sendATCommand("AT+CGDCONT?", 1);                    //find what apn is set to this simcard
  sendATCommand("AT+CGREG?", 2) ;                       // Check whether modem is registered to the network
  sendATCommand("AT+CGATT?", 2);                        // 1: Attached to network; 0:detached)   //GPRS is on or off(Default =1(ON))
  sendATCommand("AT+CGACT?", 1);   //120                  // refer 8.5.1 of application note; expected 1,1;
  sendATCommand("AT+CGREG?", 1);                       // Check whether modem is registered to the network
  cellular_rcv_time = "";
  sendATCommand("AT+CCLK?", 2);                        // getting network time to set rtc
  if (resp_st != "\0")
  {
    cellular_rcv_time = resp_st;
    Cellular_RCV_Time(cellular_rcv_time);                //  convert rcv network time to gmt time as per the regions (EX. 5:30)
    setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);            // sec min hr ,day/month/year
    delay(500);
    //   printLocalTime();
  }
  else
  {
    Serial.println("RTC not Set by cellular network");
    if ((com_sec != SEC) || (com_min != MIN) || (com_hr != HR))
    {
      Serial.println("RTC Mannualy Set");
      writeFile(SPIFFS, Mannual_rtc_enable_FILE, "1");
      SUBSTRACT_GMT_Time(HR, MIN, SEC, DAY, MONTH, YEAR);
      setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0); // sec min hr ,day/month/year
      delay(500);
      //    printLocalTime();
    }
    else
    {
      writeFile(SPIFFS, Mannual_rtc_enable_FILE, "1");
      delay(1000);
      Serial.println("Memory time set\n");
      readFile1(SPIFFS, TIME_STAMP);
      mannual_Timestamp(read_g_str_1);
      setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);                       // sec min hr ,day/month/year
    }
  }
}


void Cellular_RCV_Time(String rcv_time)
{
  int time_fragment = 0, Date_fragment = 0;
  int time_seperater = 0, date_seperater = 0;
  char hr_rcv[3] = "", min_rcv[3] = "", sec_rcv[3] = "", day_rcv[3] = "", month_rcv[3] = "", year_rcv[3] = "";
  int cl = 0, ck = 0, cj = 0, ct = 0, dk = 0, dj = 0, dt = 0;
  // cellular_rcv_time= "+CCLK: \"24/2/5,17:30:00+22\"";

  for ( cl = 0, ck = 0, cj = 0, ct = 0, dk = 0, dj = 0, dt = 0; rcv_time[cl]; cl++)
  {

    if (rcv_time[cl] == '"')
    {
      Date_fragment = 1;

    }
    else if (Date_fragment == 1)
    {
      if (rcv_time[cl] == '/')
      {
        date_seperater++;

      }
      else if (date_seperater == 0)
      {
        year_rcv[dk] = rcv_time[cl];
        dk++;
      }
      else if ( date_seperater == 1)
      {
        month_rcv[dt] =  rcv_time[cl];
        dt++;
      }
      else if (date_seperater == 2)
      {
        day_rcv[dj] =  rcv_time[cl];
        dj++;
      }
    }

    if (rcv_time[cl] == ',')
    {
      time_fragment = 1;
      Date_fragment = 0;
      day_rcv[dk] = '\0';
      month_rcv[dt] = '\0';
      year_rcv[dj] = '\0';
    }

    else if (time_fragment == 1)
    {
      if (rcv_time[cl] == ':')
      {
        time_seperater++;

      }
      else if (time_seperater == 0)
      {
        hr_rcv[ck] = rcv_time[cl];
        ck++;
      }
      else if (time_seperater == 1)
      {

        min_rcv[ct] = rcv_time[cl];
        ct++;
      }
      else if (time_seperater == 2)
      {
        if (rcv_time[cl] == '+')
        {
          hr_rcv[ck] = '\0';
          min_rcv[ct] = '\0';
          sec_rcv[cj] = '\0';

          break;
        }
        sec_rcv[cj] =  rcv_time[cl];
        cj++;
      }
    }
  }

  String hr_string  = String(hr_rcv);
  String min_string = String(min_rcv);
  String sec_string = String(sec_rcv);
  String day_string  = String(day_rcv);
  String month_string = String(month_rcv);
  String year_string = String(year_rcv);
  hr_cel = hr_string.toInt();
  min_cel = min_string.toInt();
  sec_cel = sec_string.toInt();
  day_cel = day_string.toInt();
  month_cel = month_string.toInt();
  year_cel = year_string.toInt();

  year_cel = year_cel + 2000;

  HR  = hr_cel;
  MIN = min_cel;
  SEC = sec_cel;
  DAY = day_cel;
  MONTH = month_cel;
  YEAR = year_cel;
  //  Serial.println("cellular rcv time.................... ");
  //  Serial.print("HR= ");
  //  Serial.println(HR);
  //  Serial.print("MIN= ");
  //  Serial.println(MIN);
  //  Serial.print("SEC= ");
  //  Serial.println(SEC);
  //  Serial.print("DAY= ");
  //  Serial.println(DAY);
  //  Serial.print("MONTH= ");
  //  Serial.println(MONTH);
  //  Serial.print("YEAR= ");
  //  Serial.println(YEAR);

  //  ADD_GMT_Time(hr_cel, min_cel, sec_cel, day_cel, month_cel, year_cel);
}


void ADD_GMT_Time(int hr_cel, int min_cel, int  sec_cel, int day_cel, int month_cel, int year_cel)
{
  min_cel = min_cel + GMT_MIN;

  if (min_cel >= 60)
  {
    MIN = min_cel - 60;
    HR = hr_cel + 1;
  }
  else if (min_cel < 60)
  {
    MIN = min_cel;
    HR = hr_cel;
  }

  HR = HR + GMT_HR;

  if (HR >= 23)
  {
    HR = HR - 23;
    DAY = day_cel + 1;
  }
  else
  {
    HR = HR;
    DAY = day_cel;
  }
  SEC = sec_cel;

  switch (month_cel)
  {
    case 1: No_of_Days_in_Month = 31;
      break;
    case 2: if (year_cel % 4 == 0)
      {
        No_of_Days_in_Month = 29;
      }
      else
      {
        No_of_Days_in_Month = 28;
      }
      break;
    case 3: No_of_Days_in_Month = 31;
      break;
    case 4: No_of_Days_in_Month = 30;
      break;
    case 5: No_of_Days_in_Month = 31;
      break;
    case 6: No_of_Days_in_Month = 30;
      break;
    case 7: No_of_Days_in_Month = 31;
      break;
    case 8: No_of_Days_in_Month = 31;
      break;
    case 9: No_of_Days_in_Month = 30;
      break;
    case 10: No_of_Days_in_Month = 31;
      break;
    case 11: No_of_Days_in_Month = 30;
      break;
    case 12: No_of_Days_in_Month = 31;
      break;
  }

  if (DAY > No_of_Days_in_Month)
  {
    DAY = 1;
    MONTH = month_cel + 1;
  }
  else
  {
    DAY = DAY ;
    MONTH = month_cel;
  }

  if (MONTH > 12)
  {
    MONTH = 1;
    YEAR = year_cel + 1;
  }
  else
  {
    MONTH = MONTH;
    YEAR = year_cel;
  }
  YEAR = YEAR;
}

void SUBSTRACT_GMT_Time(int hr_cel, int min_cel, int  sec_cel, int day_cel, int month_cel, int year_cel)
{
  min_cel = min_cel - GMT_MIN;

  if (min_cel < 0)
  {
    MIN = 60 + min_cel;
    HR = hr_cel - 1;
  }
  else if (min_cel >= 0)
  {
    MIN = min_cel;
    HR = hr_cel;
  }

  HR = HR - GMT_HR;

  if (HR < 0)
  {
    HR = HR + 24;
    DAY = day_cel - 1;
  }
  else
  {
    HR = HR;
    DAY = day_cel;
  }
  SEC = sec_cel;



  if (DAY < 1)
  {
    MONTH = month_cel - 1;
    if (MONTH < 1)
    {
      MONTH = 12;
      YEAR = year_cel - 1;
    }
    else
    {
      MONTH = MONTH;
      YEAR = year_cel;
    }

    switch (MONTH)
    {
      case 1: No_of_Days_in_Month = 31;
        break;
      case 2: if (year_cel % 4 == 0)
        {
          No_of_Days_in_Month = 29;
        }
        else
        {
          No_of_Days_in_Month = 28;
        }
        break;
      case 3: No_of_Days_in_Month = 31;
        break;
      case 4: No_of_Days_in_Month = 30;
        break;
      case 5: No_of_Days_in_Month = 31;
        break;
      case 6: No_of_Days_in_Month = 30;
        break;
      case 7: No_of_Days_in_Month = 31;
        break;
      case 8: No_of_Days_in_Month = 31;
        break;
      case 9: No_of_Days_in_Month = 30;
        break;
      case 10: No_of_Days_in_Month = 31;
        break;
      case 11: No_of_Days_in_Month = 30;
        break;
      case 12: No_of_Days_in_Month = 31;
        break;
    }
    DAY = No_of_Days_in_Month;
  }
  else
  {
    DAY = DAY ;
    MONTH = month_cel;
    YEAR = year_cel;
  }
  YEAR = YEAR;

  //  Serial.println("SubStracted time.................... ");
  //  Serial.print("HR= ");
  //  Serial.println(HR);
  //  Serial.print("MIN= ");
  //  Serial.println(MIN);
  //  Serial.print("SEC= ");
  //  Serial.println(SEC);
  //  Serial.print("DAY= ");
  //  Serial.println(DAY);
  //  Serial.print("MONTH= ");
  //  Serial.println(MONTH);
  //  Serial.print("YEAR= ");
  //  Serial.println(YEAR);
}

/********************* HTTP Function **********************/
void Cellular_HTTP_Post()
{
  // sendATCommand("AT+UHTTP=0,1,\"http://103.235.105.138:8082/asset/tracking/get_tracking_details_of_tag_bulk\"", 2);                // url for bulk data post

  Serial.print("total_tags_scan_time:-");
  Serial.println(total_tags_scan_time);
  if (total_tags_scan_time > (RTC_TIME_RESET * 1800000))               //3600000
  {
    Serial.println("RTC UPDATE  by Cellular network..........................");
    writeFile(SPIFFS, scan_time_file, "0");
    cellular_rcv_time = "";
    sendATCommand("AT+CCLK?", 2);                        // getting network time to set rtc

    if (resp_st != "\0")
    {
      Serial.println("time set in cellular\n");
      Serial.print("resp_st");
      Serial.println(resp_st);
      cellular_rcv_time = resp_st;
      Cellular_RCV_Time(cellular_rcv_time);                //  convert rcv network time to gmt time as per the regions (EX. 5:30)
      setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);            // sec min hr ,day/month/year
      delay(500);
      printLocalTime();
    }
    else
    {
      Serial.println("Memory time set in cellular\n");
      readFile1(SPIFFS, TIME_STAMP);
      mannual_Timestamp(read_g_str_1);
      setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);                       // sec min hr ,day/month/year
    }
  }


  SD_String_receive = "";
  SD_readFile(SD, Write_pointer);                                      // read current file pointer
  if (SD_String_receive != "\0")
  {
    SD_File_No_Write = SD_String_receive.toInt();                          // assign new file no. pointer
  }
  else
  {
    SD_File_No_Write = 0;
  }

  if (SD_File_No_Write > 0)
  {
    SD_Read_enable = 1;
  }
  else
  {
    sendATCommand("AT+CMEE=2", 1);
    // sendATCommand("AT+CGDCONT=1,\"IP\",\"Airtelgprs.com\"", 2);     // Setting of APN Airtel

    char ftpAPN[100];
    Serial.println(apnName);                                                              // Make sure this is large enough to hold the final command string
    sprintf(ftpAPN, "AT+CGDCONT=1,\"IP\",\"%s\"", apnName);
    Serial.println(ftpAPN);
    sendATCommand(ftpAPN, 1);

    sendATCommand("AT+CGACT=1,1", 2);                      // for apn and its ip address
    //    sendATCommand("AT+CGATT?", 1);                       // 1: Attached to network; 0:detached)
    //    sendATCommand("AT+CGACT?", 1);   //120                  // refer 8.5.1 of application note; expected 1,1;
    //    sendATCommand("AT+CGDCONT?", 1);                    //find what apn is set to this simcard
    sendATCommand("AT+UDELFILE=\"json\"", 1);
    int Data_len = (Data_buffer.length() + 1);
    GSM.print("AT+UDWNFILE=\"json\",");                   // Download the data in file
    GSM.print(Data_len);
    GSM.write("\r\n");
    printGSMToSerial(2);
    sendATCommand(Data_buffer, 3);                        // the actual data store in file
    sendATCommand("AT+URDFILE=\"json\"", 3);             // read the data stored in file
    sendATCommand("AT+UHTTP=0", 2);                      // set profile id 0
    sendATCommand("AT+UHTTP=0,0,\"103.235.105.138\"", 4); // set ip
    sendATCommand("AT+UHTTP=0,5,8082", 5);               // set port no.
    sendATCommand("AT+UHTTPC=0,4,\"/asset/tracking/get_tracking_details_of_tag_bulk\",\"result\",\"json\",4", 5); // command to post post txt file
    sendATCommand("AT+URDFILE=\"result\"", 5);           // stored result in this file
    String ret;
    ret = strstr(resp_st.c_str(), "200");
    if (ret != "\0")
    {
      Network_connected_flag = 1;
    }
    else
    {
      Network_connected_flag = 0;
    }
    sendATCommand("AT+UDELFILE=\"result\"", 2);          // delete file
    Serial.print("Network_connected_flag ");
    Serial.println(Network_connected_flag);
    if (Network_connected_flag == 1)
    {
      ;
    }
    else
    {
      SD_String_receive = "";
      SD_readFile(SD, Write_pointer);                                      // read current file pointer
      if (SD_String_receive != "\0")
      {
        SD_File_No_Write = SD_String_receive.toInt();                          // assign new file no. pointer
      }
      else
      {
        SD_File_No_Write = 0;
      }
      Make_file_name(SD_File_No_Write);                                 // make new file for current data store
      SD_File_No_Write++;                                               // file no. increament
      String SD_No_Write = String(SD_File_No_Write);
      writeFile(SD, Write_pointer, SD_No_Write.c_str());              // file no. pointer increament and write in file
      Serial.print("Buffer in cellular connected but Network_connected_flag is not set so store in sd file:= ");
      writeFile(SD, SD_File, Data_buffer.c_str());                     // write current asset data in new file
      file_size(SD, SD_File);                                           // file size
    }
  }

  if (SD_Read_enable == 1)                                           // if this flag is enable then only file can be read from SD card
  {
    SD_Read_enable = 0;
    deleteFile(SPIFFS, Asset_File);

    SD_String_receive = "";
    SD_readFile(SD, Read_pointer);                                   // read current file pointer
    if (SD_String_receive != "\0")
    {
      SD_File_No_Read = SD_String_receive.toInt();                      // assign new file no. pointer
    }
    else
    {
      SD_File_No_Read = 0;
    }

    while (SD_File_No_Read < SD_File_No_Write)
    {
      Make_file_name(SD_File_No_Read);                                   // make new file for current data read
      SD_String_receive = "";
      SD_readFile(SD, SD_File);
      Serial.print("Buffer in cellular connectivity and send to sever read file from sd card 1 by 1 := ");
      sendATCommand("AT+CMEE=2", 1);
      //sendATCommand("AT+CGDCONT=1,\"IP\",\"Airtelgprs.com\"", 2);     // Setting of APN Airtel

      char ftpAPN[100];
      Serial.println(apnName);                                                              // Make sure this is large enough to hold the final command string
      sprintf(ftpAPN, "AT+CGDCONT=1,\"IP\",\"%s\"", apnName);
      Serial.println(ftpAPN);
      sendATCommand(ftpAPN, 1);

      sendATCommand("AT+CGACT=1,1", 2);                      // for apn and its ip address
      sendATCommand("AT+UDELFILE=\"json\"", 1);
      int Data_len = (SD_String_receive.length() + 1);
      GSM.print("AT+UDWNFILE=\"json\",");                   // Download the data in file
      GSM.print(Data_len);
      GSM.write("\r\n");
      printGSMToSerial(2);
      //GSM.print("")
      sendATCommand(SD_String_receive, 2);                        // the actual data store in file
      sendATCommand("AT+URDFILE=\"json\"", 4);             // read the data stored in file
      sendATCommand("AT+UHTTP=0", 2);                      // set profile id 0
      sendATCommand("AT+UHTTP=0,0,\"103.235.105.138\"", 2); // set ip
      sendATCommand("AT+UHTTP=0,5,8082", 5);               // set port no.
      sendATCommand("AT+UHTTPC=0,4,\"/asset/tracking/get_tracking_details_of_tag_bulk\",\"result\",\"json\",4", 5); // command to post post txt file
      sendATCommand("AT+URDFILE=\"result\"", 5);           // stored result in this file
      String ret;
      ret = strstr(resp_st.c_str(), "200");

      if (ret != "\0")
      {
        Network_connected_flag = 1;
      }
      else
      {
        Network_connected_flag = 0;
      }
      sendATCommand("AT+UDELFILE=\"result\"", 2);          // delete file
      Serial.print("Network_connected_flag ");
      Serial.println(Network_connected_flag);
      if (Network_connected_flag == 1)
      {
        deleteFile(SD, SD_File);
        SD_File_No_Read++;                                               // file no. increament
        String SD_No_Read = String(SD_File_No_Read);
        writeFile(SD, Read_pointer, SD_No_Read.c_str());             // file no. pointer increament and write in file
      }
      else
      {
        Serial.println(" ");
        Serial.println(" ");
        Serial.println("break in cellular sd file loop because Network_connected_flag not set ");
        Serial.println(" ");
        Serial.println(" ");
        break;
      }
      delay(1000);
    }
    if (SD_File_No_Read == SD_File_No_Write)
    {
      SD_File_No_Read = 0;
      SD_File_No_Write = 0;
      SD_Read_enable = 0;
      current_data_flag = 1;
      Serial.println("in cellular all files read  then clear all pointers");
    }
    if (current_data_flag == 1)
    {
      current_data_flag = 0;
      sendATCommand("AT+CMEE=2", 1);
      // sendATCommand("AT+CGDCONT=1,\"IP\",\"Airtelgprs.com\"", 2);     // Setting of APN Airtel

      char ftpAPN[100];
      Serial.println(apnName);                                                              // Make sure this is large enough to hold the final command string
      sprintf(ftpAPN, "AT+CGDCONT=1,\"IP\",\"%s\"", apnName);
      Serial.println(ftpAPN);
      sendATCommand(ftpAPN, 1);

      sendATCommand("AT+CGACT=1,1", 2);                      // for apn and its ip address
      sendATCommand("AT+UDELFILE=\"json\"", 1);
      int Data_len = (Data_buffer.length() + 1);
      GSM.print("AT+UDWNFILE=\"json\",");                   // Download the data in file
      GSM.print(Data_len);
      GSM.write("\r\n");
      printGSMToSerial(2);
      sendATCommand(Data_buffer, 1);                        // the actual data store in file
      sendATCommand("AT+URDFILE=\"json\"", 3);             // read the data stored in file
      sendATCommand("AT+UHTTP=0", 2);                      // set profile id 0
      sendATCommand("AT+UHTTP=0,0,\"103.235.105.138\"", 2); // set ip
      sendATCommand("AT+UHTTP=0,5,8082", 5);               // set port no.
      sendATCommand("AT+UHTTPC=0,4,\"/asset/tracking/get_tracking_details_of_tag_bulk\",\"result\",\"json\",4", 5); // command to post post txt file
      sendATCommand("AT+URDFILE=\"result\"", 5);           // stored result in this file
      String ret;
      ret = strstr(resp_st.c_str(), "200");

      if (ret != "\0")
      {
        Network_connected_flag = 1;
      }
      else
      {
        Network_connected_flag = 0;
      }
      sendATCommand("AT+UDELFILE=\"result\"", 2);          // delete file
      Serial.print("Network_connected_flag ");
      Serial.println(Network_connected_flag);

      if (Network_connected_flag == 1)
      {
        ;
      }
      else
      {
        SD_String_receive = "";
        SD_readFile(SD, Write_pointer);                                      // read current file pointer
        if (SD_String_receive != "\0")
        {
          SD_File_No_Write = SD_String_receive.toInt();                          // assign new file no. pointer
        }
        else
        {
          SD_File_No_Write = 0;
        }
        Make_file_name(SD_File_No_Write);                                 // make new file for current data store
        SD_File_No_Write++;                                               // file no. increament
        String SD_No_Write = String(SD_File_No_Write);
        writeFile(SD, Write_pointer, SD_No_Write.c_str());              // file no. pointer increament and write in file
        Serial.print("Buffer in cellular connected but Network_connected_flag not set so store in sd file:= ");
        Serial.println(Data_buffer);
        writeFile(SD, SD_File, Data_buffer.c_str());                     // write current asset data in new file
        file_size(SD, SD_File);                                           // file size
      }
    }
    else
    {
      Make_file_name(SD_File_No_Write);                                 // make new file for current data store
      SD_File_No_Write++;                                               // file no. increament
      String SD_No_Write = String(SD_File_No_Write);
      writeFile(SD, Write_pointer, SD_No_Write.c_str());              // file no. pointer increament and write in file
      Serial.print("Current buffer stored in sd file beacause all files of sd card not able to send to server := ");
      Serial.println(Data_buffer);
      writeFile(SD, SD_File, Data_buffer.c_str());                     // write current asset data in new file
      file_size(SD, SD_File);                                           // file size
    }
  }
}

/***************** MQtt Functions ************************/
void MQTT_cloud_connect(void)
{
  //certificate checking and import from sara flash.
  sendATCommand("AT+ULSTFILE=0", 2);                                       // check list of files
  sendATCommand("AT+ULSTFILE=2,\"SFSRootCAG2_CA root.crt\"", 2);            //if resp == +ULSTFILE : 1188 \n OK     then certi is available
  sendATCommand("AT+ULSTFILE=2,\"embel_iot-certificate.pem.crt\"", 2);      //if resp == +ULSTFILE: 1224 \n OK  then certi is available
  sendATCommand("AT+ULSTFILE=2,\"embel_iot-private.pem.key\"", 2);          //if resp == +ULSTFILE: 1679 \n OK  then certi is available

  //importing all certi.
  sendATCommand("AT+USECMNG=1,0,\"SFSRootCAG2_CA root.crt\",\"SFSRootCAG2_CA root.crt\"", 2);   //import CA certificate
  sendATCommand("AT+USECMNG=1,1,\"embel_iot-certificate.pem.crt\",\"embel_iot-certificate.pem.crt\"", 2);   //import CC certificate
  sendATCommand("AT+USECMNG=1,2,\"embel_iot-private.pem.key\",\"embel_iot-private.pem.key\"", 2);   //import Private Key

  sendATCommand("AT+USECMNG=3", 2);
  sendATCommand("AT+ULSTFILE=0", 2);                                       // check list of files

  // set sara in hex mode and its configuration
  sendATCommand("AT+UDCONF=1,1", 2);                                          // enable HEX mode
  sendATCommand("AT+USECPRF=0", 2);
  sendATCommand("AT+USECPRF=0,0,1", 2);                                       //Set the certificate validation level 1
  sendATCommand("AT+USECPRF=0,1,0", 2);                                       //Set TLS version to any
  sendATCommand("AT+USECPRF=0,2,0", 2);                                       //Set the trusted root cert internal name
  sendATCommand("AT+USECPRF=0,3,\"SFSRootCAG2_CA root.crt\"", 2);             // set the trusted client certificate internal name
  sendATCommand("AT+USECPRF=0,5,\"embel_iot-certificate.pem.crt\"", 2);      //set the trusted private key internal name
  sendATCommand("AT+USECPRF=0,6,\"embel_iot-private.pem.key\"", 2);
}
void Cellular_MQTT_Publish()                                                // cellular mqtt publish data function
{

  /////////////////////////////////////////////////////
  //create socket and socket security parameters
  sendATCommand("AT+USOCR=6", 5);                                              // creating socket for TCP(Protocol)  //(6= tcp, local port num)
  sendATCommand("AT+USOSEC=0,1,0", 5);                                         //(=0,1,0) 0= Socket identifie;
  //  sendATCommand("AT+USOSO=0,6,1,1", 3);
  //1= enable the SSL/TLS on the socket;
  //0=Defines the USECMNG profile which specifies the SSL/TLS properties to be used for the
  //SSL/TLS connection. The range goes from 0 to 4.
  sendATCommand("AT+USOCO=0,\"a1udxktg9dbtpj-ats.iot.ap-south-1.amazonaws.com\",8883", 5);                           // connection paramerters of AWS_IOT and port number.
  sendATCommand("AT+USOWR=0,2,\"C000\"", 10);                                                                        //PING Request message
  sendATCommand("AT+USORD=0,2", 10);
  sendATCommand("AT+USOWR=0,31,\"101D00064D514973647003020E10000F7468696E672F656D62656C2D696F74\"", 10);             //CONNECT message
  sendATCommand("AT+USORD=0,4", 5);                                                                                  //CONNACK
  Mqtt_Cellular_String_split();                                                                                // asset data send to cloude 1 by 1 in this fun
  if (String(receivedChars) == "OK")
  {
    Network_connected_flag = 1;
  }
  if ((SD_Read_enable == 1) && (Network_connected_flag == 1))
  {
    Network_connected_flag = 0;
    file_size(SPIFFS, Asset_File);
    deleteFile(SPIFFS, Asset_File);
    while (SD_File_No_Read <= SD_File_No_Write)
    {
      Make_file_name(SD_File_No_Read);
      readFile1(SD, SD_File);
      Mqtt_Cellular_String_split();
      deleteFile(SD, SD_File);
      SD_File_No_Read++;
    }
    if (SD_File_No_Read > SD_File_No_Write)
    {
      SD_File_No_Read = 0;
      SD_File_No_Write = 0;
      SD_Read_enable = 0;
    }
  }

  if (Network_connected_flag == 1)
  {
    Network_connected_flag = 0;
    file_size(SPIFFS, Asset_File);
    deleteFile(SPIFFS, Asset_File);
  }
  else
  {
    SD_Read_enable = 1;
    Make_file_name(SD_File_No_Write);
    SD_File_No_Write++;
    appendFile(SD, SD_File, read_g_str_1.c_str());
    file_size(SD, SD_File);
  }
  sendATCommand("AT+USOCL=0", 5);// close socket
}

/***************** ASCII TO Hex   &   DEC TO HEX  Fun ********/
String AtoH(String input)                                                   // ascii hex converter function
{
  Serial.println(input);
  String output = "";
  byte high_b = 0, low_b = 0, s_byte = 0;
  for ( i = 0; i < input.length(); i++)
  {
    s_byte = input[i];
    high_b = s_byte >> 4;
    low_b = s_byte & 15;
    if (high_b > 9)
    {
      high_b = 'A' + (high_b - 10);
    }
    else
    {
      high_b = '0' + high_b;
    }
    if (low_b > 9)
    {
      low_b = 'A' + (low_b - 10);
    }
    else
    {
      low_b = '0' + low_b;
    }
    output = output + (char)high_b + (char)low_b;
    s_byte = 0;
    high_b = 0;
    low_b  = 0;
  }
  Serial.println(output);
  return output;
}

String DectoHex(int n)                                          // Dec To Hex Function
{
  int num_decimal = n , remainder , quotient ;
  int a = 1 , b , var ;
  char hexanum_decimal[ 100 ] = "", data1[100] = "" ;
  Data2 = "";
  quotient = num_decimal ;
  while ( quotient != 0 ) {
    var = quotient % 16 ;
    if ( var < 10 )
      var = var + 48 ;
    else
      var = var + 55 ;
    hexanum_decimal[ a++ ] = var ;
    quotient = quotient / 16;
  }
  Serial.println(num_decimal ) ;
  int i = 0;
  for ( b = a - 1, i = 0 ; b > 0 ; b--, i++ )
  {
    data1[i] = hexanum_decimal[b];
  }
  Serial.println(data1);
  Data2 = String(data1);
  memset(data1, 0, sizeof(data1));
  memset(hexanum_decimal, 0, sizeof(hexanum_decimal));
  return Data2;
}
/************************* BQ functions ************************/
void sendATCommand(String cmd, int wait)                               //  AT command Function
{
  GSM.print(cmd);
  GSM.write("\r\n");
  Serial.println(cmd);
  printGSMToSerial(wait);
}

void printGSMToSerial(int sec)                                       // Delay after At Command
{
  resp_st = "";
  char resp_char = 0;
  for (int i = 0; i < 1000 ; i++)
  {
    for (; GSM.available() > 0;)
    {
      resp_char = GSM.read();
      resp_st = resp_st + resp_char;
    }
    delay(sec);
  }
  Serial.println(resp_st);
}

void power_off()                                                    // Power off function of gateway board
{
  digitalWrite(pwr_uc, HIGH);
  delay(1000);
  digitalWrite(pwr_uc, LOW);
  delay(4000);
  //  delay(2500);
  digitalWrite(pwr_uc, HIGH);
}

void power_on()                                                     // Power on function of gateway board
{
  Serial.println("\nPower ON Starting");
  // POWER ON  //HIGH LOW 150 HIGH
  //digitalWrite(25, HIGH);
  //delay(1000);
  digitalWrite(pwr_uc, LOW);
  //delay(150);
  delay(250);
  digitalWrite(pwr_uc, HIGH);
}

void power_reset()                                                // Reset function of gateway board
{
  digitalWrite(pwr_uc, HIGH);
  delay(1000);
  digitalWrite(pwr_uc, LOW);
  digitalWrite(rst_uc, LOW);
  delay(10000);
  digitalWrite(pwr_uc, HIGH);
}

/************************ SPIFFS FUN ****************************/

void Make_file_name(int pos)
{
  const char T_file_name[] = "/000.txt";
  int temp = pos;
  if (pos < 10)
  {
    SD_File[3] = pos + 48;
  }
  else if (pos < 100)
  {
    SD_File[3] = (pos % 10) + 48;
    pos = pos / 10;
    SD_File[2] = pos + 48;
  }
  else if (pos > 99)
  {
    SD_File[3] = (pos % 10) + 48;
    pos = pos / 10;
    SD_File[2] = (pos % 10) + 48;
    SD_File[1] = (pos / 10) + 48;
  }
  Serial.println(SD_File);
}


void write_in_file(String u_data, int data_len)                      // Write in file function of Spiff
{
  // int wr_pos = write_position;
  char data_arr[500] = {0};
  u_data.toCharArray(data_arr, u_data.length() + 1);
  //Make_file_name(wr_pos);
  file_size(SPIFFS, Parameter_set_File);
  writeFile(SPIFFS, Parameter_set_File, data_arr);
  file_size(SPIFFS, Parameter_set_File);
}

void file_size(fs::FS & fs, const char * path)                    // fun for file size
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  Serial.print("  FILE: ");
  Serial.print(file.name());
  Serial.print("\tSIZE: ");
  Serial.println(file.size());
  fs_size = file.size();
  Serial.println(fs_size);
}

void listDir(fs::FS & fs, const char * dirname, uint8_t levels) // list of file in directory
{
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void SD_readFile(fs::FS & fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }
  SD_String_receive = "";
  Serial.println("- read from file:");
  if (file.available()) {
    SD_String_receive = file.readString();
  }
  Serial.print("SD_String_receive = ");
  Serial.println(SD_String_receive);
}

void readFile_2(fs::FS & fs, const char * path)                        // read fun for readFile_2 which is modbus file
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }
  read_g_str = "";
  Serial.println("- read from file:");
  while (file.available()) {
    //  Serial.println("in rd file");
    read_g_str = read_g_str + (char)file.read();

  }
  Serial.print("read_g_str = ");
  Serial.println(read_g_str);
  mry_data = read_g_str;
  modbus_mem_string_split();                            // split the spiffs data into actual variable
}

void readFile1(fs::FS & fs, const char * path)                  // read fun for readFile1
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }
  read_g_str_1 = "";
  Serial.println("- read from file:");
  if (file.available()) {
    read_g_str_1 = file.readString();
  }
  Serial.print("read_g_str_1 = ");
  Serial.println(read_g_str_1);

}

void readFile(fs::FS & fs, const char * path)              //Read file Parameter set data from bt app.
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }
  read_g_str = "";
  Serial.println("- read from file:");
  while (file.available()) {
    //  Serial.println("in rd file");
    read_g_str = read_g_str + (char)file.read();
  }
  mry_data = read_g_str;
  Spiff_string_split(mry_data);                                      // slit data recieved from file
  if (connectivity != "\0")                                 // if cellular connectivity is set then  cellular_connectivity_enable=1
  {
    cellular_connectivity_enable = 1;
  }
  else                                                     // if cellular connectivity is not set then  cellular_connectivity_enable=0
  {
    cellular_connectivity_enable = 0;
  }

  BandMask();

  if (HTTP_MQTT == 0 && cellular_connectivity_enable == 1)         // condition to enable mqtt function
  {
    MQTT_connect_flag = 1;
  }
  Time_stamp = String(time_stamp);
  timeStamp();
}
void writeFile(fs::FS & fs, const char * path, const char * message) // Write file fun
{
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
}

void appendFile(fs::FS & fs, const char * path, const char * message)  // Append data function in file
{
  Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("- message appended");
  } else {
    Serial.println("- append failed");
  }
}

void deleteFile(fs::FS & fs, const char * path) {                  //  Delete file function
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

/***************** String spilt function ************/
void Spiff_string_split(String mry_data)                                                // segrigate data from SPIFFs Memory
{
  count = 0;
  Serial.print("mry_data=");
  Serial.println(mry_data);
  for (spiff_z = 0, spiff_a = 0, spiff_b = 0; mry_data[spiff_z] != '\0'; spiff_z++)
  {
    while (mry_data[spiff_z] == '/')                // if '/' found in string count is increament
    {
      spiff_z++;
      count++;
      spiff_a = 0;
    }

    if (count == 1)                              // for count 1 wifi ssid is regrigate
    {
      time1[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 2)                        // for count 2 wifi password is regrigate
    {
      time2[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 3)                        // for count 3 connectivity_type of cellulat network is segrigate
    {
      time3[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 4)                        // for count 3 connectivity_type of cellulat network is segrigate
    {
      GMT_Time_arr[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 5)                      // for count 4 http mqtt set variable segrigate
    {
      bands[spiff_b] = mry_data[spiff_z];
      spiff_b++;
    }
    else if (count == 6)                      // for count 5 ble enable flag is segrigate
    {
      //      if (band_c == 0)
      //      {
      //
      //        band_c++;
      //      }
      time4[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 7)                      // for count 6 ble config devices are set
    {
      time5[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 8)                      // for count 6 ble config devices are set
    {
      time6[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 9)                      // for count 6 ble config devices are set
    {
      time_stamp[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 10)                      // for count 6 ble config devices are set
    {
      time7[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
    else if (count == 11)                      // for count 6 ble config devices are set
    {
      time8[spiff_a] = mry_data[spiff_z];
      spiff_a++;
    }
  }
  bands[spiff_b] = ',';
  int GMT_Fragment_flag = 0;
  for (int v = 0, w = 0; GMT_Time_arr[v]; v++)
  {
    if (GMT_Time_arr[v] == ':')
    {
      GMT_Fragment_flag = 1;
      w = 0;
      v++;
    }

    if (GMT_Fragment_flag == 1)
    {
      GMT_MIN_arr[w] = GMT_Time_arr[v];
      w++;
    }
    else
    {
      GMT_HR_arr[w] = GMT_Time_arr[v];
      w++;
    }
  }

  // all below Segrigated variable store in string variables
  String GMT_HR_str = String(GMT_HR_arr);
  String GMT_MIN_str = String(GMT_MIN_arr);
  ssid_M = String(time1);
  password_M = String(time2);
  arr_M = String(time3);
  HTTP_MQTT_M = String(time4);
  ble_enable_M = String(time5);
  config_device_M = String(time6);
  TIME_TO_SLEEP_M = String(time7);
  modbus_enable_M = String(time8);
  //  time_stamp_M = String(time8);

  // clear the char array variable for new data to store
  memset(time1, 0, sizeof(time1));
  memset(time2, 0, sizeof(time2));
  memset(time3, 0, sizeof(time3));
  memset(time4, 0, sizeof(time4));
  memset(time5, 0, sizeof(time5));
  memset(time6, 0, sizeof(time6));
  memset(time7, 0, sizeof(time7));
  memset(time8, 0, sizeof(time8));

  // memset(time8, 0, sizeof(time8));
  // All variable store in actual variable which are used for main code
  WiFi_ssid = String(ssid_M);
  WiFi_password = String(password_M);
  connectivity = String(arr_M);
  HTTP_MQTT = HTTP_MQTT_M.toInt();
  ble_enable = ble_enable_M.toInt();
  config_device = config_device_M.toInt();
  TIME_TO_SLEEP = TIME_TO_SLEEP_M.toInt();
  modbus_enable = modbus_enable_M.toInt();
  GMT_HR = GMT_HR_str.toInt();
  GMT_MIN = GMT_MIN_str.toInt();
}

void BandMask()
{
  Band_temp2 = 0;                                      // clear variable
  for (int Band_i = 0; bands[Band_i]; Band_i++)
  {
    if (bands[Band_i] == ',')                         // ',' separater of bands Ex, 2,4,5   after ',' next band store in char array
    {
      Band_j = 0;
      band_flag = 1;
    }
    else                                             // this loop for multiple char store in one  char array
    {
      Band_b[Band_j] = bands[Band_i];               // Store new data in char array
      Band_j++;
    }
    if (band_flag == 1)                            // this flag for store new data to int variable
    {
      band_flag = 0;
      Band_temp = atoi(Band_b);                   // char array to int conversion
      memset(Band_b, 0, sizeof(Band_b));          // clear array for next data
      Band_temp1 = pow(2, Band_temp - 1);         // store power of band in Band_temp variable
      Band_temp2 = Band_temp1 + Band_temp2;      // add Pevious Data + new data for summing of bands
    }
  }
  bitmask = Band_temp2;                         // Store in bitmask to set Bandmask
  Serial.print("bitmask= ");
  Serial.println(bitmask);
}

void timeStamp()
{
  Fragment_flag = 0;

  for (i = 0, j = 0; Time_stamp[i] != '\0'; i++)          // this loop for time stamp segrigation
  {
    if (Time_stamp[i] == ',')                            // ',' used for seperation after ',' data will strore in new variable
    {
      Fragment_flag++;                                  // after  ',' this flag will always inclreament
      i++;
      j = 0;
      delay(1);
    }

    if (Fragment_flag == 0)                             // this condition for sec
    {
      Sec[j] = Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 1)                      // this condition for min
    {
      Min[j] = Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 2)                     // this condition for Hour
    {
      Hr[j] = Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 3)                     // this condition for day
    {
      Day[j] = Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 4)                   // this  condition for month
    {
      Month[j] = Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 5)                  // this condition for year
    {
      Year[j] = Time_stamp[i];
      j++;
    }
  }
  /**********  all char array will String variable ****************/
  sec = String(Sec);
  Serial.print("sec in Time Stamp= ");
  Serial.println(sec);
  minit = String(Min);
  hr = String(Hr);
  daY = String(Day);
  months = String(Month);
  years = String(Year);
  /**********  all String will int variable ****************/
  SEC = sec.toInt();
  MIN = minit.toInt();
  HR = hr.toInt();
  DAY = daY.toInt();
  MONTH = months.toInt();
  YEAR = years.toInt();

  Serial.print("YEAR in Time Stamp= ");
  Serial.println(YEAR);
}

void mannual_Timestamp(String rcv_TimeStamp)
{
  String Dummy_Time_stamp = rcv_TimeStamp;
  int t1 =  Dummy_Time_stamp.length();
  //    2022-10-14 17:17:42
  Fragment_flag = 0;
  for (i = 0, j = 0; Dummy_Time_stamp[i] != '\0'; i++)          // this loop for time stamp segrigation
  {
    if (Dummy_Time_stamp[i] == '-' || Dummy_Time_stamp[i] == ':' || Dummy_Time_stamp[i] == ' ')                            // ',' used for seperation after ',' data will strore in new variable
    {
      Fragment_flag++;                                  // after  ',' this flag will always inclreament
      i++;
      j = 0;
      space_flag = 0;
    }

    if (Fragment_flag == 0)                             // this condition for sec
    {
      Year[j] = Dummy_Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 1)                      // this condition for min
    {
      Month[j] = Dummy_Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 2)                     // this condition for Hour
    {
      Day[j] = Dummy_Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 3)                     // this condition for day
    {
      Hr[j] = Dummy_Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 4)                   // this  condition for month
    {
      Min[j] = Dummy_Time_stamp[i];
      j++;
    }
    else if (Fragment_flag == 5)                  // this condition for year
    {
      Sec[j] = Dummy_Time_stamp[i];
      j++;
    }
  }
  /**********  all char array will String variable ****************/
  sec = String(Sec);
  minit = String(Min);
  hr = String(Hr);
  daY = String(Day);
  months = String(Month);
  years = String(Year);
  /**********  all String will int variable ****************/
  SEC = sec.toInt();
  MIN = minit.toInt();
  HR = hr.toInt();
  DAY = daY.toInt();
  MONTH = months.toInt();
  YEAR = years.toInt();
  Serial.println("Mannual_Time_stamp set");
  Serial.println(" ");

  Serial.print("HR= ");
  Serial.println(HR);
  Serial.print("MIN= ");
  Serial.println(MIN);
  Serial.print("SEC= ");
  Serial.println(SEC);
  Serial.print("DAY= ");
  Serial.println(DAY);
  Serial.print("MONTH= ");
  Serial.println(MONTH);
  Serial.print("YEAR= ");
  Serial.println(YEAR);
}
/*******************  Mqtt functions  *********************/
void String_split_MQTT_WiFI(AWS_IOT * myiot)            // asset String Split function for tag data send to cloud  1 by 1.  through WiFI
{
  end_flag = 0;
  n = 0;
  i = 0;
  while (end_flag == 0)                                      // this loop will rotate till all tag data get successfully senf to cloud
  {

    for (i = n, j = 0; read_g_str_1[i]; i++)
    {
      if (read_g_str_1[i] == ']')                           // if strings end with ']' then end flag set
      {
        end_flag = 1;
      }
      else if (a[i] == '[')                                // this condition for to eliminate '[' from actual json data
      {
        ;
      }
      else if ((read_g_str_1[i] == '}') && (read_g_str_1[i + 1] == ',')) // if we get '}' anf ',' then single tag data is segrigated from Bulk data
      {
        Serial.println("abcd");
        b[j] = '}';
        j++;
        i = i + 2;
        n = i;                                                          // this for next tag data initialization
        break;                                                          // if one data is send then exit the loop
      }
      else
      {
        b[j] = read_g_str_1[i];                                         // tag data store in buffer 'b'
        j++;
      }
    }

    Serial.print("read_g_str= ");
    Serial.println(read_g_str_1);
    Serial.print("b= ");
    Serial.println(b);

    for (i = 0, j = 0; b[i]; i++)                                        // store data in payload which send to cloud
    {
      payload[j] = b[i];
      j++;
    }
    aws_data(myiot);
    memset(b, 0, sizeof(b));
    delay(100);
  }
}


void Mqtt_Cellular_String_split()                                // asset String Split function for tag data send to cloud  1 by 1.  through Cellular
{
  end_flag = 0;
  n = 0;
  i = 0;
  //  read_g_str_1=a;
  while (end_flag == 0)                                                // this loop will rotate till all tag data get successfully senf to cloud
  {
    for (i = n, j = 0; read_g_str_1[i]; i++)
    {
      if (read_g_str_1[i] == ']')                                      // if strings end with ']' then end flag set
      {
        end_flag = 1;
      }
      else if (read_g_str_1[i] == '[')                                // this condition for to eliminate '[' from actual json data
      {
        ;
      }
      else if ((read_g_str_1[i] == '}') && (read_g_str_1[i + 1] == ',')) // if we get '}' anf ',' then single tag data is segrigated from Bulk data
      {
        Serial.println("abcd");
        b[j] = '}';
        j++;
        i = i + 2;
        n = i;                                                          // this for next tag data initialization
        break;                                                          // if one data is send then exit the loop
      }
      else
      {
        b[j] = read_g_str_1[i];                                         // tag data store in buffer 'b'
        j++;
      }
    }

    Serial.print("read_g_str_1= ");
    Serial.println(read_g_str_1);
    Serial.print("b= ");
    Serial.println(b);

    Msg = String(b);                                                   // single asset data store in 'msg' buffer
    String len_1, len_2;
    len_1 = DectoHex(Topic_name.length());                             // topic name length conver into hex
    Pub_Topic_len = len_1;                                             // Pub_Topic_len it will add to AT coomand
    Pub_Topic_name = AtoH(Topic_name);                                 // topic name convert into hex
    RL_data = Topic_name + Msg;                                        // addition of topic_name+Msg
    len_2 = DectoHex(RL_data.length() + 4);                            // Convert RL(remaining length) into hex
    Pub_RL = len_2;                                                    // Pub_RL which will add to AT command
    Pub_Msg = AtoH(Msg);                                               // Msg(Actual Data) Convert into Hex
    int Total_len = RL_data.length() + 6;                              // Total Lenght packet
    String Pub_Total_len = String(Total_len);                          // Total_len convert into String format

    /*******************  Below is a customise AT Command For Dynamic Data Publish to cloud************* */
    GSM.print("AT+USOWR=0,");                                          // Write command Start with this which is used to publish data to Cloud
    GSM.print(Pub_Total_len);
    GSM.print(",\"32");
    GSM.print(Pub_RL);
    GSM.print("00");
    GSM.print(Pub_Topic_len);
    GSM.print(Pub_Topic_name);
    GSM.print("0001");
    GSM.print(Pub_Msg);
    GSM.print("\"");
    GSM.write("\r\n");
    printGSMToSerial(1);                                              // delay of 1 sec
    sendATCommand("AT+USORD=0,4", 5);                                 // to get response from cloud
    memset(b, 0, sizeof(b));                                          // clear buffer for next variable
  }
}
/*******************   Modbus Functiond   ************************/
void ModBus_Data_to_Mem()                                                  // Getting Modbus Data From Bluetooth App 1 by 1 and store in the memory
{
  int i_1 = 0;

  if (Ble_Available == 1)
  {

    for (i_2 = 0, j_2 = 0, k_2 = 0; RCV_DATA[i_2]; i_2++, j_2++)         // split function
    {
      Bt_recieve[j_2] = RCV_DATA[i_2];
    }

    Ble_Available = 0;
    mod_flag_enable = 1;                                                  // This flag will set for each modbus data recieved
    j_mod++;                                                              // depend on this variable data will get seqentially from Bt App and Stored in variables
    //    Serial.print("j_mod= ");
    //    Serial.println(j_mod);
  }
  App_command = String(Bt_recieve);
  memset(Bt_recieve, 0, sizeof(Bt_recieve));                             // clear the buffer
  if ((j_mod % 2) != 0  && mod_flag_enable == 1)                         // this loop is for 'yes' or 'no' command from BT app
  {
    //    Serial.print("App_command= ");
    //    Serial.println(App_command);
    mod_flag_enable = 0;
    if (App_command == "yes")                                            // is 'yes' then Add modbus data from bt App
    {
      pCharacteristic->setValue("Add Slaveid,FunCode,RgAddr in numbers\n");
      pCharacteristic->notify();
    }
    else if (App_command == "no")                                 // if 'no' then exit the modbus loop
    {
      exit_flag = 1;
    }
    else if ((App_command != "yes") || (App_command != "no"))     // if 'yes' or 'no' command send incorrect it will print data incorrect
    {
      pCharacteristic->setValue("Please enter correct word \"yes\",\"no\")\n");
      pCharacteristic->notify();
      j_mod = j_mod - 1;
    }
  }

  if ((j_mod % 2) == 0 && mod_flag_enable == 1)                        // this condition for Enter The Modbus Data only From Bt App
  {
    mod_flag_enable = 0;                                               // it will clear for next modbus data enter
    Fragment_flag = 0;
    pCharacteristic->setValue("Add New slave data (yes/no)\n");
    pCharacteristic->notify();

    memset(Slave_ID, 0, sizeof(Slave_ID));
    memset(Fun_Code, 0, sizeof(Fun_Code));
    memset(Reg_Addr, 0, sizeof(Reg_Addr));
    for (i_2 = 0, j_2 = 0, k_2 = 0, l_2 = 0; App_command[i_2]; i_2++)  // split functon for Segrigate (slaveid,funcode,regAdd) From App command Buffer which is recieved from Bt APP
    {
      if (App_command[i_2] == ',')                                     // after ',' flag will set
      {
        Fragment_flag++;
      }
      else if (Fragment_flag == 0)                                     // slave id segrigate
      {
        Slave_ID[j_2] = char(App_command[i_2]);
        j_2++;
      }

      else if (Fragment_flag == 1)                                    // Function code Segrigate
      {
        Fun_Code[k_2] = char(App_command[i_2]);
        k_2++;
      }
      else if (Fragment_flag == 2)                                    // Reg_Add Segrigate
      {
        Reg_Addr[l_2] = char(App_command[i_2]);
        l_2++;
      }
    }

    Serial.print("App_command= ");
    Serial.println(App_command);
    Serial.print("Slave_ID= ");
    Serial.println(Slave_ID);
    Serial.print("Fun_Code= ");
    Serial.println(Fun_Code);
    Serial.print("Reg_Addr= ");
    Serial.println(Reg_Addr);


    // convert the Modbus data into string to Store in Memory
    Slave_ID_M = String(Slave_ID);
    FunCode_M = String(Fun_Code);
    RegAddr_M = String(Reg_Addr);
    Modbus_add_mry_data = slash + Slave_ID_M + slash + FunCode_M + slash + RegAddr_M + quama; // Add all Modbus vaiable in one buffer
    if (Mem_Write_count == 0)                                                                 // this condition for Writing a data in file
    {
      file_size(SPIFFS, MODBUS_FILE);                                                        // it will show data size of file
      writeFile(SPIFFS, MODBUS_FILE, Modbus_add_mry_data.c_str());                           // write data in file
      file_size(SPIFFS, MODBUS_FILE);                                                        // file size
      Mem_Write_count++;                                                                     // it for single time write in memory
    }
    else                                                                                     // after write next data will append
    {
      appendFile(SPIFFS, MODBUS_FILE, Modbus_add_mry_data.c_str());                          // append modbus data
    }

    App_command = "";
  }
}



void modbus_master_call()
{
  slave_no = 0;
  i = 0;
  j = 0;
  k = 0;
  //  Serial.print("slave_counter=");
  //  Serial.println(slave_counter);
  if (slave_no == 0)
  {
    mod_entry_flag = 1;
  }

  Serial.println("Reading slave data");

  while (slave_no <= slave_counter)
  {
    //    Serial.print("slave_no=");
    //    Serial.println(slave_no);
    delay(1000);
    if (FunCode[j] == 1)
    {
      mod_data_collection_flag = 1;
      if (!mb.slave()) {
        mb.readCoil(1, 1, coils, 20, cbWrite);
        slave_no++;
        i++;
        j++;
        k++;

        //mod_collection_flag=1;
        if (i >= slave_counter)
        {
          // slave_no = 0;
          i = 0;
          j = 0;
          k = 0;
        }

        for (int a = 0, b = 0; coils[a]; a++, b++)
        {
          Serial.print(coils[a]);
          P_coils[b] = coils[a];
        }
        Serial.println("");
        Serial.print("SlaveID=");
        Serial.println(SlaveID[i]);
        Serial.print("FunCode= ");
        Serial.println(FunCode[j]);
        Serial.print("RegAddr= ");
        Serial.println(RegAddr[k]);
      }
    }
    else if (FunCode[j] == 2)
    {
      mod_data_collection_flag = 2;
      if (!mb.slave()) {
        mb.readIsts(1, 1, input_sts, 20, cbWrite);
        slave_no++;
        i++;
        j++;
        k++;

        //mod_collection_flag=1;
        if (i >= slave_counter)
        {
          // slave_no = 0;
          i = 0;
          j = 0;
          k = 0;
        }

        for (int a = 0, b = 0; input_sts[a]; a++, b++)
        {
          Serial.print(input_sts[a]);
          P_input_sts[b] = input_sts[a];
        }

        Serial.println("");
        Serial.print("SlaveID=");
        Serial.println(SlaveID[i]);
        Serial.print("FunCode= ");
        Serial.println(FunCode[j]);
        Serial.print("RegAddr= ");
        Serial.println(RegAddr[k]);
      }
    }
    else if (FunCode[j] == 3)
    {
      mod_data_collection_flag = 3;
      if (!mb.slave()) {
        mb.readHreg(SlaveID[i], RegAddr[k],  &Holding_res_buf[i], 1, cbWrite); //(SlaevID,Address,Buffer,Range of data,Modus call)

        slave_no++;
        i++;
        j++;
        k++;

        //mod_collection_flag=1;
        if (i >= slave_counter)
        {
          // slave_no = 0;
          i = 0;
          j = 0;
          k = 0;
        }

        Serial.println(Holding_res_buf[i]);
        Serial.print("SlaveID=");
        Serial.println(SlaveID[i]);
        Serial.print("FunCode= ");
        Serial.println(FunCode[j]);
        Serial.print("RegAddr= ");
        Serial.println(RegAddr[k]);
      }
    }
    else if (FunCode[j] == 4)
    {
      mod_data_collection_flag = 4;
      if (!mb.slave()) {
        mb.readIreg(SlaveID[i], RegAddr[k],  &Input_res_buf[i], 1, cbWrite); //(SlaevID,Address,Buffer,Range of data,Modus call)
        slave_no++;
        i++;
        j++;
        k++;

        //mod_collection_flag=1;
        if (i >= slave_counter)
        {
          // slave_no = 0;
          i = 0;
          j = 0;
          k = 0;
        }
        Serial.println(Input_res_buf[i]);
        Serial.print("SlaveID=");
        Serial.println(SlaveID[i]);
        Serial.print("FunCode= ");
        Serial.println(FunCode[j]);
        Serial.print("RegAddr= ");
        Serial.println(RegAddr[k]);
      }
    }
    else if (FunCode[j] == 5)
    {
      if (!mb.slave()) {
        mb.writeCoil(SlaveID[i], RegAddr[k], 1, cbWrite); //(SlaevID,Address,Buffer,Range of data,Modus call)
        slave_no++;
        i++;
        j++;
        k++;

        //mod_collection_flag=1;
        if (i >= slave_counter)
        {
          // slave_no = 0;
          i = 0;
          j = 0;
          k = 0;
        }
        // Serial.println(Input_res_buf[i]);
        Serial.print("SlaveID=");
        Serial.println(SlaveID[i]);
        Serial.print("FunCode= ");
        Serial.println(FunCode[j]);
        Serial.print("RegAddr= ");
        Serial.println(RegAddr[k]);
      }
    }

    mb.task();
    yield();
    //     delay(1000);
    if (mod_data_collection_flag == 1)
    {
      modbus_slave_data = "{\"SlaveID\":\"" + (String)SlaveID[i] + "\",\"RegAddr\":\"" + (String)RegAddr[k] + "\",\"Data\":\"" + P_coils + "\"}";
    }
    else if (mod_data_collection_flag == 2)
    {
      modbus_slave_data = "{\"SlaveID\":\"" + (String)SlaveID[i] + "\",\"RegAddr\":\"" + (String)RegAddr[k] + "\",\"Data\":\"" + P_input_sts + "\"}";
    }
    else if (mod_data_collection_flag == 3)
    {
      modbus_slave_data = "{\"SlaveID\":\"" + (String)SlaveID[i] + "\",\"RegAddr\":\"" + (String)RegAddr[k] + "\",\"Data\":\"" + (String)Holding_res_buf[i] + "\"}";
    }
    else if (mod_data_collection_flag == 4)
    {
      modbus_slave_data = "{\"SlaveID\":\"" + (String)SlaveID[i] + "\",\"RegAddr\":\"" + (String)RegAddr[k] + "\",\"Data\":\"" + (String)Input_res_buf[i] + "\"}";
    }

    if (mod_data_collection_flag != 0)
    {
      if (mod_collection_flag == 1)
      {
        mod_collection_flag = 0;
        x = 0;
        y = 0;
        for (x = 0, y = 0; modbus_slave_data[x]; x++)
        {
          if (mod_entry_flag == 1)
          {
            Serial.println("enter in last count ");
            mod_entry_flag = 0;
            mod_send_buffer[0] = '[';                                         // add '{' at Start of first data String
            y++;
          }
          if (modbus_slave_data[x] == '}')                                   // if this condition is true means 1 data String is completed
          {
            mod_send_buffer[y] =  modbus_slave_data[x];
            y++;
            mod_send_buffer[y] = ',';                                       // add ',' one after slave data completed
            y++;
          }
          else
          {
            mod_send_buffer[y] =  modbus_slave_data[x];
            y++;
          }
        }
        if (slave_no == (slave_counter))                                    // to add ']' at last of the Total String
        {
          Serial.println("enter in last count ");
          y = y - 1;
          mod_send_buffer[y] = ']';
        }
        //        Serial.print("modbus_slave_data=");
        //        Serial.println( modbus_slave_data);
        modbus_slave_data = "";
        single_slave = String(mod_send_buffer);                                   // convert into string format
        memset(mod_send_buffer, 0, sizeof(mod_send_buffer));                      // clear buffer
        ToTal_mod_slave_data = ToTal_mod_slave_data + single_slave;               // add slave data 1 by 1 to Total_mod_slave_data
        // slave_no++;
      }
    }
  }
  Serial.print("ToTal_mod_slave_data=");
  Serial.println(ToTal_mod_slave_data);
  read_g_str_1 = ToTal_mod_slave_data ;                                        // add to main buffer which is going to be send to cloud
  ToTal_mod_slave_data = "";                                                  // clear buffer
  mod_data_collection_flag = 0;
}

void modbus_mem_string_split()                                                 // this fun used for split the data from memory to the actual varialble
{
  i = 0;
  j = 0;
  k = 0;
  z = 0;
  slave_counter = 0;
  Modbus_count = 0;
  for (z = 0, V1 = 0, V2 = 0, V3 = 0; mry_data[z] != '\0'; z++)               // "memory_data" contained data like Ex.  /1/2/3,/4/3/5,/2/3/4
  {
    if (mry_data[z] == ',')                                                   // ',' seperator of single slave data
    {
      Slave_ID_M = String(D1);
      FunCode_M = String(D2);
      RegAddr_M = String(D3);
      SlaveID[i] = Slave_ID_M.toInt();                                        // Convert into integer format for actual slave id pass to modbus function
      FunCode[j] = FunCode_M.toInt();                                         // Convert into integer format for actual Function code pass to modbus function
      RegAddr[k] = RegAddr_M.toInt();                                         // Convert into integer format for actual RegAddr pass to modbus function
      i++;
      j++;
      k++;
      z++;
      V1 = 0;
      V2 = 0;
      V3 = 0;
      memset(D1, 0, sizeof(D1));                                             // clear slave id buffer for next slave
      memset(D2, 0, sizeof(D2));                                             // clear funcode buffer for next slave
      memset(D3, 0, sizeof(D3));                                             // clear regaddr buffer for next slave
      Modbus_count = 0;                                                      // reset counter for next slave
      slave_counter++;                                                       // it will count nubber of slave in network
    }
    while (mry_data[z] == '/')                                            //   '/' is seperator of slave parameters ex.(/1/2/3): 1=slave id  2=funcode  3=regaddr
    {
      z++;
      Modbus_count++;                                                     // depend on this we can segrigate modbus parameter of single slave
    }

    if (Modbus_count == 1)                                               // this segrigate slave id
    {
      D1[V1] = mry_data[z];
      V1++;
    }
    else if (Modbus_count == 2)                                          // this segrigate funcode
    {
      D2[V2] = mry_data[z];
      V2++;
    }
    else if (Modbus_count == 3)                                         // this segrigate Regaddr
    {
      D3[V3] = mry_data[z];
      V3++;
    }
  }
}


void WiFi_Network_time()
{
  WiFi.begin(WiFi_ssid.c_str(), WiFi_password.c_str());                       // wifi begin with ssid and password
  delay(100);
  wifi_try = 0;
  wifi_connected = 0;
  while (WiFi.status() != WL_CONNECTED)                                       // modify and put wifi connect condition
  {
    wifi_try++;
    delay(500);
    Serial.print("% ");
    if (wifi_try == 15)                                                       // after try of 15 times to connect wifi then it will reminate from the loop
      break;
  }
  if (WiFi.status() != WL_CONNECTED)                                          // if wifi is not connected
  {
    Serial.println("UNABLE TO CONNECT TO WIFI\n");                            //wifi_not connected
    if ((com_sec != SEC) || (com_min != MIN) || (com_hr != HR))
    {
      Serial.println("RTC Mannualy Set");

      //      Serial.println(" Before SubStracted time.................... ");
      //      Serial.print("HR= ");
      //      Serial.println(HR);
      //      Serial.print("MIN= ");
      //      Serial.println(MIN);
      //      Serial.print("SEC= ");
      //      Serial.println(SEC);
      //      Serial.print("DAY= ");
      //      Serial.println(DAY);
      //      Serial.print("MONTH= ");
      //      Serial.println(MONTH);
      //      Serial.print("YEAR= ");
      //      Serial.println(YEAR);
      delay(500);
      //      writeFile(SPIFFS, Mannual_rtc_enable_FILE, "1");
      SUBSTRACT_GMT_Time(HR, MIN, SEC, DAY, MONTH, YEAR);
      setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);            // sec min hr ,day/month/year
      delay(500);
      //    printLocalTime();
    }
    else
    {
      writeFile(SPIFFS, Mannual_rtc_enable_FILE, "1");
      delay(1000);
      Serial.println("RTC time set from memory\n");
      readFile1(SPIFFS, TIME_STAMP);
      mannual_Timestamp(read_g_str_1);
      setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);                       // sec min hr ,day/month/year
    }
  }
  else    //wifi connected
  {
    Serial.println("CONNECTED TO WIFI\n");
    configTime(gmtOffset_sec, Network_daylightOffset_sec, ntpServer); // set network time to rtc
    delay(100);
    printLocalTime();                                                 // print date and time
    //        mannual_Timestamp(Time_stamp);
    //        ADD_GMT_Time(HR, MIN, SEC, DAY, MONTH, YEAR);                      //  convert rcv network time to gmt time as per the regions (EX. 5:30)
    //        printLocalTime();                                                 // print date and time
    //        setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);                       // sec min hr ,day/month/year
    //        printLocalTime();                                                 // print date and time

    if (local_time_flag != 1)                                         // this flag will set when we ge network time
    {
      writeFile(SPIFFS, Mannual_rtc_enable_FILE, "0");                // if Mannual Rtc enable flag =0 then then network set time will continue
      Serial.println("RTC Set by network");
    }
    else
    {
      Serial.println("RTC not Set by network");
      if ((com_sec != SEC) || (com_min != MIN) || (com_hr != HR))
      {
        Serial.println("RTC Mannualy Set");
        writeFile(SPIFFS, Mannual_rtc_enable_FILE, "1");
        SUBSTRACT_GMT_Time(HR, MIN, SEC, DAY, MONTH, YEAR);
        setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0); // sec min hr ,day/month/year
        delay(500);
        //    printLocalTime();
      }
      else
      {
        writeFile(SPIFFS, Mannual_rtc_enable_FILE, "1");
        delay(1000);
        Serial.println("Memory time set\n");
        readFile1(SPIFFS, TIME_STAMP);
        mannual_Timestamp(read_g_str_1);
        setTime(YEAR, MONTH, DAY, HR, MIN, SEC, 0);                       // sec min hr ,day/month/year
      }
    }
  }
  WiFi.disconnect();                                                // diconnect the wifi
  delay(100);
  while (WiFi.status() == WL_CONNECTED)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi is disconnectd");
      break;
    }
  }
  delay(50);
}

void Gateway_power_ON_SETUP()
{
  Serial.println( "///////////////// SETUP START ///////////////////" );
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);                         // writing value to register, disable brownout
  Serial.println("\n>> Power ON ");
  Wire.begin();                                                      // I2C initialization
  pwr_mgmt.attatch(Wire);                                            // Attaching I2C interface
  delay(1000);
  bq_reg();                                                          // Configuring BQ ic
  reg8_reg9();                                                       // read register 0x08 and 0x09 from BQ
  Serial.println(ESP.getFreeHeap());                                 // getting free heap (RAM)

  pinMode(vp,       INPUT);
  pinMode(cap_p,    INPUT);
  pinMode(cap_n,    INPUT);
  pinMode(vn,       INPUT);
  pinMode(bat_init, INPUT_PULLUP);
  pinMode(adc_1_ch, INPUT);

  pinMode(pwr_uc,   OUTPUT);//25
  pinMode(dtr_uc,   OUTPUT);//26
  pinMode(rst_uc,   OUTPUT);//27
  pinMode(rts_uc,   OUTPUT);//14
  //  pinMode(cts_uc,   INPUT);

  //pinMode(g_led,    OUTPUT);
  pinMode(rts0, OUTPUT);
  pinMode(cts0, INPUT);
  pinMode(boot_gpio, INPUT);
  pinMode(en_ic,    OUTPUT);
  pinMode(gpio_19,  INPUT);
  digitalWrite(rts0, HIGH);

  Serial.println("\nGSM Booting Process");                            // power on process
  Serial.println("\nPower OFF Start");
  power_off();
  Serial.println("\nPower OFF Done");
  digitalWrite(dtr_uc, HIGH);
  // digitalWrite(26, LOW);
  digitalWrite(rst_uc, HIGH);
  digitalWrite(pwr_uc, HIGH);
  digitalWrite(rts_uc, HIGH);
  delay(1000);
  digitalWrite(dtr_uc, LOW);
  digitalWrite(rts_uc, LOW);
  delay(1000);
  power_on();
  Serial.println("\nDONE");
  digitalWrite(en_ic, HIGH);
  delay(1000);
  Serial.println("\nPower ON sequence DONE");
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    local_time_flag = 1;
    return;
  }
  char output[80];
  Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
  strftime(output, 80, "%Y-%m-%d %H:%M:%S", &timeinfo);
  Time_stamp = String(output);
  local_time_flag = 0;
}

void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst) {
  struct tm tm;
  tm.tm_year = yr - 1900;   // Set date
  tm.tm_mon = month - 1;
  tm.tm_mday = mday;
  tm.tm_hour = hr;      // Set time
  tm.tm_min = minute;
  tm.tm_sec = sec;
  tm.tm_isdst = isDst;  // 1 or 0
  time_t t = mktime(&tm);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
}

void writeDetails()
{
  Serial.println("Writing SD details in SPIFF");

  File file1 = SPIFFS.open("/Upistring1.txt", FILE_WRITE);

  if (!file1)
  {
    Serial.println("Failed to open file 1 for writing");
    return;
  }

  file1.println(serverName);
  file1.close();

  File file2 = SPIFFS.open("/Upistring2.txt", FILE_WRITE);

  if (!file2)
  {
    Serial.println("Failed to open file 2 for writing");
    return;
  }

  file2.println(apnName);
  file2.close();

  File file3 = SPIFFS.open("/Upistring3.txt", FILE_WRITE);
  if (!file3) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file3.println(gId);                                                  // Write backup filenames line by line
  Serial.print("backup file wrote in spiff: "); Serial.println(gId);
  file3.close();

  File file4 = SPIFFS.open("/Upistring4.txt", FILE_WRITE);
  if (!file4) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file4.println(HOST_ADDRESS);                                                  // Write backup filenames line by line
  file4.close();

  File file5 = SPIFFS.open("/Upistring5.txt", FILE_WRITE);
  if (!file5) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file5.println(CLIENT_ID);                                                  // Write backup filenames line by line
  file5.close();

  File file6 = SPIFFS.open("/Upistring6.txt", FILE_WRITE);
  if (!file6) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file6.println(TOPIC_NAME);                                                  // Write backup filenames line by line
  file6.close();

  // Now you can use these strings as needed
  Serial.println("WRITE SD DETAILS TO SSPIFFS");

}

// ================= Reading SD details from file =====================================================================
void readDetails()
{
  uint64_t readArrayLength = 0;

  File file1 = SPIFFS.open("/Upistring1.txt", FILE_READ);

  if (!file1)
  {
    Serial.println("Failed to open file 1 for reading");
    return;
  }

  String input1 = file1.readStringUntil('\0');
  file1.close();

  File file2 = SPIFFS.open("/Upistring2.txt", FILE_READ);

  if (!file2)
  {
    Serial.println("Failed to open file 2 for reading");
    return;
  }

  String input2 = file2.readStringUntil('\0');
  file2.close();

  File file3 = SPIFFS.open("/Upistring3.txt", FILE_READ);
  if (!file3) {
    Serial.println("Failed to open file for reading");
    return;
  }

  String input3 = file3.readStringUntil('\0');                                         // Read sdRead value
  file3.close();
  file3.close();

  File file4 = SPIFFS.open("/Upistring4.txt", FILE_READ);
  if (!file4) {
    Serial.println("Failed to open file for reading");
    return;
  }

  String input4 = file4.readStringUntil('\0');                                         // Read sdRead value
  file4.close();
  file4.close();

  File file5 = SPIFFS.open("/Upistring5.txt", FILE_READ);

  if (!file5)
  {
    Serial.println("Failed to open file 5 for reading");
    return;
  }

  String input5 = file5.readStringUntil('\0');
  file5.close();
  
  File file6 = SPIFFS.open("/Upistring6.txt", FILE_READ);

  if (!file6)
  {
    Serial.println("Failed to open file 6 for reading");
    return;
  }

  String input6 = file6.readStringUntil('\0');
  file6.close();
  
  Serial.println("READ SD DETAILS FROM SPIFFS");

  serverName = input1.substring(0, input1.length() - 2);                          // Parse sdWrite integer
  apnName = input2.substring(0, input2.length() - 2);                           // Parse sdRead integer
  gId = input3.substring(0, input3.length() - 2);
  HOST_ADDRESS = input4.substring(0, input4.length() - 2);
  CLIENT_ID = input5.substring(0, input5.length() - 2);
  TOPIC_NAME = input6.substring(0, input6.length() - 2);

  Serial.print("The data in spiff: ");
  Serial.println(serverName);
  Serial.println(apnName);
  Serial.println(gId);
  Serial.println(HOST_ADDRESS);
  Serial.println(CLIENT_ID);
  Serial.println(TOPIC_NAME);
}

void getMQTT()
{
  String received ;
  received = command;
  Serial.println("## received is : " + received);

  if (received.length() > 0)
  {
    // Find the positions of the first and second commas in the received string
    int firstComma = received.indexOf(',');
    int secondComma = received.indexOf(',', firstComma + 1);

    // Check if both commas were found and their positions are valid
    if (firstComma > 0 && secondComma > firstComma)
    {
      // Extract the UPI ID, user name, and currency from the received string
      HOST_ADDRESS = received.substring(0, firstComma);
      CLIENT_ID = received.substring(firstComma + 1, secondComma);
      TOPIC_NAME = received.substring(secondComma + 1);

      Serial.println(HOST_ADDRESS);
      Serial.println(CLIENT_ID);
      Serial.println(TOPIC_NAME);
    }

  }

  memset(Recieved, 0, sizeof(Recieved));
}
