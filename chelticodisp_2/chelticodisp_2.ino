#include "SPI.h"
#include "AmebaILI9341.h"
#include "AmebaLogo.h"
#include <stdint.h>
#include "lvgl.h"
#include "SHT31.h"
#define SHT31_ADDRESS 0x44          // Alamat I2C default sensor SHT3x
#define CMD_MEASURE_HIGHREP 0x2400  // Perintah untuk mengukur dengan presisi tinggi
#include <WiFi.h>
#include <WiFiClient.h>
#include <FlashMemory.h>
#include "PCF8574.h"

#define FLASH_START_ADDR 0x00100000  // Alamat awal flash
#define FLASH_SIZE 0x4000            // Ukuran flash yang diminta
#define STRING_ADDR_1 0x1E00
#define STRING_ADDR_2 0x1E20//0x1F00
#define STRING_ADDR_3 0x1E40//0x2000
#define STRING_ADDR_4 0x1E60//0x2100
#define STRING_ADDR_5 0x1E80//0x2200
#define STRING_ADDR_6 0x1EA0//0x2300
#define STRING_ADDR_7 0x1EC0
#define STRING_ADDR_8 0x1EE0
#define STRING_ADDR_9 0x1F00
#define STRING_ADDR_10 0x1F20 


#define MAX_STRING_LEN 32  // Maksimum panjang string (termasuk '\0')
#define min(a, b) ((a) < (b) ? (a) : (b))

enum cmd_celthico {
  REPLY,
  WAIT
};
cmd_celthico cmline;
const int BUFFER_SIZE = 32;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

String inputString = "";  // Untuk menyimpan data serial yang diterima
bool stringComplete = false;
void caseReply(String command);


char readBuffer1[MAX_STRING_LEN] = { 0 }; //ssid
char readBuffer2[MAX_STRING_LEN] = { 0 }; // pasword wifi
char readBuffer3[MAX_STRING_LEN] = { 0 }; //adj temp
char readBuffer4[MAX_STRING_LEN] = { 0 }; //adj humidity
char readBuffer5[MAX_STRING_LEN] = { 0 }; //deviceid
char readBuffer6[MAX_STRING_LEN] = { 0 }; //serverhost
char readBuffer7[MAX_STRING_LEN] = { 0 }; //threshold
char readBuffer8[MAX_STRING_LEN] = { 0 }; //threshold
char readBuffer9[MAX_STRING_LEN] = { 0 };
char readBuffer10[MAX_STRING_LEN] = { 0 };
/* store string */
void storeString(const char *str, uint32_t addr) {
  int len = strlen(str) + 1;  // Termasuk null-terminator '\0'
  int i = 0;

  while (i < len) {
    uint32_t word = 0;
    memcpy(&word, str + i, min(4, len - i));  // Salin maksimal 4 byte
    FlashMemory.writeWord(addr + i, word);
    i += 4;
  }
}

void readString(char *buffer, uint32_t addr, int maxLen) {
  int i = 0;

  while (i < maxLen) {
    uint32_t word = FlashMemory.readWord(addr + i);
    memcpy(buffer + i, &word, min(4, maxLen - i));  // Salin kembali ke buffer
    i += 4;
    if (buffer[i - 1] == '\0') break;  // Stop jika menemukan null-terminator
  }
}
// Konfigurasi WiFi
char ssid[32] = "WirelessNet";   // Ganti dengan nama WiFi Anda
char password[32] = "eeeeeeee";  // Ganti dengan password WiFi Anda

// Konfigurasi Zabbix
char zabbix_server[32] = "34.101.137.39";  // IP atau hostname Zabbix Server
int zabbix_port = 10051;                // Port Zabbix Server

// Data yang akan dikirim
char *host = "CelthicoTester";                             // Host di Zabbix
char *key_temperature = "celthico.temperature";  // Key item di Zabbix
char *key_humidity = "celthico.humidity";
char *value = "69";  // Nilai data yang akan dikirim
WiFiClient client;


// Select 2 GPIO pins connect to TFT_RESET and TFT_DC. And default SPI_SS/SPI1_SS connect to TFT_CS.
#define TFT_RESET PA30
#define TFT_DC PB3
#define TFT_CS PA15
#define TFT_LED PB1
#define SPI_BUS SPI

#include <Wire.h>

#define PCF8574A_ADDR 0x71
#define ILI9341_SPI_FREQUENCY 20000000

wl_status_t wi_stats;

uint32_t start;
uint32_t stop;

SHT31 sht(SHT31_ADDRESS, &Wire);

AmebaILI9341 tft = AmebaILI9341(TFT_CS, TFT_DC, TFT_RESET, SPI_BUS);
//float temperature, humidity;

bool readSHT3x(float *temperature, float *humidity);
#define pinBuzzer PB1

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[ILI9341_TFTWIDTH * 10];  // Buffer tampilan
//static lv_color_t buf2[ILI9341_TFTWIDTH * 20];  // Double buffer

lv_obj_t *label_temp;
lv_obj_t *label_temp1;
lv_obj_t *label_hum;
lv_obj_t *label_hum1;
lv_obj_t *label_ssid;
lv_obj_t *label_rssid;
lv_obj_t *btn;
lv_obj_t *label;

typedef struct {
  float temp;
  float humidity;
  float threshold;
} sht_data;

sht_data sht_dat;
int suhu = 25;      // Simulasi suhu
int humidity = 60;  // Simulasi kelembaban

//const char *ssid = "WirelessNet";  // Simulasi SSID
PCF8574 PCF(0x38);
typedef struct {
  float tempadj;
  float humadj;
} adjust_sens;
adjust_sens ajd_sens;
////////////////////////////////////
//
//  INTERRUPT ROUTINE + FLAG
//
const int IRQPIN = 2;
int counter_strike;
volatile bool flagging = false;

void pcf_irq() {
  flagging = true;
}



void getDataFromSensorSHT(sht_data *s_data);
// Fungsi update tampilan suhu dan kelembaban
static void update_data(lv_timer_t *timer) {
  static char buf_temp[16];
  static char buf_hum[16];

  suhu += (rand() % 3 - 1);      // Simulasi perubahan suhu
  humidity += (rand() % 5 - 2);  // Simulasi perubahan kelembaban

  sprintf(buf_temp, "Tempe: %d°C", suhu);
  sprintf(buf_hum, "Humidity: %d%%", humidity);

  lv_label_set_text(label_temp, buf_temp);
  lv_label_set_text(label_hum, buf_hum);
}

void lv_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t w = area->x2 - area->x1 + 1;
  uint16_t h = area->y2 - area->y1 + 1;

  tft.setAddress(area->x1, area->y1, area->x2, area->y2);

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      tft.drawPixel(area->x1 + x, area->y1 + y, color_p->full);
      color_p++;
    }
  }

  lv_disp_flush_ready(disp);
}

void setup() {
  // tester
  Serial.begin(115200);
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  SPI_BUS.setDefaultFrequency(ILI9341_SPI_FREQUENCY);
  pinMode(pinBuzzer, OUTPUT);
  // Hubungkan ke WiFi
  Serial.println("Connecting to WiFi...");
  setDisplay();
  initialSetUP();
  WiFi.begin(ssid, password);

 /// while (WiFi.status() != WL_CONNECTED) {
  //  delay(500);
  //  Serial.print(".");
 // }
 // Serial.println("\nWiFi connected!");
  //Serial.println(WiFi.localIP());

  Wire.begin();
  Wire.setClock(100000);
  sht.begin();
  PCF.begin();
  
  pinMode(IRQPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRQPIN), pcf_irq, FALLING);


  uint16_t stat = sht.readStatus();
  Serial.print(stat, HEX);
  Serial.println();
  //initialSetUP();
  
  
  inputString.reserve(200);  // Alokasi memori untuk string
  cmline = WAIT;
  counter_strike=0;
  //ajd_sens.tempadj=0;
  //ajd_sens.humadj=0;
  lv_timer_handler();
}

void loop() {
 // lv_timer_handler();
 


  char temp_str[32];
  char temp_str_payload[32];
  char hum_str[32];
  char rssid_str[32];
  char hum_str_payload[32];

  
  //baca dari sensor sht
  getDataFromSensorSHT(&sht_dat);
  //update text suhu
  sprintf(temp_str, "Temp: %.2f °C", sht_dat.temp);
  sprintf(temp_str_payload, "%.2f", sht_dat.temp);

  //lv_label_set_text(label_temp, temp_str);


  sprintf(hum_str, "Humidity: %.2f %%", sht_dat.humidity);
  sprintf(hum_str_payload, "%.2f", sht_dat.humidity);
  //lv_label_set_text(label_hum, hum_str);
  //Serial.println("ajust temp" + String(ajd_sens.tempadj));
  //Serial.println("ajust humidity" + String(ajd_sens.humadj));
  /* trying to send temperature */




  uint32_t now = millis();
  if (flagging) {
    flagging = false;
    int x = PCF.read8();
    Serial.print("READ:\t");
    Serial.print('\t');
    Serial.print(now);
    Serial.print('\t');
    Serial.println(x);

    switch(x){
      case 241: ajd_sens.tempadj++; break;
      case 233: ajd_sens.tempadj--; break;
      case 229: ajd_sens.humadj++; break;
      case 227: ajd_sens.humadj--; break;
      case 249: //set ajd
      break;
      case 231: //toogle alarm
      break;

      default: break; 
    }
    updateLabelSensor(sht_dat.temp, ajd_sens.tempadj, sht_dat.humidity, ajd_sens.humadj);
    lv_timer_handler();

  }
  //  do other things here
  delay(1);


  //  digitalWrite(pinBuzzer, HIGH);
  // delay(5000);
  digitalWrite(pinBuzzer, LOW);
  // delay(5000);
  counter_strike++;
  if(counter_strike==1000){
    updateLabelSensor(sht_dat.temp, ajd_sens.tempadj, sht_dat.humidity, ajd_sens.humadj);
    lv_timer_handler();

    char temp_str_payload[32];
    char hum_str_payload[32];

    sprintf(temp_str_payload, "%.2f", sht_dat.temp+ajd_sens.tempadj);
    sprintf(hum_str_payload, "%.2f", sht_dat.humidity+ajd_sens.humadj);
    sendData(temp_str_payload, hum_str_payload);
    
    counter_strike=0;
  }

  if(counter_strike==1000){
   
   

   // lv_label_set_text(label_hum, hum_str);
  

  }

}

static void setAdjust(){
  // save to server

  // save to memory

}

static void initialSetUP(){
  // baca dari memory

  //ssid
  readString(readBuffer1, STRING_ADDR_1, MAX_STRING_LEN);
  String wifiAccess = String(readBuffer1);
  // convert to variable
  //wifiAccess.toCharArray(ssid, 32);

  strcpy(ssid, readBuffer1);
  

  // wifipass
  readString(readBuffer2, STRING_ADDR_2, MAX_STRING_LEN);
  String wifiPass = String(readBuffer2);
 //strcpy(password, readbBuffer2);
  wifiPass.toCharArray(password, 32);



  //temperature
  readString(readBuffer3, STRING_ADDR_3, MAX_STRING_LEN);
  String adjtemp_string = String(readBuffer3);

  ajd_sens.tempadj=adjtemp_string.toFloat();

  // humidity
  readString(readBuffer4, STRING_ADDR_4, MAX_STRING_LEN);
  String adjhum_string = String(readBuffer4);

  ajd_sens.humadj=adjhum_string.toFloat();

  // device id
  readString(readBuffer5, STRING_ADDR_5, MAX_STRING_LEN);
  String devid_string = String(readBuffer5);
  //strcpy();

  // hostname
  readString(readBuffer6, STRING_ADDR_6, MAX_STRING_LEN);
  String host_string = String(readBuffer6);
  strcpy(zabbix_server, readBuffer6);
  // hostname
  readString(readBuffer7, STRING_ADDR_7, MAX_STRING_LEN);
  String threshold_string = String(readBuffer7);
  sht_dat.threshold=threshold_string.toFloat();
  
  readString(readBuffer8, STRING_ADDR_8, MAX_STRING_LEN);
  String port_string = String(readBuffer8);
  zabbix_port = port_string.toInt();

  readString(readBuffer9, STRING_ADDR_9, MAX_STRING_LEN);
  String timer_post = String(readBuffer9);

  readString(readBuffer10, STRING_ADDR_10, MAX_STRING_LEN);
  String mode_low_or_not = String(readBuffer10);
  

  char temp_label[32];
  char hum_label[32];
  char ssid_label[32];
  char devid_label[32];
  sprintf(temp_label, "--.--/%.2f°C",  adjtemp_string.toFloat());
  lv_label_set_text(label_temp, temp_label);
  
  sprintf(hum_label, "--.--/%.2f%%",  adjhum_string.toFloat());
  lv_label_set_text(label_hum, hum_label);

  sprintf(ssid_label, "%s : NC", ssid);
  lv_label_set_text(label_ssid, ssid_label);

  sprintf(devid_label, "%s : --%", devid_string.c_str());
  lv_label_set_text(label_rssid, devid_label);


  lv_timer_handler();

}

static void updateLabelSensor(float temp, float temp_ajd, float hum, float hum_ajd){
  char temp_label[32];
  char hum_label[32];
  char temp_label1[32];
  char hum_label1[32];
  
  sprintf(temp_label1, "%.1f°C", temp+temp_ajd);
  lv_label_set_text(label_temp1, temp_label1);

  sprintf(hum_label1, "%.1f%%", hum+hum_ajd);
  lv_label_set_text(label_hum1, hum_label1);

  sprintf(temp_label, "%.2f/%.2f°C", temp, temp_ajd);
  lv_label_set_text(label_temp, temp_label);
  sprintf(hum_label, "%.2f/%.2f%%", hum, hum_ajd);
  lv_label_set_text(label_hum, hum_label);

  

}
static void sendData( char* temp_str_payload, char* hum_str_payload){
  char rssid_str[64];
  if (WiFi.status() != WL_CONNECTED) {
    lv_label_set_text(label_ssid, "SSID: Not Connected");
  } else {
    long rssi = WiFi.RSSI();
    // Serial.println("RSSI :");
    //Serial.println(rssi);
    sprintf(rssid_str, "%s : %d dBM", ssid, rssi);
    lv_label_set_text(label_ssid, rssid_str);
    //lv_label_set_text(label_ssid, ssid);
    /* then we will send the payload to the server */

    // lets parse the key 
    char key_temperature_temp[64];
    char key_humidity_temp[64];
    readString(readBuffer5, STRING_ADDR_5, MAX_STRING_LEN);

    sprintf(key_temperature_temp, "%s.%s",key_temperature, readBuffer5);
    sprintf(key_humidity_temp, "%s.%s",key_humidity, readBuffer5);

    Serial.println(key_temperature_temp);
    Serial.println(key_humidity_temp);
    send_payload_toZabbix(key_temperature_temp, temp_str_payload);
    send_payload_toZabbix(key_humidity_temp, hum_str_payload);
  }
}
static void setDisplay() {
  lv_init();

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, ILI9341_TFTWIDTH * 10);
  //wifi_off_debug_mode();

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = ILI9341_TFTHEIGHT;
  disp_drv.ver_res = ILI9341_TFTWIDTH;
  disp_drv.flush_cb = lv_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Label Suhu
  // Label Suhu
  btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn, 140, 40);
  lv_obj_align(btn, LV_ALIGN_LEFT_MID, 10, -90);
  label_temp = lv_label_create(btn);
  lv_label_set_text(label_temp, "T: -- °C Adj: -- °C");
  lv_obj_center(label_temp);
  
  // Label Suhu
  btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn, 140, 80);
  lv_obj_align(btn, LV_ALIGN_LEFT_MID, 10, -30);
  label_temp1 = lv_label_create(btn);
  lv_label_set_text(label_temp1, "12.42");
 lv_obj_set_style_text_font(label_temp1, &lv_font_montserrat_38, 0);
  lv_obj_center(label_temp1);

  //lv_obj_align(label_temp, LV_ALIGN_TOP_MID, 0, 10);
  btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn, 140, 40);
  lv_obj_align(btn, LV_ALIGN_RIGHT_MID, -10, -90);
  // Label Kelembab
  label_hum = lv_label_create(btn);
  lv_label_set_text(label_hum, "H: -- % Adj: -- %");
  lv_obj_center(label_hum);
  //lv_obj_align(label_hum, LV_ALIGN_TOP_MID, 0, 40);
  btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn, 140, 80);
  lv_obj_align(btn, LV_ALIGN_RIGHT_MID, -10, -30);
  // Label Kelembaban
  label_hum1 = lv_label_create(btn);
  lv_label_set_text(label_hum1, "--.--");
   lv_obj_set_style_text_font(label_hum1, &lv_font_montserrat_38, 0);
  lv_obj_center(label_hum1);


  btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn, 240, 40);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 40);
  label_ssid = lv_label_create(btn);
  lv_label_set_text(label_ssid, "SSID: Not Connected");
  lv_obj_center(label_ssid);
  //lv_obj_align(label_ssid, LV_ALIGN_TOP_MID, 0, 70);

  btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn, 240, 40);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -10);
  label_rssid = lv_label_create(btn);
  lv_label_set_text(label_rssid, "RSSID: Not Connected");
  lv_obj_center(label_rssid);





  // Timer untuk update suhu dan kelembaban
  // lv_timer_create(update_data, 2000, NULL);  // Update tiap 2 detik
}

void getDataFromSensorSHT(sht_data *s_data) {
  start = micros();
  sht.read(false);  //  default = true/fast       slow = false
  stop = micros();
  //Serial.print("\t");
  //Serial.print(stop - start);
  //Serial.print("\t");
  //Serial.print(sht.getTemperature(), 1);
  //Serial.print("\t");
  //Serial.println(sht.getHumidity(), 1);
  s_data->humidity = sht.getHumidity();
  s_data->temp = sht.getTemperature();
}

void send_payload_toZabbix(char *item_key, char *item_value) {
  /* just prepare the payload */
  String jsonPayload = String("{\"request\":\"sender data\",\"data\":[{\"host\":\"") + host + "\",\"key\":\"" + item_key + "\",\"value\":\"" + item_value + "\"}]}";

  // Membuat header sesuai protokol Zabbix
  uint32_t jsonLength = jsonPayload.length();
  char header[5] = { 'Z', 'B', 'X', 'D', 1 };
  uint8_t lengthBytes[8] = {
    (uint8_t)(jsonLength & 0xFF),
    (uint8_t)((jsonLength >> 8) & 0xFF),
    (uint8_t)((jsonLength >> 16) & 0xFF),
    (uint8_t)((jsonLength >> 24) & 0xFF),
    0, 0, 0, 0
  };

  // Kirim data ke Zabbix
  if (client.connect(zabbix_server, zabbix_port)) {
    // Serial.println("Connected to Zabbix server ");
    //Serial.println("Payload : "+ jsonPayload);

    // Kirim header
    client.write(header, 5);
    client.write(lengthBytes, 8);

    // Kirim payload JSON
    client.print(jsonPayload);

    // Membaca respons
    while (client.available() == 0) {
      delay(100);
    }

    while (client.available()) {
      String response = client.readString();
      //     Serial.println("Response from Zabbix:");
      //   Serial.println(response);
    }

    client.stop();
  } else {
    Serial.println("Failed to connect to Zabbix server");
  }
}

void serialEvent() {

  while (Serial.available()) {
    char receivedChar = Serial.read();  // Baca karakter
    if (receivedChar == '\n') {
      stringComplete = true;   // Tandai data lengkap
      caseReply(inputString);  // Cetak data yang diterima
      inputString = "";        // Reset string
      stringComplete = false;
      break;
    } else {
      inputString += receivedChar;  // Tambahkan ke string
    }
  }
}

void caseReply(String command) {


  if (command == "help") {
    Serial.println("Gunakan perintah berikut");
    Serial.println("wifiAP=[namawifi]");
    Serial.println("wifiPass=[passwordwifi]");
    Serial.println("adjt=[temperature adjust]");
    Serial.println("adjh=[humidity adjust]");
    Serial.println("ZabbixHost=[Zabbix Hostname]");
    Serial.println("ZabbixPort=[Zabbix Port]");
    Serial.println("devid=[device identifier]");
    Serial.println("portzbx=[port zabbix]");
    Serial.println("threshold=[Nilai threshold dalam celcius]");
    Serial.println("timerpost=[semakin banyak angkanya makin hemat]");
    Serial.println("modelow=[on atau off]");
    Serial.println("viewall");

  } else if (command.indexOf("wifiAp=") > -1) {

    String ssidAP = command.substring(command.indexOf("=") + 1);
    Serial.println("SSID : " + ssidAP);
    storeString(ssidAP.c_str(), STRING_ADDR_1);

  } else if (command.indexOf("threshold=") > -1) {

    String threshold_temp = command.substring(command.indexOf("=") + 1);
    Serial.println("Thresshold temp " + threshold_temp);
    storeString(threshold_temp.c_str(), STRING_ADDR_7);

  } else if (command.indexOf("wifiPass=") > -1) {

    String passwd = command.substring(command.indexOf("=") + 1);
    Serial.println("Password mant : " + passwd);
    storeString(passwd.c_str(), STRING_ADDR_2);

  } else if (command.indexOf("hostname=") > -1) {

    String hostname_zabbix = command.substring(command.indexOf("=") + 1);
    Serial.println("hostname : " + hostname_zabbix);
    storeString(hostname_zabbix.c_str(), STRING_ADDR_6);

  } else if (command.indexOf("portzbx=") > -1) {

    String port_host_zabbix = command.substring(command.indexOf("=") + 1);
    Serial.println("portzbx : " + port_host_zabbix);
    storeString(port_host_zabbix.c_str(), STRING_ADDR_8);

  }
  else if (command.indexOf("adjt=") > -1) {
    String adj_sub = command.substring(command.indexOf("=") + 1);
    // check fload if ok save if false end
    if (adj_sub.toFloat() != 0) {
      Serial.println(adj_sub.toFloat());
      storeString(adj_sub.c_str(), STRING_ADDR_3);
    } else {
      Serial.println("Nilai yang anda masukkan salah");
    }

  } else if (command.indexOf("adjh=") > -1) {
    String adj_sub = command.substring(command.indexOf("=") + 1);
    // check fload if ok save if false end
    if (adj_sub.toFloat() != 0) {
      Serial.println(adj_sub.toFloat());
      storeString(adj_sub.c_str(), STRING_ADDR_4);
    } else {
      Serial.println("Nilai yang anda masukkan salah");
    }

  } else if (command.indexOf("devid=") > -1) {

    String devid = command.substring(command.indexOf("=") + 1);
    Serial.println("device identifier : " + devid);
    storeString(devid.c_str(), STRING_ADDR_5);

  } else if (command.indexOf("timerpost=") > -1) {

    String timerpost = command.substring(command.indexOf("=") + 1);
    Serial.println("timerpost : " + timerpost);
    storeString(timerpost.c_str(), STRING_ADDR_9);
 
  } else if (command.indexOf("modelow=") > -1) {

    String modelow = command.substring(command.indexOf("=") + 1);
    Serial.println("modelow : " + modelow);
    storeString(modelow.c_str(), STRING_ADDR_10);

  }

  else if (command == "viewall") {

    readString(readBuffer1, STRING_ADDR_1, MAX_STRING_LEN);
    String wifiAcces = String(readBuffer1);

    readString(readBuffer2, STRING_ADDR_2, MAX_STRING_LEN);
    String wifiPass = String(readBuffer2);

    readString(readBuffer3, STRING_ADDR_3, MAX_STRING_LEN);
    String adjt_string = String(readBuffer3);

    readString(readBuffer4, STRING_ADDR_4, MAX_STRING_LEN);
    String adjh_string = String(readBuffer4);

    readString(readBuffer5, STRING_ADDR_5, MAX_STRING_LEN);
    String device_name = String(readBuffer5);

    readString(readBuffer6, STRING_ADDR_6, MAX_STRING_LEN);
    String host_name_zabbix = String(readBuffer6);

    readString(readBuffer7, STRING_ADDR_7, MAX_STRING_LEN);
    String thres_device = String(readBuffer7);

    readString(readBuffer8, STRING_ADDR_8, MAX_STRING_LEN);
    String string_port = String(readBuffer8);

    readString(readBuffer9, STRING_ADDR_9, MAX_STRING_LEN);
    String string_timer = String(readBuffer9);

    readString(readBuffer10, STRING_ADDR_10, MAX_STRING_LEN);
    String low_power_on = String(readBuffer10);

    Serial.println("Wifi_AP = " + wifiAcces);
    Serial.println("Wifi_pass = " + wifiPass);
    Serial.println("Adjt = " + adjt_string);
    Serial.println("adjt = " + adjh_string);
    Serial.println("device name = "+ device_name);
    Serial.println("host_name_zabbix = "+ host_name_zabbix);
    Serial.println("thresdevice = "+ thres_device);
    Serial.println("port zabbix = "+string_port);
    Serial.println("timer post = "+string_timer);
    Serial.println("low power mode = "+low_power_on);
  } else {
    Serial.println("Perintah tidak ditemukan");
  }
}

void CommandLine() {
  //Serial.println("Hallo ");
  //delay(10000);

  if (cmline == WAIT) {
    while (Serial.available() > 0) {
      char receivedChar = Serial.read();  // Baca karakter dari Serial

      // Periksa apakah karakter yang diterima adalah '\n'
      if (receivedChar == '\n') {
        buffer[bufferIndex] = '\0';  // Tambahkan null-terminator di akhir string
        //Serial.print("Received: ");
        caseReply(buffer);  // Cetak isi buffer

        // Reset buffer untuk menerima data baru
        bufferIndex = 0;
      } else {
        // Tambahkan karakter ke buffer jika masih ada ruang
        if (bufferIndex < BUFFER_SIZE - 1) {
          buffer[bufferIndex++] = receivedChar;
        }
      }
    }
  }
}