/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Wire.h>
#include <Temperature_LM75_Derived.h> 
#include <Adafruit_MPL3115A2.h>
#include <WiFi.h>
#include <esp_now.h>
Generic_LM75 temperature(0x48);
Adafruit_MPL3115A2 baro;
uint8_t broadcastAddress0[] = {0xE4, 0x65, 0xB8, 0x1E, 0x1E, 0x78};
uint8_t broadcastAddress1[] = {0xCC, 0xDB, 0xA7, 0x31, 0x9E, 0x70};
uint8_t broadcastAddress2[] = {0xCC, 0xDB, 0xA7, 0x31, 0x2B, 0x3B};
uint8_t broadcastAddress3[] = {0xEC, 0x64, 0xC9, 0x5E, 0xBE, 0x2C};

typedef struct struct_message {
  char a[32];
  float p;
  float c;
} struct_message;
struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  WiFi.mode(WIFI_MODE_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
// Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("\nI2C Scanner");
  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }
  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  baro.setSeaPressure(1013.26);
  baro.setAltitudeOffset(76);
  
}
void loop() {
  float pressure = baro.getPressure();
  float altitude = baro.getAltitude();
  float temperatureb = baro.getTemperature();

  strcpy(myData.a, "THIS IS A CHAR");
  myData.c = temperatureb;
  myData.p = pressure;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    //Serial.println(nDevices);
    //Serial.println(address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      //Serial.println(nDevices);
      //Serial.println(address);
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.print("TemperatureA = ");
    Serial.print(temperature.readTemperatureC());
    Serial.println(" C");
    Serial.print("TemperatureB = ");
    Serial.print(temperatureb);
    Serial.println(" C");
    Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("altitude = "); Serial.print(altitude); Serial.println(" m");
    Serial.println("done\n");
  }
  Serial.println(WiFi.macAddress());
  delay(5000);          
}