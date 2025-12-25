#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <IRsend.h>
#include <ir_Daikin.h>
#include <ir_Panasonic.h>
#include <ir_LG.h>
#include <ir_Samsung.h>
#include <ir_Mitsubishi.h>
#include <ir_Fujitsu.h>
#include <ir_Gree.h>
#include <ir_Haier.h>
#include <ir_Hitachi.h>
#include <ir_Midea.h>
#include <ir_Toshiba.h>
#include <ir_Whirlpool.h>

// Static network configuration
IPAddress local_IP(192, 168, 1, 122); 
IPAddress gateway(192, 168, 1, 1);   
IPAddress subnet(255, 255, 255, 0);  
IPAddress primaryDNS(8, 8, 8, 8);     
IPAddress secondaryDNS(8, 8, 4, 4);   

// WiFi mode configuration
const char* apSSID = "ESP_Config";
const char* apPassword = "config123";
const int configButtonPin = 0;
bool configMode = false;

// IR pin configuration
const uint16_t kRecvPin = 5; 
const uint16_t kIrLed = 14; 
const uint16_t kCaptureBufferSize = 1024; 
const uint8_t kTimeout = 50; 
const uint16_t USECPERTICK = 50; 

IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
IRsend irsend(kIrLed);
decode_results results;

// Store model information and raw IR code
uint8_t savedProtocol = 0;
const int MAX_RAWBUF_SIZE = 600;
uint16_t rawData[MAX_RAWBUF_SIZE];
uint16_t rawLength = 0;

ESP8266WebServer server(80);

// Define brands
enum ACBrand {
  BRAND_DAIKIN = 0,       
  BRAND_DAIKIN2 = 13,                   
  BRAND_DAIKIN128 = 14,   
  BRAND_DAIKIN152 = 15,  
  BRAND_DAIKIN64 = 16,    
  BRAND_DAIKIN160 = 17,   
  BRAND_DAIKIN176 = 18,  
  BRAND_DAIKIN216 = 20,   
  BRAND_PANASONIC = 1,
  BRAND_LG = 2,
  BRAND_SAMSUNG = 3,
  BRAND_MITSUBISHI = 4,
  BRAND_FUJITSU = 5,
  BRAND_GREE = 6,
  BRAND_HAIER = 7,
  BRAND_HITACHI = 8,
  BRAND_MIDEA = 9,
  BRAND_TOSHIBA = 10,
  BRAND_WHIRLPOOL = 11,
  BRAND_UNKNOWN = 12
};

ACBrand currentBrand = BRAND_DAIKIN;

// Initialize AC objects
IRDaikinESP daikinAC(kIrLed);         
IRDaikin2 daikin2AC(kIrLed);
IRDaikin128 daikin128AC(kIrLed);
IRDaikin152 daikin152AC(kIrLed);
IRDaikin64 daikin64AC(kIrLed);
IRDaikin160 daikin160AC(kIrLed);
IRDaikin176 daikin176AC(kIrLed);
IRDaikin216 daikin216AC(kIrLed);
IRPanasonicAc panasonicAC(kIrLed);
IRLgAc lgAC(kIrLed);
IRSamsungAc samsungAC(kIrLed);
IRMitsubishiAC mitsubishiAC(kIrLed);
IRFujitsuAC fujitsuAC(kIrLed);
IRGreeAC greeAC(kIrLed);
IRHaierAC haierAC(kIrLed);
IRHitachiAc hitachiAC(kIrLed);
IRMideaAC mideaAC(kIrLed);
IRToshibaAC toshibaAC(kIrLed);
IRWhirlpoolAc whirlpoolAC(kIrLed);

// Struct to store common AC state
struct ACState {
  bool power = true;
  int mode = 3; // Cooling mode
  int fan = 0;  // Auto
  int temp = 24;
  bool swingV = false;
  bool swingH = false;
} currentState;

// CORS headers function
void setCrossOrigin() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Max-Age", "600");
  server.sendHeader("Access-Control-Allow-Methods", "PUT,POST,GET,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
}

void handleOptions() {
  setCrossOrigin();
  server.send(204);
}

// EEPROM support functions
void saveProtocol(uint8_t protocol) {
  EEPROM.write(0, protocol);
  EEPROM.commit();
}

uint8_t loadProtocol() {
  return EEPROM.read(0);
}

void saveRawData(uint16_t* data, uint16_t length) {
  if (length > MAX_RAWBUF_SIZE) length = MAX_RAWBUF_SIZE;
  for (int i = 0; i < length; i++) {
    EEPROM.write(1 + i * 2, (data[i] >> 8) & 0xFF);
    EEPROM.write(2 + i * 2, data[i] & 0xFF);
  }
  EEPROM.write(1201, (length >> 8) & 0xFF);
  EEPROM.write(1202, length & 0xFF);
  EEPROM.commit();
}

uint16_t loadRawData(uint16_t* data) {
  uint16_t length = ((EEPROM.read(1201) << 8) | EEPROM.read(1202));
  if (length > MAX_RAWBUF_SIZE) length = MAX_RAWBUF_SIZE;
  for (int i = 0; i < length; i++) {
    data[i] = ((EEPROM.read(1 + i * 2) << 8) | EEPROM.read(2 + i * 2));
  }
  return length;
}

void saveACState() {
  EEPROM.write(1203, currentBrand);
  EEPROM.write(1204, currentState.power);
  EEPROM.write(1205, currentState.mode);
  EEPROM.write(1206, currentState.fan);
  EEPROM.write(1207, currentState.temp);
  EEPROM.write(1208, currentState.swingV);
  EEPROM.write(1209, currentState.swingH);
  EEPROM.commit();
}

void loadACState() {
  currentBrand = (ACBrand)EEPROM.read(1203);
  currentState.power = EEPROM.read(1204);
  currentState.mode = EEPROM.read(1205);
  currentState.fan = EEPROM.read(1206);
  currentState.temp = EEPROM.read(1207);
  currentState.swingV = EEPROM.read(1208);
  currentState.swingH = EEPROM.read(1209);
}

// Map brand names
ACBrand getBrandFromString(String brandName) {
  brandName.toLowerCase();
  if (brandName == "daikin") return BRAND_DAIKIN;
  else if (brandName == "daikin2") return BRAND_DAIKIN2;
  else if (brandName == "daikin128") return BRAND_DAIKIN128;
  else if (brandName == "daikin152") return BRAND_DAIKIN152;
  else if (brandName == "daikin64") return BRAND_DAIKIN64;
  else if (brandName == "daikin160") return BRAND_DAIKIN160;
  else if (brandName == "daikin176") return BRAND_DAIKIN176;
  else if (brandName == "daikin216") return BRAND_DAIKIN216;
  else if (brandName == "panasonic") return BRAND_PANASONIC;
  else if (brandName == "lg") return BRAND_LG;
  else if (brandName == "samsung") return BRAND_SAMSUNG;
  else if (brandName == "mitsubishi") return BRAND_MITSUBISHI;
  else if (brandName == "fujitsu") return BRAND_FUJITSU;
  else if (brandName == "gree") return BRAND_GREE;
  else if (brandName == "haier") return BRAND_HAIER;
  else if (brandName == "hitachi") return BRAND_HITACHI;
  else if (brandName == "midea") return BRAND_MIDEA;
  else if (brandName == "toshiba") return BRAND_TOSHIBA;
  else if (brandName == "whirlpool") return BRAND_WHIRLPOOL;
  else return BRAND_UNKNOWN;
}

String getBrandName(ACBrand brand) {
  switch (brand) {
    case BRAND_DAIKIN: return "daikin";
    case BRAND_DAIKIN2: return "daikin2";
    case BRAND_DAIKIN128: return "daikin128";
    case BRAND_DAIKIN152: return "daikin152";
    case BRAND_DAIKIN64: return "daikin64";
    case BRAND_DAIKIN160: return "daikin160";
    case BRAND_DAIKIN176: return "daikin176";
    case BRAND_DAIKIN216: return "daikin216";
    case BRAND_PANASONIC: return "panasonic";
    case BRAND_LG: return "lg";
    case BRAND_SAMSUNG: return "samsung";
    case BRAND_MITSUBISHI: return "mitsubishi";
    case BRAND_FUJITSU: return "fujitsu";
    case BRAND_GREE: return "gree";
    case BRAND_HAIER: return "haier";
    case BRAND_HITACHI: return "hitachi";
    case BRAND_MIDEA: return "midea";
    case BRAND_TOSHIBA: return "toshiba";
    case BRAND_WHIRLPOOL: return "whirlpool";
    case BRAND_UNKNOWN: return "unknown";
    default: return "unknown";
  }
}

// Apply settings to AC
void applySettingsToAC() {
  Serial.println("Applying settings, Brand: " + getBrandName(currentBrand));
  Serial.println("Power: " + String(currentState.power ? "ON" : "OFF"));
  Serial.println("Mode: " + String(currentState.mode));
  Serial.println("Temp: " + String(currentState.temp) + "°C");
  
  switch (currentBrand) {
    case BRAND_DAIKIN:
      daikinAC.begin();
      daikinAC.setPower(currentState.power);
      daikinAC.setMode(currentState.mode);
      daikinAC.setFan(currentState.fan);
      daikinAC.setTemp(currentState.temp);
      daikinAC.setSwingVertical(currentState.swingV);
      daikinAC.setSwingHorizontal(currentState.swingH);
      daikinAC.send();
      break;
    case BRAND_DAIKIN2:
      daikin2AC.begin();
      daikin2AC.setPower(currentState.power);
      daikin2AC.setMode(currentState.mode);
      daikin2AC.setFan(currentState.fan);
      daikin2AC.setTemp(currentState.temp);
      daikin2AC.setSwingVertical(currentState.swingV);
      daikin2AC.setSwingHorizontal(currentState.swingH);
      daikin2AC.send();
      break;
    case BRAND_DAIKIN128:
      daikin128AC.begin();
      daikin128AC.setPowerToggle(currentState.power);
      daikin128AC.setMode(currentState.mode);
      daikin128AC.setFan(currentState.fan);
      daikin128AC.setTemp(currentState.temp);
      daikin128AC.setSwingVertical(currentState.swingV);
      daikin128AC.send();
      break;
    case BRAND_DAIKIN152:
      daikin152AC.begin();
      daikin152AC.setPower(currentState.power);
      daikin152AC.setMode(currentState.mode);
      daikin152AC.setFan(currentState.fan);
      daikin152AC.setTemp(currentState.temp);
      daikin152AC.setSwingV(currentState.swingV);
      daikin152AC.send();
      break;
    case BRAND_DAIKIN64:
      daikin64AC.begin();
      daikin64AC.setPowerToggle(currentState.power);
      daikin64AC.setMode(currentState.mode);
      daikin64AC.setFan(currentState.fan);
      daikin64AC.setTemp(currentState.temp);
      daikin64AC.setSwingVertical(currentState.swingV);
      daikin64AC.send();
      break;
    case BRAND_DAIKIN160:
      daikin160AC.begin();
      daikin160AC.setPower(currentState.power);
      daikin160AC.setMode(currentState.mode);
      daikin160AC.setFan(currentState.fan);
      daikin160AC.setTemp(currentState.temp);
      daikin160AC.setSwingVertical(currentState.swingV);
      daikin160AC.send();
      break;
    case BRAND_DAIKIN176:
      daikin176AC.begin();
      daikin176AC.setPower(currentState.power);
      daikin176AC.setMode(currentState.mode);
      daikin176AC.setFan(currentState.fan);
      daikin176AC.setTemp(currentState.temp);
      daikin176AC.send();
      break;
    case BRAND_DAIKIN216:
      daikin216AC.begin();
      daikin216AC.setPower(currentState.power);
      daikin216AC.setMode(currentState.mode);
      daikin216AC.setFan(currentState.fan);
      daikin216AC.setTemp(currentState.temp);
      daikin216AC.setSwingVertical(currentState.swingV);
      daikin216AC.setSwingHorizontal(currentState.swingH);
      daikin216AC.send();
      break;
    case BRAND_PANASONIC:
      panasonicAC.begin();
      panasonicAC.setPower(currentState.power);
      panasonicAC.setMode(currentState.mode);
      panasonicAC.setFan(currentState.fan);
      panasonicAC.setTemp(currentState.temp);
      panasonicAC.setSwingVertical(currentState.swingV ? kPanasonicAcSwingVAuto : kPanasonicAcSwingVLowest);
      panasonicAC.setSwingHorizontal(currentState.swingH ? kPanasonicAcSwingHAuto : kPanasonicAcSwingHMiddle);
      panasonicAC.send();
      break;
    case BRAND_LG:
      lgAC.begin();
      lgAC.setPower(currentState.power);
      lgAC.setMode(currentState.mode);
      lgAC.setFan(currentState.fan);
      lgAC.setTemp(currentState.temp);
      lgAC.setSwingV(currentState.swingV);
      lgAC.setSwingH(currentState.swingH);
      lgAC.send();
      break;
    case BRAND_SAMSUNG:
      samsungAC.begin();
      samsungAC.setPower(currentState.power);
      samsungAC.setMode(currentState.mode);
      samsungAC.setFan(currentState.fan);
      samsungAC.setTemp(currentState.temp);
      samsungAC.setSwing(currentState.swingV);
      samsungAC.send();
      break;
    case BRAND_MITSUBISHI:
      mitsubishiAC.begin();
      mitsubishiAC.setPower(currentState.power);
      mitsubishiAC.setMode(currentState.mode);
      mitsubishiAC.setFan(currentState.fan);
      mitsubishiAC.setTemp(currentState.temp);
      mitsubishiAC.setVane(currentState.swingV ? kMitsubishiAcVaneAuto : kMitsubishiAcVaneAutoMove);
      mitsubishiAC.send();
      break;
    case BRAND_FUJITSU:
      fujitsuAC.begin();
      fujitsuAC.setMode(currentState.mode);
      fujitsuAC.setTemp(currentState.temp);
      fujitsuAC.setFanSpeed(currentState.fan);
      fujitsuAC.setSwing(currentState.swingV ? kFujitsuAcSwingVert : kFujitsuAcSwingOff);
      fujitsuAC.send();
      break;
    case BRAND_GREE:
      greeAC.begin();
      greeAC.setPower(currentState.power);
      greeAC.setMode(currentState.mode);
      greeAC.setFan(currentState.fan);
      greeAC.setTemp(currentState.temp);
      greeAC.setSwingVertical(currentState.swingV, 0);
      greeAC.send();
      break;
    case BRAND_HAIER:
      haierAC.begin();
      haierAC.setCommand(currentState.power ? kHaierAcCmdOn : kHaierAcCmdOff);
      haierAC.setMode(currentState.mode);
      haierAC.setFan(currentState.fan);
      haierAC.setTemp(currentState.temp);
      haierAC.setSwingV(currentState.swingV ? kHaierAc160SwingVAuto : kHaierAcSwingVOff);
      haierAC.send();
      break;
    case BRAND_HITACHI:
      hitachiAC.begin();
      hitachiAC.setPower(currentState.power);
      hitachiAC.setMode(currentState.mode);
      hitachiAC.setFan(currentState.fan);
      hitachiAC.setTemp(currentState.temp);
      hitachiAC.setSwingVertical(currentState.swingV);
      hitachiAC.setSwingHorizontal(currentState.swingH);
      hitachiAC.send();
      break;
    case BRAND_MIDEA:
      mideaAC.begin();
      mideaAC.setPower(currentState.power);
      mideaAC.setMode(currentState.mode);
      mideaAC.setFan(currentState.fan);
      mideaAC.setTemp(currentState.temp);
      mideaAC.send();
      break;
    case BRAND_TOSHIBA:
      toshibaAC.begin();
      toshibaAC.setPower(currentState.power);
      toshibaAC.setMode(currentState.mode);
      toshibaAC.setFan(currentState.fan);
      toshibaAC.setTemp(currentState.temp);
      toshibaAC.setSwing(currentState.swingV ? kToshibaAcSwingOn : kToshibaAcSwingOff);
      toshibaAC.send();
      break;
    case BRAND_WHIRLPOOL:
      whirlpoolAC.begin();
      whirlpoolAC.setCommand(currentState.power ? 0x00 : 0x20);
      whirlpoolAC.setMode(currentState.mode);
      whirlpoolAC.setFan(currentState.fan);
      whirlpoolAC.setTemp(currentState.temp);
      whirlpoolAC.send();
      break;
    case BRAND_UNKNOWN:
      if (rawLength > 0) {
        Serial.println("Sending raw IR code, Length: " + String(rawLength));
        irsend.sendRaw(rawData, rawLength, 38);
      } else {
        Serial.println("Error: No raw data available");
      }
      break;
  }
  saveACState();
  Serial.println("IR signal sent successfully!");
}

// Endpoint /learn
void handleLearn() {
  setCrossOrigin();
  Serial.println("Learning mode activated - waiting for IR signal...");
  
  if (irrecv.decode(&results)) {
    savedProtocol = results.decode_type;
    saveProtocol(savedProtocol);

    rawLength = results.rawlen;
    if (rawLength > 0 && rawLength <= MAX_RAWBUF_SIZE) {
      for (int i = 0; i < rawLength; i++) {
        rawData[i] = results.rawbuf[i] * USECPERTICK;
      }
      saveRawData(rawData, rawLength);
    }

    String human = resultToHumanReadableBasic(&results);
    Serial.println("IR signal learned!");
    Serial.println(human);
    Serial.println("Protocol: " + String(typeToString(results.decode_type, false)));
    Serial.println("Bits: " + String(results.bits));
    Serial.println("Raw length: " + String(rawLength));

    switch (results.decode_type) {
      case DAIKIN: currentBrand = BRAND_DAIKIN; break;
      case DAIKIN2: currentBrand = BRAND_DAIKIN2; break;
      case DAIKIN128: currentBrand = BRAND_DAIKIN128; break;
      case DAIKIN152: currentBrand = BRAND_DAIKIN152; break;
      case DAIKIN64: currentBrand = BRAND_DAIKIN64; break;
      case DAIKIN160: currentBrand = BRAND_DAIKIN160; break;
      case DAIKIN176: currentBrand = BRAND_DAIKIN176; break;
      case DAIKIN216: currentBrand = BRAND_DAIKIN216; break;
      case PANASONIC_AC: case PANASONIC_AC32: currentBrand = BRAND_PANASONIC; break;
      case LG: case LG2: currentBrand = BRAND_LG; break;
      case SAMSUNG_AC: currentBrand = BRAND_SAMSUNG; break;
      case MITSUBISHI_AC: case MITSUBISHI112: case MITSUBISHI136: currentBrand = BRAND_MITSUBISHI; break;
      case FUJITSU_AC: currentBrand = BRAND_FUJITSU; break;
      case GREE: currentBrand = BRAND_GREE; break;
      case HAIER_AC: case HAIER_AC_YRW02: currentBrand = BRAND_HAIER; break;
      case HITACHI_AC: case HITACHI_AC1: case HITACHI_AC2: case HITACHI_AC3:
      case HITACHI_AC344: case HITACHI_AC424: case HITACHI_AC264: case HITACHI_AC296: currentBrand = BRAND_HITACHI; break;
      case MIDEA: currentBrand = BRAND_MIDEA; break;
      case TOSHIBA_AC: currentBrand = BRAND_TOSHIBA; break;
      case WHIRLPOOL_AC: currentBrand = BRAND_WHIRLPOOL; break;
      default: currentBrand = BRAND_UNKNOWN; break;
    }
    saveACState();

    irrecv.resume();

    server.send(200, "application/json",
      "{\"status\":\"ok\",\"protocol\":\"" + typeToString(results.decode_type, false) + 
      "\",\"bits\":" + String(results.bits) + ",\"raw_length\":" + String(rawLength) + 
      ",\"brand\":\"" + getBrandName(currentBrand) + "\"}");
  } else {
    server.send(200, "application/json", "{\"status\":\"waiting\"}");
  }
}

// Endpoint /brands
void handleBrands() {
  setCrossOrigin();
  StaticJsonDocument<512> doc;
  JsonArray brands = doc.createNestedArray("supported_brands");
  brands.add("daikin");
  brands.add("panasonic");
  brands.add("lg");
  brands.add("samsung");
  brands.add("mitsubishi");
  brands.add("fujitsu");
  brands.add("gree");
  brands.add("haier");
  brands.add("hitachi");
  brands.add("midea");
  brands.add("toshiba");
  brands.add("whirlpool");
  brands.add("unknown");
  
  doc["current_brand"] = getBrandName(currentBrand);
  doc["total_brands"] = 13;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// Endpoint /brand
void handleSetBrand() {
  setCrossOrigin();
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No data provided\"}");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) { 
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  if (!doc.containsKey("brand")) {
    server.send(400, "application/json", "{\"error\":\"Missing brand parameter\"}");
    return;
  }

  String brandName = doc["brand"].as<String>();
  ACBrand newBrand = getBrandFromString(brandName);
  
  if (newBrand != currentBrand) {
    currentBrand = newBrand;
    switch (currentBrand) {
      case BRAND_DAIKIN: daikinAC.begin(); break;
      case BRAND_PANASONIC: panasonicAC.begin(); break;
      case BRAND_LG: lgAC.begin(); break;
      case BRAND_SAMSUNG: samsungAC.begin(); break;
      case BRAND_MITSUBISHI: mitsubishiAC.begin(); break;
      case BRAND_FUJITSU: fujitsuAC.begin(); break;
      case BRAND_GREE: greeAC.begin(); break;
      case BRAND_HAIER: haierAC.begin(); break;
      case BRAND_HITACHI: hitachiAC.begin(); break;
      case BRAND_MIDEA: mideaAC.begin(); break;
      case BRAND_TOSHIBA: toshibaAC.begin(); break;
      case BRAND_WHIRLPOOL: whirlpoolAC.begin(); break;
      case BRAND_UNKNOWN: break;
    }
    applySettingsToAC();
  }

  StaticJsonDocument<128> response;
  response["status"] = "ok";
  response["current_brand"] = getBrandName(currentBrand);
  response["message"] = "Brand switch successful";
  
  String responseStr;
  serializeJson(response, responseStr);
  server.send(200, "application/json", responseStr);
}

// Endpoint /status
void handleStatus() {
  setCrossOrigin();
  StaticJsonDocument<512> doc;
  doc["brand"] = getBrandName(currentBrand);
  doc["protocol"] = typeToString((decode_type_t)savedProtocol, false);
  doc["bits"] = results.bits;
  doc["raw_length"] = rawLength;
  doc["power"] = currentState.power ? "on" : "off";
  doc["mode"] = currentState.mode;
  doc["fan"] = currentState.fan;
  doc["temp"] = currentState.temp;
  doc["swing_v"] = currentState.swingV;
  doc["swing_h"] = currentState.swingH;
  doc["timestamp"] = millis();

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
  
  Serial.println("Status requested - Power: " + String(currentState.power ? "ON" : "OFF") + 
                 ", Temp: " + String(currentState.temp) + "°C");
}

// Individual property endpoints for Homebridge
void handleGetTemp() {
  setCrossOrigin();
  Serial.println("GET /temp - Returning: " + String(currentState.temp));
  server.send(200, "text/plain", String(currentState.temp));
}

void handleGetMode() {
  setCrossOrigin();
  Serial.println("GET /mode - Returning: " + String(currentState.mode));
  server.send(200, "text/plain", String(currentState.mode));
}

void handleGetPower() {
  setCrossOrigin();
  String powerState = currentState.power ? "on" : "off";
  Serial.println("GET /power - Returning: " + powerState);
  server.send(200, "text/plain", powerState);
}

void handleGetFan() {
  setCrossOrigin();
  Serial.println("GET /fan - Returning: " + String(currentState.fan));
  server.send(200, "text/plain", String(currentState.fan));
}

void handleGetSwingV() {
  setCrossOrigin();
  server.send(200, "text/plain", currentState.swingV ? "true" : "false");
}

void handleGetSwingH() {
  setCrossOrigin();
  server.send(200, "text/plain", currentState.swingH ? "true" : "false");
}

// Endpoint /control - UPDATED for Homebridge compatibility with CORS
void handleControl() {
  setCrossOrigin();
  
  Serial.println("POST /control received");
  
  if (!server.hasArg("plain")) {
    Serial.println("Error: No data provided");
    server.send(400, "application/json", "{\"error\":\"No data provided\"}");
    return;
  }

  String body = server.arg("plain");
  Serial.println("Request body: " + body);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.println("Error: Invalid JSON - " + String(error.c_str()));
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  // Update brand if specified
  if (doc.containsKey("brand")) {
    String brandName = doc["brand"].as<String>();
    ACBrand newBrand = getBrandFromString(brandName);
    if (newBrand != currentBrand) {
      currentBrand = newBrand;
      Serial.println("Brand changed to: " + getBrandName(currentBrand));
      switch (currentBrand) {
        case BRAND_DAIKIN: daikinAC.begin(); break;
        case BRAND_PANASONIC: panasonicAC.begin(); break;
        case BRAND_LG: lgAC.begin(); break;
        case BRAND_SAMSUNG: samsungAC.begin(); break;
        case BRAND_MITSUBISHI: mitsubishiAC.begin(); break;
        case BRAND_FUJITSU: fujitsuAC.begin(); break;
        case BRAND_GREE: greeAC.begin(); break;
        case BRAND_HAIER: haierAC.begin(); break;
        case BRAND_HITACHI: hitachiAC.begin(); break;
        case BRAND_MIDEA: mideaAC.begin(); break;
        case BRAND_TOSHIBA: toshibaAC.begin(); break;
        case BRAND_WHIRLPOOL: whirlpoolAC.begin(); break;
        case BRAND_UNKNOWN: break;
      }
    }
  }

  // Update state based on JSON parameters
  if (doc.containsKey("power")) {
    String pwr = doc["power"].as<String>();
    currentState.power = (pwr == "on");
    Serial.println("Power set to: " + String(currentState.power ? "ON" : "OFF"));
  }
  if (doc.containsKey("mode")) {
    int mode = doc["mode"];
    if (mode >= 0 && mode <= 4) {
      currentState.mode = mode;
      Serial.println("Mode set to: " + String(mode));
    }
  }
  if (doc.containsKey("fan")) {
    int fan = doc["fan"];
    if (fan >= 0 && fan <= 5) {
      currentState.fan = fan;
      Serial.println("Fan set to: " + String(fan));
    }
  }
  if (doc.containsKey("temp")) {
    int temp = doc["temp"];
    if (temp >= 16 && temp <= 32) {
      currentState.temp = temp;
      Serial.println("Temperature set to: " + String(temp) + "°C");
    }
  }
  if (doc.containsKey("swing_v")) {
    currentState.swingV = doc["swing_v"];
    Serial.println("Swing V set to: " + String(currentState.swingV));
  }
  if (doc.containsKey("swing_h")) {
    currentState.swingH = doc["swing_h"];
    Serial.println("Swing H set to: " + String(currentState.swingH));
  }

  // Apply settings to AC
  applySettingsToAC();

  // Return response based on what was requested
  String lastUpdated = "";
  if (doc.containsKey("temp")) lastUpdated = "temp";
  else if (doc.containsKey("mode")) lastUpdated = "mode";
  else if (doc.containsKey("power")) lastUpdated = "power";

  // Send simple text response for single property updates (Homebridge compatibility)
  if (lastUpdated == "temp") {
    server.send(200, "text/plain", String(currentState.temp));
  } else if (lastUpdated == "mode") {
    server.send(200, "text/plain", String(currentState.mode));
  } else if (lastUpdated == "power") {
    server.send(200, "text/plain", currentState.power ? "on" : "off");
  } else {
    // Full JSON response for multiple parameters or unknown requests
    StaticJsonDocument<512> response;
    response["status"] = "ok";
    response["brand"] = getBrandName(currentBrand);
    response["power"] = currentState.power ? "on" : "off";
    response["mode"] = currentState.mode;
    response["fan"] = currentState.fan;
    response["temp"] = currentState.temp;
    response["swing_v"] = currentState.swingV;
    response["swing_h"] = currentState.swingH;

    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
  }
  
  Serial.println("Control command executed successfully!");
}

// Endpoint /info
void handleInfo() {
  setCrossOrigin();
  StaticJsonDocument<1024> doc;
  doc["current_brand"] = getBrandName(currentBrand);
  doc["firmware_version"] = "2.1";
  doc["supported_brands_count"] = 13;
  
  JsonObject modes = doc.createNestedObject("modes");
  modes["0"] = "auto";
  modes["1"] = "heat";
  modes["2"] = "dry";
  modes["3"] = "cool";
  modes["4"] = "fan";
  
  JsonObject fans = doc.createNestedObject("fan_speeds");
  fans["0"] = "auto";
  fans["1"] = "minimum";
  fans["2"] = "low";
  fans["3"] = "medium";
  fans["4"] = "high";
  fans["5"] = "maximum";
  
  JsonObject temp = doc.createNestedObject("temperature");
  temp["min"] = 16;
  temp["max"] = 32;
  temp["unit"] = "°C";
  
  JsonObject swing = doc.createNestedObject("swing");
  swing["vertical"] = "supported";
  swing["horizontal"] = "depends on brand";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// Endpoint /test
void handleTest() {
  setCrossOrigin();
  Serial.println("Test signal requested");
  
  StaticJsonDocument<256> response;
  response["status"] = "ok";
  response["brand"] = getBrandName(currentBrand);
  response["message"] = "Test signal sent";
  
  applySettingsToAC();
  
  String responseStr;
  serializeJson(response, responseStr);
  server.send(200, "application/json", responseStr);
}

// Enter WiFi configuration mode
void enterWiFiConfigMode() {
  configMode = true;
  Serial.println("Entering WiFi configuration mode");
  WiFi.disconnect();
  
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  if (!wm.startConfigPortal(apSSID, apPassword)) {
    Serial.println("WiFi connection failed. Restarting...");
    delay(3000);
    ESP.restart();
  }
  
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Unable to configure static IP");
  }
  
  Serial.println("WiFi connected:");
  Serial.println(WiFi.localIP());
  configMode = false;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  EEPROM.begin(1210);

  Serial.println("\n=== Multi-brand AC controller v2.1 (Homebridge + CORS) ===");

  pinMode(configButtonPin, INPUT_PULLUP);
  
  if (digitalRead(configButtonPin) == LOW) {
    enterWiFiConfigMode();
  } else {
    WiFiManager wm;
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("Unable to configure static IP");
    }
    bool res = wm.autoConnect(apSSID, apPassword);
    if (!res) {
      Serial.println("WiFi connection failed. Entering configuration mode...");
      enterWiFiConfigMode();
    } else {
      Serial.println("WiFi connected successfully!");
      Serial.println("IP Address: " + WiFi.localIP().toString());
      Serial.println("MAC Address: " + WiFi.macAddress());

      irrecv.enableIRIn();
      irsend.begin();

      daikinAC.begin();
      savedProtocol = loadProtocol();
      rawLength = loadRawData(rawData);
      loadACState();

      Serial.println("Current state loaded from EEPROM:");
      Serial.println("- Brand: " + getBrandName(currentBrand));
      Serial.println("- Power: " + String(currentState.power ? "ON" : "OFF"));
      Serial.println("- Mode: " + String(currentState.mode));
      Serial.println("- Temp: " + String(currentState.temp) + "°C");

      // Define API routes with CORS support
      server.on("/", HTTP_GET, []() {
        setCrossOrigin();
        String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        html += "<title>AC Controller</title></head><body>";
        html += "<h1>Multi-brand AC Controller v2.1</h1>";
        html += "<h2>Current Status</h2>";
        html += "<ul>";
        html += "<li><b>Brand:</b> " + getBrandName(currentBrand) + "</li>";
        html += "<li><b>Temperature:</b> " + String(currentState.temp) + "°C</li>";
        html += "<li><b>Mode:</b> " + String(currentState.mode) + "</li>";
        html += "<li><b>Power:</b> " + String(currentState.power ? "ON" : "OFF") + "</li>";
        html += "<li><b>IP:</b> " + WiFi.localIP().toString() + "</li>";
        html += "</ul>";
        html += "<h2>Available APIs:</h2>";
        html += "<ul>";
        html += "<li>GET /learn - Learn IR signal</li>";
        html += "<li>GET /brands - List supported brands</li>";
        html += "<li>POST /brand - Set AC brand</li>";
        html += "<li>GET /status - Get current status (JSON)</li>";
        html += "<li>GET /temp - Get temperature</li>";
        html += "<li>GET /mode - Get mode</li>";
        html += "<li>GET /power - Get power state</li>";
        html += "<li>GET /fan - Get fan speed</li>";
        html += "<li>GET /swing_v - Get vertical swing</li>";
        html += "<li>GET /swing_h - Get horizontal swing</li>";
        html += "<li>POST /control - Control AC (JSON)</li>";
        html += "<li>GET /info - Get detailed information</li>";
        html += "<li>POST /test - Test IR signal</li>";
        html += "</ul>";
        html += "<p><small>CORS enabled for all endpoints</small></p>";
        html += "</body></html>";
        server.send(200, "text/html", html);
      });
      
      // Register all endpoints with OPTIONS support for CORS
      server.on("/learn", HTTP_OPTIONS, handleOptions);
      server.on("/learn", HTTP_GET, handleLearn);
      
      server.on("/brands", HTTP_OPTIONS, handleOptions);
      server.on("/brands", HTTP_GET, handleBrands);
      
      server.on("/brand", HTTP_OPTIONS, handleOptions);
      server.on("/brand", HTTP_POST, handleSetBrand);
      
      server.on("/status", HTTP_OPTIONS, handleOptions);
      server.on("/status", HTTP_GET, handleStatus);
      
      server.on("/temp", HTTP_OPTIONS, handleOptions);
      server.on("/temp", HTTP_GET, handleGetTemp);
      
      server.on("/mode", HTTP_OPTIONS, handleOptions);
      server.on("/mode", HTTP_GET, handleGetMode);
      
      server.on("/power", HTTP_OPTIONS, handleOptions);
      server.on("/power", HTTP_GET, handleGetPower);
      
      server.on("/fan", HTTP_OPTIONS, handleOptions);
      server.on("/fan", HTTP_GET, handleGetFan);
      
      server.on("/swing_v", HTTP_OPTIONS, handleOptions);
      server.on("/swing_v", HTTP_GET, handleGetSwingV);
      
      server.on("/swing_h", HTTP_OPTIONS, handleOptions);
      server.on("/swing_h", HTTP_GET, handleGetSwingH);
      
      server.on("/control", HTTP_OPTIONS, handleOptions);
      server.on("/control", HTTP_POST, handleControl);
      
      server.on("/info", HTTP_OPTIONS, handleOptions);
      server.on("/info", HTTP_GET, handleInfo);
      
      server.on("/test", HTTP_OPTIONS, handleOptions);
      server.on("/test", HTTP_POST, handleTest);

      server.enableCORS(true);
      server.begin();
      
      Serial.println("\n=== HTTP API ready! ===");
      Serial.println("All endpoints support CORS");
      Serial.println("Supported brands: Daikin (all variants), Panasonic, LG, Samsung, Mitsubishi, Fujitsu, Gree, Haier, Hitachi, Midea, Toshiba, Whirlpool, Unknown");
      Serial.println("Web interface: http://" + WiFi.localIP().toString());
      Serial.println("Homebridge endpoints: /temp, /mode, /power, /control");
      Serial.println("=======================================\n");
    }
  }
}

void loop() {
  server.handleClient();
  
  if (!configMode) {
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 30000) {
      Serial.println("[Heartbeat] AC controller running - Brand: " + getBrandName(currentBrand) + 
                     " | Temp: " + String(currentState.temp) + "°C | Power: " + 
                     (currentState.power ? "ON" : "OFF") + " | Uptime: " + String(millis()/1000) + "s");
      lastHeartbeat = millis();
    }
  }

  if (digitalRead(configButtonPin) == LOW && !configMode) {
    Serial.println("Configuration button pressed, entering WiFi configuration mode");
    enterWiFiConfigMode();
  }
}