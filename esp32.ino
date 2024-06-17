#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "AndroidAP252A";
const char* password = "irkf6463";
const char* botToken = "7309581329:AAGKIlrIn1r_c1ihNhuvcjwU5Q0eJbjAWA8";
const char* chatID = "6392851336";

bool wifiConnected = false; // Flag to track Wi-Fi connection status
void connectToWiFi() { //mazbut!
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi!");
}
void sendTelegramMessage(String message) { //mazbut!
  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(botToken) + "/sendMessage";
  String payload = "chat_id=" + String(chatID) + "&text=" + message;
  
  http.begin(url);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  int httpResponseCode = http.POST(payload);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Payload: " + response);
  } else {
    Serial.print("Error on sending message: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial2.begin(9600,SERIAL_8N1,16,17);
 connectToWiFi();
}

void loop() 
{
   if (WiFi.status() == WL_CONNECTED && !wifiConnected)
  { //mazbut!
    sendTelegramMessage("Wi-Fi Connected"); // Send message when Wi-Fi is connected for the first time
    wifiConnected = true; // Set flag to true to avoid sending multiple messages
   }
  // put your main code here, to run repeatedly:
Serial.print("Data Received");
Serial.println(Serial2.readString());
delay(200);
sendTelegramMessage(Serial2.readString());
delay(200);


}



