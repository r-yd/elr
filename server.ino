#include <WiFi.h>
#include "BluetoothSerial.h"
#include <PubSubClient.h>
const char* ssid = "NAME";
const char* password = "password";              
#define LED_BUILTIN 2
int bat_status;
const char* mqtt_server = "192.168.241.128";

WiFiClient espClient;
PubSubClient client(espClient);
BluetoothSerial ESP_BT;
long now = millis();
long lastMeasure = 0;

boolean BT_cnx = false;
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    digitalWrite(LED_BUILTIN, HIGH);
    BT_cnx = true;
  }
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    digitalWrite(LED_BUILTIN, LOW);
    BT_cnx = false;
    ESP.restart();
  }
}
void setup_wifi() {
  delay(10);
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
                                                                        bat_status = analogRead(A4);
                                                                        if(bat_status<=3289){
                                                                        digitalWrite(27, HIGH);
                                                                        }
                                                                        if(bat_status > 3289){
                                                                         digitalWrite(27, LOW);
                                                                        }
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}
void reconnect() {
  
  while (!client.connected()) {
                                                                    bat_status = analogRead(A4);
                                                                    if(bat_status<=3289){
                                                                    digitalWrite(27, HIGH);
                                                                    }
                                                                    if(bat_status > 3289){
                                                                    digitalWrite(27, LOW);
                                                                    }
    Serial.print("Attempting MQTT connection...");
   

    if (client.connect("ESP32_new")) {
      Serial.println("connected");  
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // 5 sec rertying delay
      delay(5000);
    }
  }
}
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println();
  pinMode(41, INPUT); // LO +
  pinMode(40, INPUT); // LO -
  pinMode(14, INPUT);
  pinMode(27, OUTPUT);
  client.setServer(mqtt_server, 1883);
}

void loop() {
                                           bat_status = analogRead(A4);
                                           if(bat_status<=3289){
                                           digitalWrite(27, HIGH);
                                           }
                                           if(bat_status > 3289){
                                           digitalWrite(27, LOW);
                                           }
  int pB_state;
  pB_state=digitalRead(14);
  switch(pB_state){
    case 1:
          ESP_BT.register_callback(callback);
            if(!ESP_BT.begin("ESP32_ECG")){
              Serial.println("An error occurred initializing Bluetooth");
            }
            else{
                Serial.println("Bluetooth initialized... Bluetooth Device is Ready to Pair...");
            }
             while(1){
                                                 bat_status = analogRead(A4);
                                                 if(bat_status<=3289){
                                                 digitalWrite(27, HIGH);
                                                 }
                                                 if(bat_status > 3289){
                                                 digitalWrite(27, LOW);
                                                 }
              if((digitalRead(40) == 1)||(digitalRead(41) == 1)){
              Serial.println('!');
              ESP_BT.println('!');
              }
               else{
               
                Serial.println(analogRead(A0));
             
                if(BT_cnx){
                  ESP_BT.print('E'); 
                  ESP_BT.println(analogRead(A0));
                  } 
                  }
                  delay(1);
                  pB_state=digitalRead(14);
                  if(pB_state!=1){
                    break;
                  }
                 }
                break;
        default:  
          setup_wifi();
            while(1){
                                                  bat_status = analogRead(A4);
                                                  if(bat_status<=3289){
                                                    digitalWrite(27, HIGH);
                                                  }
                                                  if(bat_status > 3289){
                                                    digitalWrite(27, LOW);
                                                  }
                      if (!client.connected()) {
                        reconnect();
                      }
                      if(!client.loop())
                        client.connect("ESP32_new");
                    
                      now = millis();
                    
                      if (now - lastMeasure > 2) {
                        lastMeasure = now;
                        int temp = analogRead(A0);
                        Serial.println(temp);
                        boolean rc;
                        char msg_out[20];
                        sprintf(msg_out, "%d",temp);
                        client.publish("ecg", msg_out );
                        Serial.print("ECG: ");
                        Serial.println(temp);
                      }
                      pB_state=digitalRead(14);
                       if (pB_state==1){
                         break;
                          } 
                      }
            }
}
