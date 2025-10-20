/*
  Taller de IoT con Ubidots
  Dispositivo de Gestión de Comunicaciones IoT

  Autores:
    - Freddy Palacios Ángel
    - Juan Manuel Tamayo Gutiérrez

  Versión: 1.0.0

  Descripción: Obtener datos de humedad y realizar acciones de control de la
  humedad del suelo en planta doméstica, recibiendo y enviando los datos de
  proceso desde y hacia el servidor de Ubidots.

  El presente dispositivo realiza la gestión de las comunicaciones. La
  comunicación con Ubidots se establece mediante el protocolo MQTT. Para obtener
  la humedad y realizar las acciones de control, este dispositivo se comunica
  con otro via protocolo UART cuya función es el manejo de periféricos.

  Se requiere crear un módulo "secrets.h" en el cual se incluyan los secretos
  para la gestión de la conexión con el servicio WiFi y el servidor MQTT. Las
  variables que se deben instanciar en él son:
      - WIFI_SSID
      - WIFI_PASSWORD
      - MQTT_SERVER
      - MQTT_PORT
      - MQTT_USER
      - MQTT_PASSWORD
      - MQTT_TOPIC_BASE

  Links de Interés:
    - MQTT: https://mqtt.org
    - Protocolos Seriales:
  https://docs.arduino.cc/micropython/micropython-course/course/serial/
    - Ubidots: https://ubidots.com

  Placa de Desarrollo:
    - Nombre: ESP32 C3 SUPER MINI
    - Documentación:
  https://randomnerdtutorials.com/getting-started-esp32-c3-super-mini/
*/

// Importación de Librerías
#include <PubSubClient.h>
#include <WiFi.h>

// Importación de Secretos:
#include "secrets.h"

// Etiquetado de Pines I/O:
#define BUILTIN_LED                                                            \
  8 // Pin incorporado de la placa para indicar error en comunicaciones
#define CONTROLLER_RX_PIN                                                      \
  7 // Pin de transmisión de datos via UART con dispositivo de control
#define CONTROLLER_TX_PIN                                                      \
  6 // Pin de transmisión de datos via UART con dispositivo de control

// Definición de Constantes:
#define BAUD_RATE 115200 // Velocidad de comunicación serial para depuración
#define CONTROLLER_BAUD_RATE                                                   \
  9600 // Velocidad de comunicación serial con controlador [Arduino UNO]

#define WIFI_RETRY_MILLISECONDS                                                \
  1000 // Tiempo máximo de espera por conexión exitosa con servicio WiFi

#define MQTT_CLIENT_ID                                                         \
  "r3-esp32" // Identificador único del dispositivo en servidor MQTT
#define MQTT_RETRY_MILLISECONDS                                                \
  1000 // Tiempo máximo de espera por conexión exitosa con servidor MQTT
#define MQTT_TOPIC_ENABLE_WATER_FLOW                                           \
  "enable-water-flow/lv" // Tópico para habilitar o deshabilitar sistema de
                         // bombeo de agua
#define MQTT_TOPIC_SOIL_MOISTURE                                               \
  "soil-moisture" // Tópico para monitorear humedad del suelo de la planta
#define MQTT_TOPIC_SOIL_MOISTURE_SAMPLING_TIME_MILLISECONDS                    \
  60000 // Tiempo de muestreo de la humedad de la planta

#define CONTROLLER_TIMEOUT_MILLISECONDS                                        \
  250 // Tiempo máximo de espera por datos del dispositivo de control
#define CONTROLLER_REQUEST_DELAY_MILLISECONDS                                  \
  50 // Tiempo de retardo luego de petición a dispositivo de control
#define COMMAND_SEPARATOR                                                      \
  ":" // Separador entre comando y acción, cuya estructura es
      // {comando}{separador}{acción}
#define SOIL_MOISTURE_SENSOR_COMMAND                                           \
  "soil-moisture-sensor" // Comando para obtener medidas del sensor de humedad
                         // del suelo
#define WATER_FLOW_ACTUATOR_COMMAND                                            \
  "water-flow-actuator" // Comando para habilitar sistema de bombeo de agua a la
                        // planta

// Definición de Variables:
WiFiClient espClient = WiFiClient();
PubSubClient mqttClient = PubSubClient(espClient);

String enableWaterFlowTopic =
    String(MQTT_TOPIC_BASE) + "/" + String(MQTT_TOPIC_ENABLE_WATER_FLOW);
String soilMoistureTopic =
    String(MQTT_TOPIC_BASE) + "/" + String(MQTT_TOPIC_SOIL_MOISTURE);

unsigned long lastSoilMoistureSample = 0;

// Definición de Subrutinas y Funciones:

/*
  Conectarse a Servicio WiFi

  Descripción: Conectarse a internet vía WiFi a partir de usuario y contraseña
  de la red. Cuando la comunicación se establece exitosamente, el led
  incorporado de la placa estará encendido.
*/
void connectToWifi() {
  lightOnLed(false);

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (WiFi.status() != WL_CONNECTED) {
    if (retryTimeMs >= WIFI_RETRY_MILLISECONDS) {
      Serial.println("Connection to WiFi network failed. SSID: " +
                     String(WIFI_SSID));
      return;
    }

    delay(100);
    retryTimeMs = millis() - startTimeMs;
  }

  Serial.println("Connected to WiFi network. SSID: " + String(WIFI_SSID) +
                 ". IP: " + String(WiFi.localIP().toString().c_str()));

  lightOnLed(true);
}

/*
  Conectarse a Servidor MQTT

  Descripción: Conectarse a servidor MQTT a partir de su url, puerto, usuario y
  contraseña. Adicionalmente, se suscribe al tópico para el control del sistema
  de bombeo de la planta. Cuando la comunicación se establece exitosamente, el
  led incorporado de la placa estará encendido.
*/
void connectToMqtt() {
  lightOnLed(false);

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!mqttClient.connected()) {
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      mqttClient.subscribe(enableWaterFlowTopic.c_str());
      break;
    }

    if (retryTimeMs >= MQTT_RETRY_MILLISECONDS) {
      Serial.println("Connection to MQTT server failed. Server: " +
                     String(MQTT_SERVER));
      return;
    }

    delay(100);
    retryTimeMs = millis() - startTimeMs;
  }

  lightOnLed(true);
}

/*
  Callback para Eventos Suscritos en Servidor MQTT

  Descripción: Función que se ejecuta cuando se recibe un mensaje de un tópico
  suscrito en servidor MQTT. Cuando se recibe un mensaje válido para controlar
  el sistema de bombeo, se ejecuta la función que envía la petición
  correspondiente al dispositivo de control.
*/
void mqttSubscriptionCallback(char *topic, byte *payload, unsigned int length) {
  String topicStr = topic;
  String payloadString = (char *)payload;
  payloadString.remove(length);

  if (topicStr.equals(enableWaterFlowTopic) &&
      (payloadString.equals("0") || payloadString.equals("1") ||
       payloadString.equals("0.0") || payloadString.equals("1.0"))) {
    Serial.println("Received message from MQTT server. Topic: " + topicStr +
                   ". Payload: " + payloadString);
    bool action = payloadString.toInt() == 1 ? true : false;
    enableWaterFlowActuator(action);

  } else {
    Serial.println(
        "Received unimplemented message from MQTT server. Topic: " + topicStr +
        ". Payload: " + payloadString + ". Ignoring message.");

    return;
  }
}

/*
  Encender Led Incorporado de la Placa

  Descripción: Permite encender o apagar totalmente el led incorporado de la
  placa, el cual tiene lógica invertida.
*/
void lightOnLed(bool on) {
  unsigned int action = on ? LOW : HIGH;
  digitalWrite(BUILTIN_LED, action);
}

/*
  Enviar Solicitud para Encendido de Sistema de Bombeo

  Descripción: Envía petición a dispositivo de control para activar o desactivar
  el sistema de bombeo de agua para la planta. La comunicación se realiza vía
  UART.
*/
void enableWaterFlowActuator(bool on) {
  String action = on ? "1" : "0";
  Serial1.println(WATER_FLOW_ACTUATOR_COMMAND + String(COMMAND_SEPARATOR) +
                  action);
}

/*
  Enviar Solicitud para Monitorear la Humedad del Suelo

  Descripción: Envía petición a dispositivo de control para obtener datos de la
  humedad del suelo de la planta. La comunicación se realiza vía UART.
*/
float soilMoistureRequest() {
  String action = "1";
  Serial1.println(SOIL_MOISTURE_SENSOR_COMMAND + String(COMMAND_SEPARATOR) +
                  action);
  delay(CONTROLLER_REQUEST_DELAY_MILLISECONDS);

  const unsigned long startTime = millis();
  while (millis() - startTime < CONTROLLER_TIMEOUT_MILLISECONDS) {
    if (Serial1.available() > 0) {
      String response = Serial1.readStringUntil('\n');
      response.trim();

      int separatorIndex = response.indexOf(COMMAND_SEPARATOR);
      if (separatorIndex == -1) {
        Serial.print("error:");
        Serial.println("The response could not be interpreted, it must be in "
                       "the format {command}" +
                       String(COMMAND_SEPARATOR) + "{value}");
        return NAN;
      }

      String responseCommand = response.substring(0, separatorIndex);
      responseCommand.trim();

      String responseValueString = response.substring(separatorIndex + 1);
      responseValueString.trim();
      return responseValueString.toFloat();
    }
  }

  Serial.print("error:");
  Serial.println("Timeout waiting for controller response");
  return NAN;
}

// Subrutina para Configuración del Proceso a Ejecutar:
void setup() {
  // Configuración de pines I/O:
  pinMode(BUILTIN_LED, OUTPUT);

  // Limpieza de salidas:
  lightOnLed(false);

  // Configuración de comunicaciones:
  Serial.begin(BAUD_RATE);
  while (!Serial)
    continue;

  Serial1.begin(CONTROLLER_BAUD_RATE, SERIAL_8N1, CONTROLLER_RX_PIN,
                CONTROLLER_TX_PIN);
  while (!Serial1)
    continue;

  mqttClient.setCallback(mqttSubscriptionCallback);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  // Inicio de variables de proceso:
  lastSoilMoistureSample =
      millis() - MQTT_TOPIC_SOIL_MOISTURE_SAMPLING_TIME_MILLISECONDS;
}

// Loop principal, en el cual se gestionan las conexiones y las acciones de
// control:
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWifi();

  } else if (!mqttClient.connected()) {
    connectToMqtt();

  } else {
    if (millis() - lastSoilMoistureSample >=
        MQTT_TOPIC_SOIL_MOISTURE_SAMPLING_TIME_MILLISECONDS) {
      float soilMoisture = soilMoistureRequest();
      if (!isnan(soilMoisture))
        mqttClient.publish(soilMoistureTopic.c_str(),
                           String(soilMoisture).c_str());

      lastSoilMoistureSample = millis();
    }
  }

  mqttClient.loop();
}