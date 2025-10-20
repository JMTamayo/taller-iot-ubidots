/*
  Taller de IoT con Ubidots
  Dispositivo de Control de Periféricos

  Autores:
    - Freddy Palacios Ángel
    - Juan Manuel Tamayo Gutiérrez

  Versión: 1.0.0

  Descripción: Obtener datos de humedad y realizar acciones de control de la
  humedad del suelo en planta doméstica, recibiendo las instrucciones de parte
  del dispositivo principal.

  El presente dispositivo realiza la comunicación directa con los periféricos,
  los cuales son sensor de humedad del suelo y actuador que habilita el sistema
  de bombeo de agua. Las acciones están determinadas por las instrucciones que
  recibe desde el dispositivo principal a quien le reporta los resultados vía
  protocolo UART.

  Links de Interés:
    - Protocolos Seriales:
  https://docs.arduino.cc/micropython/micropython-course/course/serial/

  Placa de Desarrollo:
    - Nombre: ARDUINO UNO R3
    - Documentación: https://docs.arduino.cc/hardware/uno-rev3/
*/

// Etiquetado de Pines I/O:
#define SOIL_MOISTURE_SENSOR_PIN                                               \
  A5 // Pin para lectura analógica de sensor de humedad
#define WATER_FLOW_ACTUATOR_PIN                                                \
  2 // Pin de salida digital para control de sistema de bombeo

// Definición de Constantes:
#define BAUD_RATE                                                              \
  9600 // Velocidad de comunicación serial con dispositivo principal

#define SOIL_MOISTURE_SENSOR_COMMAND                                           \
  "soil-moisture-sensor" // Comando para obtener medidas del sensor de humedad
                         // del suelo
#define WATER_FLOW_ACTUATOR_COMMAND                                            \
  "water-flow-actuator" // Comando para habilitar sistema de bombeo de agua a la
                        // planta
#define COMMAND_SEPARATOR                                                      \
  ":" // Separador entre comando y acción, cuya estructura es
      // {comando}{separador}{acción}

#define SOIL_MOISTURE_SENSOR_MIN_VALUE                                         \
  245 // Valor mínimo del sensor de humedad, medido en agua
#define SOIL_MOISTURE_SENSOR_MAX_VALUE                                         \
  560 // Valor máximo del sensor de humiedad, medido en aire

// Definición de Variables:
const unsigned int SOIL_MOISTURE_READING_RANGE =
    SOIL_MOISTURE_SENSOR_MAX_VALUE - SOIL_MOISTURE_SENSOR_MIN_VALUE;

String requestCommand = "";
String requestValue = "";

// Definición de Subrutinas y Funciones:

/*
  Interpretar Comando Recibido

  Descripción: Interpreta el comando y la accio2n a llevar a cabo a partir de la
  información recibida vía UART desde el dispositivo principal
*/
bool unwrapRequest(String request) {
  requestCommand = "";
  requestValue = "";

  request.trim();

  int separatorIndex = request.indexOf(COMMAND_SEPARATOR);
  if (separatorIndex == -1) {
    Serial.print("error:");
    Serial.println("The request could not be interpreted, it must be in the "
                   "format {command}" +
                   String(COMMAND_SEPARATOR) + "{value}");
    return true;
  }

  requestCommand = request.substring(0, separatorIndex);
  requestCommand.trim();

  requestValue = request.substring(separatorIndex + 1);
  requestValue.trim();

  return false;
}

// Subrutina para Configuración del Proceso a Ejecutar:
void setup() {
  // Configuración de pines I/O:
  pinMode(SOIL_MOISTURE_SENSOR_PIN, INPUT);
  pinMode(WATER_FLOW_ACTUATOR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Limpieza de salidas:
  digitalWrite(WATER_FLOW_ACTUATOR_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

  // Configuración de comunicaciones:
  Serial.begin(BAUD_RATE);
  while (!Serial)
    continue;
}

// Loop principal, en el cual se gestionan las acciones de monitoreo y control:
void loop() {
  if (Serial.available() > 0) {
    String request = Serial.readStringUntil('\n');
    bool error = unwrapRequest(request);

    if (!error) {
      if (requestCommand.equals(WATER_FLOW_ACTUATOR_COMMAND) &&
          (requestValue.equals("0") || requestValue.equals("1"))) {
        unsigned int controlAction = requestValue.equals("1") ? HIGH : LOW;
        digitalWrite(WATER_FLOW_ACTUATOR_PIN, controlAction);

      } else if (requestCommand.equals(SOIL_MOISTURE_SENSOR_COMMAND) &&
                 requestValue.equals("1")) {
        unsigned int sensorValue = analogRead(SOIL_MOISTURE_SENSOR_PIN);
        float normalizedValue =
            (1.0f - ((float)sensorValue - SOIL_MOISTURE_SENSOR_MIN_VALUE) /
                        SOIL_MOISTURE_READING_RANGE);
        float moisture = 100.0f * constrain(normalizedValue, 0.0f, 1.0f);

        Serial.println(requestCommand + COMMAND_SEPARATOR +
                       String(moisture, 2));
      }
    }
  }
}
