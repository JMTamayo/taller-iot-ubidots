/*
  Taller de IoT con Ubidots
  Calibración del sensor de humedad del suelo resistivo
  Autores:
    - Freddy Palacios Ángel
    - Juan Manuel Tamayo Gutiérrez

  Versión: 1.0.0

  Descripción: Calibración del sensor de humedad del suelo resistivo.
  El sensor se conecta al pin A0 del Arduino Mega 2560 y se lee el valor
  analógico (ADC) y el voltaje equivalente.
*/

// Etiquetado de Pines I/O:
#define SENSOR_PIN A0 // Pin analógico donde está conectado el sensor

// Definición de Constantes:
#define VREF 5.0 // Voltaje de referencia ADC del Arduino Mega

// Definición de Subrutinas y Funciones:

/*
  Loop principal

  Descripción: Loop principal del programa.
  Se lee el valor del sensor de humedad del suelo resistivo y se muestra por
  el monitor serial.
*/
void loop() {
  int valorADC = analogRead(SENSOR_PIN);      // Leer valor ADC (0-1023)
  float voltaje = (valorADC * VREF) / 1023.0; // Convertir a voltaje

  Serial.print("ADC: ");
  Serial.print(valorADC);
  Serial.print("  |  Voltaje: ");
  Serial.print(voltaje, 2);
  Serial.println(" V");

  delay(1000); // Leer cada 1 segundo
}
