/*

#include<Arduino.h>

void anotherTask(void *parameter);

void setup() {
  Serial.begin(112500);

  xTaskCreate(
      anotherTask,     
      "another Task",  
      10000,           
      NULL,             
      1,               
      NULL              
  );
}

void loop() {
  Serial.println("this is ESP32 Task");
  delay(1000);
}

void anotherTask(void *parameter) {
  
  for (;;) {
    Serial.println("this is another Task");
    delay(1000);
  }

  vTaskDelete(NULL);
}

*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LED_ROJO 47  // LED rojo en el pin 47
#define LED_VERDE 39 // LED verde en el pin 39

SemaphoreHandle_t xSemaphore;

void tareaEncenderRojo(void *pvParameters) {
    while (1) {
        xSemaphoreTake(xSemaphore, portMAX_DELAY);
        digitalWrite(LED_ROJO, HIGH);
        Serial.println("LED Rojo Encendido");
        vTaskDelay(pdMS_TO_TICKS(1000));
        digitalWrite(LED_ROJO, LOW);
        Serial.println("LED Rojo Apagado");
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void tareaEncenderVerde(void *pvParameters) {
    while (1) {
        xSemaphoreTake(xSemaphore, portMAX_DELAY);
        digitalWrite(LED_VERDE, HIGH);
        Serial.println("LED Verde Encendido");
        vTaskDelay(pdMS_TO_TICKS(1000));
        digitalWrite(LED_VERDE, LOW);
        Serial.println("LED Verde Apagado");
        xSemaphoreGive(xSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_ROJO, OUTPUT);
    pinMode(LED_VERDE, OUTPUT);

    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore == NULL) {
        Serial.println("Error creando el semáforo");
        return;
    }

    xSemaphoreGive(xSemaphore);

    xTaskCreate(tareaEncenderRojo, "Tarea Encender Rojo", 4096, NULL, 1, NULL);
    xTaskCreate(tareaEncenderVerde, "Tarea Encender Verde", 4096, NULL, 1, NULL);
}

void loop() {
    // No hacer nada en loop, las tareas de FreeRTOS manejan el flujo
}