# P4_AlexandrePascual_MartiVila

## Sistemes operatius en temps real

## Objectius

Aprendre el funcionament de un sistema operatiu en temps real en la nostra placa ESP32 utilitzant 'FreeRTOS' per a controlar diverses feines a dur a terme alhora.
Per això en el primer apartat es mostra com hem aprés a crear "tareas" en 'FreeRTOS' i en el segon es mostrarà com ho hem implementat al laboratori en un exemple més pràctic com la creació d'un semàfor on s'encenen i apagen LEDs.
En esta práctica se ha explorado el funcionamiento de un _Sistema operativo en tiempo real_ en el **ESP32**, utilizando **FreeRTOS** para manejar múltiples tareas en paralelo.

## 1.Creació de feines en FreeRTOS:

### **Objectiu**
Implementar y executar diverses feines simultaneament en el ESP32 utilitzant FreeRTOS.

#### **Codi**
```c++
#include<Arduino.h>

void anotherTask(void *parameter);

void setup() {
  Serial.begin(112500); //Inicia una comunicació serial

  xTaskCreate( // Crea una `Task'
      anotherTask,     
      "another Task",  
      10000,           
      NULL,             
      1,               
      NULL              
  );
}

void loop() {
  Serial.println("this is ESP32 Task"); // Aquest bucle anira imprimint cada segon el missatge indicat
  delay(1000);
}

void anotherTask(void *parameter) {
  
  for (;;) {
    Serial.println("this is another Task"); // Aquest també anira imprimint el missatge indicat cada segon, de manera simultanea
    delay(1000);
  }

  vTaskDelete(NULL);
}
```

### Explicació

Es crea un **task** en _FreeRTOS_ ('anotherTask') en el 'xTaskCreate' i el que farà serà que s'imprimeixi, a cada segon, el missatge de **"this is another Task"**, mentre que simultaneament, també cada segon, el bucle imprimirà el missatge al monitor de **"this is ESP32 Task"**.


### Sortida Esperada
```
this is ESP32 Task
this is another Task
this is ESP32 Task
this is another Task
...
```

## Semàfor(Sincronització de tasks)

### Objectiu
Utilitzar semàfors en FreeRTOS per sincronitzar tasks que fagin interaccionar **LEDs**.
Per això, hem implementat dos tasks, **la primera** encen i apaga el LED **vermell**, **la segona** farà el mateix amb un LED **verd**. 
Utilitzem un semàfor per evitar una engegada alhora de LEDs, es a dir, per evitar malfuncionaments.

#### Codi
```c++
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
    //Les Tasks de FreeRTOS s'encarregen del funcionament dels LEDs
}
```

### Explicació

Es defineixen les dues tasks que controlen el funciomnament dels LEDs, aquestes són `tareaEncenderRojo()`, que s'encarrega del vermell, i `tareaEncenderVerde()`, que s'encarrega del verd. Afegim un semàfor (`xSemaphore`) per evitar el malfuncionament que voliem evitar comentat a l'apartat d'objectiu. Cada LED s'encendrà per **1 segon** i després s'apagarà segons el funcionament del semàfor.

El 'loop()' esta buit ja que el control dels LEDs es gestionat per les tasks.

### Sortida Esperada 
```
LED Rojo Encendido
LED Rojo Apagado
LED Verde Encendido
LED Verde Apagado
...
```

## Resum

Hem aprés a com utilitzar tasks de 'FreeRTOS' per a dur a terme feines de manera simultanea, i eficient, i en el segon apartat hi hem aplicat un semàfor amb el qual em aprés a sincronitzar tasks i evitar així malfuncionament com la sobreposició d'una task envers una altre.
Al final em aconseguit controlar els LEDs de la manera esperada, es a dir que funcionaven de manera sincronitzada i sense sobreposició, en un apartat més pràctic de les posibles aplicacions de 'FreeRTOS'.
