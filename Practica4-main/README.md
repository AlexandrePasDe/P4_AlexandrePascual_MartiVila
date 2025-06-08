# Practica 4: Sistemas operativos en tiempo real(FreeRTOS)
Participants: Alexandre Pascual / Marti Vila


## Introducción
En esta práctica se ha explorado el funcionamiento de un _Sistema operativo en tiempo real_ en el **ESP32**, utilizando **FreeRTOS** para manejar múltiples tareas en paralelo.

El laboratorio se divide en dos partes:
1. **Creación de tareas en FreeRTOS**.
2. **Semáforos**, controlando el encendido de LEDs.


## Parte 1: Creación de Tareas con FreeRTOS

### **Objetivo**
Implementar y ejecutar **múltiples tareas simultàneas** en el ESP32 utilizando FreeRTOS.

#### **Código Implementado**
```c++
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
```

### **Explicación del Código**
- Se crea una tarea en _FreeRTOS_ (`anotherTask`) usando `xTaskCreate()`.
- La tarea imprime `"this is another Task"` cada segundo en el **monitor serie**.
- Simultániamente, `loop()` imprime `"this is ESP32 Task"`.

### **Salida Esperada en el Monitor Serie**
```
this is ESP32 Task
this is another Task
this is ESP32 Task
this is another Task
...
```

## Parte 2: Sincronización de Tareas con Semáforos

### **Objetivo**
Utilizar **semáforos en FreeRTOS** para sincronizar dos tareas que controlan dos **LEDs**.

### **Desarrollo**
Se implementaron dos tareas, donde:
- Una tarea enciende y apaga el LED **rojo**.
- Otra tarea enciende y apaga el LED **verde**.
- Se usa un **semáforo** para evitar que ambos LEDs se enciendan al mismo tiempo.

#### **Código Implementado**
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
    // No hacer nada en loop, las tareas de FreeRTOS manejan el flujo
}
```

### **Explicación del Código**
- Se definen dos tareas: `tareaEncenderRojo()` y `tareaEncenderVerde()`.
- Se usa un **semáforo** (`xSemaphore`) para evitar que ambos LEDs se enciendan a la vez.
- Cada LED se **enciende por 1 segundo** y luego se apaga según el funcionamiento del semáforo.
- El `loop()` está vacío, ya que el control de los LEDs lo gestionan las tareas de FreeRTOS.

### **Salida Esperada en el Monitor Serie**
```
LED Rojo Encendido
LED Rojo Apagado
LED Verde Encendido
LED Verde Apagado
...
```

## Conclusiones
- **FreeRTOS permite la ejecución de múltiples tareas** de forma eficiente en el ESP32.
- Implementamos un **paralelismo real**, mostrando cómo dos tareas pueden ejecutarse por separado.
- Se usó un **semáforo para sincronización** para evitar que se superponga una tarea con la otra.
- Se ha logrado **controlar LEDs de manera sincronizada**, demostrando un uso práctico de FreeRTOS.

