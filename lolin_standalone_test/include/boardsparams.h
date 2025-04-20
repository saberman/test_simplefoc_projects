#include "Arduino.h"
#include "esp_heap_caps.h"

void doParamsCheck() {
    Serial.begin(115200);
    if (psramFound()) {
      Serial.println("PSRAM is found and enabled!");
    } else {
      Serial.println("PSRAM not found.");
    }

    
    Serial.print("Total PSRAM: ");
    Serial.println(ESP.getPsramSize());
    Serial.print("Free PSRAM: ");
    Serial.println(ESP.getFreePsram());

    size_t internalRam = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    Serial.print("Free internal SRAM: ");
    Serial.println(internalRam);

    size_t spiRam = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    Serial.print("Free external PSRAM (if available): ");
    Serial.println(spiRam);

    Serial.begin(115200);
    Serial.print("Total Heap Size: ");
    Serial.println(ESP.getHeapSize());
  
    Serial.print("Free heap memory (SRAM)");
    Serial.println(ESP.getFreeHeap());
    Serial.println("This will print the current available SRAM size (heap memory part). Typically, you'll see around 200KB in idle state, as the ESP32 chip has approximately 520KB of built-in SRAM, part of which is used for stack, program execution, RTOS, and other purposes.");
    
    Serial.println("");

    Serial.print("Minimum Free Heap since boot: ");
    Serial.println(ESP.getMinFreeHeap());
}