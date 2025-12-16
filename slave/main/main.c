#include <stdio.h>
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Tu librería encapsulada (El .h que definimos antes)
#include "CANopen_LSS.h" 

// Definiciones de pines según tu placa
#define GPIO_BOTON_EMERGENCIA   GPIO_NUM_0 
#define GPIO_CAN_ENABLE         GPIO_NUM_16 

static const char *TAG = "APP_MAIN";

// SETUP HARDWARE (Transceptor y Pines)
void setup_hardware_externo()
{
    ESP_LOGI(TAG, "Configurando hardware externo...");
    
    // 1. Configurar PIN ENABLE del Transceptor CAN (Importante)
    // Muchos transceptores necesitan este pin en HIGH para funcionar.
    gpio_reset_pin(GPIO_CAN_ENABLE);
    gpio_set_direction(GPIO_CAN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_CAN_ENABLE, 1); // Activamos el chip CAN

    // 2. Configurar Botón (Redundante, la librería también lo hace, pero está bien dejarlo)
    gpio_reset_pin(GPIO_BOTON_EMERGENCIA);
    gpio_set_direction(GPIO_BOTON_EMERGENCIA, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BOTON_EMERGENCIA, GPIO_PULLUP_ONLY);
}

// MAIN
void app_main(void)
{
    // 1. Inicializar Flash NVS 
    // CRÍTICO: CANopenNode usa esto para guardar el NodeID (LSS) y parámetros.
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_LOGW(TAG, "NVS corrupta, borrando y reiniciando...");
        nvs_flash_erase();
        nvs_flash_init();
    // }

    // 2. Configurar pines físicos
    setup_hardware_externo();

    ESP_LOGI(TAG, "--- ARRANCANDO NODO CANOPEN (Híbrido PDO/Emergencia) ---");

    // 3. Arrancar la librería CANopen
    // - 500: Bitrate en kbps
    // - 0x20: NodeID por defecto (32 decimal). 
    //   NOTA: Si ya se guardó un ID por LSS en la flash, el 0x20 se ignora.
    CO_ESP32_LSS_Run(500, 0xFF);

    // 4. Bucle infinito para mantener vivo el main (aunque la tarea CAN va por libre)
    while(1) {
        // Solo para depuración visual de que el chip no se ha colgado
        vTaskDelay(pdMS_TO_TICKS(10000)); 
        // ESP_LOGI(TAG, "Main vivo...");
    }
}