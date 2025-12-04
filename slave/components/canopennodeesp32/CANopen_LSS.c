#include "CANopen.h"
#include "CO_LSSslave.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "OD.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- CONFIGURACIÓN ---
#define PIN_BOTON_EMERGENCIA GPIO_NUM_0
#define PIN_LED_ESTADO       GPIO_NUM_2
#define TAG "CO_LOGIC"

// Prioridades
#define MAIN_TASK_PRIO       4
#define PERIODIC_TASK_PRIO   5 

// TIEMPOS (10ms mínimo para evitar Watchdog)
#define MAIN_INTERVAL_MS     100
#define PERIODIC_INTERVAL_MS 10   

// Control NMT corregido
#define NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

static CO_t *CO = NULL;
static uint32_t heapMemoryUsed = 0;

// Variables Globales
static uint16_t g_bitRate;
static uint8_t  g_nodeId;
static bool b_emergencia_activa = false;

// Variables para el envío automático
static uint64_t last_auto_send_time_us = 0;
static uint8_t  contador_dummy = 0;

TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t periodicTaskHandle = NULL;

static void CO_mainTask(void *pxParam);
static void CO_periodicTask(void *pxParam);

// --- STORAGE ---
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
static CO_storage_t storage; 
static CO_storage_entry_t storageEntry1;
static CO_storage_entry_t *storageEntries[] = {&storageEntry1};
static uint8_t storageEntriesCount = 0;
static uint32_t storageInitError = 0;

static int config_storage(void){
    storageEntry1 = (CO_storage_entry_t){
        .addr = &OD_PERSIST_COMM, .len = sizeof(OD_PERSIST_COMM), .subIndexOD = 2,
        .attr = CO_storage_cmd | CO_storage_restore, .addrNV = NULL 
    };
    storageEntriesCount = 1;
    return CO_storageESP32_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters, OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount, &storageInitError);
}
#endif

static uint32_t getSerialNumberFromMAC() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BASE);
    return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | ((uint32_t)mac[5]);
}

// -------------------------------------------------------------------------
// FUNCIÓN DE ARRANQUE
// -------------------------------------------------------------------------
void CO_ESP32_LSS_Run(uint16_t pendingBitRate, uint8_t pendingNodeId) {
    g_bitRate = pendingBitRate;
    g_nodeId = pendingNodeId;

    // Hardware
    gpio_reset_pin(PIN_BOTON_EMERGENCIA);
    gpio_set_direction(PIN_BOTON_EMERGENCIA, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BOTON_EMERGENCIA, GPIO_PULLUP_ONLY);
    gpio_reset_pin(PIN_LED_ESTADO);
    gpio_set_direction(PIN_LED_ESTADO, GPIO_MODE_OUTPUT);

    xTaskCreatePinnedToCore(CO_mainTask, "CO_Main", 4096, NULL, MAIN_TASK_PRIO, &mainTaskHandle, 1);
}

// -------------------------------------------------------------------------
// TAREA PRINCIPAL (100ms) - Monitor y Gestión
// -------------------------------------------------------------------------
static void CO_mainTask(void *pxParam) {
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    void* CANptr = NULL;

    CO = CO_new(NULL, &heapMemoryUsed);
    
    #if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    config_storage();
    #endif

    while (reset != CO_RESET_APP) {
        ESP_LOGI(TAG, "Iniciando Comunicacion...");
        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);

        if(CO_CANinit(CO, CANptr, g_bitRate) != CO_ERROR_NO) {
            ESP_LOGE(TAG, "Error CAN Init"); vTaskDelay(pdMS_TO_TICKS(100)); continue;
        }

        uint32_t serial_number = getSerialNumberFromMAC();
        CO_LSS_address_t lss_address = {
            .identity = { .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                          .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                          .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                          .serialNumber = serial_number }
        };
        uint8_t actualNodeId = g_nodeId;
        CO_LSSinit(CO, &lss_address, &actualNodeId, &g_bitRate);

        uint32_t errInfo = 0;
        CO_CANopenInit(CO, NULL, NULL, OD, NULL, NMT_CONTROL, 500, 1000, 500, false, actualNodeId, &errInfo);
        CO_CANopenInitPDO(CO, CO->em, OD, actualNodeId, &errInfo);

        if (periodicTaskHandle == NULL) {
            ESP_LOGI(TAG, "Creando Tarea Periodica...");
            xTaskCreatePinnedToCore(CO_periodicTask, "CO_Periodic", 4096, NULL, PERIODIC_TASK_PRIO, &periodicTaskHandle, 1);
        }

        // Activamos alertas del driver
        twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED, NULL);

        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        ESP_LOGI(TAG, "NODO OPERATIVO. ID: %d", actualNodeId);

        uint32_t co_timer_us = MAIN_INTERVAL_MS * 1000;

        while (reset == CO_RESET_NOT) {
            vTaskDelay(pdMS_TO_TICKS(MAIN_INTERVAL_MS)); 

            reset = CO_process(CO, false, co_timer_us, NULL);
            if (CO->LSSslave) CO_LSSslave_process(CO->LSSslave);

            // --- MONITOR DE TRÁFICO ---
            // Esto te dirá si las tramas realmente salen al cable
            uint32_t alerts = 0;
            if (twai_read_alerts(&alerts, 0) == ESP_OK) {
                if (alerts & TWAI_ALERT_TX_SUCCESS) {
                    // Solo imprimimos un punto para no saturar si es automático
                    // Pero si quieres verlo claro, descomenta la siguiente línea:
                    ESP_LOGI(TAG, "<<< [BUS] TX OK (ACK)"); 
                }
                if (alerts & TWAI_ALERT_TX_FAILED) {
                    ESP_LOGE(TAG, "XXX [BUS] TX FALLIDO (Nadie escucha)");
                }
                if (alerts & TWAI_ALERT_RX_DATA) {
                    ESP_LOGI(TAG, ">>> [BUS] RX DATA");
                }
            }
        }
        
        CO_CANsetConfigurationMode(CANptr);
        CO_CANmodule_disable(CO->CANmodule);
    }

    if(periodicTaskHandle != NULL) { vTaskDelete(periodicTaskHandle); periodicTaskHandle = NULL; }
    CO_delete(CO);
    vTaskDelete(NULL);
}

// -------------------------------------------------------------------------
// TAREA PERIÓDICA (10ms) - Lógica de Usuario
// -------------------------------------------------------------------------
static void CO_periodicTask(void *pxParam) {
    uint32_t co_timer_us = PERIODIC_INTERVAL_MS * 1000;
    last_auto_send_time_us = esp_timer_get_time();

    while(1) {
        // Retraso de 10ms (Seguro para Watchdog)
        vTaskDelay(pdMS_TO_TICKS(PERIODIC_INTERVAL_MS)); 

        if (!CO->CANmodule->CANnormal) continue; 

        uint64_t now_us = esp_timer_get_time();

        // Procesar PDOs
        bool syncWas = false; 
        CO_process_RPDO(CO, syncWas, co_timer_us, NULL); 
        CO_process_TPDO(CO, syncWas, co_timer_us, NULL);

        // ============================================================
        // A. ENVÍO AUTOMÁTICO (Cada 1 Segundo)
        // ============================================================
        if ((now_us - last_auto_send_time_us) > 1000000) {
            last_auto_send_time_us = now_us;
            contador_dummy++;
            
            // 1. Escribimos en el OD
            OD_RAM.x6000_readDigitalInput8_bit[0] = contador_dummy;
            // 2. ¡AQUÍ ESTÁ EL TRUCO! 
            // 2. FORZAR ENVÍO (CORREGIDO: USAR PUNTO, NO FLECHA)
            #if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
                        CO->TPDO[0].sendRequest = 1;
            #endif

            // 2. AVISO VISUAL (He añadido esto para que sepas que el timer funciona)
            ESP_LOGI(TAG, "[AUTO] Actualizando OD 0x6000 a: %d", contador_dummy);
        }

        // ============================================================
        // B. EMERGENCIA (BOTÓN)
        // ============================================================
        bool boton_activo = (gpio_get_level(PIN_BOTON_EMERGENCIA) == 0);
        
        if (boton_activo) {
            if (!b_emergencia_activa) {
                b_emergencia_activa = true;
                
                // --- HE RESTAURADO EL LOG DE EMERGENCIA ---
                ESP_LOGE(TAG, "!!! EMERGENCIA ACTIVADA (Boton pulsado) !!!");
                
                CO_errorReport(CO->em, 1, CO_EMC_GENERIC, 0x5050);
            }
        } else {
            if (b_emergencia_activa) {
                b_emergencia_activa = false;
                ESP_LOGI(TAG, "Emergencia Desactivada.");
                CO_errorReset(CO->em, 1, 0);
            }
        }

        // C. LED
        uint8_t led_val = OD_RAM.x6200_writeDigitalOutput8_bit[0];
        gpio_set_level(PIN_LED_ESTADO, (led_val & 0x01));
    }
}