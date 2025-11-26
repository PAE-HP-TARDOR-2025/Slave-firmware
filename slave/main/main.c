#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "CO_driver.h"
#include "CO_NMT_Heartbeat.h"
#include "CANopen.h"
#include "CO_LSSslave.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_efuse.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "OD.h"


/*---------------------- Configuracio Macros ------------------*/
//Revisar el que estigui posat al driver i al target. 
#define NMT_CONTROL (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION )//diferents estats del NMT. 
#define FIRST_HB_TIME        500 //temps en milisegons per enviar primer heartbeat
#define SDO_SRV_TIMEOUT_TIME 1000 //temps espera servidor SDO
#define SDO_CLI_TIMEOUT_TIME 500 //temps espera client SDO
#define SDO_CLI_BLOCK        false //habilitar/deshabilitar transferència de blocs en SDO. 
#define OD_STATUS_BITS       NULL //array de bits que indiquen estat de cada objecte del OD (si està en error, per exemple)
/*definim això per poder inicialitzar CANopen. En funció de les prestacions hauríem de poder canviar-ho*/

/*---------------------- Globals ------------------------------*/
static CO_t *CO = NULL;//&CO_instance;//estruct principal de CanOpen.  NMT, SDO, PDO, SYNC, etc.
static CO_config_t *config_ptr = 0; 
static uint32_t heapMemoryUsed = 0;
static const char *TAG = "MAIN";


/*---------------------- Storage Variables -------------------*/
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
static CO_storage_t storage; 
static CO_storage_entry_t storageEntries[1]; // Ajusta el tamaño según tus entries
static uint8_t storageEntriesCount = 0;
static uint32_t storageInitError = 0;
#endif

/*Storage en aquest cas ens cal perquè sinó cada vegada que féssim un reset no només s'eborrarien
les configuracions del node sinó que també esborrariem altres configuracions del diccionari i etc.*/

/*---------------------- Config Storage ----------------------*/
int config_storage(void){
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_ReturnError_t err;

    // Definimos los objetos que queremos almacenar
    storageEntries[0] = (CO_storage_entry_t){
        .addr = &OD_PERSIST_COMM,
        .len = sizeof(OD_PERSIST_COMM),
        .subIndexOD = 2,
        .attr = CO_storage_cmd | CO_storage_restore,
        .addrNV = NULL // ESP32 usará NVS
    };

    storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    storageInitError = 0;

    // Inicializamos el módulo de storage
    err = CO_storageBlank_init(&storage,
                               CO->CANmodule,
                               OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters,
                               storageEntries,
                               storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        ESP_LOGE(TAG, "Error initializing storage: %d", storageInitError);
        return -1;
    }

#endif
    return 0;
}


/*----------------------  Funcions Complementaries ------------*/
void config_microcontroller(){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

uint32_t getSerialNumberFromMAC() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BASE);
    //Dona una direcció MAC BASE (mac unica grabada en l'eFuse de fabrica de la ESP32. )

    return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8)  | ((uint32_t)mac[5]);

}

uint64_t get_time_ESP(){
    return esp_timer_get_time();
}

void app_main(void){
    CO_ReturnError_t err; //Per fer saltar errors
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT; //quin tipus de reinicialització li estem demanant
    void* CANptr = NULL; 

    //la gràcia es intentar inicialitzar això des del LSS. 
    uint16_t pendingBitRate = 250;// Aixì espera 250 kbps
    uint8_t pending_node_id = 10;
    uint8_t active_node_id = 10; //Ha de ser del [1..127]   
    uint32_t serial_number = 0;
    uint64_t lastTime_us = 0; 
    /*(1 <= pendingNodeID <= 0x7F)
    If pendingNodeID is not valid (pendingNodeID == 0xFF), then only LSS slave is initialized and processed in run time.
    In that state pendingNodeID can be configured and after successful configuration reset communication with all CANopen
    object is activated automatically.
    read from dip switches or nonvolatile memory, configurable by LSS slave */


    config_ptr = NULL;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        printf("Error: Can't allocate memory");
    } else {
        printf("Allocated bytes for CANopen objects");
    }

    config_storage(); //ha d'anar després d'iniciar el CO
    if (config_storage() != 0) {
        CO_delete(CO); // alliberem memoria si l'hem guardat
    }
    config_microcontroller(); 

    //Reinicialitza el CAN i els seus mòduls
    while(reset != CO_RESET_APP){
        
        printf("Reboot SLAVE has started.... Config mode");
        
        CO->CANmodule->CANnormal = false; //no s'envvia res mentre hardware fa la reconfiguració. Node NO en mode normal
        CO_CANsetConfigurationMode(CANptr);//CAN controller en mode config
        CO_CANmodule_disable(CO->CANmodule); //desactiva modul CAN per lliberar buffers, no enviar ni rebre
        //desactivar el modul CAN fisic associat al stack CANopenNode. 

        //Inicialitza el módul CAN
        err = CO_CANinit(CO, CANptr, pendingBitRate); //punter objecte canOpen, punter config nostra estructura, baudrate
        if (err != CO_ERROR_NO){
            ESP_LOGE(TAG, "Error: CAN initialization failed %d", err); 
        }

        //M'agradaria canviar-los
        //tot això s'hauria de definir a l'OD, no aquí. De moment ho faig així, però sense OD no faig res. 
        serial_number = getSerialNumberFromMAC();
        CO_LSS_address_t lss_address = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                    .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                    .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                    .serialNumber = serial_number}};
       
        /*pending node ID: Nou node-ID que ha estat rebut pel node a través del protocol LSS*/
        
        err = CO_LSSinit(CO, &lss_address, &pending_node_id, &pendingBitRate);
        if (err != CO_ERROR_NO){
            ESP_LOGE(TAG, "Error: CAN initialization failed: %d", err);
        }

        active_node_id = pending_node_id; /*active Node Id: Node-ID que el node utilitzarà per la transmissió CAN*/ //és una assignacio normal
        
        //inicialització CANOpen
        uint32_t errInfo = 0;
        err = CO_CANopenInit(CO, NULL, NULL, OD, OD_STATUS_BITS, NMT_CONTROL, FIRST_HB_TIME, SDO_SRV_TIMEOUT_TIME, SDO_CLI_TIMEOUT_TIME, SDO_CLI_BLOCK, active_node_id, &errInfo);
        /*CANOpen object, alternate NMT, alternate em, Object Dictionary, Optional OD_statusBit, CO_NMT_CONTROL_t, firstHBTime_ms, SDOserverTimeoutTime_ms, SDOclientTimeoutTime_ms, SDOclientBlockTransfer*/
        //alternate NMT: estructura alternativa NMT. No necessària. NULL
        //alternate em: Punter opcional per una implementació alternativa al Emergency Message. NULL. 
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                ESP_LOGE(TAG, "Error: Object Dictionary entry");
            } else {
                ESP_LOGE(TAG, "Error: CANopen initialization failed: %d", err);
            }
        }

        //inicialització del PDO
        err = CO_CANopenInitPDO(CO, CO->em, OD, active_node_id, &errInfo);
        if (err != CO_ERROR_NO) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                ESP_LOGE(TAG,"Error: Object Dictionary entry");
            } else {
                ESP_LOGE(TAG, "Error: PDO initialization failed: %d", err);
            }
        }

        //mirem si el node està iniciat. Si no, missatge error. Si està configurat i l'emmagatzematge està habilitat
        //el que fa és dir-nos si hi ha hagut problemes en inicialitzar la memòria volàtil
        if (!CO->nodeIdUnconfigured) { //si igual a true, node NO configurat
            //i la configuració està declarada i enabled, 
            #if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
                if (storageInitError != 0) {
                    CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
                }
            #endif
        }
        else {
            printf("CANopenNode - Node-id not initialized");
        }
        

        CO_CANsetNormalMode(CO->CANmodule); /* start CAN. Mode del controlador CAN en normal (TX/RX). Si no el node concret no entra dins de la xarxa*/
        reset = CO_RESET_NOT;/*Avisa que el node no s'ha de reiniciar. Aquest valor pot canviar si CO_process() detecta algun event de reset (NMT o error greu)*/
        lastTime_us = get_time_ESP();

        printf("Inici Funcionament CAN");
        fflush(stdout); //assegurar que el missatge s'imprimeixi JA

        //inicia el bucle principal.
        //Bucle continu mentre el node en estat normal
        /***************************** Normal Node Execution **************************/
        while (reset == CO_RESET_NOT) {
            /* get time difference since last function call */
            
            uint64_t nowTime_us = get_time_ESP();
            uint32_t timeDifference_us = (uint32_t)(nowTime_us - lastTime_us);
            lastTime_us = get_time_ESP();

            /*Actualitzem l'estat del node CANopen. Passa un valor de reset si cal que es reinici*/
            reset = CO_process(CO, false, timeDifference_us, NULL);
            /*punter a CO, 1ms? calcular o no el temps, temps des de l'última 
            trucada en ms, altApp: punter opcional a una aplicatió alternativa*/

            /*Aquí aniria la lògica del programa*/
            
            
            /*Aquí surts dels dos whiles*/
            if(reset == CO_RESET_COMM){
                printf("Communication reset requested");
                break; 
            }
        }

        CO_CANsetConfigurationMode((void*)&CANptr);
        CO_CANmodule_disable(CO->CANmodule); 
        printf("CAN open stack stopped");
    }

    /* program exit ***************************************************************/
    /* stop threads */
    /* delete objects from memory */
    
    CO_delete(CO);
    printf("Finalitzat");

    /* reset */
    return;
}

