#ifndef CANOPEN_LSS_H
#define CANOPEN_LSS_H

#include <stdint.h>

// -------------------------------------------------------------------------
// DECLARACIÓN DE FUNCIONES PÚBLICAS
// -------------------------------------------------------------------------

/**
 * @brief Arranca el nodo CANopen en el ESP32.
 * * Esta función inicializa el driver CAN, el Diccionario de Objetos,
 * y lanza las tareas de FreeRTOS necesarias (Principal y Periódica).
 * * @param pendingBitRate Velocidad del bus CAN en kbps (ej: 500).
 * @param pendingNodeId  ID del nodo por defecto (ej: 0x20). 
 * Nota: Si hay un ID guardado en Flash (LSS), se usará ese.
 */
void CO_ESP32_LSS_Run(uint16_t pendingBitRate, uint8_t pendingNodeId);

#endif // CANOPEN_LSS_H