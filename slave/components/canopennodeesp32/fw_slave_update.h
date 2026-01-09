#ifndef FW_SLAVE_UPDATE_H
#define FW_SLAVE_UPDATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_MAX_IMAGE_SIZE_BYTES (1024U * 512U)
#define FW_CHUNK_SIZE_BYTES     64U

typedef enum {
    FW_STAGE_IDLE = 0,
    FW_STAGE_METADATA_READY,
    FW_STAGE_ERASING_FLASH,
    FW_STAGE_RECEIVING_BLOCKS,
    FW_STAGE_VERIFYING,
    FW_STAGE_READY_TO_BOOT
} fw_stage_t;

typedef struct {
    fw_stage_t stage;
    uint32_t expectedSize;
    uint32_t receivedBytes;
    uint16_t expectedCrc;
    uint16_t runningCrc;
    uint8_t currentBank;
    bool metadataReceived;
    bool flashPrepared;
    bool crcMatched;
} fw_update_context_t;

void fw_slave_reset_context(fw_update_context_t* ctx);
uint16_t fw_slave_crc16_step(uint16_t seed, uint8_t data);
bool fw_slave_prepare_storage(fw_update_context_t* ctx);
bool fw_slave_store_metadata(fw_update_context_t* ctx, uint32_t size, uint16_t crc, uint8_t bank);
bool fw_slave_receive_chunk(fw_update_context_t* ctx, const uint8_t* data, uint32_t len, uint32_t offset);
bool fw_slave_finalize(fw_update_context_t* ctx);
void fw_slave_dump_context(const fw_update_context_t* ctx);

#ifdef __cplusplus
}
#endif

#endif /* FW_SLAVE_UPDATE_H */
