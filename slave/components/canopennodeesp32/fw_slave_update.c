#include "fw_slave_update.h"

#include <stdio.h>
#include <string.h>

#ifndef FW_SLAVE_LOG
#define FW_SLAVE_LOG(fmt, ...) printf("[FW-SLAVE] " fmt, ##__VA_ARGS__)
#endif

#ifndef FW_SLAVE_ERR
#define FW_SLAVE_ERR(fmt, ...) printf("[FW-ERR ] " fmt, ##__VA_ARGS__)
#endif

#ifndef FW_SLAVE_WARN
#define FW_SLAVE_WARN(fmt, ...) printf("[FW-WARN] " fmt, ##__VA_ARGS__)
#endif

#define RETURN_IF_FALSE(cond, msg, ...)                                                              \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            FW_SLAVE_ERR(msg "\n", ##__VA_ARGS__);                                                  \
            return false;                                                                            \
        }                                                                                            \
    } while (0)

void
fw_slave_reset_context(fw_update_context_t* ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->stage = FW_STAGE_IDLE;
    ctx->currentBank = 0U;
}

uint16_t
fw_slave_crc16_step(uint16_t seed, uint8_t data) {
    seed ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++) {
        if (seed & 0x8000U) {
            seed = (uint16_t)((seed << 1) ^ 0x1021U);
        } else {
            seed <<= 1;
        }
    }
    return seed;
}

bool
fw_slave_prepare_storage(fw_update_context_t* ctx) {
    FW_SLAVE_LOG("Preparing flash bank %u for new image...\n", ctx->currentBank);
    RETURN_IF_FALSE(ctx->stage == FW_STAGE_METADATA_READY, "Cannot erase flash before metadata step");

    ctx->flashPrepared = true;
    ctx->stage = FW_STAGE_ERASING_FLASH;
    FW_SLAVE_LOG("Flash bank %u erased successfully (simulated).\n", ctx->currentBank);
    ctx->stage = FW_STAGE_RECEIVING_BLOCKS;
    return true;
}

bool
fw_slave_store_metadata(fw_update_context_t* ctx, uint32_t size, uint16_t crc, uint8_t bank) {
    FW_SLAVE_LOG("Received metadata: size=%lu crc=0x%04X bank=%u\n", (unsigned long)size, crc, bank);

    RETURN_IF_FALSE(size > 0U, "Metadata rejected: size is zero");
    RETURN_IF_FALSE(size <= FW_MAX_IMAGE_SIZE_BYTES, "Metadata rejected: size %lu exceeds limit", (unsigned long)size);
    RETURN_IF_FALSE(crc != 0x0000U, "Metadata rejected: CRC cannot be zero");

    ctx->expectedSize = size;
    ctx->expectedCrc = crc;
    ctx->currentBank = bank;
    ctx->stage = FW_STAGE_METADATA_READY;
    ctx->metadataReceived = true;
    ctx->runningCrc = 0xFFFFU;
    FW_SLAVE_LOG("Metadata accepted; expecting %lu bytes.\n", (unsigned long)ctx->expectedSize);
    return true;
}

bool
fw_slave_receive_chunk(fw_update_context_t* ctx, const uint8_t* data, uint32_t len, uint32_t offset) {
    RETURN_IF_FALSE(ctx->stage == FW_STAGE_RECEIVING_BLOCKS, "Chunk rejected: wrong stage %d", ctx->stage);
    RETURN_IF_FALSE(ctx->flashPrepared, "Chunk rejected: flash not prepared");
    RETURN_IF_FALSE(data != NULL, "Chunk rejected: NULL pointer");
    RETURN_IF_FALSE(len > 0U, "Chunk rejected: length zero");
    RETURN_IF_FALSE(offset == ctx->receivedBytes, "Chunk rejected: expected offset %lu got %lu",
                    (unsigned long)ctx->receivedBytes, (unsigned long)offset);

    ctx->receivedBytes += len;

    for (uint32_t i = 0; i < len; i++) {
        ctx->runningCrc = fw_slave_crc16_step(ctx->runningCrc, data[i]);
    }

    FW_SLAVE_LOG("Chunk @%lu (%u bytes) accepted; total=%lu/%lu\n", (unsigned long)offset, (unsigned)len,
                 (unsigned long)ctx->receivedBytes, (unsigned long)ctx->expectedSize);

    RETURN_IF_FALSE(ctx->receivedBytes <= ctx->expectedSize, "Overflow: received %lu > expected %lu",
                    (unsigned long)ctx->receivedBytes, (unsigned long)ctx->expectedSize);
    return true;
}

bool
fw_slave_finalize(fw_update_context_t* ctx) {
    RETURN_IF_FALSE(ctx->receivedBytes == ctx->expectedSize, "Finalize refused: size mismatch");
    ctx->stage = FW_STAGE_VERIFYING;

    ctx->crcMatched = (ctx->runningCrc == ctx->expectedCrc);
    if (!ctx->crcMatched) {
        FW_SLAVE_ERR("CRC mismatch: computed 0x%04X expected 0x%04X\n", ctx->runningCrc, ctx->expectedCrc);
        return false;
    }

    ctx->stage = FW_STAGE_READY_TO_BOOT;
    FW_SLAVE_LOG("CRC validated (0x%04X). Image ready in bank %u\n", ctx->runningCrc, ctx->currentBank);
    return true;
}

void
fw_slave_dump_context(const fw_update_context_t* ctx) {
    FW_SLAVE_LOG("--- Firmware context snapshot ---\n");
    FW_SLAVE_LOG(" stage          : %d\n", ctx->stage);
    FW_SLAVE_LOG(" metadata ready : %s\n", ctx->metadataReceived ? "yes" : "no");
    FW_SLAVE_LOG(" flash prepared : %s\n", ctx->flashPrepared ? "yes" : "no");
    FW_SLAVE_LOG(" expected size  : %lu bytes\n", (unsigned long)ctx->expectedSize);
    FW_SLAVE_LOG(" received bytes : %lu bytes\n", (unsigned long)ctx->receivedBytes);
    FW_SLAVE_LOG(" expected crc   : 0x%04X\n", ctx->expectedCrc);
    FW_SLAVE_LOG(" running crc    : 0x%04X\n", ctx->runningCrc);
    FW_SLAVE_LOG(" crc matched    : %s\n", ctx->crcMatched ? "yes" : "no");
    FW_SLAVE_LOG("----------------------------------\n");
}
