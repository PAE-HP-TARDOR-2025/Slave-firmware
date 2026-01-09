#include "fw_update_server.h"

#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include <esp_timer.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "OD.h"

#define FW_CTRL_CMD_START 0x01U

#ifndef CONFIG_DEMO_SLAVE_MAX_CHUNK_BYTES
#define CONFIG_DEMO_SLAVE_MAX_CHUNK_BYTES 256
#endif

#ifndef CONFIG_DEMO_SLAVE_MAX_IMAGE_BYTES
#define CONFIG_DEMO_SLAVE_MAX_IMAGE_BYTES (512 * 1024)
#endif

static const char *TAG = "fw_server";
static esp_timer_handle_t s_rebootTimer;
static bool s_rebootScheduled;

/* NVS storage for firmware CRC and version - stores after successful OTA */
#define FW_NVS_NAMESPACE "fw_update"
#define FW_NVS_KEY_CRC   "fw_crc"
#define FW_NVS_KEY_VER   "fw_ver"

/* Forward declaration for NVS CRC retrieval */
static bool fw_load_crc_from_nvs(uint16_t *crc);
static bool fw_load_version_from_nvs(uint16_t *version);

typedef enum {
    FW_STAGE_IDLE = 0,
    FW_STAGE_METADATA_READY,
    FW_STAGE_ERASING_FLASH,
    FW_STAGE_RECEIVING_BLOCKS,
    FW_STAGE_VERIFYING,
    FW_STAGE_READY_TO_BOOT
} fw_stage_t;

typedef struct {
    uint32_t imageBytes;  /* Total firmware size in bytes */
    uint16_t crc;         /* CRC-16 CCITT of the firmware image */
    uint8_t imageType;    /* Image type identifier */
    uint8_t bank;         /* Target bank/slot */
    uint16_t version;     /* Firmware version - for version check */
} fw_metadata_record_t;  /* Total: 10 bytes */

typedef struct {
    fw_stage_t stage;
    uint32_t expectedSize;
    uint32_t receivedBytes;
    uint32_t currentChunkBase;
    uint16_t expectedCrc;
    uint16_t expectedVersion;
    uint16_t runningCrc;
    uint8_t currentBank;
    uint8_t imageType;
    bool metadataReceived;
    bool flashPrepared;
    bool crcMatched;
    bool chunkInProgress;
    const esp_partition_t *targetPartition;
    esp_ota_handle_t otaHandle;
    bool otaOpen;
} fw_update_context_t;

typedef struct {
    CO_t *co;
    fw_update_context_t ctx;
    OD_extension_t metaExt;
    OD_extension_t ctrlExt;
    OD_extension_t dataExt;
    OD_extension_t statusExt;
    OD_extension_t runningCrcExt;
    OD_extension_t runningVerExt;
    uint16_t runningFirmwareCrc;
    uint16_t runningFirmwareVersion;
} fw_server_state_t;

static fw_server_state_t s_server = {0};

static void fw_reset_context(fw_update_context_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->stage = FW_STAGE_IDLE;
    ctx->runningCrc = 0xFFFFU;
}

static void fw_reboot_cb(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Restarting to boot new firmware");
    esp_restart();
}

static void fw_schedule_reboot(void) {
    if (s_rebootScheduled) {
        return;
    }
    s_rebootScheduled = true;
    if (s_rebootTimer == NULL) {
        const esp_timer_create_args_t timerArgs = {
            .callback = fw_reboot_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "fw_reboot"
        };
        if (esp_timer_create(&timerArgs, &s_rebootTimer) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create reboot timer, restarting immediately");
            esp_restart();
        }
    }
    if (s_rebootTimer != NULL) {
        (void)esp_timer_start_once(s_rebootTimer, 500000);
    }
}

static uint16_t fw_crc16_step(uint16_t seed, uint8_t data) {
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

static uint16_t fw_compute_running_firmware_crc(void) {
    /* First, try to load CRC from NVS - this is reliable after successful OTA */
    uint16_t nvsCrc;
    if (fw_load_crc_from_nvs(&nvsCrc)) {
        ESP_LOGI(TAG, "Running firmware CRC from NVS: 0x%04X", nvsCrc);
        return nvsCrc;
    }
    ESP_LOGW(TAG, "No CRC in NVS, computing from flash (may be inaccurate)");
    
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        ESP_LOGE(TAG, "Cannot determine running partition");
        return 0U;
    }
    esp_app_desc_t appDesc;
    if (esp_ota_get_partition_description(running, &appDesc) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read app description");
        return 0U;
    }
    /* Read the entire app image and compute CRC */
    uint16_t crc = 0xFFFFU;
    size_t imageSize = running->size;
    /* Limit to actual image size from app header if available */
    const size_t chunkSize = 1024;
    uint8_t *buf = malloc(chunkSize);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Cannot allocate buffer for CRC computation");
        return 0U;
    }
    size_t offset = 0;
    while (offset < imageSize) {
        size_t toRead = (imageSize - offset) < chunkSize ? (imageSize - offset) : chunkSize;
        if (esp_partition_read(running, offset, buf, toRead) != ESP_OK) {
            break;
        }
        /* Stop at first 0xFF run (end of actual image data) */
        bool endFound = false;
        for (size_t i = 0; i < toRead; i++) {
            if (buf[i] == 0xFFU) {
                /* Check if rest is 0xFF (erased flash) */
                bool allFF = true;
                for (size_t j = i; j < toRead && allFF; j++) {
                    if (buf[j] != 0xFFU) allFF = false;
                }
                if (allFF) {
                    toRead = i;
                    endFound = true;
                    break;
                }
            }
            crc = fw_crc16_step(crc, buf[i]);
        }
        if (endFound) break;
        offset += toRead;
    }
    free(buf);
    ESP_LOGI(TAG, "Running firmware CRC: 0x%04X (partition %s)", crc, running->label);
    return crc;
}

static bool fw_store_metadata(fw_update_context_t *ctx, const fw_metadata_record_t *meta) {
    if (meta->imageBytes == 0U) {
        ESP_LOGE(TAG, "Metadata rejected: size is zero");
        return false;
    }
    if (meta->imageBytes > CONFIG_DEMO_SLAVE_MAX_IMAGE_BYTES) {
        ESP_LOGE(TAG, "Metadata rejected: size %u exceeds limit", (unsigned)meta->imageBytes);
        return false;
    }
    if (meta->crc == 0U) {
        ESP_LOGE(TAG, "Metadata rejected: CRC cannot be zero");
        return false;
    }

    ctx->expectedSize = meta->imageBytes;
    ctx->expectedCrc = meta->crc;
    ctx->expectedVersion = meta->version;
    ctx->imageType = meta->imageType;
    ctx->currentBank = meta->bank;
    ctx->receivedBytes = 0U;
    ctx->currentChunkBase = 0U;
    ctx->chunkInProgress = false;
    ctx->targetPartition = NULL;
    ctx->otaHandle = 0;
    ctx->otaOpen = false;
    ctx->runningCrc = 0xFFFFU;
    ctx->stage = FW_STAGE_METADATA_READY;
    ctx->metadataReceived = true;
    ctx->flashPrepared = false;
    ctx->crcMatched = false;

    ESP_LOGI(TAG, "Metadata accepted: size=%u bytes crc=0x%04X ver=%u bank=%u type=%u", 
             (unsigned)ctx->expectedSize, ctx->expectedCrc, ctx->expectedVersion, 
             ctx->currentBank, ctx->imageType);
    return true;
}

static bool fw_prepare_storage(fw_update_context_t *ctx) {
    if (!ctx->metadataReceived || ctx->stage != FW_STAGE_METADATA_READY) {
        ESP_LOGE(TAG, "Cannot prepare storage before valid metadata");
        return false;
    }

    const esp_partition_t *updatePart = esp_ota_get_next_update_partition(NULL);
    if (updatePart == NULL) {
        ESP_LOGE(TAG, "No OTA partition available for update");
        return false;
    }
    if (ctx->expectedSize > updatePart->size) {
        ESP_LOGE(TAG, "Image size %u exceeds OTA partition %s size %u", (unsigned)ctx->expectedSize,
                 updatePart->label, (unsigned)updatePart->size);
        return false;
    }

    esp_err_t err = esp_ota_begin(updatePart, ctx->expectedSize, &ctx->otaHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed for %s (err=0x%X)", updatePart->label, (unsigned)err);
        return false;
    }

    ctx->targetPartition = updatePart;
    ctx->otaOpen = true;
    ctx->stage = FW_STAGE_ERASING_FLASH;
    ESP_LOGI(TAG, "Prepared OTA partition %s (%u bytes)", updatePart->label, (unsigned)updatePart->size);
    ctx->flashPrepared = true;
    ctx->stage = FW_STAGE_RECEIVING_BLOCKS;
    return true;
}

static bool fw_receive_chunk(fw_update_context_t *ctx, const uint8_t *data, uint32_t len, uint32_t offset) {
    if (!ctx->flashPrepared || ctx->stage != FW_STAGE_RECEIVING_BLOCKS) {
        ESP_LOGE(TAG, "Chunk rejected: flash not prepared or wrong stage (%d)", (int)ctx->stage);
        return false;
    }
    if (!ctx->otaOpen || ctx->targetPartition == NULL) {
        ESP_LOGE(TAG, "Chunk rejected: OTA partition not ready");
        return false;
    }
    if (offset != ctx->receivedBytes) {
        ESP_LOGE(TAG, "Chunk rejected: expected offset %u got %u", (unsigned)ctx->receivedBytes, (unsigned)offset);
        return false;
    }
    if ((ctx->receivedBytes + len) > ctx->expectedSize) {
        ESP_LOGE(TAG, "Chunk rejected: would overflow image size (%u)", (unsigned)ctx->expectedSize);
        return false;
    }
    esp_err_t err = esp_ota_write(ctx->otaHandle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed at offset %u (err=0x%X)", (unsigned)offset, (unsigned)err);
        return false;
    }
    ctx->receivedBytes += len;
    for (uint32_t i = 0; i < len; i++) {
        ctx->runningCrc = fw_crc16_step(ctx->runningCrc, data[i]);
    }
    ESP_LOGI(TAG, "Chunk @%u accepted (%u bytes, total %u/%u)", (unsigned)offset, (unsigned)len,
             (unsigned)ctx->receivedBytes, (unsigned)ctx->expectedSize);
    return true;
}

/**
 * Save verified firmware CRC to NVS for reliable retrieval after reboot.
 * This is more reliable than recomputing from flash since we know the exact size.
 */
static void fw_save_crc_to_nvs(uint16_t crc) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FW_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Cannot open NVS to save CRC: 0x%X", err);
        return;
    }
    err = nvs_set_u16(handle, FW_NVS_KEY_CRC, crc);
    if (err == ESP_OK) {
        nvs_commit(handle);
        ESP_LOGI(TAG, "Saved firmware CRC 0x%04X to NVS", crc);
    } else {
        ESP_LOGW(TAG, "Failed to save CRC to NVS: 0x%X", err);
    }
    nvs_close(handle);
}

/**
 * Load firmware CRC from NVS. Returns true if found, false otherwise.
 */
static bool fw_load_crc_from_nvs(uint16_t *crc) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FW_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }
    err = nvs_get_u16(handle, FW_NVS_KEY_CRC, crc);
    nvs_close(handle);
    return (err == ESP_OK);
}

/**
 * Save verified firmware version to NVS.
 */
static void fw_save_version_to_nvs(uint16_t version) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FW_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Cannot open NVS to save version: 0x%X", err);
        return;
    }
    err = nvs_set_u16(handle, FW_NVS_KEY_VER, version);
    if (err == ESP_OK) {
        nvs_commit(handle);
        ESP_LOGI(TAG, "Saved firmware version %u to NVS", version);
    } else {
        ESP_LOGW(TAG, "Failed to save version to NVS: 0x%X", err);
    }
    nvs_close(handle);
}

/**
 * Load firmware version from NVS. Returns true if found, false otherwise.
 */
static bool fw_load_version_from_nvs(uint16_t *version) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FW_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }
    err = nvs_get_u16(handle, FW_NVS_KEY_VER, version);
    nvs_close(handle);
    return (err == ESP_OK);
}

static bool fw_finalize(fw_update_context_t *ctx, uint16_t crc) {
    if (ctx->stage != FW_STAGE_RECEIVING_BLOCKS) {
        ESP_LOGE(TAG, "Finalize refused: wrong stage %d", ctx->stage);
        return false;
    }
    if (!ctx->otaOpen || ctx->targetPartition == NULL) {
        ESP_LOGE(TAG, "Finalize refused: OTA session not active");
        return false;
    }
    if (ctx->receivedBytes != ctx->expectedSize) {
        ESP_LOGE(TAG, "Finalize refused: received %u bytes but expected %u", (unsigned)ctx->receivedBytes,
                 (unsigned)ctx->expectedSize);
        return false;
    }
    ctx->stage = FW_STAGE_VERIFYING;
    if (ctx->runningCrc != crc || ctx->runningCrc != ctx->expectedCrc) {
        ESP_LOGE(TAG, "CRC mismatch: computed 0x%04X expected 0x%04X (declared 0x%04X)", ctx->runningCrc,
                 crc, ctx->expectedCrc);
        return false;
    }
    esp_err_t err = esp_ota_end(ctx->otaHandle);
    ctx->otaOpen = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (err=0x%X)", (unsigned)err);
        return false;
    }

    err = esp_ota_set_boot_partition(ctx->targetPartition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set boot partition to %s (err=0x%X)", ctx->targetPartition->label, (unsigned)err);
        return false;
    }

    ctx->crcMatched = true;
    ctx->stage = FW_STAGE_READY_TO_BOOT;
    ESP_LOGI(TAG, "Firmware image validated (crc=0x%04X, ver=%u). Next boot will use partition %s", 
             ctx->runningCrc, ctx->expectedVersion, ctx->targetPartition->label);
    
    /* Save the verified CRC and version to NVS so we can reliably report them after reboot */
    fw_save_crc_to_nvs(ctx->runningCrc);
    fw_save_version_to_nvs(ctx->expectedVersion);
    
    fw_schedule_reboot();
    return true;
}

static fw_server_state_t *fw_get_server(OD_stream_t *stream) {
    (void)stream;
    return &s_server;
}

static ODR_t fw_write_metadata(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten) {
    if (stream->subIndex == 0U) {
        return OD_writeOriginal(stream, buf, count, countWritten);
    }
    if (stream->subIndex != 1U) {
        return ODR_SUB_NOT_EXIST;
    }
    if (buf == NULL || count == 0U) {
        return ODR_NO_DATA;
    }
    if ((stream->dataOffset + count) > sizeof(fw_metadata_record_t)) {
        return ODR_DATA_LONG;
    }

    ODR_t ret = OD_writeOriginal(stream, buf, count, countWritten);
    if (ret == ODR_PARTIAL || ret != ODR_OK) {
        return ret;
    }

    const fw_metadata_record_t *meta = (const fw_metadata_record_t *)stream->dataOrig;
    if (meta == NULL) {
        return ODR_DEV_INCOMPAT;
    }

    fw_server_state_t *server = fw_get_server(stream);
    if (!fw_store_metadata(&server->ctx, meta)) {
        return ODR_INVALID_VALUE;
    }
    return ODR_OK;
}

static ODR_t fw_write_control(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten) {
    if (stream->subIndex == 0U) {
        return OD_writeOriginal(stream, buf, count, countWritten);
    }
    if (stream->subIndex != 1U) {
        return ODR_SUB_NOT_EXIST;
    }
    if (stream->dataOffset != 0U || count != 3U || buf == NULL) {
        return ODR_DATA_LONG;
    }
    const uint8_t *payload = (const uint8_t *)buf;
    fw_server_state_t *server = fw_get_server(stream);
    if (payload[0] != FW_CTRL_CMD_START) {
        ESP_LOGE(TAG, "Unsupported control command 0x%02X", payload[0]);
        return ODR_INVALID_VALUE;
    }
    if (!server->ctx.metadataReceived) {
        ESP_LOGE(TAG, "Start command received before metadata");
        return ODR_INVALID_VALUE;
    }
    if (!fw_prepare_storage(&server->ctx)) {
        return ODR_INVALID_VALUE;
    }
    ODR_t ret = OD_writeOriginal(stream, buf, count, countWritten);
    if (ret == ODR_OK && countWritten != NULL) {
        *countWritten = count;
    }
    return ret;
}

static ODR_t fw_write_data(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten) {
    if (stream->subIndex == 0U) {
        return ODR_READONLY;
    }
    if (stream->subIndex != 1U) {
        return ODR_SUB_NOT_EXIST;
    }
    if (count == 0U || buf == NULL) {
        return ODR_NO_DATA;
    }
    if (count > CONFIG_DEMO_SLAVE_MAX_CHUNK_BYTES) {
        ESP_LOGE(TAG, "Chunk too large (%u > %u)", (unsigned)count, CONFIG_DEMO_SLAVE_MAX_CHUNK_BYTES);
        return ODR_DATA_LONG;
    }
    fw_server_state_t *server = fw_get_server(stream);
    fw_update_context_t *ctx = &server->ctx;
    if (stream->dataOffset == 0U) {
        ctx->currentChunkBase = ctx->receivedBytes;
        ctx->chunkInProgress = true;
    }
    uint32_t absoluteOffset = ctx->currentChunkBase + (uint32_t)stream->dataOffset;
    if (!fw_receive_chunk(ctx, (const uint8_t *)buf, (uint32_t)count, absoluteOffset)) {
        return ODR_INVALID_VALUE;
    }
    OD_size_t nextOffset = stream->dataOffset + count;
    stream->dataOffset = nextOffset;
    if (countWritten != NULL) {
        *countWritten = count;
    }
    bool finalChunk = (stream->dataLength != 0U) && (nextOffset >= stream->dataLength);
    if (finalChunk) {
        ctx->chunkInProgress = false;
        ctx->currentChunkBase = ctx->receivedBytes;
    }
    return finalChunk ? ODR_OK : ODR_PARTIAL;
}

static ODR_t fw_write_status(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *countWritten) {
    if (stream->subIndex == 0U) {
        return OD_writeOriginal(stream, buf, count, countWritten);
    }
    if (stream->subIndex != 1U) {
        return ODR_SUB_NOT_EXIST;
    }
    if (stream->dataOffset != 0U || count != 2U || buf == NULL) {
        return ODR_DATA_LONG;
    }
    fw_server_state_t *server = fw_get_server(stream);
    const uint8_t *payload = (const uint8_t *)buf;
    uint16_t crc = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    if (!fw_finalize(&server->ctx, crc)) {
        return ODR_INVALID_VALUE;
    }
    ODR_t ret = OD_writeOriginal(stream, buf, count, countWritten);
    if (ret == ODR_OK && countWritten != NULL) {
        *countWritten = count;
    }
    return ret;
}

bool fw_server_init(CO_t *co) {
    if (co == NULL || OD == NULL) {
        return false;
    }
    s_server.co = co;
    fw_reset_context(&s_server.ctx);

    /* Get running firmware CRC - try NVS first (reliable), then compute from flash (fallback) */
    uint16_t nvsCrc = 0;
    if (fw_load_crc_from_nvs(&nvsCrc)) {
        s_server.runningFirmwareCrc = nvsCrc;
        ESP_LOGI(TAG, "Running firmware CRC from NVS: 0x%04X", s_server.runningFirmwareCrc);
    } else {
        /* No NVS entry - compute from flash (first boot or factory firmware) */
        s_server.runningFirmwareCrc = fw_compute_running_firmware_crc();
        ESP_LOGI(TAG, "Running firmware CRC computed: 0x%04X (no NVS entry)", s_server.runningFirmwareCrc);
    }

    /* Get running firmware version from NVS (or default from Kconfig) */
    uint16_t nvsVer = 0;
    if (fw_load_version_from_nvs(&nvsVer)) {
        s_server.runningFirmwareVersion = nvsVer;
        ESP_LOGI(TAG, "Running firmware version from NVS: %u", s_server.runningFirmwareVersion);
    } else {
#ifdef CONFIG_DEMO_SLAVE_FW_VERSION
        s_server.runningFirmwareVersion = (uint16_t)(CONFIG_DEMO_SLAVE_FW_VERSION & 0xFFFF);
        ESP_LOGI(TAG, "Running firmware version from Kconfig: %u", s_server.runningFirmwareVersion);
#else
        s_server.runningFirmwareVersion = 0;
        ESP_LOGI(TAG, "Running firmware version: 0 (no NVS/Kconfig)");
#endif
    }

    /* Store in OD so SDO reads return the correct value */
#ifdef OD_ENTRY_H1F5B_runningFirmwareCrc
    OD_RAM.x1F5B_runningFirmwareCrc.runningCrc = s_server.runningFirmwareCrc;
#endif
#ifdef OD_ENTRY_H1F5C_runningFirmwareVersion
    OD_RAM.x1F5C_runningFirmwareVersion.runningVersion = s_server.runningFirmwareVersion;
#endif

    s_server.metaExt.object = &s_server;
    s_server.metaExt.read = OD_readOriginal;
    s_server.metaExt.write = fw_write_metadata;
    if (OD_extension_init(OD_ENTRY_H1F57_programIdentification, &s_server.metaExt) != ODR_OK) {
        return false;
    }

    s_server.ctrlExt.object = &s_server;
    s_server.ctrlExt.read = OD_readOriginal;
    s_server.ctrlExt.write = fw_write_control;
    if (OD_extension_init(OD_ENTRY_H1F51_programControl, &s_server.ctrlExt) != ODR_OK) {
        return false;
    }

    s_server.dataExt.object = &s_server;
    s_server.dataExt.read = NULL;
    s_server.dataExt.write = fw_write_data;
    if (OD_extension_init(OD_ENTRY_H1F50_programDownload, &s_server.dataExt) != ODR_OK) {
        return false;
    }

    s_server.statusExt.object = &s_server;
    s_server.statusExt.read = OD_readOriginal;
    s_server.statusExt.write = fw_write_status;
    if (OD_extension_init(OD_ENTRY_H1F5A_programStatus, &s_server.statusExt) != ODR_OK) {
        return false;
    }

    /* Register running firmware CRC object 0x1F5B if defined in OD */
#ifdef OD_ENTRY_H1F5B_runningFirmwareCrc
    s_server.runningCrcExt.object = &s_server;
    s_server.runningCrcExt.read = OD_readOriginal;
    s_server.runningCrcExt.write = NULL; /* read-only */
    if (OD_extension_init(OD_ENTRY_H1F5B_runningFirmwareCrc, &s_server.runningCrcExt) != ODR_OK) {
        ESP_LOGW(TAG, "Could not register 0x1F5B extension");
    }
#endif

    /* Register running firmware version object 0x1F5C if defined in OD */
#ifdef OD_ENTRY_H1F5C_runningFirmwareVersion
    s_server.runningVerExt.object = &s_server;
    s_server.runningVerExt.read = OD_readOriginal;
    s_server.runningVerExt.write = NULL; /* read-only */
    if (OD_extension_init(OD_ENTRY_H1F5C_runningFirmwareVersion, &s_server.runningVerExt) != ODR_OK) {
        ESP_LOGW(TAG, "Could not register 0x1F5C extension");
    }
#endif

    ESP_LOGI(TAG, "Firmware download objects registered");
    return true;
}

uint16_t fw_server_get_running_crc(void) {
    return s_server.runningFirmwareCrc;
}

uint16_t fw_server_get_running_version(void) {
    return s_server.runningFirmwareVersion;
}
