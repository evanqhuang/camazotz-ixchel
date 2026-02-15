/*
 * SDIO Logger Implementation
 * Ring-buffered CSV logging for navigation data to SD card
 */

#include "drivers/sdio_logger.hpp"
#include "pico/stdlib.h"

/* FatFS headers */
extern "C" {
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
}

#include <cstdio>
#include <cstring>

/* Directory names for log file organization */
static constexpr const char* NAV_DIR   = "nav";
static constexpr const char* EVENT_DIR = "events";

/* Create log directories if they do not already exist. */
static bool ensure_directories() {
    FRESULT fr = f_mkdir(NAV_DIR);
    if (fr != FR_OK && fr != FR_EXIST) {
        printf("[SD] Failed to create %s/: %s (%d)\n", NAV_DIR, FRESULT_str(fr), fr);
        return false;
    }
    fr = f_mkdir(EVENT_DIR);
    if (fr != FR_OK && fr != FR_EXIST) {
        printf("[SD] Failed to create %s/: %s (%d)\n", EVENT_DIR, FRESULT_str(fr), fr);
        return false;
    }
    return true;
}

/*============================================================================
 * Lifecycle
 *============================================================================*/

SDIO_Logger::~SDIO_Logger() {
    deinit();
}

bool SDIO_Logger::init() {
    stats_ = {};  /* Reset statistics */

    if (!mount_filesystem()) {
        critical_error_ = true;
        return false;
    }

    /* Scan existing files to determine next index */
    boot_count_ = find_next_index();

    if (!create_files()) {
        deinit();
        critical_error_ = true;
        return false;
    }

    mounted_ = true;
    stats_.mounted = true;
    return true;
}

void SDIO_Logger::deinit() {
    /* Flush any remaining buffered records */
    if (mounted_ && nav_file_ != nullptr) {
        flush();
        sync();
    }

    /* Close files */
    if (nav_file_ != nullptr) {
        f_close(static_cast<FIL*>(nav_file_));
        nav_file_ = nullptr;
    }
    if (event_file_ != nullptr) {
        f_close(static_cast<FIL*>(event_file_));
        event_file_ = nullptr;
    }

    /* Unmount filesystem */
    if (fatfs_ != nullptr) {
        f_unmount("");
        fatfs_ = nullptr;
    }

    mounted_ = false;
    stats_.mounted = false;
}

/*============================================================================
 * Filesystem Operations
 *============================================================================*/

bool SDIO_Logger::mount_filesystem() {
    static FATFS fatfs;  /* Static allocation for FatFS */
    fatfs_ = &fatfs;

    FRESULT fr = f_mount(&fatfs, "", 1);  /* Mount immediately */
    if (fr != FR_OK) {
        printf("[SD] Mount failed: %s (%d)\n", FRESULT_str(fr), fr);
        fatfs_ = nullptr;
        return false;
    }

    printf("[SD] Mounted OK\n");
    return true;
}

uint32_t SDIO_Logger::find_next_index() {
    DIR dir;
    FILINFO fno;
    uint32_t max_idx = 0;
    bool found = false;

    /* Scan nav/ directory for ???.csv files to find highest existing index */
    FRESULT fr = f_findfirst(&dir, &fno, NAV_DIR, "???.csv");
    while (fr == FR_OK && fno.fname[0] != '\0') {
        /* Parse index from "XXX.csv" */
        unsigned long idx = 0;
        if (sscanf(fno.fname, "%lu.csv", &idx) == 1) {
            if (!found || static_cast<uint32_t>(idx) > max_idx) {
                max_idx = static_cast<uint32_t>(idx);
                found = true;
            }
        }
        fr = f_findnext(&dir, &fno);
    }
    f_closedir(&dir);

    uint32_t next = found ? max_idx + 1U : 0U;
    printf("[SD] Next log index: %lu\n", static_cast<unsigned long>(next));
    return next;
}

bool SDIO_Logger::create_files() {
    static FIL nav_fil;
    static FIL event_fil;
    char filename[32];

    /* Ensure log directories exist */
    if (!ensure_directories()) {
        return false;
    }

    /* Create navigation log file */
    snprintf(filename, sizeof(filename), "%s/%03lu.csv", NAV_DIR,
             static_cast<unsigned long>(boot_count_));

    FRESULT fr = f_open(&nav_fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("[SD] Failed to create %s: %s (%d)\n", filename, FRESULT_str(fr), fr);
        return false;
    }
    nav_file_ = &nav_fil;

    if (!write_nav_header()) {
        return false;
    }
    printf("[SD] Created %s\n", filename);

    /* Create events log file */
    snprintf(filename, sizeof(filename), "%s/%03lu.csv", EVENT_DIR,
             static_cast<unsigned long>(boot_count_));

    fr = f_open(&event_fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("[SD] Failed to create %s: %s (%d)\n", filename, FRESULT_str(fr), fr);
        return false;
    }
    event_file_ = &event_fil;

    if (!write_event_header()) {
        return false;
    }
    printf("[SD] Created %s\n", filename);

    /* Initial sync to persist headers */
    sync();

    return true;
}

bool SDIO_Logger::write_nav_header() {
    FIL* fil = static_cast<FIL*>(nav_file_);
    const char* header = "timestamp_ms,seq,angular_delta,qw,qx,qy,qz,px,py,pz,delta_dist,flags\n";

    UINT bw;
    FRESULT fr = f_write(fil, header, strlen(header), &bw);
    if (fr != FR_OK || bw != strlen(header)) {
        printf("[SD] Header write failed: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    return true;
}

bool SDIO_Logger::write_event_header() {
    FIL* fil = static_cast<FIL*>(event_file_);
    const char* header = "timestamp_ms,seq,tag,flags\n";

    UINT bw;
    FRESULT fr = f_write(fil, header, strlen(header), &bw);
    if (fr != FR_OK || bw != strlen(header)) {
        printf("[SD] Event header write failed: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    return true;
}

/*============================================================================
 * Logging API
 *============================================================================*/

bool SDIO_Logger::log_state(const nav_state_compact_t& state) {
    if (!mounted_ || critical_error_) {
        return false;
    }

    /* Check for buffer overflow */
    uint32_t next_write = (write_idx_ + 1U) % RECORD_COUNT;
    if (next_write == read_idx_) {
        /* Buffer full - drop record */
        if (stats_.records_dropped < UINT32_MAX) {
            stats_.records_dropped++;
        }
        return false;
    }

    /* Convert nav_state_compact_t to LogRecord */
    LogRecord& rec = buffer_[write_idx_];
    rec.timestamp_ms = to_ms_since_boot(get_absolute_time());
    rec.sequence_num = sequence_num_++;
    rec.angular_delta = state.angular_delta;
    rec.quat_w = state.quat_w;
    rec.quat_x = state.quat_x;
    rec.quat_y = state.quat_y;
    rec.quat_z = state.quat_z;
    rec.pos_x = state.pos_x;
    rec.pos_y = state.pos_y;
    rec.pos_z = state.pos_z;
    rec.delta_dist = state.delta_dist;
    rec.status_flags = state.status_flags;

    /* Advance write pointer */
    write_idx_ = next_write;

    return true;
}

bool SDIO_Logger::log_event(const char* tag, uint8_t flags) {
    if (!mounted_ || critical_error_ || tag == nullptr) {
        return false;
    }

    uint32_t current_ms = to_ms_since_boot(get_absolute_time());
    uint32_t tag_hash = hash_tag(tag);

    /* Check for duplicate (1s cooldown) */
    if (is_event_duplicate(tag_hash, current_ms)) {
        return false;
    }

    /* Write event line directly (events are infrequent) */
    if (!write_event_line(current_ms, tag, flags)) {
        return false;
    }

    /* Record for deduplication */
    record_event(tag_hash, current_ms);

    if (stats_.events_written < UINT32_MAX) {
        stats_.events_written++;
    }

    return true;
}

bool SDIO_Logger::write_event_line(uint32_t timestamp_ms, const char* tag, uint8_t flags) {
    FIL* fil = static_cast<FIL*>(event_file_);
    char line[128];

    int len = snprintf(line, sizeof(line), "%lu,%lu,%s,0x%02X\n",
                       static_cast<unsigned long>(timestamp_ms),
                       static_cast<unsigned long>(event_sequence_++),
                       tag,
                       static_cast<unsigned>(flags));

    if (len <= 0 || len >= static_cast<int>(sizeof(line))) {
        return false;
    }

    UINT bw;
    FRESULT fr = f_write(fil, line, static_cast<UINT>(len), &bw);
    if (fr != FR_OK || bw != static_cast<UINT>(len)) {
        if (stats_.write_errors < UINT32_MAX) {
            stats_.write_errors++;
        }
        return false;
    }

    return true;
}

/*============================================================================
 * Flush and Sync
 *============================================================================*/

uint32_t SDIO_Logger::flush() {
    if (!mounted_ || critical_error_ || nav_file_ == nullptr) {
        return 0;
    }

    FIL* fil = static_cast<FIL*>(nav_file_);
    uint32_t count = 0;
    char line[128];

    /* Process all buffered records */
    while (read_idx_ != write_idx_) {
        const LogRecord& rec = buffer_[read_idx_];

        int len = snprintf(line, sizeof(line),
                           "%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.6f,0x%02X\n",
                           static_cast<unsigned long>(rec.timestamp_ms),
                           static_cast<unsigned long>(rec.sequence_num),
                           static_cast<double>(rec.angular_delta),
                           static_cast<double>(rec.quat_w),
                           static_cast<double>(rec.quat_x),
                           static_cast<double>(rec.quat_y),
                           static_cast<double>(rec.quat_z),
                           static_cast<double>(rec.pos_x),
                           static_cast<double>(rec.pos_y),
                           static_cast<double>(rec.pos_z),
                           static_cast<double>(rec.delta_dist),
                           static_cast<unsigned>(rec.status_flags));

        if (len <= 0 || len >= static_cast<int>(sizeof(line))) {
            /* Format error - skip record */
            read_idx_ = (read_idx_ + 1U) % RECORD_COUNT;
            continue;
        }

        UINT bw;
        FRESULT fr = f_write(fil, line, static_cast<UINT>(len), &bw);
        if (fr != FR_OK || bw != static_cast<UINT>(len)) {
            /* Write error - stop and set critical error */
            if (stats_.write_errors < UINT32_MAX) {
                stats_.write_errors++;
            }
            critical_error_ = true;
            stats_.critical_error = true;
            printf("[SD] Write error: %s (%d) - entering recovery mode\n",
                   FRESULT_str(fr), fr);
            break;
        }

        read_idx_ = (read_idx_ + 1U) % RECORD_COUNT;
        count++;

        if (stats_.records_written < UINT32_MAX) {
            stats_.records_written++;
        }
    }

    if (count > 0 && stats_.flush_count < UINT32_MAX) {
        stats_.flush_count++;
    }

    return count;
}

bool SDIO_Logger::sync() {
    if (!mounted_ || critical_error_) {
        return false;
    }

    bool success = true;

    if (nav_file_ != nullptr) {
        FIL* fil = static_cast<FIL*>(nav_file_);
        FRESULT fr = f_sync(fil);
        if (fr != FR_OK) {
            printf("[SD] Nav sync failed: %s (%d)\n", FRESULT_str(fr), fr);
            success = false;
        }
    }

    if (event_file_ != nullptr) {
        FIL* fil = static_cast<FIL*>(event_file_);
        FRESULT fr = f_sync(fil);
        if (fr != FR_OK) {
            printf("[SD] Event sync failed: %s (%d)\n", FRESULT_str(fr), fr);
            success = false;
        }
    }

    if (success && stats_.sync_count < UINT32_MAX) {
        stats_.sync_count++;
    }

    return success;
}

/*============================================================================
 * Recovery
 *============================================================================*/

bool SDIO_Logger::try_recovery() {
    if (!critical_error_) {
        return true;  /* Already operational */
    }

    if (stats_.recovery_attempts < UINT32_MAX) {
        stats_.recovery_attempts++;
    }

    printf("[SD] Attempting recovery (attempt %lu)...\n",
           static_cast<unsigned long>(stats_.recovery_attempts));

    /* Close any open file handles */
    if (nav_file_ != nullptr) {
        f_close(static_cast<FIL*>(nav_file_));
        nav_file_ = nullptr;
    }
    if (event_file_ != nullptr) {
        f_close(static_cast<FIL*>(event_file_));
        event_file_ = nullptr;
    }

    /* Unmount if mounted */
    if (fatfs_ != nullptr) {
        f_unmount("");
        fatfs_ = nullptr;
    }
    mounted_ = false;

    /* Attempt remount */
    if (!mount_filesystem()) {
        printf("[SD] Recovery mount failed\n");
        return false;
    }

    /* Ensure directories exist after remount */
    if (!ensure_directories()) {
        printf("[SD] Recovery directory creation failed\n");
        f_unmount("");
        fatfs_ = nullptr;
        return false;
    }

    /* Reopen files in append mode (continue existing log) */
    static FIL nav_fil;
    static FIL event_fil;
    char filename[32];

    snprintf(filename, sizeof(filename), "%s/%03lu.csv", NAV_DIR,
             static_cast<unsigned long>(boot_count_));
    FRESULT fr = f_open(&nav_fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        printf("[SD] Recovery nav open failed: %s (%d)\n", FRESULT_str(fr), fr);
        f_unmount("");
        fatfs_ = nullptr;
        return false;
    }
    nav_file_ = &nav_fil;

    snprintf(filename, sizeof(filename), "%s/%03lu.csv", EVENT_DIR,
             static_cast<unsigned long>(boot_count_));
    fr = f_open(&event_fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        printf("[SD] Recovery event open failed: %s (%d)\n", FRESULT_str(fr), fr);
        f_close(&nav_fil);
        nav_file_ = nullptr;
        f_unmount("");
        fatfs_ = nullptr;
        return false;
    }
    event_file_ = &event_fil;

    /* Log recovery event */
    log_event("SD_RECOVERED", 0);

    mounted_ = true;
    critical_error_ = false;
    stats_.mounted = true;
    stats_.critical_error = false;

    printf("[SD] Recovery successful\n");
    return true;
}

/*============================================================================
 * Event Deduplication
 *============================================================================*/

uint32_t SDIO_Logger::hash_tag(const char* tag) const {
    /* Simple djb2 hash */
    uint32_t hash = 5381U;
    while (*tag != '\0') {
        hash = ((hash << 5U) + hash) + static_cast<uint32_t>(*tag);
        tag++;
    }
    return hash;
}

bool SDIO_Logger::is_event_duplicate(uint32_t tag_hash, uint32_t current_ms) {
    for (uint32_t i = 0; i < MAX_EVENT_ENTRIES; i++) {
        if (event_cache_[i].tag_hash == tag_hash) {
            uint32_t elapsed = current_ms - event_cache_[i].last_time_ms;
            return elapsed < LOG_EVENT_COOLDOWN_MS;
        }
    }
    return false;
}

void SDIO_Logger::record_event(uint32_t tag_hash, uint32_t current_ms) {
    /* Check if tag already exists in cache */
    for (uint32_t i = 0; i < MAX_EVENT_ENTRIES; i++) {
        if (event_cache_[i].tag_hash == tag_hash) {
            event_cache_[i].last_time_ms = current_ms;
            return;
        }
    }

    /* Add new entry (circular replacement) */
    event_cache_[event_cache_idx_].tag_hash = tag_hash;
    event_cache_[event_cache_idx_].last_time_ms = current_ms;
    event_cache_idx_ = (event_cache_idx_ + 1U) % MAX_EVENT_ENTRIES;
}

/*============================================================================
 * Status
 *============================================================================*/

LoggerStats SDIO_Logger::get_stats() const {
    stats_.mounted = mounted_;
    stats_.critical_error = critical_error_;
    return stats_;
}

bool SDIO_Logger::is_operational() const {
    return mounted_ && !critical_error_;
}
