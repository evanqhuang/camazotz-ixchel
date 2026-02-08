/*
 * SDIO Logger - SD Card Logging for Navigation Data
 * Ring-buffered CSV logger for nav_state_compact_t records
 */

#ifndef SDIO_LOGGER_HPP
#define SDIO_LOGGER_HPP

#include "types.h"
#include "config.h"
#include <cstdint>

/*============================================================================
 * Logger Statistics
 *============================================================================*/
struct LoggerStats {
    uint32_t records_written;      /* Total nav records written to SD */
    uint32_t records_dropped;      /* Records lost due to buffer overflow */
    uint32_t events_written;       /* Total events written to SD */
    uint32_t flush_count;          /* Number of flush operations */
    uint32_t sync_count;           /* Number of sync operations */
    uint32_t write_errors;         /* SD write errors */
    uint32_t recovery_attempts;    /* Mount recovery attempts */
    bool mounted;                  /* SD card currently mounted */
    bool critical_error;           /* Unrecoverable error state */
};

/*============================================================================
 * SDIO Logger Class
 *============================================================================*/
class SDIO_Logger {
public:
    SDIO_Logger() = default;
    ~SDIO_Logger();

    /* Disable copy/move */
    SDIO_Logger(const SDIO_Logger&) = delete;
    SDIO_Logger& operator=(const SDIO_Logger&) = delete;
    SDIO_Logger(SDIO_Logger&&) = delete;
    SDIO_Logger& operator=(SDIO_Logger&&) = delete;

    /**
     * @brief Initialize SD card and create log files.
     * Mounts filesystem, reads/increments boot count, creates CSV files with headers.
     * @return true if initialization successful, false otherwise
     */
    bool init();

    /**
     * @brief Shutdown logger gracefully.
     * Flushes buffer, syncs filesystem, closes files, unmounts.
     */
    void deinit();

    /**
     * @brief Queue navigation state for logging.
     * Adds record to ring buffer. Non-blocking, O(1).
     * @param state Navigation state to log
     * @return true if queued, false if buffer full (record dropped)
     */
    bool log_state(const nav_state_compact_t& state);

    /**
     * @brief Log a critical event.
     * Events are deduplicated by tag (1s cooldown per unique tag).
     * @param tag Event identifier (e.g., "ENCODER_LOST", "IMU_RESET")
     * @param flags Current status flags at time of event
     * @return true if logged, false if deduplicated or error
     */
    bool log_event(const char* tag, uint8_t flags);

    /**
     * @brief Write buffered records to SD card.
     * Should be called at ~10Hz from main loop.
     * @return Number of records written
     */
    uint32_t flush();

    /**
     * @brief Sync filesystem to ensure data persistence.
     * Should be called every 5 seconds.
     * @return true if sync successful
     */
    bool sync();

    /**
     * @brief Attempt to recover from mount failure.
     * Called during heartbeat when in error state.
     * @return true if recovery successful
     */
    bool try_recovery();

    /**
     * @brief Get current logger statistics.
     * @return LoggerStats structure
     */
    LoggerStats get_stats() const;

    /**
     * @brief Check if logger is operational.
     * @return true if mounted and no critical error
     */
    bool is_operational() const;

private:
    /* Ring buffer record (internal format, 48 bytes) */
    struct LogRecord {
        uint32_t timestamp_ms;
        uint32_t sequence_num;
        float angular_delta;
        float quat_w, quat_x, quat_y, quat_z;
        float pos_x, pos_y, pos_z;
        float delta_dist;
        uint8_t status_flags;
        uint8_t _pad[3];
    };
    static_assert(sizeof(LogRecord) == 48U, "LogRecord must be 48 bytes");

    /* Ring buffer sizing */
    static constexpr uint32_t RECORD_SIZE = sizeof(LogRecord);
    static constexpr uint32_t RECORD_COUNT = LOG_BUFFER_SIZE_BYTES / RECORD_SIZE;  /* 682 */

    /* Event deduplication entry */
    struct EventEntry {
        uint32_t tag_hash;
        uint32_t last_time_ms;
    };
    static constexpr uint32_t MAX_EVENT_ENTRIES = 16U;

    /* Private methods */
    bool mount_filesystem();
    bool create_files();
    bool write_nav_header();
    bool write_event_header();
    bool write_record(const LogRecord& rec);
    bool write_event_line(uint32_t timestamp_ms, const char* tag, uint8_t flags);
    uint32_t hash_tag(const char* tag) const;
    bool is_event_duplicate(uint32_t tag_hash, uint32_t current_ms);
    void record_event(uint32_t tag_hash, uint32_t current_ms);
    uint32_t find_next_index();

    /* Ring buffer (SPSC: single-producer single-consumer, both on Core 0) */
    LogRecord buffer_[RECORD_COUNT] = {};
    volatile uint32_t write_idx_ = 0;
    volatile uint32_t read_idx_ = 0;

    /* Event deduplication */
    EventEntry event_cache_[MAX_EVENT_ENTRIES] = {};
    uint32_t event_cache_idx_ = 0;

    /* File handles (opaque pointers to avoid FatFS header pollution) */
    void* nav_file_ = nullptr;      /* FIL* for nav_bootXXX.csv */
    void* event_file_ = nullptr;    /* FIL* for events_bootXXX.csv */
    void* fatfs_ = nullptr;         /* FATFS* for mounted filesystem */

    /* State */
    uint32_t boot_count_ = 0;
    uint32_t sequence_num_ = 0;
    uint32_t event_sequence_ = 0;
    bool mounted_ = false;
    bool critical_error_ = false;

    /* Statistics */
    mutable LoggerStats stats_ = {};
};

#endif /* SDIO_LOGGER_HPP */
