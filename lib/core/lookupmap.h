#ifndef LOOKUPMAP_H
#define LOOKUPMAP_H

#include <stdint.h>
#include "lookuptable.h"
#include "freertos/FreeRTOS.h"

static const uint8_t MAX_LOOKUP_CACHE = 5u;

struct LookupCache {
    float x_val;
    float y_val;
    uint32_t timestamp_ms;
};

struct LookupCacheReadEntry {
    uint8_t slot_id;
    LookupCache cache;
};

class LookupMap {
    public:
        float get_value(const float xValue, const float yValue);
        float get_value(const float xValue, const float yValue, uint8_t cache_slot);
        void get_y_headers(uint16_t *size, int16_t **headers);
        float get_x_header_interpolated(const float value, const int16_t y) const;
        int16_t* get_current_data(void) const;
        void get_x_headers(uint16_t *size, int16_t **headers);
        uint16_t data_size();
        void copy_lookup_cache(uint8_t *entry_count, LookupCacheReadEntry *entries, uint8_t max_entries, uint32_t now_ms, uint32_t max_age_ms) const;
        void clear_lookup_cache(void);
    protected:
        LookupTable* table;
        LookupHeader* yHeader;
        uint16_t yHeaderSize;
    private:
        void record_lookup_cache(const float xValue, const float yValue, uint8_t cache_idx);
        void copy_lookup_cache_snapshot(LookupCache *snapshot, uint8_t snapshot_count) const;
        void clear_lookup_cache_locked(void);

        LookupCache lookup_cache[MAX_LOOKUP_CACHE] = {};
        mutable portMUX_TYPE lookup_cache_mutex = portMUX_INITIALIZER_UNLOCKED;
};

class LookupAllocMap : public LookupMap {
    public:
        LookupAllocMap(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _yHeader, const uint16_t _yHeaderSize, const int16_t* _data, const uint16_t _dataSize);
        bool add_data(const int16_t* map, const uint16_t size);
        bool is_allocated(void) const;
        ~LookupAllocMap();
};

class LookupRefMap : public LookupMap {
    public:
        LookupRefMap(int16_t* _xHeader, const uint16_t _xHeaderSize, int16_t* _yHeader, const uint16_t _yHeaderSize, int16_t* _data, const uint16_t _dataSize);
};

class LookupByteMap : public LookupMap {
    public:
        LookupByteMap(uint8_t* _xHeader, const uint16_t _xHeaderSize, uint8_t* _yHeader, const uint16_t _yHeaderSize, uint8_t* _data, const uint16_t _dataSize);
        bool is_allocated(void) const;
        bool add_data(const uint8_t* map, const uint16_t size);
        ~LookupByteMap();
    private:
        int16_t* x_alloc;
        int16_t* y_alloc;
        int16_t* z_alloc;
        uint16_t z_size;
};

#endif /* lookupmap.h */
