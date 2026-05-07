#ifndef LOOKUPMAP_H
#define LOOKUPMAP_H

#include <stdint.h>
#include "lookuptable.h"

static const uint8_t LOOKUP_TRACE_SLOT_COUNT = 8u;

typedef struct {
    int16_t x;
    int16_t y;
    uint32_t timestamp_ms;
} LookupTraceEntry;

class LookupMap {
    public:
        float get_value(const float xValue, const float yValue);
        float get_value(const float xValue, const float yValue, uint8_t trace_slot);
        void get_y_headers(uint16_t *size, int16_t **headers);
        float get_x_header_interpolated(const float value, const int16_t y) const;
        int16_t* get_current_data(void) const;
        void get_x_headers(uint16_t *size, int16_t **headers);
        uint16_t data_size();
        void get_trace_entries(uint8_t *slot_count, uint8_t *valid_mask, const LookupTraceEntry **entries) const;
        void clear_trace_entries(void);
    protected:
        LookupTable* table;
        LookupHeader* yHeader;
        uint16_t yHeaderSize;
    private:
        void record_lookup_trace(const float xValue, const float yValue, uint8_t trace_slot);
        static bool trace_float_to_i16(const float value, int16_t *dest);

        LookupTraceEntry trace_entries[LOOKUP_TRACE_SLOT_COUNT] = {};
        uint8_t trace_valid_mask = 0u;
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
