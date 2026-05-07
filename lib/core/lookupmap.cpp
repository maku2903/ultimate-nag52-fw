#include "lookupmap.h"
#include "tcu_maths_impl.h"
#include "tcu_alloc.h"
#include "../../src/clock.hpp"
#include <limits.h>

int16_t LookupMap::trace_float_to_i16(const float value)
{
    if (value > (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (value < (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

void LookupMap::record_lookup_trace(const float xValue, const float yValue, uint8_t trace_slot)
{
    if (trace_slot >= LOOKUP_TRACE_SLOT_COUNT) {
        return;
    }
    this->trace_entries[trace_slot].x = trace_float_to_i16(xValue);
    this->trace_entries[trace_slot].y = trace_float_to_i16(yValue);
    this->trace_entries[trace_slot].timestamp_ms = GET_CLOCK_TIME();
    this->trace_valid_mask |= (1u << trace_slot);
}

float LookupMap::get_value(const float xValue, const float yValue)
{
    return this->get_value(xValue, yValue, 0u);
}

float LookupMap::get_value(const float xValue, const float yValue, uint8_t trace_slot)
{
    uint16_t    x_idx_min;
    uint16_t    x_idx_max;
    uint16_t    y_idx_min;
    uint16_t    y_idx_max;
    const LookupHeader* xHeader = this->table->get_header();
    const int16_t* data = this->table->get_current_data();

    // part 1a - identification of the indices for x-value
    search_value<int16_t>(xValue, xHeader->get_data(), xHeader->get_size(), &x_idx_min, &x_idx_max);
    
    // part 1b - identification of the indices for y-value
    search_value<int16_t>(yValue, yHeader->get_data(), yHeader->get_size(), &y_idx_min, &y_idx_max);
    
    // part 2: do the interpolation
    const int16_t x1 = xHeader->get_value(x_idx_min);
    const int16_t x2 = xHeader->get_value(x_idx_max);
    const int16_t y1 = yHeader->get_value(y_idx_min);
    const int16_t y2 = yHeader->get_value(y_idx_max);

    // some precalculations for making the code more readable, although somewhat inefficient
    const float f_11 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_min];
    const float f_12 = (float)data[(y_idx_min * xHeader->get_size()) + x_idx_max];
    const float f_21 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_min];
    const float f_22 = (float)data[(y_idx_max * xHeader->get_size()) + x_idx_max];

    // interpolation on x-axis for smaller y-index
    const float f_11f_12_interpolated = interpolate(f_11, f_12, x1, x2, xValue);
    // interpolation on x-axis for greater y-index
    const float f_21f_22_interpolated = interpolate(f_21, f_22, x1, x2, xValue);
    // bilinear interpolation, not always efficient, but with more or less constant runtime
    // also see https://en.wikipedia.org/wiki/Bilinear_interpolation, https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/ for mathematical background
    float ret = interpolate(f_11f_12_interpolated, f_21f_22_interpolated, y1, y2, yValue);
    this->record_lookup_trace(xValue, yValue, trace_slot);
    return ret;
}

void LookupMap::get_y_headers(uint16_t *size, int16_t **headers){
    *size = yHeaderSize;
    *headers = yHeader->get_data();
}

int16_t* LookupMap::get_current_data(void) const {
    return this->table->get_current_data();
}

void LookupMap::get_x_headers(uint16_t *size, int16_t **headers) {
    return table->get_x_headers(size, headers);
}

uint16_t LookupMap::data_size() {
    return this->table->data_size();
}

void LookupMap::get_trace_entries(uint8_t *slot_count, uint8_t *valid_mask, const LookupTraceEntry **entries) const
{
    *slot_count = LOOKUP_TRACE_SLOT_COUNT;
    *valid_mask = this->trace_valid_mask;
    *entries = this->trace_entries;
}

void LookupMap::clear_trace_entries(void)
{
    for (uint8_t i = 0; i < LOOKUP_TRACE_SLOT_COUNT; i++) {
        this->trace_entries[i] = {};
    }
    this->trace_valid_mask = 0u;
}

float LookupMap::get_x_header_interpolated(const float value, const int16_t y) const
{
    const LookupHeader* xHeader = this->table->get_header();
    const int16_t* data = this->table->get_current_data();
    // isolate the row
    int16_t row[xHeader->get_size()] = {0};
    for (uint16_t i = 0; i < xHeader->get_size(); i++)
    {
        row[i] = data[i*yHeaderSize];
    }
    
    uint16_t    idvalue_min;
    uint16_t    idvalue_max;

    // part 1 - identification of the indices for x-value
    search_value<int16_t>(value, row, xHeader->get_size(), &idvalue_min, &idvalue_max);

    // part 2: do the interpolation
    const float value1 = (float)xHeader->get_value(idvalue_min);
    const float value2 = (float)xHeader->get_value(idvalue_max);
    
    return value1 + progress_between_targets(value, row[idvalue_min], row[idvalue_max]) * (value2 - value1);
}

LookupAllocMap::LookupAllocMap(const int16_t* _xHeader, const uint16_t _xHeaderSize, const int16_t* _yHeader, const uint16_t _yHeaderSize, const int16_t* _data, const uint16_t _dataSize) {
    this->table = new LookupAllocTable(_xHeader, _xHeaderSize, _data, _dataSize);
    this->yHeader = new LookupAllocHeader(_yHeader, _yHeaderSize);
    this->yHeaderSize = _yHeaderSize;
}

LookupAllocMap::~LookupAllocMap() {
    delete this->yHeader;
    delete this->table;
}

bool LookupAllocMap::add_data(const int16_t* map, const uint16_t size) {
    return reinterpret_cast<LookupAllocTable*>(this->table)->add_data(map, size);
}

bool LookupAllocMap::is_allocated(void) const {
    return reinterpret_cast<LookupAllocTable*>(this->table)->is_allocated();
}

LookupRefMap::LookupRefMap(int16_t* _xHeader, const uint16_t _xHeaderSize, int16_t* _yHeader, const uint16_t _yHeaderSize, int16_t* _data, const uint16_t _dataSize) {
    this->table = new LookupRefTable(_xHeader, _xHeaderSize, _data, _dataSize);
    this->yHeader = new LookupRefHeader(_yHeader, _yHeaderSize);
    this->yHeaderSize = _yHeaderSize;
}

LookupByteMap::LookupByteMap(uint8_t* _xHeader, const uint16_t _xHeaderSize, uint8_t* _yHeader, const uint16_t _yHeaderSize, uint8_t* _data, const uint16_t _dataSize) {
    this->x_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_xHeaderSize * sizeof(int16_t)));
    this->y_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_yHeaderSize * sizeof(int16_t)));
    this->z_alloc = static_cast<int16_t*>(TCU_HEAP_ALLOC(_dataSize * sizeof(int16_t)));

    for (auto i = 0; i < _xHeaderSize; i++) {
        this->x_alloc[i] = _xHeader[i];
    }
    for (auto i = 0; i < _yHeaderSize; i++) {
        this->y_alloc[i] = _yHeader[i];
    }
    for (auto i = 0; i < _dataSize; i++) {
        this->z_alloc[i] = _data[i];
    }
    if (nullptr != this->x_alloc && nullptr != this->y_alloc && nullptr != this->z_alloc) {
        this->table = new LookupRefTable(x_alloc, _xHeaderSize, z_alloc, _dataSize);
        this->yHeader = new LookupRefHeader(y_alloc, _yHeaderSize);
        this->yHeaderSize = _yHeaderSize;
    }
}

bool LookupByteMap::is_allocated(void) const {
    return nullptr != this->x_alloc && nullptr != this->y_alloc && nullptr != this->z_alloc;
}

bool LookupByteMap::add_data(const uint8_t* map, const uint16_t size) {
    if (size != this->z_size) {
        return false;
    } else {
        for (auto i = 0; i < size; i++) {
            this->z_alloc[i] = map[i];
        }
        return true;
    }
}

LookupByteMap::~LookupByteMap() {
    delete this->table;
    delete this->yHeader;
    TCU_FREE(this->x_alloc);
    TCU_FREE(this->y_alloc);
    TCU_FREE(this->z_alloc);
}
