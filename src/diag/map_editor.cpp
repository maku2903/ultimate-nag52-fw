#include "map_editor.h"
#include "kwp2000_defines.h"
#include "nvs/eeprom_config.h"
#include "../maps.h"
#include "string.h"
#include "speaker.h"
#include "profiles.h"
#include "pressure_manager.h"
#include "gearbox.h"
#include "tcu_alloc.h"
#include "clock.hpp"
#include <limits>

static_assert(sizeof(float) == 4u, "Map lookup cache wire format requires 32-bit floats");
static_assert(std::numeric_limits<float>::is_iec559, "Map lookup cache wire format requires IEEE 754 floats");
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "Map lookup cache wire format requires a little-endian target");
static_assert(MAP_LOOKUP_CACHE_ENTRY_SIZE == 13u, "Map lookup cache wire entry size must stay 13 bytes");
static const uint16_t MAP_LOOKUP_CACHE_HEADER_SIZE = 4u;
static const uint32_t MAP_LOOKUP_CACHE_MAX_AGE_MS = 60000u;
static_assert((MAP_LOOKUP_CACHE_HEADER_SIZE + (MAX_LOOKUP_CACHE * MAP_LOOKUP_CACHE_ENTRY_SIZE)) <= UINT16_MAX, "Map lookup cache response must fit in uint16_t");

static void write_u32_le(uint8_t* dest, uint32_t value) {
    dest[0] = static_cast<uint8_t>(value & 0x000000FFu);
    dest[1] = static_cast<uint8_t>((value >> 8u) & 0x000000FFu);
    dest[2] = static_cast<uint8_t>((value >> 16u) & 0x000000FFu);
    dest[3] = static_cast<uint8_t>((value >> 24u) & 0x000000FFu);
}

static void write_f32_le(uint8_t* dest, float value) {
    uint32_t raw = 0u;
    memcpy(&raw, &value, sizeof(raw));
    write_u32_le(dest, raw);
}

static uint16_t lookup_cache_payload_size(uint8_t entry_count) {
    const uint16_t header_size = MAP_LOOKUP_CACHE_HEADER_SIZE;
    const uint16_t entry_size = MAP_LOOKUP_CACHE_ENTRY_SIZE;
    return static_cast<uint16_t>(header_size + (static_cast<uint16_t>(entry_count) * entry_size));
}

StoredMap* get_map(uint8_t map_id) {
    switch(map_id) {
        case A_UPSHIFT_MAP_ID:
            return agility->get_upshift_map();
        case C_UPSHIFT_MAP_ID:
            return comfort->get_upshift_map();
        case S_UPSHIFT_MAP_ID:
            return standard->get_upshift_map();
        case A_DOWNSHIFT_MAP_ID:
            return agility->get_downshift_map();
        case C_DOWNSHIFT_MAP_ID:
            return comfort->get_downshift_map();
        case S_DOWNSHIFT_MAP_ID:
            return standard->get_downshift_map();
        case TCC_PWM_MAP_ID:
            return pressure_manager->get_tcc_pwm_map();
        case FILL_TIME_MAP_ID:
            return pressure_manager->get_fill_time_map();
        case FILL_PRESSURE_MAP_ID:
            return pressure_manager->get_fill_pressure_map();
        case FILL_PRESSURE_LOW_MAP_ID:
            return pressure_manager->get_low_fill_pressure_map();
        case A_UPTIME_MAP_ID:
            return agility->get_upshift_time_map();
        case A_DNTIME_MAP_ID:
            return agility->get_downshift_time_map();
        case S_UPTIME_MAP_ID:
            return standard->get_upshift_time_map();
        case S_DNTIME_MAP_ID:
            return standard->get_downshift_time_map();
        case C_UPTIME_MAP_ID:
            return comfort->get_upshift_time_map();
        case C_DNTIME_MAP_ID:
            return comfort->get_downshift_time_map();
        case W_UPTIME_MAP_ID:
            return winter->get_upshift_time_map();
        case W_DNTIME_MAP_ID:
            return winter->get_downshift_time_map();
        case M_UPTIME_MAP_ID:
            return manual->get_upshift_time_map();
        case M_DNTIME_MAP_ID:
            return manual->get_downshift_time_map();
        case TCC_ADAPT_SLIP_MAP_ID:
            return gearbox->tcc->get_slip_map();
        case TCC_ADAPT_LOCK_MAP_ID:
            return gearbox->tcc->get_lock_map();
        case TCC_RPM_SLIP_MAP:
            return gearbox->tcc->get_rpm_slip_map();
        default:
            return nullptr;
    }
}

#define CHECK_MAP(map_id) \
    StoredMap* ptr = get_map(map_id); \
    if (ptr == nullptr) { \
        return NRC_SUB_FUNC_NOT_SUPPORTED_INVALID_FORMAT; \
    }

kwp_result_t MapEditor::read_map_data(uint8_t map_id, uint8_t read_type, uint16_t *dest_size_bytes, uint8_t** buffer) {
    CHECK_MAP(map_id)
    // Map valid
    uint16_t size = ptr->get_map_element_count();
    uint8_t* b = static_cast<uint8_t*>(TCU_HEAP_ALLOC((size*sizeof(int16_t))));
    if (nullptr == b) {
        ESP_LOGE("MAP_EDITOR_R", "Could not allocate read array!");
        return NRC_UN52_NO_MEM;
    }
    if (read_type ==  MAP_READ_TYPE_PRG) {
        memcpy(b, ptr->get_default_map_data(), size*sizeof(int16_t));
    } else if (read_type == MAP_READ_TYPE_MEM) {
        memcpy(b, ptr->get_current_data(), size*sizeof(int16_t));
    } else if (read_type == MAP_READ_TYPE_STO) {
        int16_t* eeprom_data = ptr->get_current_eeprom_map_data();
        memcpy(b, eeprom_data, size*sizeof(int16_t));
        TCU_FREE(eeprom_data);
    } else {
        TCU_FREE(buffer);
        return NRC_GENERAL_REJECT;
    }
    *buffer = b;
    *dest_size_bytes = size*sizeof(int16_t);
    return NRC_OK;
}

kwp_result_t MapEditor::read_map_metadata(uint8_t map_id, uint16_t *dest_size_bytes, uint8_t** buffer) {
    CHECK_MAP(map_id)
    // X meta, Y meta, KEY_NAME
    int16_t* x_ptr;
    int16_t* y_ptr;
    const char* k_ptr;
    uint16_t x_size;
    uint16_t y_size;
    uint16_t k_size;

    ptr->get_x_headers(&x_size, &x_ptr);
    ptr->get_y_headers(&y_size, &y_ptr);
    k_ptr = ptr->get_map_name();
    k_size = strlen(k_ptr);
    // 6 bytes for size data
    uint16_t size = 6+k_size+((x_size+y_size)*sizeof(int16_t));
    uint8_t* b = static_cast<uint8_t*>(TCU_HEAP_ALLOC(size));
    if (nullptr == b) {
        return NRC_UN52_NO_MEM;
    }
    b[1] = x_size >> 8;
    b[0] = x_size & 0xFF;
    b[3] = y_size >> 8;
    b[2] = y_size & 0xFF;
    b[5] = k_size >> 8;
    b[4] = k_size & 0xFF;
    memcpy(&b[6], x_ptr, x_size*sizeof(int16_t));
    memcpy(&b[6+(x_size*sizeof(int16_t))], y_ptr, y_size*sizeof(int16_t));
    memcpy(&b[6+((x_size+y_size)*sizeof(int16_t))], k_ptr, k_size);
    *buffer = b;
    *dest_size_bytes = size;
    return NRC_OK;
}

kwp_result_t MapEditor::read_map_lookup_cache(uint8_t map_id, uint16_t *dest_size_bytes, uint8_t** buffer) {
    CHECK_MAP(map_id)

    uint8_t entry_count = 0;
    const uint32_t now = GET_CLOCK_TIME();
    LookupCacheReadEntry entries[MAX_LOOKUP_CACHE] = {};
    ptr->copy_lookup_cache(&entry_count, entries, MAX_LOOKUP_CACHE, now, MAP_LOOKUP_CACHE_MAX_AGE_MS);

    const uint16_t size = lookup_cache_payload_size(entry_count);
    uint8_t* b = static_cast<uint8_t*>(TCU_HEAP_ALLOC(size));
    if (nullptr == b) {
        return NRC_UN52_NO_MEM;
    }

    b[0] = entry_count;
    b[1] = MAP_LOOKUP_CACHE_ENTRY_SIZE;
    b[2] = 0u;
    b[3] = 0u;

    uint8_t* dest = &b[MAP_LOOKUP_CACHE_HEADER_SIZE];
    for (uint8_t i = 0; i < entry_count; i++) {
        const LookupCacheReadEntry& wire_entry = entries[i];
        const LookupCache& entry = wire_entry.cache;
        dest[0] = wire_entry.slot_id;
        write_f32_le(&dest[1], entry.x_val);
        write_f32_le(&dest[5], entry.y_val);
        write_u32_le(&dest[9], entry.timestamp_ms);
        dest += MAP_LOOKUP_CACHE_ENTRY_SIZE;
    }

    *buffer = b;
    *dest_size_bytes = size;
    return NRC_OK;
}
    
kwp_result_t MapEditor::write_map_data(uint8_t map_id, uint16_t dest_size, int16_t* buffer) {
    CHECK_MAP(map_id)
    if (ptr->replace_data_content(buffer, dest_size) == ESP_OK) {
        return NRC_OK;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

kwp_result_t MapEditor::burn_to_eeprom(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ptr->save_to_eeprom() == ESP_OK) {
        return NRC_OK;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

uint8_t MapEditor::reset_to_program_default(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ESP_OK == ptr->reset_from_flash()) {
        return 0;
    } else {
        return NRC_GENERAL_REJECT;
    }
}

kwp_result_t MapEditor::undo_changes(uint8_t map_id) {
    CHECK_MAP(map_id)
    if (ptr->reload_from_eeprom() == ESP_OK) {
        return NRC_OK;
    } else {
        return NRC_GENERAL_REJECT;
    }
}
