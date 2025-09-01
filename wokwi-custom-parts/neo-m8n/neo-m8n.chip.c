#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    pin_t pin_vcc;
    pin_t pin_gnd;
    pin_t pin_tx;
    pin_t pin_rx;
    uint32_t led_fix;
    
    // GPS simulation state
    uint32_t time_ms;
    uint32_t last_sentence_time;
    uint8_t fix_state;        // 0=no fix, 1=acquiring, 2=fixed
    uint8_t satellites;
    uint16_t sentence_counter;
    
    // Current simulated position (San Francisco)
    double latitude;
    double longitude;
    float altitude;
    float hdop;
} chip_state_t;

static chip_state_t chip;

// Calculate NMEA checksum
static uint8_t calculate_checksum(const char* sentence) {
    uint8_t checksum = 0;
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
        checksum ^= sentence[i];
    }
    return checksum;
}

// Generate GGA sentence (Global Positioning System Fix Data)
static void send_gga_sentence(void) {
    char sentence[128];
    char lat_str[16], lon_str[16];
    
    // Convert decimal degrees to NMEA format (DDMM.MMMM)
    double lat_abs = chip.latitude >= 0 ? chip.latitude : -chip.latitude;
    double lon_abs = chip.longitude >= 0 ? chip.longitude : -chip.longitude;
    
    int lat_deg = (int)lat_abs;
    int lon_deg = (int)lon_abs;
    double lat_min = (lat_abs - lat_deg) * 60.0;
    double lon_min = (lon_abs - lon_deg) * 60.0;
    
    sprintf(lat_str, "%02d%07.4f", lat_deg, lat_min);
    sprintf(lon_str, "%03d%07.4f", lon_deg, lon_min);
    
    // Current time (simulated)
    uint32_t hours = (chip.time_ms / 3600000) % 24;
    uint32_t minutes = (chip.time_ms / 60000) % 60;
    uint32_t seconds = (chip.time_ms / 1000) % 60;
    
    sprintf(sentence, "$GPGGA,%02lu%02lu%02lu.00,%s,%c,%s,%c,%d,%02d,%.1f,%.1f,M,0.0,M,,*",
            hours, minutes, seconds,
            lat_str, chip.latitude >= 0 ? 'N' : 'S',
            lon_str, chip.longitude >= 0 ? 'E' : 'W',
            chip.fix_state >= 2 ? 1 : 0,  // Fix quality
            chip.satellites,
            chip.hdop,
            chip.altitude);
    
    // Add checksum
    uint8_t checksum = calculate_checksum(sentence);
    char final_sentence[140];
    sprintf(final_sentence, "%s%02X\r\n", sentence, checksum);
    
    // Send via UART
    for (int i = 0; final_sentence[i] != '\0'; i++) {
        pin_dac_write(chip.pin_tx, final_sentence[i] * 255 / 256);
    }
}

// Generate RMC sentence (Recommended Minimum Course)
static void send_rmc_sentence(void) {
    char sentence[128];
    char lat_str[16], lon_str[16];
    
    // Convert coordinates
    double lat_abs = chip.latitude >= 0 ? chip.latitude : -chip.latitude;
    double lon_abs = chip.longitude >= 0 ? chip.longitude : -chip.longitude;
    
    int lat_deg = (int)lat_abs;
    int lon_deg = (int)lon_abs;
    double lat_min = (lat_abs - lat_deg) * 60.0;
    double lon_min = (lon_abs - lon_deg) * 60.0;
    
    sprintf(lat_str, "%02d%07.4f", lat_deg, lat_min);
    sprintf(lon_str, "%03d%07.4f", lon_deg, lon_min);
    
    // Current time and date
    uint32_t hours = (chip.time_ms / 3600000) % 24;
    uint32_t minutes = (chip.time_ms / 60000) % 60;
    uint32_t seconds = (chip.time_ms / 1000) % 60;
    
    sprintf(sentence, "$GPRMC,%02lu%02lu%02lu.00,%c,%s,%c,%s,%c,0.0,0.0,010125,0.0,E*",
            hours, minutes, seconds,
            chip.fix_state >= 2 ? 'A' : 'V',  // Status: A=active, V=void
            lat_str, chip.latitude >= 0 ? 'N' : 'S',
            lon_str, chip.longitude >= 0 ? 'E' : 'W');
    
    // Add checksum
    uint8_t checksum = calculate_checksum(sentence);
    char final_sentence[140];
    sprintf(final_sentence, "%s%02X\r\n", sentence, checksum);
    
    // Send via UART
    for (int i = 0; final_sentence[i] != '\0'; i++) {
        pin_dac_write(chip.pin_tx, final_sentence[i] * 255 / 256);
    }
}

// Timer callback for GPS sentence generation
void chip_timer_event(void *user_data) {
    chip.time_ms += 1000;  // 1 second increment
    
    // Update GPS fix state based on time
    if (chip.time_ms < 5000) {
        chip.fix_state = 0;  // No fix for first 5 seconds
        chip.satellites = 0;
    } else if (chip.time_ms < 15000) {
        chip.fix_state = 1;  // Acquiring fix
        chip.satellites = (chip.time_ms - 5000) / 2500;  // Gradually acquire sats
    } else {
        chip.fix_state = 2;  // Fixed
        chip.satellites = 8;
    }
    
    // Update LED based on fix state
    pin_write(chip.led_fix, chip.fix_state >= 2 ? HIGH : LOW);
    
    // Send NMEA sentences every second
    if (chip.time_ms % 1000 == 0) {
        send_gga_sentence();
        send_rmc_sentence();
    }
}

void chip_init(void) {
    chip.pin_vcc = pin_init("VCC", INPUT_PULLDOWN);
    chip.pin_gnd = pin_init("GND", INPUT_PULLDOWN);
    chip.pin_tx = pin_init("TX", OUTPUT);
    chip.pin_rx = pin_init("RX", INPUT);
    chip.led_fix = pin_init("led", OUTPUT);
    
    // Initialize GPS state
    chip.time_ms = 0;
    chip.fix_state = 0;
    chip.satellites = 0;
    chip.sentence_counter = 0;
    
    // San Francisco coordinates for simulation
    chip.latitude = 37.7749;
    chip.longitude = -122.4194;
    chip.altitude = 16.0;
    chip.hdop = 1.2;
    
    // Set up timer for 1Hz GPS updates
    const timer_config_t timer_config = {
        .callback = chip_timer_event,
        .user_data = &chip,
    };
    timer_t timer_id = timer_init(&timer_config);
    timer_start(timer_id, 1000, true);  // 1000ms interval, repeating
}