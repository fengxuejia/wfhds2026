#include "i2c.h"
#include "ti_msp_dl_config.h"

#define LINE_SENSOR_I2C_ADDRESS 0x48U
#define LINE_SENSOR_PACKET_SIZE 7U
#define I2C_POLL_TIMEOUT        200000U

static uint8_t normalize_sensor(uint8_t value)
{
    if (value == (uint8_t)'0') {
        return 0U;
    }
    if (value == (uint8_t)'1') {
        return 1U;
    }
    return value & 1U;
}

static bool wait_for_idle(void)
{
    uint32_t timeout = I2C_POLL_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout-- == 0U) {
            return false;
        }
    }
    return true;
}

static bool read_sensor_packet(uint8_t packet[LINE_SENSOR_PACKET_SIZE])
{
    uint32_t timeout;

    if (!wait_for_idle()) {
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_INST, LINE_SENSOR_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, LINE_SENSOR_PACKET_SIZE);

    for (uint8_t i = 0U; i < LINE_SENSOR_PACKET_SIZE; i++) {
        timeout = I2C_POLL_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST)) {
            if (timeout-- == 0U) {
                DL_I2C_resetControllerTransfer(I2C_INST);
                return false;
            }
        }
        packet[i] = DL_I2C_receiveControllerData(I2C_INST);
    }

    if (!wait_for_idle()) {
        return false;
    }

    return true;
}

void I2C_Init(void)
{
    /* Clock, pins, and controller mode are initialized by SysConfig. */
}

bool I2C_IsControllerBusy(void)
{
    return (DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U;
}

bool I2C_ReadLineSensors(uint8_t sensors[12])
{
    static uint8_t first_half[6];
    static uint8_t second_half[6];
    static uint8_t received_halves;
    uint8_t packet[LINE_SENSOR_PACKET_SIZE];

    if (!read_sensor_packet(packet)) {
        received_halves = 0U;
        return false;
    }

    if (packet[0] == (uint8_t)'#') {
        for (uint8_t i = 0U; i < 6U; i++) {
            first_half[i] = packet[i + 1U];
        }
        received_halves |= 0x01U;
    } else if (packet[0] == (uint8_t)'!') {
        for (uint8_t i = 0U; i < 6U; i++) {
            second_half[i] = packet[i + 1U];
        }
        received_halves |= 0x02U;
    } else {
        received_halves = 0U;
        return false;
    }

    /* The sensor module sends two 7-byte packets for one 12-sensor frame. */
    if (received_halves != 0x03U) {
        return false;
    }

    for (uint8_t i = 0U; i < 6U; i++) {
        sensors[i] = normalize_sensor(first_half[i]);
        sensors[i + 6U] = normalize_sensor(second_half[i]);
    }

    received_halves = 0U;
    return true;
}
