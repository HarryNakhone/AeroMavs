#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <arm_math.h>

#define REG_BANK_SEL 0x7F
#define USER_BANK_0 0x00
#define USER_BANK_2 0x20
#define REG_ACCEL_XOUT_H 0x2D
#define GYRO_CONFIG_1 0x01
#define GYRO_SMPLRT_DIV 0x00

#define GYRO_SENSITIVITY 65.5f  // LSB/°/s at ±500 dps

// CMSIS Q31 moving average params for bias calculation
#define NUM_TAPS   512
#define BLOCK_SIZE 1

q31_t firCoeffs[NUM_TAPS];  // 1/NUM_TAPS in Q31 for all
q31_t firState_x[NUM_TAPS + BLOCK_SIZE - 1];
q31_t firState_y[NUM_TAPS + BLOCK_SIZE - 1];
q31_t firState_z[NUM_TAPS + BLOCK_SIZE - 1];

arm_fir_instance_q31 fir_x, fir_y, fir_z;

static struct spi_dt_spec icm_spi = SPI_DT_SPEC_GET(DT_NODELABEL(icm20649), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

// Forward declaration so static analysis is happy
static int icm_read_burst_raw(int16_t accel[3], int16_t *temp, int16_t gyro[3]);

static int icm_spi_write(uint8_t reg, const uint8_t *buf, size_t len) {
    if (!device_is_ready(icm_spi.bus)) return -ENODEV;
    uint8_t addr = reg & 0x7F;
    struct spi_buf bufs[2] = {{.buf = &addr, .len = 1}, {.buf = (void *)buf, .len = len}};
    const struct spi_buf_set tx = {.buffers = bufs, .count = 2};
    return spi_write_dt(&icm_spi, &tx);
}

static int icm_spi_read(uint8_t reg, uint8_t *buf, size_t len) {
    if (!device_is_ready(icm_spi.bus)) return -ENODEV;
    uint8_t addr = reg | 0x80;
    struct spi_buf txb = {.buf = &addr, .len = 1};
    struct spi_buf rxb[2] = {{.buf = NULL, .len = 1}, {.buf = buf, .len = len}};
    const struct spi_buf_set tx = {.buffers = &txb, .count = 1};
    const struct spi_buf_set rx = {.buffers = rxb, .count = 2};
    return spi_transceive_dt(&icm_spi, &tx, &rx);
}

static int icm_set_bank(uint8_t bank) {
    return icm_spi_write(REG_BANK_SEL, &bank, 1);
}

// Bias storage (int32 for improved precision)
static int32_t gyro_bias[3] = {0, 0, 0};

static int icm_init(void) {
    int rc;
    uint8_t v;

    // Select User Bank 0
    v = USER_BANK_0;
    rc = icm_spi_write(REG_BANK_SEL, &v, 1);
    if (rc) return rc;

    // PWR_MGMT_1: enable PLL, wake device
    v = 0x01; rc = icm_spi_write(0x06, &v, 1); if (rc) return rc; k_sleep(K_MSEC(10));
    v = 0x00; rc = icm_spi_write(0x07, &v, 1); if (rc) return rc;

    // Select User Bank 2, configure DLPF, sample rate
    v = USER_BANK_2; rc = icm_spi_write(REG_BANK_SEL, &v, 1); if (rc) return rc;

    uint8_t cur = 0; rc = icm_spi_read(GYRO_CONFIG_1, &cur, 1); if (rc) return rc;
    uint8_t dlpf_code = 0x04;
    uint8_t new_val = (cur & ~0x39) | ((dlpf_code << 3) & 0x38) | 0x01;
    rc = icm_spi_write(GYRO_CONFIG_1, &new_val, 1); if (rc) return rc;

    v = 4; rc = icm_spi_write(GYRO_SMPLRT_DIV, &v, 1); if (rc) return rc;
    v = USER_BANK_0; rc = icm_spi_write(REG_BANK_SEL, &v, 1); if (rc) return rc;
    k_sleep(K_MSEC(5));
    return 0;
}

static int icm_read_burst_raw(int16_t accel[3], int16_t *temp, int16_t gyro[3]) {
    int rc = icm_set_bank(USER_BANK_0); if (rc) return rc;
    uint8_t buf[14]; rc = icm_spi_read(REG_ACCEL_XOUT_H, buf, sizeof(buf)); if (rc) return rc;

    if (accel) {
        accel[0] = (int16_t)sys_get_be16(&buf[0]);
        accel[1] = (int16_t)sys_get_be16(&buf[2]);
        accel[2] = (int16_t)sys_get_be16(&buf[4]);
    }
    if (temp) *temp = (int16_t)sys_get_be16(&buf[6]);
    if (gyro) {
        gyro[0] = (int16_t)sys_get_be16(&buf[8]);
        gyro[1] = (int16_t)sys_get_be16(&buf[10]);
        gyro[2] = (int16_t)sys_get_be16(&buf[12]);
    }
    return 0;
}

int main(void) {
    int rc = icm_init();
    if (rc != 0) { printk("Init failed: %d\n", rc); return 0; }

    for (int i = 0; i < NUM_TAPS; i++) firCoeffs[i] = (q31_t)(0x7FFFFFFF / NUM_TAPS);

    arm_fir_init_q31(&fir_x, NUM_TAPS, firCoeffs, firState_x, BLOCK_SIZE);
    arm_fir_init_q31(&fir_y, NUM_TAPS, firCoeffs, firState_y, BLOCK_SIZE);
    arm_fir_init_q31(&fir_z, NUM_TAPS, firCoeffs, firState_z, BLOCK_SIZE);

    printk("Calibrating gyro bias: applying moving average of %d samples...\n", NUM_TAPS);
    for (int i = 0; i < NUM_TAPS; i++) {
        int16_t gyro[3];
        rc = icm_read_burst_raw(NULL, NULL, gyro);
        if (rc) { printk("Bias read failed: %d\n", rc); return rc; }

        q31_t sample_x = (q31_t)gyro[0] * 0x10000;
        q31_t sample_y = (q31_t)gyro[1] * 0x10000;
        q31_t sample_z = (q31_t)gyro[2] * 0x10000;

        q31_t out_x, out_y, out_z;
        arm_fir_q31(&fir_x, &sample_x, &out_x, BLOCK_SIZE);
        arm_fir_q31(&fir_y, &sample_y, &out_y, BLOCK_SIZE);
        arm_fir_q31(&fir_z, &sample_z, &out_z, BLOCK_SIZE);

        gyro_bias[0] = (int32_t)(out_x / 0x10000);
        gyro_bias[1] = (int32_t)(out_y / 0x10000);
        gyro_bias[2] = (int32_t)(out_z / 0x10000);

        k_sleep(K_MSEC(5));  
    }
    printk("Bias done. Bias X: %ld, Y: %ld, Z: %ld (LSB)\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    while (1) {
        int16_t gyro_raw[3];
        rc = icm_read_burst_raw(NULL, NULL, gyro_raw);
        if (rc) { printk("Read failed: %d\n", rc); k_sleep(K_MSEC(100)); continue; }

        int32_t corrected_x = gyro_raw[0] - gyro_bias[0];
        int32_t corrected_y = gyro_raw[1] - gyro_bias[1];
        int32_t corrected_z = gyro_raw[2] - gyro_bias[2];

        float dps_x = (float)corrected_x / GYRO_SENSITIVITY;
        float dps_y = (float)corrected_y / GYRO_SENSITIVITY;
        float dps_z = (float)corrected_z / GYRO_SENSITIVITY;
        printk("Gyro (bias-comp). X=%.2f Y=%.2f Z=%.2f deg/s | Bias: %ld %ld %ld\n",
               (double)dps_x, (double)dps_y, (double)dps_z,
               gyro_bias[0], gyro_bias[1], gyro_bias[2]);
        k_sleep(K_MSEC(100));  
    }
    return 0;
}
