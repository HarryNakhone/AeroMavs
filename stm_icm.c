#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

/* Devicetree handle to our SPI child node */
#define ICM_NODE DT_NODELABEL(icm20649)

/* Validate node presence */
#if !DT_NODE_HAS_STATUS(ICM_NODE, okay)
#error "icm20649 node not found in devicetree"
#endif

/* SPI config: 8-bit words, mode to match part, single line.
 * Most InvenSense parts use SPI Mode 0 or 3; check the ICM-20649 datasheet’s SPI section and timing diagrams.
 * Start with Mode 0, switch to Mode 3 if reads are scrambled.
 */
#define ICM_SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA) /* Mode 3 trial */
 /* Alternative trial: Mode 0
  * #define ICM_SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
  */

static const struct spi_dt_spec icm_spi = SPI_DT_SPEC_GET(ICM_NODE, ICM_SPI_OP, 0);

enum {
	REG_BANK_SEL = 0x7F,   /* User bank select (Bank 0 register) */
	/* Bank 0 registers */
	REG_WHO_AM_I = 0x00,   /* WHO_AM_I should read 0xE1 for ICM-20649 */
	REG_ACCEL_XOUT_H = 0x2D, /* Burst start: ACCEL(6), TEMP(2), GYRO(6) */
};

/* WHO_AM_I expected value per datasheet */
#define ICM20649_WHO_AM_I_EXPECTED 0xE1

/* SPI protocol on InvenSense parts:
 * - Read: MSB=1 on register address; Write: MSB=0.
 * - Multi-byte read is supported by incrementing internal addr.
 * Confirm with datasheet's SPI protocol section.
 */
static int icm_spi_read(uint8_t reg, uint8_t *buf, size_t len)
{
	if (!device_is_ready(icm_spi.bus)) {
		return -ENODEV;
	}
	uint8_t addr = reg | 0x80; /* read bit */
	struct spi_buf txb = { .buf = &addr, .len = 1 };
	struct spi_buf rxb[2] = {
		{ .buf = NULL, .len = 1 },
		{ .buf = buf, .len = len },
	};
	const struct spi_buf_set tx = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rx = { .buffers = rxb, .count = 2 };
	return spi_transceive_dt(&icm_spi, &tx, &rx);
}

static int icm_spi_write(uint8_t reg, const uint8_t *buf, size_t len)
{
	if (!device_is_ready(icm_spi.bus)) {
		return -ENODEV;
	}
	uint8_t addr = reg & 0x7F; /* write bit cleared */
	struct spi_buf bufs[2] = {
		{ .buf = &addr, .len = 1 },
		{ .buf = (void *)buf, .len = len },
	};
	const struct spi_buf_set tx = { .buffers = bufs, .count = 2 };
	return spi_write_dt(&icm_spi, &tx);
}

static int icm_set_bank(uint8_t bank)
{
	/* REG_BANK_SEL bits select user bank 0..3; datasheet defines the field location. */
	uint8_t v = (bank << 4); /* common mapping is bits [5:4]; adjust if needed per DS */
	return icm_spi_write(REG_BANK_SEL, &v, 1);
}

static int icm_read_whoami(uint8_t *out)
{
	int rc = icm_set_bank(0);
	if (rc) return rc;
	return icm_spi_read(REG_WHO_AM_I, out, 1);
}

static int icm_read_burst_raw(int16_t accel[3], int16_t *temp, int16_t gyro[3])
{
	int rc = icm_set_bank(0);
	if (rc) return rc;

	uint8_t buf[14];
	rc = icm_spi_read(REG_ACCEL_XOUT_H, buf, sizeof(buf));
	if (rc) return rc;

	accel[0] = (int16_t)sys_get_be16(&buf[0]);
	accel[1] = (int16_t)sys_get_be16(&buf[2]);
	accel[2] = (int16_t)sys_get_be16(&buf[4]);
	*temp    = (int16_t)sys_get_be16(&buf[6]);
	gyro[0]  = (int16_t)sys_get_be16(&buf[8]);
	gyro[1]  = (int16_t)sys_get_be16(&buf[10]);
	gyro[2]  = (int16_t)sys_get_be16(&buf[12]);
	return 0;
}

void main(void)
{
	k_sleep(K_MSEC(50)); /* power-up settle; adjust per DS */

	uint8_t wai = 0;
	int rc = icm_read_whoami(&wai);
	if (rc) {
		printk("ICM20649: WHO_AM_I read failed: %d\n", rc);
		return;
	}
	printk("ICM20649: WHO_AM_I=0x%02X\n", wai);

	if (wai != ICM20649_WHO_AM_I_EXPECTED) {
		printk("ICM20649: unexpected WHO_AM_I (expected 0xE1)\n");
	}

	/* Example burst read loop */
	while (1) {
		int16_t ax[3], g[3], t;
		rc = icm_read_burst_raw(ax, &t, g);
		if (rc) {
			printk("burst read failed: %d\n", rc);
		} else {
			printk("AX:%d %d %d  T:%d  GX:%d %d %d\n",
			       ax[0], ax[1], ax[2], t, g[0], g[1], g[2]);
		}
		k_sleep(K_MSEC(100));
	}
}
