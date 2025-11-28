/* main.c - Application main entry point */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/*imu modules*/
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
static struct sensor_value accel_xyz_out[3];

K_MSGQ_DEFINE(imu_msgq, sizeof(int32_t) * 3 * 2, 450, 4);

static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig);

					/**/

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

#if !defined(CONFIG_BT_EXT_ADV)
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
#endif /* !CONFIG_BT_EXT_ADV */

/* Use atomic variable, 2 bits for connection and disconnection state */
static ATOMIC_DEFINE(state, 2U);

#define STATE_CONNECTED    1U
#define STATE_DISCONNECTED 2U

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");

		(void)atomic_set_bit(state, STATE_CONNECTED);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	(void)atomic_set_bit(state, STATE_DISCONNECTED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	    // get driver for the accelerometer
	struct sensor_value odr_attr;
	const struct device *lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
    if (lsm6dsl_dev == NULL) {
        printf("Could not get lsm6dsl_dev device\n");
        return EXIT_FAILURE;
    }

	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 52;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printf("Cannot set sampling frequency for accelerometer.\n");
		return EXIT_FAILURE;
	}

	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
	}
	
	struct esb_payload tx = {
        .noack  = true,
        .length = sizeof(float) * 3
    };

	float f_val_buffer[3];
	struct sensor_value acc_buffer[3];
	while (1) {
		k_msgq_get(&imu_msgq, acc_buffer, K_FOREVER);
		if (k_msgq_num_used_get(&imu_msgq) > (150 * 3 / 4)) {
								printk("IMU queue > 75%% full!\n");
		}
		f_val_buffer[0]= sensor_value_to_float(&acc_buffer[0]);
		f_val_buffer[1]= sensor_value_to_float(&acc_buffer[1]);
		f_val_buffer[2]= sensor_value_to_float(&acc_buffer[2]);
		//printk("This are the walues form sensor Ax:%f Ay:%f Az:%f \n",f_val_buffer[0], f_val_buffer[1], f_val_buffer[2]);

		/**/ //bt_serial_print("This are the walues form sensor Ax:%f Ay:%f Az:%f \n",f_val_buffer[0], f_val_buffer[1], f_val_buffer[2]);
		if (err) {
			printk("esb_write_payload failed, err: %d\n", err);
			err = esb_flush_tx();
			if (err) {
				printk("Flush tx failed, err: %d", err);
			}
		}
    }
#if !defined(CONFIG_BT_EXT_ADV)
	printk("Starting Legacy Advertising (connectable and scannable)\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}
#endif /* CONFIG_BT_EXT_ADV */

	printk("Advertising successfully started\n");

#if defined(HAS_LED)
	err = blink_setup();
	if (err) {
		return 0;
	}

	blink_start();
#endif /* HAS_LED */

	/* Implement notification. */
	while (1) {
		k_sleep(K_SECONDS(1));


		if (atomic_test_and_clear_bit(state, STATE_CONNECTED)) {
			/* Connected callback executed */

#if defined(HAS_LED)
			blink_stop();
#endif /* HAS_LED */
		} else if (atomic_test_and_clear_bit(state, STATE_DISCONNECTED)) {
#if !defined(CONFIG_BT_EXT_ADV)
			printk("Starting Legacy Advertising (connectable and scannable)\n");
			err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
					      ARRAY_SIZE(sd));
			if (err) {
				printk("Advertising failed to start (err %d)\n", err);
				return 0;
			}
#endif /* CONFIG_BT_EXT_ADV */

#if defined(HAS_LED)
			blink_start();
#endif /* HAS_LED */
		}
	}

	return 0;
}

static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig) {

	static struct sensor_value accel_x, accel_y, accel_z;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
	accel_xyz_out[0]=accel_x;
	accel_xyz_out[1]=accel_y;
	accel_xyz_out[2]=accel_z;
	int err = k_msgq_put(&imu_msgq, accel_xyz_out, K_NO_WAIT);
	if(err){
		printk("Couldn t send packet err:%d\n", err);
	}
}