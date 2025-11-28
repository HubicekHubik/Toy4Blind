/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define DEVICE_NAME		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static struct sensor_value accel_xyz_out[3];

K_MSGQ_DEFINE(imu_msgq, sizeof(int32_t) * 3 * 2, 450, 4);

static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig);

static void notif_enabled(bool enabled, void *ctx)
{
	ARG_UNUSED(ctx);

	printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);

	printk("%s() - Len: %d, Message: %.*s\n", __func__, len, len, (char *)data);
}

struct bt_nus_cb nus_listener = {
	.notif_enabled = notif_enabled,
	.received = received,
};

int main(void)
{
	int err;

	printk("Sample - Bluetooth Peripheral NUS\n");
	
	err = bt_enable(NULL);
	if (err) {
		printk("Failed to enable bluetooth: %d\n", err);
		return err;
	}

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		printk("Failed to register NUS callback: %d\n", err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Failed to start advertising: %d\n", err);
		return err;
	}

	printk("Initialization complete\n");

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

	float f_val_buffer[3];
	struct sensor_value acc_buffer[3];

	while (true) {
		k_msgq_get(&imu_msgq, acc_buffer, K_FOREVER);
		if (k_msgq_num_used_get(&imu_msgq) > (150 * 3 / 4)) {
								printk("IMU queue > 75%% full!\n");
		}
		f_val_buffer[0]= sensor_value_to_float(&acc_buffer[0]);
		f_val_buffer[1]= sensor_value_to_float(&acc_buffer[1]);
		f_val_buffer[2]= sensor_value_to_float(&acc_buffer[2]);
		//printk("This are the walues form sensor Ax:%f Ay:%f Az:%f \n",f_val_buffer[0], f_val_buffer[1], f_val_buffer[2]);
		
		err = bt_nus_send(NULL, f_val_buffer, sizeof(f_val_buffer));
		
		printk("Data seAnd - Result: %d\n", err);

		if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
			return err;
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