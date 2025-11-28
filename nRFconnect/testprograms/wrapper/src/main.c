/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/kernel.h>


#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

static struct sensor_value accel_xyz_out[3];

K_MSGQ_DEFINE(imu_msgq, sizeof(int32_t) * 3 * 2, 150, 4);

static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig)
{
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

#define IMU_THREAD_STACK_SIZE 1024
#define IMU_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(imu_stack_area, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

static void imu_trigger(void *arg1, void *arg2, void *arg3);

#include <ei_wrapper.h>

#include "input_data.h"

static void result_ready_cb(int err)
{
	if (err) {
		printk("Result ready callback returned error (err: %d)\n", err);
		return;
	}

	const char *label;
	float value;
	float anomaly;

	printk("\nClassification results\n");
	printk("======================\n");

	while (true) {
		err = ei_wrapper_get_next_classification_result(&label, &value, NULL);

		if (err) {
			if (err == -ENOENT) {
				err = 0;
			}
			break;
		}

		printk("Value: %.2f\tLabel: %s\n", (double)value, label);
	}

	if (err) {
		printk("Cannot get classification results (err: %d)", err);
	} else {
		if (ei_wrapper_classifier_has_anomaly()) {
			err = ei_wrapper_get_anomaly(&anomaly);
			if (err) {
				printk("Cannot get anomaly (err: %d)\n", err);
			} else {
				printk("Anomaly: %.2f\n", (double)anomaly);
			}
		}
	}

	err = ei_wrapper_start_prediction(0, 9);
	if (err) {
		printk("Cannot restart prediction (err: %d)\n", err);
	} else {
		printk("Prediction restarted...\n");
	}
}

int main(void)
{
	k_thread_create(&imu_thread_data, imu_stack_area,
	K_THREAD_STACK_SIZEOF(imu_stack_area),
	imu_trigger,
	NULL, NULL, NULL,
	5, 0, K_NO_WAIT);

	return 0;
}

static void imu_trigger(void *arg1, void *arg2, void *arg3) {

	int err;
	struct sensor_value odr_attr;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	if (!device_is_ready(lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
	}
	
	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 52;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
	}

	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
	}

	err = ei_wrapper_init(result_ready_cb);

	if (err) {
		printk("Edge Impulse wrapper failed to initialize (err: %d)\n",
		       err);
		return 0;
	};

	if (ARRAY_SIZE(input_data) < ei_wrapper_get_window_size()) {
		printk("Not enough input data\n");
		return 0;
	}

	if (ARRAY_SIZE(input_data) % ei_wrapper_get_frame_size() != 0) {
		printk("Improper number of input samples\n");
		return 0;
	}

	printk("Machine learning model sampling frequency: %zu\n",
	       ei_wrapper_get_classifier_frequency());
	printk("Labels assigned by the model:\n");
	for (size_t i = 0; i < ei_wrapper_get_classifier_label_count(); i++) {
		printk("- %s\n", ei_wrapper_get_classifier_label(i));
	}
	printk("\n");

	size_t cnt = 0;

	err = ei_wrapper_start_prediction(0, 0);
	if (err) {
		printk("Cannot start prediction (err: %d)\n", err);
	} else {
		printk("Prediction started...\n");
	}

	float f_val_buffer[3];
	struct sensor_value acc_buffer[3];
	while (1) {
		k_msgq_get(&imu_msgq, acc_buffer, K_FOREVER);
		if (k_msgq_num_used_get(&imu_msgq) > (150 * 3 / 4)) {
								printk("Audio queue > 75%% full!\n");
		}
		f_val_buffer[0]= sensor_value_to_float(&acc_buffer[0]);
		f_val_buffer[1]= sensor_value_to_float(&acc_buffer[1]);
		f_val_buffer[2]= sensor_value_to_float(&acc_buffer[2]);
		err = ei_wrapper_add_data(f_val_buffer,3);
		if (err) {
			printk("Cannot provide input data (err: %d)\n", err);
			printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
		}
    }
}