#include "ei_imu_modul.h"
#include "app_state.h"
#include "toy_utils.h"
//=======================================//
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/bluetooth/services/nus/inst.h>
//========================================//
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(EI_IMU, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#include <ei_wrapper.h>
#define SAMPLING_FREQ 104
#define RESULTS_PRINT 0
bool ei_canceled = false;
#define HISTORY_LEN 14
#define NUM_CLASSES 5
#define MIN_DETECTION_COUNT 6
#define STRAIGHT_IDX 5
#define DEBUG_RESULT_CB // coment this to use the old function of result_ready
#define IDLEFOLLOWUP_TREASHOLD 500

uint8_t hystor_index;
float label_history[HISTORY_LEN][NUM_CLASSES];
float labels_value[NUM_CLASSES];
uint8_t idlefollowup = 0;

static void result_ready_cb(int err);

static struct sensor_value accel_gyro_xyz_out[6];

K_MSGQ_DEFINE(imu_msgq, sizeof(struct sensor_value) * 6, 200, 4);

#define IMU_THREAD_STACK_SIZE 1024
#define IMU_THREAD_PRIORITY 4
K_THREAD_STACK_DEFINE(imu_stack_area, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

static void lsm6dsl_trigger_handler(const struct device *dev,
									const struct sensor_trigger *trig);

static void imu_ini_data_forw(void *arg1, void *arg2, void *arg3);

int imu_ei_init() {
    k_thread_create(&imu_thread_data, imu_stack_area,
                        K_THREAD_STACK_SIZEOF(imu_stack_area),
                        imu_ini_data_forw,
                        NULL, NULL, NULL,
                        IMU_THREAD_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

void lsm6dsl_sleep() {
	int err;
    const struct device *lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

    // 1) Zastavit trigger
    sensor_trigger_set(lsm6dsl_dev, NULL, NULL);

    // 2) Vypnout ODR → IMU přestane posílat data
    struct sensor_value off_odr = {0, 0};
    sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY, &off_odr);

    // 3) Vyčistit queue
    k_msgq_purge(&imu_msgq);
	bool canceld;
	err = ei_wrapper_clear_data(&canceld);
    	LOG_INF("Value of cnaceled is:%d and err: %d", canceld, err);
	if(!err && canceld) {
	    LOG_INF("IMU stopped & queue cleared, system ready for sleep\n");
	}
}

void lsm6dsl_ei_wake() {
	int err;
	if(!device_running){
		LOG_WRN("Device is turned off skipping lsm_ei_wake");
		return 0;
	}
	/*LOG_INF("Machine learning model sampling frequency: %zu",
		   ei_wrapper_get_classifier_frequency());
	LOG_INF("Labels assigned by the model:");
	for (size_t i = 0; i < ei_wrapper_get_classifier_label_count(); i++) {
		LOG_INF("- %s", ei_wrapper_get_classifier_label(i));
	}
	*/
	err = ei_wrapper_start_prediction(1, 0);
	if (err) {
		LOG_ERR("Cannot start prediction (err: %d)", err);
	}
	else {
		LOG_INF("Prediction started...");
	}

	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
	
	if (!device_is_ready(lsm6dsl_dev)) {
		LOG_ERR("IMU not ready");
	}	

	struct sensor_value odr_attr;
	odr_attr.val1 = SAMPLING_FREQ;
	odr_attr.val2 = 0;

	//struct sensor_value has_freq;
	//sensor_attr_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &has_freq);
	//if(has_freq.val1 != 0 && has_freq.val2 != 0) {}
	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
						SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		LOG_WRN("Cannot set sampling frequency for accelerometer.");
	}

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		LOG_WRN("Cannot set sampling frequency for gyro.\n");
	}
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		LOG_ERR("Could not set sensor type and channel");
	}
}

static void lsm6dsl_trigger_handler(const struct device *dev,
									const struct sensor_trigger *trig)
{

	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
	accel_gyro_xyz_out[0] = accel_x;
	accel_gyro_xyz_out[1] = accel_y;
	accel_gyro_xyz_out[2] = accel_z;
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
	accel_gyro_xyz_out[3]=gyro_x;
	accel_gyro_xyz_out[4]=gyro_y;
	accel_gyro_xyz_out[5]=gyro_z;
	int err = k_msgq_put(&imu_msgq, accel_gyro_xyz_out, K_NO_WAIT);
	if (err) {
		printk("IMU queue full dropping packet:%d\n", err);
	}
}

static void imu_ini_data_forw(void *arg1, void *arg2, void *arg3) {
	int err;
	err = ei_wrapper_init(result_ready_cb);

	if (err) {
		LOG_INF("Edge Impulse wrapper failed to initialize (err: %d)\n",
			   err);
		return;
	};
	lsm6dsl_ei_wake();

	float f_val_buffer[6];
	struct sensor_value acc_gyro_buffer[6];
	while (true) {
		//k_sem_take(&run_sem, K_FOREVER); // pokud je OFF, vlákno se zastaví
		//k_sem_give(&run_sem);

		k_msgq_get(&imu_msgq, acc_gyro_buffer, K_FOREVER);
		if (k_msgq_num_used_get(&imu_msgq) > (200 * 3 / 4))
		{
			LOG_WRN("Audio queue > 75%% full!\n");
		}
		f_val_buffer[0] = sensor_value_to_float(&acc_gyro_buffer[0]);
		f_val_buffer[1] = sensor_value_to_float(&acc_gyro_buffer[1]);
		f_val_buffer[2] = sensor_value_to_float(&acc_gyro_buffer[2]);
		f_val_buffer[3] = sensor_value_to_float(&acc_gyro_buffer[3]);
		f_val_buffer[4] = sensor_value_to_float(&acc_gyro_buffer[4]);
		f_val_buffer[5] = sensor_value_to_float(&acc_gyro_buffer[5]);
		//LOG_INF("This are the walues form sensor Ax:%f Ay:%f Az:%f Gx:%f Gy:%f  Gz:%f",f_val_buffer[0], f_val_buffer[1], f_val_buffer[2], f_val_buffer[3], f_val_buffer[4], f_val_buffer[5]);
		if (!audioPlaying) {
			err = ei_wrapper_add_data(f_val_buffer, 6);
			if (err) {
				LOG_INF("Cannot provide input data (err: %d)\n", err);
				LOG_ERR("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
			}
		} else {
		}
	}
}

#ifdef DEBUG_RESULT_CB
static void result_ready_cb(int err) {
	if (err) {
		LOG_INF("Result ready callback returned error (err: %d)\n", err);
		return;
	}

	const char *label;
	float temp_value;
	float values[5];
	float anomaly = 404.0f;
	int idx;
	char buf[200];
	int len = 0;

	if (RESULTS_PRINT) {
		LOG_INF("Classification results");
		LOG_INF("======================");
	}
	while (true) {    
		err = ei_wrapper_get_next_classification_result(&label, &temp_value, &idx);
		values[idx] = temp_value;

		if (err)
		{
			if (err == -ENOENT)
			{
				err = 0;
			}
			break;
		}
		if (RESULTS_PRINT) {
			LOG_INF("Value: %.2f\tLabel: %s", (double)temp_value, label);
		}
	}

	if (err) {
		LOG_INF("Cannot get classification results (err: %d)", err);
	}else {
		if (ei_wrapper_classifier_has_anomaly()) {
			err = ei_wrapper_get_anomaly(&anomaly);
			if (err) {
				LOG_INF("Cannot get anomaly (err: %d)", err);
			} else if (RESULTS_PRINT) {
				LOG_INF("Anomaly: %.2f", (double)anomaly);
			}
		}
	}

	for (size_t i = 0; i < ei_wrapper_get_classifier_label_count(); i++) {
		if(master_conn != NULL) {
		len += snprintf(buf + len, sizeof(buf) - len, 
                            "Value: %.2f\tLabel: %s\n", 
                            (double)values[i], ei_wrapper_get_classifier_label(i));
		}
		if (( i != 4) && (values[i] > 0.95f) &&
			((i != 2) || (idlefollowup >= IDLEFOLLOWUP_TREASHOLD))) {
			struct toy_events tx_gest = {
				.type = MT_GEST,
				.payload.gest.type = i,
				.payload.gest.prob = (uint8_t)(values[i] * 100.0f),
				.payload.gest.anomaly = (uint16_t)(anomaly *anomaly * 100.0f),
				.payload.gest.sign = 0
			};
			if (anomaly < 0) {
				tx_gest.payload.gest.sign = 1;
			}
			if (!audioPlaying) {
				LOG_INF("Gesture detected: %s index is: %d prob: %2f anomaly: %f", ei_wrapper_get_classifier_label(i), i, (double)values[i], (double)anomaly);
				bool c;
            	ei_wrapper_clear_data(&c); 
				k_msgq_put(&sysfb_msgq, &tx_gest, K_NO_WAIT);
			}
			idlefollowup = 0;
		}

		if (i == 2) {
			if (values[i] >= 0.99f)
			{
				idlefollowup += 1;
				//printk("idlefollowup increased to %d\n", idlefollowup);
			}
			else
			{
				idlefollowup = 0;
				//printk("idlefollowup voided se to 0\n");
			}
		}
	}

	if(master_conn != NULL) {
		len += snprintf(buf + len, sizeof(buf) - len, "Anomaly: %.2f", (double)anomaly);
        uint8_t packet[210];
        packet[0] = MT_DEBUG_DATA;
        memcpy(&packet[1], buf, len);
        bt_nus_send(master_conn, packet, len + 1);
	}

	err = ei_wrapper_start_prediction(1, 0);

	if (RESULTS_PRINT) {
		if (err) {
			LOG_INF("Cannot restart prediction (err: %d)", err);
		}
		else {
			LOG_INF("Prediction restarted...");
		}
	}
}
#else
static void result_ready_cb(int err)
{
	if (err)
	{
		printk("Result error: %d\n", err);
		return;
	}

	/* 1. Vynuluj řádek, který právě přepisujeme
	   a odečti jeho příspěvek ze součtového pole */
	for (int c = 0; c < NUM_CLASSES; c++)
	{
		labels_value[c] -= label_history[hystor_index][c];
		label_history[hystor_index][c] = 0.0f;
	}

	/* 2. Získej všechny výsledky pro aktuální okno */
	const char *dummy_label; /* řetězec ignorujeme */
	float prob;
	size_t lbl_idx; /* ← tady dostaneme číselný index!   */

	while (!ei_wrapper_get_next_classification_result(&dummy_label, &prob, &lbl_idx))
	{
		if (lbl_idx < NUM_CLASSES)
		{
			label_history[hystor_index][lbl_idx] = prob;
			labels_value[lbl_idx] += prob; /* inkrementální součet */
		}
	}

	/* 3. Posuň index v kruhovém bufferu */
	hystor_index = (hystor_index + 1) % HISTORY_LEN;

	/* 4. Najdi třídu s největším součtem pravděpodobností */
	int max_i = 0;
	float max_v = labels_value[0];
	for (int c = 1; c < NUM_CLASSES; c++)
	{
		if (labels_value[c] > max_v)
		{
			max_v = labels_value[c];
			max_i = c;
		}
	}

	/* 5. Přepočítej na „počty hlasů“ (≈ jestli průměr > 0.5) */
	int votes = 0;
	for (int k = 0; k < HISTORY_LEN; k++)
	{
		if (label_history[k][max_i] > 0.55f)
		{ /* práh pro „hlas“ */
			votes++;
		}
	}

	if (max_i == 0)
	{
		++idlefollowup;
		// printk("Increasing folllowup to %d\n", idlefollowup);
	}
	else
	{
		idlefollowup = 0;
	}

	/* 6. Vyhlášení gesta – ignorujeme 'straight' */
	if (votes >= MIN_DETECTION_COUNT && max_i != STRAIGHT_IDX && max_i != 3 && ((max_i != 0) || (idlefollowup == 230)))
	{
		struct audio_ev g = {
			.cmd = PKT_GEST,
			.gesture_id = max_i, /* posíláme číslo labelu */
			.pipeprefix = DEVICE_PREFIX};

		if (!audioPlaying) // maby there is a need of tracking state of audiostream
		{
			printk("Gesture detected: %s\n", ei_wrapper_get_classifier_label(max_i));
			k_msgq_put(&aud_event_msgq, &g, K_NO_WAIT);
		}

		/* vyčisti buffer, ať se gesto nehlásí znovu hned za chvíli */
		memset(label_history, 0, sizeof(label_history));
		memset(labels_value, 0, sizeof(labels_value));
		hystor_index = 0;
		idlefollowup = 0;
	}

	/* 7. Spusť další sliding-window inferenci */
	ei_wrapper_start_prediction(0, 9);
}
#endif