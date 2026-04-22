#include "ei_imu_modul.h"
#include "app_state.h"
#include "toy_utils.h"
#include "math.h"
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
#define SAMPLING_FREQ 52
//#define DEBUG_RESULT_CB   comment this out to use debug result_ready
#define SEND_DEBUG_DATA 0 //set this to 0 for not sending debug data to app
#define RESULTS_PRINT 0

#define IDLEFOLLOWUP_TREASHOLD 60

uint8_t get_mapped_gesture(int original_index);

typedef enum {
    MY_GEST_CIRCLE = 0,
    MY_GEST_DROP   = 1,
    MY_GEST_IDLE   = 2,
    MY_GEST_ROLL   = 4,
    MY_GEST_SHAKE  = 5,
    MY_GEST_TAP    = 6,
    MY_GEST_TOS    = 7,
    MY_GEST_NOISE  = 3
} my_gesture_t;

struct ei_ev{
	float values[8];
	float anomaly;
};

uint8_t by_game_tresholds[GAME_MODE_CNT][8] = {
	[GAME_MODE_DEFAULT] = {2, 1, IDLEFOLLOWUP_TREASHOLD, 255, 1, 2, 2, 1},
	[GAME_MODE_CHAINS] = {2, 1, IDLEFOLLOWUP_TREASHOLD / 2, 255, 1, 2, 2, 1},
	[GAME_MODE_SWAPPED] = {2, 1, IDLEFOLLOWUP_TREASHOLD, 255, 1, 2, 2, 1},
	[GAME_MODE_MIRROR] = {2, 1, IDLEFOLLOWUP_TREASHOLD, 255, 1, 2, 2, 1},
};

static void result_ready_cb(int err);

void resolve_classification(float values[8], float anomaly);

#define CLASS_THREAD_STACK_SIZE 2024
#define CLASS_THREAD_PRIORITY 6
K_THREAD_STACK_DEFINE(class_stack_area, CLASS_THREAD_STACK_SIZE);
static struct k_thread class_thread_data;

K_MSGQ_DEFINE(class_msgq, sizeof(struct ei_ev), 16, 4);

void class_processing_thread(void *p1, void *p2, void *p3);

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

	k_thread_create(&class_thread_data, class_stack_area,
					K_THREAD_STACK_SIZEOF(class_stack_area),
					class_processing_thread,
					NULL, NULL, NULL,
					CLASS_THREAD_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

void lsm6dsl_sleep() {
	int err;
    const struct device *lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

    sensor_trigger_set(lsm6dsl_dev, NULL, NULL);

    struct sensor_value off_odr = {0, 0};
    sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY, &off_odr);

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
	err = ei_wrapper_start_prediction(0, 13);
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
	float Am_Gm[2];
	struct sensor_value acc_gyro_buffer[6];
	while (true) {
		//k_sem_take(&run_sem, K_FOREVER); // pokud je OFF, vlákno se zastaví
		//k_sem_give(&run_sem);

		k_msgq_get(&imu_msgq, acc_gyro_buffer, K_FOREVER);
		if (k_msgq_num_used_get(&imu_msgq) > (200 * 3 / 4)) {
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
			Am_Gm[0] = sqrt( f_val_buffer[0]*f_val_buffer[0] + f_val_buffer[1]*f_val_buffer[1] + f_val_buffer[2]*f_val_buffer[2]);
			Am_Gm[1] = sqrt( f_val_buffer[3]*f_val_buffer[3] + f_val_buffer[4]*f_val_buffer[4] + f_val_buffer[5]*f_val_buffer[5]);
			err = ei_wrapper_add_data(Am_Gm, 2);
			if (err) {
				//LOG_INF("Cannot provide input data (err: %d)\n", err);
				//LOG_ERR("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
			}
		} else {
		}
	}
}

#ifdef DEBUG_RESULT_CB
static void result_ready_cb(int err) {
	LOG_INF("Resutl ready err: %d",err);
}
#else
static void result_ready_cb(int err) {
	if (err) {
		LOG_INF("Result ready callback returned error (err: %d)\n", err);
		return;
	}

	struct ei_ev tx_class = {
		.anomaly = 0.0f,
		.values = {0}
	};
	const char *label;
	float temp_value;
	int idx;

	if (RESULTS_PRINT) {
		LOG_INF("Classification results");
		LOG_INF("======================");
	}
	while (true) {    
		err = ei_wrapper_get_next_classification_result(&label, &temp_value, &idx);
		if (err)
		{
			if (err == -ENOENT)
			{
				err = 0;
			}
			break;
		}

		tx_class.values[idx] = temp_value;

		if (RESULTS_PRINT) {
			LOG_INF("Value: %.2f\tLabel: %s", (double)temp_value, label);
		}
	}

	if (err) {
		LOG_INF("Cannot get classification results (err: %d)", err);
	}else {
		if (ei_wrapper_classifier_has_anomaly()) {
			err = ei_wrapper_get_anomaly(&tx_class.anomaly);
			if (err) {
				LOG_INF("Cannot get anomaly (err: %d)", err);
			} else if (RESULTS_PRINT) {
				LOG_INF("Anomaly: %.2f", (double)tx_class.anomaly);
			}
		}
	}

	k_msgq_put(&class_msgq, &tx_class, K_NO_WAIT);

	err = ei_wrapper_start_prediction(0, 13);

	if (RESULTS_PRINT) {
		if (err) {
			LOG_INF("Cannot restart prediction (err: %d)", err);
		}
		else {
			LOG_INF("Prediction restarted...");
		}
	}
}
#endif

void class_processing_thread(void *p1, void *p2, void *p3) {
	int err;
	LOG_INF("Starting class processing thread");
	struct ei_ev rx_class;

	while (true) {
		k_msgq_get(&class_msgq, &rx_class, K_FOREVER);

		resolve_classification(rx_class.values, rx_class.anomaly);
	}
	
}

void resolve_classification(float values[8], float anomaly) {
	char buf[240];
	int len = 0;
	int max_res = ei_wrapper_get_classifier_label_count();
	int err;
	static uint8_t counter[8] = {0};
	const uint8_t *tresholds = by_game_tresholds[current_game];
	static float prob_val[8] = {0.95, 0.95, 0.99, 0.95, 0.95, 0.95, 0.95, 0.95};

	for (size_t i = 0; i < max_res; i++) {
		if((master_conn != NULL) && SEND_DEBUG_DATA ){
		len += snprintf(buf + len, sizeof(buf) - len, 
                            "Value: %.2f\tLabel: %s\n", 
                            (double)values[i], ei_wrapper_get_classifier_label(i));
		}
		if (i == 3) continue;

		if (values[i] >= prob_val[i]) {
			counter[i]++;
		} else {
			counter[i] = 0;
		}
	}

	for (size_t i = 0; i < max_res; i++) {
		if (i == 3) continue;
		if (counter[i] == tresholds[i]) {
			uint8_t i_maped = get_mapped_gesture(i);
			struct toy_events tx_gest = {
				.type = MT_GEST,
				.payload.gest.type = i_maped,
				.payload.gest.prob = (uint8_t)(values[i] * 100.0f),
				.payload.gest.anomaly = (uint16_t)(anomaly *anomaly * 100.0f),
				.payload.gest.sign = 0
			};
			bool canceled;
			if (current_game == GAME_MODE_SWAPPED || current_game == GAME_MODE_MIRROR) {
				struct toy_events tx_game = {
					.type = MT_GEST,
					.len = sizeof(struct game_ev),
					.payload.game.gest_id = i_maped,
					.payload.game.mode = current_game,
					.payload.game.prob = (uint8_t)(values[i] * 100.0f),
				};

				bt_nus_send(remote_conn, &tx_game, sizeof(tx_game));
				ei_wrapper_clear_data(&canceled);
				ei_wrapper_start_prediction(0, 1);
			}else if (!audioPlaying) {
				LOG_INF("Gesture detected: %s index is: %d prob: %2f anomaly: %f", ei_wrapper_get_classifier_label(tx_gest.payload.gest.type), tx_gest.payload.gest.type, (double)values[i], (double)anomaly);
				k_msgq_put(&sysfb_msgq, &tx_gest, K_NO_WAIT);
				ei_wrapper_clear_data(&canceled);
				ei_wrapper_start_prediction(0, 1);
			}
			memset(&counter, 0, sizeof(counter));
			break;
		}
	}
	
	if(master_conn != NULL && SEND_DEBUG_DATA) {
		len += snprintf(buf + len, sizeof(buf) - len, "Anomaly: %.2f", (double)anomaly);
        uint8_t packet[240];
        packet[0] = MT_DEBUG_DATA;
        memcpy(&packet[1], buf, len);
        bt_nus_send(master_conn, packet, len + 1);
	}
}

uint8_t get_mapped_gesture(int original_index) {
    switch(original_index) {
        case 3: return 7;
        case 7: return 3;
        default: return original_index;
    }
}