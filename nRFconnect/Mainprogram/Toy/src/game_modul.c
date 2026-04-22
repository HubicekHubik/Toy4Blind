#include "game_modul.h"
#include "app_state.h"
#include "toy_utils.h"
#include <zephyr/kernel.h>

#include "audio_modul.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(game_modul, CONFIG_LOG_DEFAULT_LEVEL);

#define GAME_EVENT_MSGQ_LENGTH 16
#define GEST_COUNT 7 

K_MSGQ_DEFINE(game_ev_msgq, sizeof(struct toy_events), GAME_EVENT_MSGQ_LENGTH, 4);

#define GAME_THREAD_PRIORITY 13
#define GAME_THREAD_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(game_stack_area, GAME_THREAD_STACK_SIZE);
static struct k_thread game_thread_data;

static void game_thread(void *arg1, void *arg2, void *arg3);

int game_modul_init() {
    k_thread_create(&game_thread_data, game_stack_area,
                    K_THREAD_STACK_SIZEOF(game_stack_area),
                    game_thread,
                    NULL, NULL, NULL,
                    GAME_THREAD_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

static void game_thread(void *arg1, void *arg2, void *arg3) {
    LOG_INF("Game thread started");
    struct toy_events rx_event;
    struct toy_events tx_audio;
    uint8_t chain[CHAIN_TRESHOLD];
    memset(chain, 255, sizeof(chain));
    int chain_ocupied = 0;
    bool gest_detected[GEST_COUNT];
    memset(gest_detected, false, GEST_COUNT);

    while (true) {
        if(k_msgq_get(&game_ev_msgq, &rx_event, K_FOREVER) == 0){
            if (rx_event.type != MT_GEST) continue;
            switch (current_game) {
                case GAME_MODE_CHAINS:
                    if (gest_detected[rx_event.payload.gest.type]) {
                        LOG_INF("Deteced same gest as before try another one");
                    } else {
                        chain[chain_ocupied] = (char)rx_event.payload.gest.type;
                        gest_detected[rx_event.payload.gest.type] = true;
                        tx_audio.type = MT_GEST;
                        tx_audio.len = sizeof(struct audio_ev);
                        tx_audio.payload.audio.id = rx_event.payload.gest.type;
                        k_msgq_put(&aud_event_msgq, &tx_audio, K_NO_WAIT);
                        chain_ocupied++;
                    }
                    if (chain_ocupied == CHAIN_TRESHOLD) {
                        tx_audio.type = MT_CHAIN;
                        memcpy(&tx_audio.payload.audio.chain, &chain, sizeof(chain));
                        k_msgq_put(&aud_event_msgq, &tx_audio, K_NO_WAIT);
                        chain_ocupied = 0;
                        memset(chain, 255, sizeof(chain));
                        memset(gest_detected, false, GEST_COUNT);
                    }
                    break;
                
                default:
                    break;
            }
        }
    }
    
}