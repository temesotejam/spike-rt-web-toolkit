#include <kernel.h>                  // RTOSの基本機能
#include <stdbool.h>                // bool型
#include <stdint.h>                 // uint16_t型
#include <stdlib.h>                 // exit()
#include <t_syslog.h>               // ログ出力
#include <TS_S4.h>
#include "spike/hub/display.h"      // Hubの5×5 LED表示
#include "spike/hub/speaker.h"      // スピーカー
#include "spike/pup/forcesensor.h"  // フォースセンサ

// 1つのタスクだけで、演奏・表示・ボタン確認を順番に短時間ずつ実行する。
#define CHECK_INTERVAL_MS 10
#define CHAR_ON_MS        200
#define CHAR_OFF_MS       100

typedef struct {
    uint16_t note;         // 音階（周波数）
    uint16_t duration_ms;  // 音の長さ（ミリ秒）
} melody_note_t;

// スター・ウォーズのメインテーマ冒頭（短いアレンジ）。
// 音階や時間を編集すると、別のメロディにもできる。
static const melody_note_t melody[] = {
    { NOTE_G4, 180 }, { NOTE_G4, 180 }, { NOTE_G4, 180 },
    { NOTE_C5, 600 }, { NOTE_G5, 600 },
    { NOTE_F5, 180 }, { NOTE_E5, 180 }, { NOTE_D5, 180 },
    { NOTE_C6, 600 }, { NOTE_G5, 360 },
    { NOTE_F5, 180 }, { NOTE_E5, 180 }, { NOTE_D5, 180 },
    { NOTE_C6, 600 }, { NOTE_G5, 360 },
    { NOTE_F5, 180 }, { NOTE_E5, 180 }, { NOTE_F5, 180 },
    { NOTE_D5, 600 }
};

// 文字表示はhub_display_text()を使わず、1文字ずつ更新する。
// hub_display_text()は表示完了まで戻らず、ボタンを確認できないため。
static void update_pressed_display(unsigned int *elapsed_ms, unsigned int *index)
{
    const char message[] = "Pressed!";
    const unsigned int length = sizeof(message) - 1;

    if (*index >= length) return;

    *elapsed_ms += CHECK_INTERVAL_MS;

    if (*elapsed_ms == CHAR_ON_MS) {
        hub_display_off();
    }

    if (*elapsed_ms >= CHAR_ON_MS + CHAR_OFF_MS) {
        *elapsed_ms = 0;
        (*index)++;
        if (*index < length) {
            hub_display_char(message[*index]);
        }
    }
}

// true: 再生中の2回目の押下で中断、false: 最後まで演奏した。
static bool play_sound(pup_device_t *force_sensor)
{
    unsigned int display_elapsed_ms = 0;
    unsigned int display_index = 0;
    bool released_after_start = false;

    // 最初の押下で直ちに文字表示と演奏を開始する。
    hub_display_char('P');

    for (unsigned int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
        // 再生時間を指定しないので、途中でもセンサを確認できる。
        // 次の音をそのまま開始し、音と音の間で明示的には停止しない。
        hub_speaker_play_tone(melody[i].note, SOUND_MANUAL_STOP);

        for (unsigned int elapsed = 0; elapsed < melody[i].duration_ms;
             elapsed += CHECK_INTERVAL_MS) {

            // 初回に押したままでも、2回目の押下と誤認しない。
            // 一度離した後、もう一度押した場合だけ停止する。
            bool pressed = pup_force_sensor_touched(force_sensor);
            if (!pressed) {
                released_after_start = true;
            } else if (released_after_start) {
                hub_speaker_stop();
                hub_display_off();
                return true;
            }

            // 10msの待機中も、スピーカーはハードウェア側で鳴り続ける。
            dly_tsk(CHECK_INTERVAL_MS * 1000);
            update_pressed_display(&display_elapsed_ms, &display_index);
        }
    }

    hub_speaker_stop();
    hub_display_off();
    return false;
}

void Main(intptr_t exinf)
{
    syslog(LOG_NOTICE, "TS_S4 started.");

    // TS_S1と同じく、フォースセンサはDポートに接続する。
    pup_device_t *force_sensor = pup_force_sensor_get_device(PBIO_PORT_ID_D);
    if (force_sensor == NULL) {
        syslog(LOG_ERROR, "Force sensor is not connected to port D.");
        exit(1);
    }

    hub_speaker_set_volume(70);

    // 1回目の押下を10ms間隔で確認する。
    while (!pup_force_sensor_touched(force_sensor)) {
        dly_tsk(CHECK_INTERVAL_MS * 1000);
    }

    syslog(LOG_NOTICE, "Force sensor pressed. Starting playback.");

    // マルチタスクや割り込み処理を追加せず、単一タスクで動作する。
    bool cancelled = play_sound(force_sensor);

    if (cancelled) {
        syslog(LOG_NOTICE, "Playback stopped by second press.");
    } else {
        syslog(LOG_NOTICE, "Playback completed.");
    }
    exit(0);
}
