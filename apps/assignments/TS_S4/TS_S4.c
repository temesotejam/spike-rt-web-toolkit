#include <kernel.h>                  // RTOSの基本機能
#include <stdbool.h>                // bool型
#include <stdint.h>                 // uint16_t型
#include <stdlib.h>                 // exit()
#include <t_syslog.h>               // ログ出力
#include <TS_S4.h>
#include "spike/hub/display.h"      // Hubの5×5 LED表示
#include "spike/hub/speaker.h"      // スピーカー
#include "spike/pup/forcesensor.h"  // フォースセンサ

// マルチタスクを使わずに、曲を鳴らしながら10ms間隔で押下を確認する。
#define CHECK_INTERVAL_MS 10

// メインテーマの基本テンポ：約100 BPM（4分音符 = 600ms）。
// 3連符 = 1拍を3等分、2分音符 = 2拍。
// リズムを変える場合は TEMPO_BPM を調整する。
#define TEMPO_BPM        100
#define QUARTER_MS       (60000 / TEMPO_BPM)
#define TRIPLET_MS       (QUARTER_MS / 3)
#define HALF_MS          (QUARTER_MS * 2)

// 同じ音が連続する箇所だけ短く切り、各音が区別できるようにする。
#define SAME_NOTE_GAP_MS 30

typedef struct {
    uint16_t note;         // 音階（周波数）
    uint16_t duration_ms;  // 音の長さ（ミリ秒）
} melody_note_t;

// スター・ウォーズのメインテーマ冒頭（短いアレンジ）。
// 3連符→長い2分音符→下降3連符、というリズムに調整。
// 音階の順番は以前のまま（音程は変更しない）。
static const melody_note_t melody[] = {
    { NOTE_G4, TRIPLET_MS }, { NOTE_G4, TRIPLET_MS }, { NOTE_G4, TRIPLET_MS },
    { NOTE_C5, HALF_MS },    { NOTE_G5, HALF_MS },
    { NOTE_F5, TRIPLET_MS }, { NOTE_E5, TRIPLET_MS }, { NOTE_D5, TRIPLET_MS },
    { NOTE_C6, HALF_MS },    { NOTE_G5, QUARTER_MS },
    { NOTE_F5, TRIPLET_MS }, { NOTE_E5, TRIPLET_MS }, { NOTE_D5, TRIPLET_MS },
    { NOTE_C6, HALF_MS },    { NOTE_G5, QUARTER_MS },
    { NOTE_F5, TRIPLET_MS }, { NOTE_E5, TRIPLET_MS }, { NOTE_F5, TRIPLET_MS },
    { NOTE_D5, HALF_MS }
};

// 再生は2回目の押下があるまで繰り返す（通常は戻らない）。
static void play_sound(pup_device_t *force_sensor)
{
    bool released_after_start = false;
    unsigned int release_ticks = 0;
    unsigned int press_ticks = 0;

    // 曲が終わったら止めずに先頭から繰り返す。
    // 再生を終える条件は2回目の押下だけ。
    while (1) {
      for (unsigned int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
        // 非ブロッキング再生なので、長い音の途中でもセンサを確認できる。
        // 異なる音はつなげ、同じ音が続くときだけ30msの区切りを入れる。
        hub_speaker_play_tone(melody[i].note, SOUND_MANUAL_STOP);

        for (unsigned int elapsed = 0; elapsed < melody[i].duration_ms;
             elapsed += CHECK_INTERVAL_MS) {

            // 同音が続く場合、次の音の直前だけ短い無音を作る。
            // この無音の間もセンサ確認は続ける。
            const unsigned int note_count = sizeof(melody) / sizeof(melody[0]);
            if (i + 1 < note_count && melody[i].note == melody[i + 1].note &&
                elapsed == melody[i].duration_ms - SAME_NOTE_GAP_MS) {
                hub_speaker_stop();
            }

            // 初回に押したままでも、2回目の押下と誤認しない。
            // 一度離した後、もう一度押した場合だけ停止する。
            bool pressed = pup_force_sensor_touched(force_sensor);
            if (!released_after_start) {
                // 最初の押下を離したことを30ms連続で確認。
                // 接点のチャタリングで誤って2回目と判定しないため。
                release_ticks = pressed ? 0 : release_ticks + 1;
                if (release_ticks >= 3) {
                    released_after_start = true;
                }
            } else {
                // 離した後の押下を20ms連続で確認したら停止。
                press_ticks = pressed ? press_ticks + 1 : 0;
                if (press_ticks >= 2) {
                    hub_speaker_stop();
                    return;
                }
            }

            // 音を鳴らしたまま待機し、10msごとにボタンを確認する。
            dly_tsk(CHECK_INTERVAL_MS * 1000);
        }
      }
    }
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

    // 演奏中はLCDを表示しない。
    hub_display_off();
    // 課題用に音量を5%に抑える。
    hub_speaker_set_volume(5);

    // 1回目の押下を10ms間隔で確認する。
    while (!pup_force_sensor_touched(force_sensor)) {
        dly_tsk(CHECK_INTERVAL_MS * 1000);
    }

    syslog(LOG_NOTICE, "Force sensor pressed. Starting playback.");

    // マルチタスクや割り込み処理を追加せず、単一タスクで動作する。
    play_sound(force_sensor);
    syslog(LOG_NOTICE, "Playback stopped by second press.");

    // 2回目の押下で音を停止した後にだけ「Pressed!」を表示する。
    // hub_display_text()は表示終了まで待機するが、既に音を止めているため問題ない。
    hub_display_text("Pressed!", 200, 100);
    exit(0);
}
