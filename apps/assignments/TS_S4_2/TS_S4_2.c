#include <kernel.h>                // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                // exit() を使うため
#include <t_syslog.h>              // シリアルモニタにメッセージを出す
#include <TS_S4_2.h>
#include "spike/hub/display.h"     // ハブのLEDマトリクス表示
#include "spike/pup/forcesensor.h" // フォースセンサー（押す強さを検出）
#include "spike/hub/speaker.h"   // 音楽を鳴らす

// 音程と長さを順番に並べる（単位: ms）。
// TS_S4と同じ100 BPMの短いスター・ウォーズのメロディ。
static const struct {
    uint16_t note;
    int duration_ms;
} melody[] = {
    { NOTE_G4, 200 },
    { NOTE_G4, 200 },
    { NOTE_G4, 200 },
    { NOTE_C5, 1200 },
    { NOTE_G5, 1200 },
    { NOTE_F5, 200 },
    { NOTE_E5, 200 },
    { NOTE_D5, 200 },
    { NOTE_C6, 1200 },
    { NOTE_G5, 600 },
    { NOTE_F5, 200 },
    { NOTE_E5, 200 },
    { NOTE_D5, 200 },
    { NOTE_C6, 1200 },
    { NOTE_G5, 600 },
    { NOTE_F5, 200 },
    { NOTE_E5, 200 },
    { NOTE_F5, 200 },
    { NOTE_D5, 1200 },
};


// ──────────────────────────────
// Main関数（RTOSが最初に実行する関数）
// ──────────────────────────────
void Main(intptr_t exinf)
{
    // 起動時にメッセージをシリアルモニタに出力（デバッグ用）
    syslog(LOG_NOTICE, "Program started.");

    // ──────────────────────────────
    // フォースセンサーを使うための変数を宣言
    // pup_device_t は「センサーやモータなどの装置（デバイス）」を表す型です。
    // *（アスタリスク）は「ポインタ」と呼ばれ、実際のセンサーを直接持たず、
    // その“場所（アドレス）”を指す。    
    // つまり、ForceSensor は「Dポートに接続されたフォースセンサーを操作するためのリモコン」
    // のようなもの。
    // ──────────────────────────────
    pup_device_t *ForceSensor;

    // Dポートに接続されたフォースセンサーを取得
    ForceSensor = pup_force_sensor_get_device(PBIO_PORT_ID_D);

    // ──────────────────────────────
    // センサーが押されるまで待機するループ
    // pup_force_sensor_touched() は「センサーが押されたら true（1）」を返す。
    // 押されていない間は 0（false）なので、while内で待ち続ける。
    // ──────────────────────────────
    while (!pup_force_sensor_touched(ForceSensor)) {
        dly_tsk(100000);  // 0.1秒待つ（CPUを休ませる）
    }

    // 1回目の押下で再生開始。課題用の音量は5%。
    hub_display_off();
    hub_speaker_set_volume(5);

    // 最初に押していたボタンを一度離すまで、停止操作を受け付けない。
    bool released = false;

    // メロディを繰り返し再生する。
    while (1) {
        for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
            // 同じ音が連続する場合だけ短く区切る。
            if (i > 0 && melody[i].note == melody[i - 1].note) {
                hub_speaker_stop();
                dly_tsk(20000);
            }

            // 音を鳴らしたまま次の処理へ進む。
            hub_speaker_play_tone(melody[i].note, SOUND_MANUAL_STOP);

            // 50msごとにセンサーを確認する（マルチタスクなし）。
            for (int ms = 0; ms < melody[i].duration_ms; ms += 50) {
                if (!pup_force_sensor_touched(ForceSensor)) {
                    released = true;
                } else if (released) {
                    // 2回目の押下で、曲の途中でもすぐに停止する。
                    hub_speaker_stop();
                    hub_display_text("Pressed!", 200, 100);
                    exit(0);
                }
                dly_tsk(50000);
            }
        }
    }
}
