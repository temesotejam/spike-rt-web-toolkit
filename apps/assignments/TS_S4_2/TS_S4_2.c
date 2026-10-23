#include <kernel.h>                // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                // exit() を使うため
#include <stdbool.h>                // 押下状態を記録する
#include <stdint.h>                 // uint16_t型
#include <t_syslog.h>              // シリアルモニタにメッセージを出す
#include <TS_S4_2.h>
#include "spike/hub/display.h"     // ハブのLEDマトリクス表示
#include "spike/pup/forcesensor.h" // フォースセンサー（押す強さを検出）
#include "spike/hub/speaker.h"     // 音楽を鳴らす

// 1音の再生が終わったら、ボタンを確認する。
// 最初の押下を離した後の押下だけを「2回目」と判定する。
static bool check_stop(pup_device_t *sensor, bool *released)
{
    if (!pup_force_sensor_touched(sensor)) {
        *released = true;
    } else if (*released) {
        return true;
    }
    return false;
}

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

    // 1回目の押下で音楽を開始する。課題用の音量は5%。
    hub_display_off();
    hub_speaker_set_volume(5);

    // いったんボタンを離した後の押下を、終了操作とする。
    bool released = false;

    // SOUND_S2のように音を1つずつ記述し、各音の後でボタンを確認する。
    // 途中で止めなければ、曲を最初から繰り返す。
    while (1) {
        hub_speaker_play_tone(NOTE_G4, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_G4, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_G4, 200);
        if (check_stop(ForceSensor, &released)) break;

        hub_speaker_play_tone(NOTE_C5, 1200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_G5, 1200);
        if (check_stop(ForceSensor, &released)) break;

        hub_speaker_play_tone(NOTE_F5, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_E5, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_D5, 200);
        if (check_stop(ForceSensor, &released)) break;

        hub_speaker_play_tone(NOTE_C6, 1200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_G5, 600);
        if (check_stop(ForceSensor, &released)) break;

        hub_speaker_play_tone(NOTE_F5, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_E5, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_D5, 200);
        if (check_stop(ForceSensor, &released)) break;

        hub_speaker_play_tone(NOTE_C6, 1200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_G5, 600);
        if (check_stop(ForceSensor, &released)) break;

        hub_speaker_play_tone(NOTE_F5, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_E5, 200);
        if (check_stop(ForceSensor, &released)) break;
        hub_speaker_play_tone(NOTE_F5, 200);
        if (check_stop(ForceSensor, &released)) break;

        hub_speaker_play_tone(NOTE_D5, 1200);
        if (check_stop(ForceSensor, &released)) break;
    }

    // 2回目の押下を確認したときだけ、LCDに表示して終了する。
    hub_display_text("Pressed!", 200, 100);
    exit(0);
}
