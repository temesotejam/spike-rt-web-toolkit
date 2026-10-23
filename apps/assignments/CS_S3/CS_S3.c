#include <kernel.h>                // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                // exit() を使うため
#include <t_syslog.h>              // シリアルモニタにメッセージを出す
#include <CS_S3.h>
#include "spike/pup/colorsensor.h" // カラーセンサーを使う
#include "spike/hub/speaker.h"     // スピーカーを使う

// SOUND_S3と同じ「きらきら星」を演奏する。
void play_sound(void)
{
    // 音の長さ（単位：ミリ秒）。テンポを変える場合はここを変更する。
    const int32_t quarter = 300;
    const int32_t half    = 600;

    // きらきら星：ド・ド・ソ・ソ・ラ・ラ・ソ
    hub_speaker_play_tone(NOTE_C4, quarter);
    hub_speaker_play_tone(NOTE_C4, quarter);
    hub_speaker_play_tone(NOTE_G4, quarter);
    hub_speaker_play_tone(NOTE_G4, quarter);
    hub_speaker_play_tone(NOTE_A4, quarter);
    hub_speaker_play_tone(NOTE_A4, quarter);
    hub_speaker_play_tone(NOTE_G4, half);

    // ファ・ファ・ミ・ミ・レ・レ・ド
    hub_speaker_play_tone(NOTE_F4, quarter);
    hub_speaker_play_tone(NOTE_F4, quarter);
    hub_speaker_play_tone(NOTE_E4, quarter);
    hub_speaker_play_tone(NOTE_E4, quarter);
    hub_speaker_play_tone(NOTE_D4, quarter);
    hub_speaker_play_tone(NOTE_D4, quarter);
    hub_speaker_play_tone(NOTE_C4, half);

    // ソ・ソ・ファ・ファ・ミ・ミ・レ（2回）
    for (int i = 0; i < 2; i++) {
        hub_speaker_play_tone(NOTE_G4, quarter);
        hub_speaker_play_tone(NOTE_G4, quarter);
        hub_speaker_play_tone(NOTE_F4, quarter);
        hub_speaker_play_tone(NOTE_F4, quarter);
        hub_speaker_play_tone(NOTE_E4, quarter);
        hub_speaker_play_tone(NOTE_E4, quarter);
        hub_speaker_play_tone(NOTE_D4, half);
    }

    // 最初のメロディをもう一度
    hub_speaker_play_tone(NOTE_C4, quarter);
    hub_speaker_play_tone(NOTE_C4, quarter);
    hub_speaker_play_tone(NOTE_G4, quarter);
    hub_speaker_play_tone(NOTE_G4, quarter);
    hub_speaker_play_tone(NOTE_A4, quarter);
    hub_speaker_play_tone(NOTE_A4, quarter);
    hub_speaker_play_tone(NOTE_G4, half);

    hub_speaker_play_tone(NOTE_F4, quarter);
    hub_speaker_play_tone(NOTE_F4, quarter);
    hub_speaker_play_tone(NOTE_E4, quarter);
    hub_speaker_play_tone(NOTE_E4, quarter);
    hub_speaker_play_tone(NOTE_D4, quarter);
    hub_speaker_play_tone(NOTE_D4, quarter);
    hub_speaker_play_tone(NOTE_C4, half);
}

// ──────────────────────────────
// Main関数（RTOSが最初に実行する関数）
// ──────────────────────────────
void Main(intptr_t exinf)
{
    // 起動メッセージをシリアルモニタに表示
    syslog(LOG_NOTICE, "Program started.");

    // Dポートに接続されたカラーセンサーを取得
    // pup_device_t* はセンサーを操作するための変数
    pup_device_t *ColorSensor = pup_color_sensor_get_device(PBIO_PORT_ID_D);

    // ──────────────────────────────
    // 環境光が5以上になるまで待機するループ
    // ──────────────────────────────
    while (1)
    {
        // カラーセンサーの環境光の明るさを取得
        int32_t ambient = pup_color_sensor_ambient(ColorSensor);

        // シリアルモニタに表示
        syslog(LOG_NOTICE, "Ambient: %ld", ambient);

        // 環境光が5以上ならループを抜ける
        if (ambient >= 5) {
            break;
        }

        // 0.1秒待ってもう一度確認
        dly_tsk(100000);
    }

    // 環境光が5以上になったら、きらきら星を1回演奏して終了する
    hub_speaker_set_volume(5);  // 課題用に音量を小さくする
    play_sound();

    exit(0);
}
