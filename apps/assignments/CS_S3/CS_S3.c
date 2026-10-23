#include <kernel.h>                // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                // exit() を使うため
#include <t_syslog.h>              // シリアルモニタにメッセージを出す
#include <CS_S3.h>
#include "spike/pup/colorsensor.h" // カラーセンサーを使う
#include "spike/hub/speaker.h"     // スピーカーを使う

// SOUND_S3と同じ「きらきら星」を演奏する。
void play_sound(pup_device_t *ColorSensor)
{
    // 音の長さ（単位：ミリ秒）。テンポを変える場合はここを変更する。
    const int32_t quarter = 300;
    const int32_t half    = 600;

    // きらきら星：ド・ド・ソ・ソ・ラ・ラ・ソ
    hub_speaker_play_tone(NOTE_C4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_C4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_A4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_A4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, half);

    // ファ・ファ・ミ・ミ・レ・レ・ド
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_D4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_D4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_C4, half);

    // ソ・ソ・ファ・ファ・ミ・ミ・レ（1回目）
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_D4, half);

    // ソ・ソ・ファ・ファ・ミ・ミ・レ（2回目）
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_D4, half);

    // 最初のメロディをもう一度
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_C4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_C4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_A4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_A4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_G4, half);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;

    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_F4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_E4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_D4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_D4, quarter);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
    hub_speaker_play_tone(NOTE_C4, half);
    if (pup_color_sensor_ambient(ColorSensor) <= 5) return;
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

    // 教室で使うため音量を5%にする。
    hub_speaker_set_volume(5);

    // 環境光が5より大きい間は、きらきら星を繰り返し演奏する。
    // 5以下になったら現在の1音の終了後に止め、6以上で再開する。
    while (1)
    {
        int32_t ambient = pup_color_sensor_ambient(ColorSensor);
        syslog(LOG_NOTICE, "Ambient: %ld", ambient);

        if (ambient > 5) {
            play_sound(ColorSensor);
        } else {
            dly_tsk(100000);  // 暗い間は0.1秒ごとに確認
        }
    }
}
