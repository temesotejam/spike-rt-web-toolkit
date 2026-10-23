#include <kernel.h>                // RTOS（リアルタイムOS）の基本機能
#include <stdlib.h>                // exit() を使うため
#include <t_syslog.h>              // シリアルモニタにメッセージを出す
#include <CS_S3.h>
#include "spike/pup/colorsensor.h" // カラーセンサーを使う
#include "spike/hub/speaker.h"     // スピーカーを使う

// ──────────────────────────────
// Main関数（RTOSが最初に実行する関数）
// ──────────────────────────────
void Main(intptr_t exinf)
{
    // 起動メッセージをシリアルモニタに表示
    syslog(LOG_NOTICE, "Program started.");

    // Dポートに接続されたカラーセンサーを取得
    pup_device_t *ColorSensor = pup_color_sensor_get_device(PBIO_PORT_ID_D);

    // SOUND_S3と同じきらきら星（音の長さはミリ秒）
    const int32_t quarter = 300;
    const int32_t half = 600;

    // 次に鳴らす音の番号を記録する（0～41）。
    // 暗くなっても番号を消さないので、続きを再生できる。
    int nextNote = 0;

    // 課題用に音量を5%に抑える。
    hub_speaker_set_volume(5);

    while (1)
    {
        // 1音が終わるたびに環境光を確認する。
        // 5以下になったら音を止め、その場で明るくなるまで待つ。
        if (pup_color_sensor_ambient(ColorSensor) <= 5) {
            hub_speaker_stop();
            while (pup_color_sensor_ambient(ColorSensor) <= 5) {
                dly_tsk(100000);  // 5以下の間は0.1秒ごとに確認
            }
        }

        // 6以上に戻ったらnextNoteをそのまま使い、次の音から再開。
        switch (nextNote) {
            // きらきら星：ド・ド・ソ・ソ・ラ・ラ・ソ
            case 0: hub_speaker_play_tone(NOTE_C4, quarter); break;
            case 1: hub_speaker_play_tone(NOTE_C4, quarter); break;
            case 2: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 3: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 4: hub_speaker_play_tone(NOTE_A4, quarter); break;
            case 5: hub_speaker_play_tone(NOTE_A4, quarter); break;
            case 6: hub_speaker_play_tone(NOTE_G4, half); break;

            // ファ・ファ・ミ・ミ・レ・レ・ド
            case 7: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 8: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 9: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 10: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 11: hub_speaker_play_tone(NOTE_D4, quarter); break;
            case 12: hub_speaker_play_tone(NOTE_D4, quarter); break;
            case 13: hub_speaker_play_tone(NOTE_C4, half); break;

            // ソ・ソ・ファ・ファ・ミ・ミ・レ（1回目）
            case 14: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 15: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 16: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 17: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 18: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 19: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 20: hub_speaker_play_tone(NOTE_D4, half); break;

            // ソ・ソ・ファ・ファ・ミ・ミ・レ（2回目）
            case 21: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 22: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 23: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 24: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 25: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 26: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 27: hub_speaker_play_tone(NOTE_D4, half); break;

            // 最初のメロディをもう一度：ド・ド・ソ・ソ・ラ・ラ・ソ
            case 28: hub_speaker_play_tone(NOTE_C4, quarter); break;
            case 29: hub_speaker_play_tone(NOTE_C4, quarter); break;
            case 30: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 31: hub_speaker_play_tone(NOTE_G4, quarter); break;
            case 32: hub_speaker_play_tone(NOTE_A4, quarter); break;
            case 33: hub_speaker_play_tone(NOTE_A4, quarter); break;
            case 34: hub_speaker_play_tone(NOTE_G4, half); break;

            // ファ・ファ・ミ・ミ・レ・レ・ド
            case 35: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 36: hub_speaker_play_tone(NOTE_F4, quarter); break;
            case 37: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 38: hub_speaker_play_tone(NOTE_E4, quarter); break;
            case 39: hub_speaker_play_tone(NOTE_D4, quarter); break;
            case 40: hub_speaker_play_tone(NOTE_D4, quarter); break;
            case 41: hub_speaker_play_tone(NOTE_C4, half); break;
        }

        // 次の音に進む。曲の最後まで演奏した場合だけ先頭へ戻る。
        nextNote++;
        if (nextNote >= 42) {
            nextNote = 0;
        }
    }
}
