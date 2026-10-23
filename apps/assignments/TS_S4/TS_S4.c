#include <kernel.h>                  // RTOSの基本機能
#include <stdlib.h>                  // exit()を使うため
#include <t_syslog.h>                // シリアルモニタへのログ出力
#include <TS_S4.h>
#include "spike/hub/display.h"       // Hubの5×5 LED表示
#include "spike/hub/speaker.h"       // メロディの演奏
#include "spike/pup/forcesensor.h"   // フォースセンサ

// レポート課題：フォースセンサを押すと「Pressed!」を表示し、
// スター・ウォーズのメインテーマ冒頭を演奏して終了する。
void play_sound(void)
{
    // 音の長さはミリ秒。音階・長さを変えると別のメロディも作れる。
    const int32_t short_note = 180;
    const int32_t medium_note = 360;
    const int32_t long_note = 600;

    // スター・ウォーズのメインテーマ冒頭（短いアレンジ）
    hub_speaker_play_tone(NOTE_G4, short_note);
    hub_speaker_play_tone(NOTE_G4, short_note);
    hub_speaker_play_tone(NOTE_G4, short_note);

    hub_speaker_play_tone(NOTE_C5, long_note);
    hub_speaker_play_tone(NOTE_G5, long_note);
    hub_speaker_play_tone(NOTE_F5, short_note);
    hub_speaker_play_tone(NOTE_E5, short_note);
    hub_speaker_play_tone(NOTE_D5, short_note);

    hub_speaker_play_tone(NOTE_C6, long_note);
    hub_speaker_play_tone(NOTE_G5, medium_note);
    hub_speaker_play_tone(NOTE_F5, short_note);
    hub_speaker_play_tone(NOTE_E5, short_note);
    hub_speaker_play_tone(NOTE_D5, short_note);

    hub_speaker_play_tone(NOTE_C6, long_note);
    hub_speaker_play_tone(NOTE_G5, medium_note);
    hub_speaker_play_tone(NOTE_F5, short_note);
    hub_speaker_play_tone(NOTE_E5, short_note);
    hub_speaker_play_tone(NOTE_F5, short_note);
    hub_speaker_play_tone(NOTE_D5, long_note);
}

void Main(intptr_t exinf)
{
    syslog(LOG_NOTICE, "TS_S4 started.");

    // TS_S1と同じく、フォースセンサはDポートに接続する。
    pup_device_t *ForceSensor = pup_force_sensor_get_device(PBIO_PORT_ID_D);
    if (ForceSensor == NULL) {
        syslog(LOG_ERROR, "Force sensor is not connected to port D.");
        exit(1);
    }

    // センサが押されるまで、0.1秒ごとに状態を確認する。
    while (!pup_force_sensor_touched(ForceSensor)) {
        dly_tsk(100000);
    }

    syslog(LOG_NOTICE, "Force sensor pressed.");

    // 文字を順番に表示。2つの引数は点灯・消灯時間（ミリ秒）。
    hub_display_text("Pressed!", 200, 100);

    // 表示が終わったら、音楽を再生して終了する。
    hub_speaker_set_volume(70);
    play_sound();

    syslog(LOG_NOTICE, "TS_S4 finished.");
    exit(0);
}
