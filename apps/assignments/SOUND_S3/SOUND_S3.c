#include <kernel.h>              // RTOSの基本機能
#include <stdlib.h>              // exit()を使うため
#include <t_syslog.h>            // ログ出力
#include <SOUND_S3.h>
#include "spike/hub/display.h"   // ハブの画面表示
#include "spike/hub/speaker.h"   // スピーカー制御
#include "spike/hub/battery.h"   // バッテリー情報

// レポート問題1：SOUND_S2を改造して好きな曲のメロディを鳴らす
// 例として「きらきら星」を演奏する。音階と長さを変えて好きな曲にしよう。
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

void Main(intptr_t exinf)
{
    syslog(LOG_NOTICE, "SOUND_S3 started.");

    // 教室で複数台が鳴っても大きくならないよう、音量を5%に抑える。
    hub_speaker_set_volume(5);

    // ハブの画面には開始時の目印として「3」を表示する。
    hub_display_number(3);

    play_sound();

    syslog(LOG_NOTICE, "SOUND_S3 finished.");
    exit(0);
}
