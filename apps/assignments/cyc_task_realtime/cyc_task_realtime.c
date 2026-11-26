/*
 * cyc_task_realtime.c
 *
 * 【元のcyc_task.cからの変更点】
 * 1. 1秒周期の周期ハンドラによる表示処理ではなく、カラーセンサーを
 *    10ms周期で確認する高優先度のセンサータスクをMainにする。
 * 2. スピーカー処理をSOUND_TASKへ分離し、センサー監視を妨げない。
 * 3. SOUND_MANUAL_STOPで音を開始し、蓋が閉じたときはセンサータスクから
 *    hub_speaker_stop()を直接呼んで停止する。
 * 4. 誤検出防止のため2回連続で状態を確認する。検出待ちは20msを目安とする。
 *
 * カラーセンサーはDポートに接続し、蓋を閉じたときにセンサー面が覆われ、
 * 開いたときに周囲の光が入る向きに取り付ける。
 */
#include <kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <t_syslog.h>
#include "cyc_task_realtime.h"
#include "kernel_cfg.h"

#include "spike/pup/colorsensor.h"
#include "spike/hub/speaker.h"

#define SENSOR_INTERVAL_US       10000   /* 10msごとに状態を確認 */
#define REQUIRED_SAMPLES         2       /* 2回連続で確認してチャタリングを抑制 */
#define NOTE_CHECK_INTERVAL_MS   10
#define SPEAKER_VOLUME           5  /* 教室向けの小音量設定 */

/* 1: 環境光で判定（標準）。0: 反射光で判定する場合に切り替える。 */
#define LID_DETECT_USE_AMBIENT   1

#if LID_DETECT_USE_AMBIENT
/* 環境光の値は0～100%。実機で開閉時の値を測定して調整する。 */
#define LID_OPEN_THRESHOLD       12
#define LID_CLOSE_THRESHOLD      8
#else
/* 反射光も0～100%。蓋の内側の色・距離に合わせて実測調整する。 */
#define LID_OPEN_THRESHOLD       25
#define LID_CLOSE_THRESHOLD     18
#endif

/* Main（センサー監視タスク）とSOUND_TASKで共有する状態 */
static volatile bool g_lid_open = false;

#define TEMPO_BPM       100
#define QUARTER_MS      (60000 / TEMPO_BPM)
#define TRIPLET_MS      (QUARTER_MS / 3)
#define HALF_MS         (QUARTER_MS * 2)

typedef struct {
    uint16_t note;
    uint16_t duration_ms;
} melody_note_t;

/* TS_S4からスター・ウォーズのメロディーを再利用 */
static const melody_note_t star_wars[] = {
    { NOTE_G4, TRIPLET_MS }, { NOTE_G4, TRIPLET_MS }, { NOTE_G4, TRIPLET_MS },
    { NOTE_C5, HALF_MS },    { NOTE_G5, HALF_MS },
    { NOTE_F5, TRIPLET_MS }, { NOTE_E5, TRIPLET_MS }, { NOTE_D5, TRIPLET_MS },
    { NOTE_C6, HALF_MS },    { NOTE_G5, QUARTER_MS },
    { NOTE_F5, TRIPLET_MS }, { NOTE_E5, TRIPLET_MS }, { NOTE_D5, TRIPLET_MS },
    { NOTE_C6, HALF_MS },    { NOTE_G5, QUARTER_MS },
    { NOTE_F5, TRIPLET_MS }, { NOTE_E5, TRIPLET_MS }, { NOTE_F5, TRIPLET_MS },
    { NOTE_D5, HALF_MS }
};

/* 【追加】環境光／反射光のどちらを使うかを1か所で切り替える */
static int32_t read_lid_light(pup_device_t *sensor)
{
#if LID_DETECT_USE_AMBIENT
    return pup_color_sensor_ambient(sensor);
#else
    return pup_color_sensor_reflection(sensor);
#endif
}

/*------------------------------------------------------------
 * MAIN_TASK：蓋の開閉を最優先で監視する
 *
 * 【元サンプルからの変更】
 * 元のMainは曲の再生を続けていたが、ここではセンサー監視に専念。
 * 10msごとに読み取り、閉じた判定になったら即座に音を止める。
 *-----------------------------------------------------------*/
void Main(intptr_t exinf)
{
    pup_device_t *sensor = pup_color_sensor_get_device(PBIO_PORT_ID_D);

    if (sensor == NULL) {
        syslog(LOG_ERROR, "Color sensor not found on port D");
        ext_tsk();
    }

    hub_speaker_set_volume(SPEAKER_VOLUME);
    syslog(LOG_NOTICE, "cyc_task_realtime: sensor monitor started");

    /* 音楽タスクはセンサー初期化後に起動する */
    act_tsk(SOUND_TASK);

    int open_samples = 0;
    int close_samples = 0;

    while (1) {
        int32_t light_level = read_lid_light(sensor);

        if (!g_lid_open) {
            /* 閉状態から開状態へ：しきい値以上を2回連続で確認 */
            if (light_level >= LID_OPEN_THRESHOLD) {
                open_samples++;
            } else {
                open_samples = 0;
            }

            if (open_samples >= REQUIRED_SAMPLES) {
                g_lid_open = true;
                open_samples = 0;
                close_samples = 0;
                syslog(LOG_NOTICE, "LID OPEN: light=%ld", light_level);
            }
        } else {
            /* 開状態から閉状態へ：別のしきい値以下を2回連続で確認 */
            if (light_level <= LID_CLOSE_THRESHOLD) {
                close_samples++;
            } else {
                close_samples = 0;
            }

            if (close_samples >= REQUIRED_SAMPLES) {
                /* 【リアルタイム性の要点】状態変更後、ログより先に音を止める */
                g_lid_open = false;
                hub_speaker_stop();
                close_samples = 0;
                open_samples = 0;
                syslog(LOG_NOTICE, "LID CLOSED: light=%ld", light_level);
            }
        }

        dly_tsk(SENSOR_INTERVAL_US);
    }
}

/*------------------------------------------------------------
 * SOUND_TASK：蓋が開いている間だけスター・ウォーズを再生
 *
 * hub_speaker_play_tone(..., SOUND_MANUAL_STOP)は音を開始してすぐ戻る。
 * 10msごとに状態を確認するが、停止操作自体はMain側が直接行うため、
 * SOUND_TASKの待ち時間を待たずに閉状態へ反応できる。
 *-----------------------------------------------------------*/
void sound_task(intptr_t exinf)
{
    const unsigned int note_count =
        sizeof(star_wars) / sizeof(star_wars[0]);
    unsigned int note_index = 0;

    while (1) {
        if (!g_lid_open) {
            note_index = 0;
            dly_tsk(NOTE_CHECK_INTERVAL_MS * 1000);
            continue;
        }

        /* 【追加】非ブロッキングで音を開始し、再生時間はタスク側で管理 */
        hub_speaker_play_tone(star_wars[note_index].note, SOUND_MANUAL_STOP);

        for (unsigned int elapsed = 0;
             elapsed < star_wars[note_index].duration_ms;
             elapsed += NOTE_CHECK_INTERVAL_MS) {

            if (!g_lid_open) {
                /* Main側も閉状態検知時に停止するが、念のためここでも停止 */
                hub_speaker_stop();
                break;
            }

            dly_tsk(NOTE_CHECK_INTERVAL_MS * 1000);
        }

        if (g_lid_open) {
            note_index++;
            if (note_index >= note_count) {
                note_index = 0;
            }
        } else {
            note_index = 0;
        }
    }
}
