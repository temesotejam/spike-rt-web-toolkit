/*
 * cyc_task_multi.c
 *
 * 【元のcyc_task.cからの変更点】
 * 1. 元のMainにあったメロディー再生を残し、スター・ウォーズの旋律に変更。
 * 2. LCDカウント表示をcount_taskへ分離。1秒周期の周期ハンドラは
 *    isig_sem()でcount_taskを起こすだけにし、LCD操作はタスク側で行う。
 * 3. M_S6のA/Eポートモータ制御をmove_taskとして追加し、50cm直進と
 *    90度旋回を4回行う。
 *
 * 3つのタスクは同じCPUを時間分割して使う。音再生中やモータ角度の
 * 確認待ちにはdly_tsk()が入るため、その間に他のタスクが実行される。
 */
#include <kernel.h>
#include <stdint.h>
#include <t_syslog.h>
#include "cyc_task_multi.h"
#include "kernel_cfg.h"

#include "spike/hub/display.h"
#include "spike/hub/speaker.h"
#include "spike/pup/motor.h"

#define VOLUME              30
#define TASK_DELAY_US       10000

/* M_S6と同じAdvanced Driving Base想定。実機の寸法に合わせて調整する。 */
#define PI                  3.14159265f
#define WHEEL_DIAMETER_MM   88.0f
#define WHEEL_TRACK_MM      144.0f
#define SIDE_LENGTH_MM      500.0f

#define TEMPO_BPM           100
#define QUARTER_MS          (60000 / TEMPO_BPM)
#define TRIPLET_MS          (QUARTER_MS / 3)
#define HALF_MS             (QUARTER_MS * 2)

/* TS_S4のスター・ウォーズのメロディーを再利用。
 * 元のcyc_taskの音階列を大きく作り直さず、曲データだけ置き換える。 */
typedef struct {
    uint16_t note;
    uint16_t duration_ms;
} melody_note_t;

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

/*------------------------------------------------------------
 * 周期ハンドラ：1秒ごとにCOUNT_TASKへ実行タイミングを通知
 *
 * 【元のcyc_task.cからの変更】
 * 元サンプルは周期ハンドラ内でhub_display_number()を呼んでいた。
 * ここでは非タスクコンテキストでLCD APIを呼ばず、isig_sem()だけを実行。
 * isig_sem()は非タスクコンテキストから呼び出せるサービスコール。
 *-----------------------------------------------------------*/
void test_spike_cychdr(intptr_t exinf)
{
    isig_sem(SEM_COUNT);
}

/*------------------------------------------------------------
 * MAIN_TASK：スター・ウォーズを繰り返し再生
 *
 * hub_speaker_play_tone()はduration_msの間、呼び出し元タスクを
 * dly_tsk()で待たせる。その間はCOUNT_TASKやMOVE_TASKも実行できる。
 *-----------------------------------------------------------*/
void Main(intptr_t exinf)
{
    const unsigned int note_count =
        sizeof(star_wars) / sizeof(star_wars[0]);

    syslog(LOG_NOTICE, "cyc_task_multi started");
    hub_speaker_set_volume(VOLUME);

    /* 元サンプルと同じ周期ハンドラを使用。cfg側はTA_NULLにし、
       ここで明示的に開始する。 */
    sta_cyc(TEST_SPIKE_CYC1);

    while (1) {
        for (unsigned int i = 0; i < note_count; i++) {
            hub_speaker_play_tone(star_wars[i].note,
                                  star_wars[i].duration_ms);
        }

        /* 元のcyc_taskにあった曲間の小休止を残す */
        dly_tsk(300000);
    }
}

/*------------------------------------------------------------
 * COUNT_TASK：1秒ごとにLCDのカウントを更新
 *
 * 【追加したタスク】
 * 周期ハンドラからSEM_COUNTが通知されるまで待つ。
 * 周期ハンドラは通知だけ、LCD表示はこのタスクが担当する。
 *-----------------------------------------------------------*/
void count_task(intptr_t exinf)
{
    int32_t count = 0;

    while (1) {
        wai_sem(SEM_COUNT);
        count++;
        hub_display_number(count);
    }
}

/*
 * 【直進制御の改善】
 * 以前はA/Eへ同じ速度を順番に指令し、A側のエンコーダだけで終了判定していた。
 * そのため、始動遅れや左右の負荷差があると、片側が先に目標へ達した時点で
 * 両輪を停止してしまい、左右の走行距離に差が残っていた。
 *
 * 改善後は両輪のエンコーダ差を10msごとに確認し、先行側を減速・遅れ側を増速する。
 * また、A/Eそれぞれが目標角度に達した時点で個別にブレーキをかける。
 * これにより、モータ指令の開始時刻に差があっても、左右の最終回転量をそろえる。
 */
#define STRAIGHT_BASE_SPEED  300
#define STRAIGHT_MIN_SPEED   120
#define STRAIGHT_MAX_SPEED   360
#define STRAIGHT_SYNC_GAIN   3
#define STRAIGHT_UPDATE_US   10000

static int32_t clamp_straight_speed(int32_t speed)
{
    if (speed < STRAIGHT_MIN_SPEED) return STRAIGHT_MIN_SPEED;
    if (speed > STRAIGHT_MAX_SPEED) return STRAIGHT_MAX_SPEED;
    return speed;
}

/* 直進成功なら1、ストール検知なら0を返す */
static int drive_straight(pup_motor_t *motorA,
                          pup_motor_t *motorE,
                          int32_t target_angle)
{
    int doneA = 0;
    int doneE = 0;

    pup_motor_reset_count(motorA);
    pup_motor_reset_count(motorE);

    /* モータAPIは個別指令なので完全な同時始動ではない。
       直後からエンコーダ差のフィードバックで始動遅れを補正する。 */
    pup_motor_set_speed(motorA, STRAIGHT_BASE_SPEED);
    pup_motor_set_speed(motorE, STRAIGHT_BASE_SPEED);

    while (!doneA || !doneE) {
        int32_t countA = pup_motor_get_count(motorA);
        int32_t countE = pup_motor_get_count(motorE);

        if (pup_motor_is_stalled(motorA) ||
            pup_motor_is_stalled(motorE)) {
            pup_motor_stop(motorA);
            pup_motor_stop(motorE);
            return 0;
        }

        /* 片側が目標に達したら、その側だけ先に停止する */
        if (!doneA && countA >= target_angle) {
            pup_motor_brake(motorA);
            doneA = 1;
        }
        if (!doneE && countE >= target_angle) {
            pup_motor_brake(motorE);
            doneE = 1;
        }

        if (doneA && doneE) {
            break;
        }

        /* Aが先行していればAを遅く、Eを速くする。逆なら反対に補正。 */
        {
            int32_t error = countA - countE;
            int32_t correction = error * STRAIGHT_SYNC_GAIN;
            int32_t speedA = clamp_straight_speed(
                STRAIGHT_BASE_SPEED - correction);
            int32_t speedE = clamp_straight_speed(
                STRAIGHT_BASE_SPEED + correction);

            if (!doneA) {
                pup_motor_set_speed(motorA, speedA);
            }
            if (!doneE) {
                pup_motor_set_speed(motorE, speedE);
            }
        }

        dly_tsk(STRAIGHT_UPDATE_US);
    }

    pup_motor_brake(motorA);
    pup_motor_brake(motorE);
    return 1;
}

/*------------------------------------------------------------
 * MOVE_TASK：M_S6のモータ制御を基に50cm四方を1周
 *
 * 【追加したタスク】
 * M_S6からA/Eポートの初期化、エンコーダによる距離制御を再利用。
 * メニュー操作は省き、50cm直進→右90度旋回を4回行う。
 * 旋回はM_S6と同じく左車輪を支点に右車輪を逆転させる方式。
 *-----------------------------------------------------------*/
void move_task(intptr_t exinf)
{
    pup_motor_t *motorA = pup_motor_get_device(PBIO_PORT_ID_A);
    pup_motor_t *motorE = pup_motor_get_device(PBIO_PORT_ID_E);

    if (motorA == NULL || motorE == NULL) {
        syslog(LOG_ERROR, "MOVE_TASK: motor not found on A/E");
        ext_tsk();
    }

    if (pup_motor_setup(motorA, PUP_DIRECTION_COUNTERCLOCKWISE, true)
            != PBIO_SUCCESS ||
        pup_motor_setup(motorE, PUP_DIRECTION_CLOCKWISE, true)
            != PBIO_SUCCESS) {
        syslog(LOG_ERROR, "MOVE_TASK: motor setup failed");
        ext_tsk();
    }

    /* 500mm直進に必要な車輪角度 [deg] */
    const int32_t straight_angle =
        (int32_t)(360.0f * SIDE_LENGTH_MM /
                  (PI * WHEEL_DIAMETER_MM) + 0.5f);

    /* 左車輪を支点に右車輪だけ動かすM_S6方式の90度旋回角度 */
    const int32_t turn_angle =
        (int32_t)(360.0f * WHEEL_TRACK_MM /
                  (2.0f * WHEEL_DIAMETER_MM) + 0.5f);

    syslog(LOG_NOTICE, "MOVE_TASK: square path start");

    for (int side = 0; side < 4; side++) {
        /* 1辺目～4辺目：左右エンコーダを同期させながら50cm直進 */
        if (!drive_straight(motorA, motorE, straight_angle)) {
            goto motor_stalled;
        }
        dly_tsk(100000);

        /* 90度右旋回：左車輪を止め、右車輪だけ逆転 */
        pup_motor_reset_count(motorE);
        pup_motor_set_speed(motorE, -220);

        while (pup_motor_get_count(motorE) > -turn_angle) {
            if (pup_motor_is_stalled(motorE)) {
                goto motor_stalled;
            }
            dly_tsk(TASK_DELAY_US);
        }

        pup_motor_brake(motorE);
        dly_tsk(100000);
    }

    pup_motor_brake(motorA);
    pup_motor_brake(motorE);
    syslog(LOG_NOTICE, "MOVE_TASK: square path completed");
    ext_tsk();

motor_stalled:
    syslog(LOG_ERROR, "MOVE_TASK: motor stalled");
    pup_motor_stop(motorA);
    pup_motor_stop(motorE);
    ext_tsk();
}
