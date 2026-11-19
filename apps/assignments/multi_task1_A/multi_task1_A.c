/*------------------------------------------------------------
 *  multi_task1_A：SUB_TASKを先に完了し、その後MAIN_TASKを実行する
 *  元サンプルを基に、優先度とセマフォの初期値・受け渡しを変更
 *-----------------------------------------------------------*/
#include <kernel.h>
#include <stdlib.h>
#include <t_syslog.h>
#include <multi_task1_A.h>
#include "spike/hub/display.h"
#include "spike/hub/speaker.h"
#include "spike/hub/battery.h"
#include <t_stdlib.h>
#include "kernel_cfg.h"

#define COUNT           10000
#define MAIN_DELAY_MS   5
#define SUB_DELAY_MS    5

/*------------------------------------------------------------
 *  MAIN_TASK
 *  SUB_TASKが完了するまでセマフォで待ち、その後カウントする
 *-----------------------------------------------------------*/
void Main(intptr_t exinf)
{
    int i = 0;

    /* SUB_TASKがSEM_MAINを解放するまで待つ */
    wai_sem(SEM_MAIN);

    syslog(LOG_NOTICE, "Start multi_task1_A");
    syslog(LOG_NOTICE, "MAIN_TASK: start");

    for (i = 0; i <= COUNT; i++) {
        syslog(LOG_NOTICE, "MAIN TASK = %d", i);
        tslp_tsk(MAIN_DELAY_MS);
    }

    syslog(LOG_NOTICE, "MAIN_TASK: end.");
    ext_tsk();
}

/*------------------------------------------------------------
 *  SUB_TASK
 *  初期セマフォを受け取って先にカウントし、完了後MAIN_TASKを解放
 *-----------------------------------------------------------*/
void sub_task(intptr_t exinf)
{
    int i;

    /* SEM_SUBの初期値は1なので、ここから実行できる */
    wai_sem(SEM_SUB);

    syslog(LOG_NOTICE, "SUB_TASK: start.");

    for (i = 0; i <= COUNT; i++) {
        syslog(LOG_NOTICE, "SUB TASK = %d", i);
        tslp_tsk(SUB_DELAY_MS);
    }

    syslog(LOG_NOTICE, "SUB_TASK: end.");

    /* SUB_TASKの完了をMAIN_TASKへ通知 */
    sig_sem(SEM_MAIN);
    ext_tsk();
}
