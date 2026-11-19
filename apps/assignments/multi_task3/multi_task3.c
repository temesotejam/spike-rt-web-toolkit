/*------------------------------------------------------------
 *  multi_task3：MAIN_TASK、SUB_TASK1_A、SUB_TASK2の3タスクを
 *  セマフォで順番に切り替えながらカウントするサンプル
 *-----------------------------------------------------------*/
#include <kernel.h>
#include <t_syslog.h>
#include <multi_task3.h>
#include "kernel_cfg.h"

/* 0～COUNTを各タスクが出力する */
#define COUNT       100
#define TASK_DELAY_MS 5

/*------------------------------------------------------------
 *  MAIN_TASK
 *  MAIN -> SUB_TASK1_A -> SUB_TASK2 -> MAIN の順に実行権を渡す
 *-----------------------------------------------------------*/
void Main(intptr_t exinf)
{
    int i;

    syslog(LOG_NOTICE, "Start multi_task3");
    syslog(LOG_NOTICE, "MAIN_TASK: start");

    for (i = 0; i <= COUNT; i++) {
        wai_sem(SEM_MAIN);
        syslog(LOG_NOTICE, "MAIN_TASK = %d", i);
        tslp_tsk(TASK_DELAY_MS);
        sig_sem(SEM_SUB1_A);
    }

    /* 最後のSUB_TASK2が完了するまで待ってからMAIN_TASKを終了 */
    wai_sem(SEM_MAIN);
    syslog(LOG_NOTICE, "MAIN_TASK: end.");
    ext_tsk();
}

/*------------------------------------------------------------
 *  SUB_TASK1_A
 *  MAIN_TASKから実行権を受け取り、SUB_TASK2へ渡す
 *-----------------------------------------------------------*/
void sub_task1_A(intptr_t exinf)
{
    int i;

    syslog(LOG_NOTICE, "SUB_TASK1_A: start");

    for (i = 0; i <= COUNT; i++) {
        wai_sem(SEM_SUB1_A);
        syslog(LOG_NOTICE, "SUB_TASK1_A = %d", i);
        tslp_tsk(TASK_DELAY_MS);
        sig_sem(SEM_SUB2);
    }

    syslog(LOG_NOTICE, "SUB_TASK1_A: end.");
    ext_tsk();
}

/*------------------------------------------------------------
 *  SUB_TASK2
 *  SUB_TASK1_Aから実行権を受け取り、次のMAIN_TASKへ戻す
 *-----------------------------------------------------------*/
void sub_task2(intptr_t exinf)
{
    int i;

    syslog(LOG_NOTICE, "SUB_TASK2: start");

    for (i = 0; i <= COUNT; i++) {
        wai_sem(SEM_SUB2);
        syslog(LOG_NOTICE, "SUB_TASK2 = %d", i);
        tslp_tsk(TASK_DELAY_MS);

        /* 最終回は終了ログを出してからMAIN_TASKへ戻す */
        if (i < COUNT) {
            sig_sem(SEM_MAIN);
        }
    }

    syslog(LOG_NOTICE, "SUB_TASK2: end.");
    sig_sem(SEM_MAIN);
    ext_tsk();
}
