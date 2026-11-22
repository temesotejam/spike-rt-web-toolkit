# 課題フォルダ

課題のプログラムは、このフォルダ以下に一つずつ保存します。元のサンプル（`apps/`直下）は変更しません。

## 課題プログラム一覧

| 課題 | 簡単な説明 |
| --- | --- |
| [LCD_S3](LCD_S3/LCD_S3.c) | ハブの画面に0～59を1秒ごとに表示し、カウントが60になると「Game Over!」を表示して終了します。 |
| [SOUND_S3](SOUND_S3/SOUND_S3.c) | ハブの画面に「3」を表示し、音量5%で「きらきら星」を1回演奏して終了します。 |
| [TS_S4](TS_S4/TS_S4.c) | Dポートのフォースセンサーを押すと「スター・ウォーズ」のメインテーマ冒頭を繰り返し演奏します。一度離してもう一度押すと停止し、「Pressed!」を表示して終了します。アプリ側ではマルチタスクを使わず、単一タスクで演奏中も10msごとに押下を確認します（[仕組みは下記](#ts_s4の仕組みマルチタスクを使わない演奏と押下確認)）。 |
| [TS_S4_2](TS_S4_2/TS_S4_2.c) | TS_S4と同じく、Dポートのフォースセンサーで演奏を開始・終了します。音を1つずつ記述した簡易版で、押下は各音の終了後に確認するため、停止まで待ち時間が生じることがあります。 |
| [CS_S3](CS_S3/CS_S3.c) | Dポートのカラーセンサーで周囲の明るさを調べ、環境光の値が6以上なら「きらきら星」を繰り返し演奏します。各音の終了後に値を確認し、5以下なら一時停止、明るくなると続きの音から再開します。 |
| [DC_S2](DC_S2/DC_S2.c) | Eポートの超音波センサーで距離を測り、中央ボタンのライトを250mm未満で赤、250mm以上500mm未満で黄、500mm以上で青にします。LCDには「Red／Yellow／Blue」を1文字ずつ繰り返し表示します。計測エラー時はライトを消灯し、LCDに「Err」を表示します。距離の数値はシリアルモニタに出力します。 |
| [M_S6](M_S6/M_S6.c) | 左右ボタンで円・正方形を選び、中央ボタンで決定すると走行します。正方形の直進では左右エンコーダ差をフィードバック補正し、両輪の目標回転量をそろえます。 |
| [PID_Line_Tracer](PID_Line_Tracer/PID_Line_Tracer.c) | 左右2つのカラーセンサー（B/F）の反射光差をPID制御してライントレースします。Cポートのフォースセンサーを1回押すと開始、再度押すと停止します。 |
| [PID_Line_Tracer_V2](PID_Line_Tracer_V2/PID_Line_Tracer_V2.c) | 起動時にHubの左右ボタンでライントレース／校正を選択。白→黒→白の横切り校正、左右センサー個別補正、ライン状態分類、エンコーダオドメトリに加え、内蔵IMUの3軸データとエンコーダ旋回速度を比較ログ出力します。IMUは現段階では診断専用で操舵には未使用です。 |
| [multi_task1_A](multi_task1_A/multi_task1_A.c) | 元サンプルを基に、SUB_TASKを先に完了させてからMAIN_TASKを実行します。 |
| [multi_task3](multi_task3/multi_task3.c) | MAIN_TASK、SUB_TASK1_A、SUB_TASK2の3タスクがセマフォで実行権を渡しながらカウントします。 |
| [cyc_task_multi](cyc_task_multi/cyc_task_multi.c) | スター・ウォーズ再生、1秒ごとのLCDカウント、50cm四方の経路をMOVE_TASKが無限に繰り返し走行する3タスク並列プログラムです。 |
| [cyc_task_realtime](cyc_task_realtime/cyc_task_realtime.c) | Dポートのカラーセンサーで蓋の開閉を監視し、開くと演奏、閉じると停止します。環境光／反射光を切り替えられます。 |

## サンプルから課題を作る

1. [Actions → Create assignment from sample](https://github.com/temesotejam/spike-rt-web-toolkit/actions/workflows/create-assignment.yml) を開き「Run workflow」を押します。
2. `sample_id` にサンプルのフォルダ名（例 `pd_multi_task1`）、`assignment_id` に課題ID（例 `kadai_01`）、`title` に表示名を指定します。
3. 完了すると `apps/assignments/kadai_01/kadai_01.c` と、元サンプルにあった必要な設定ファイルができます。
4. 課題の `.c`（必要に応じて `.cfg` など）を編集・コミットします。通常のアプリと同じように自動ビルドされ、Web画面の「課題」グループから選択できます。

課題IDには半角英字で始まる英数字・アンダースコアを使用してください。既存のアプリIDとの重複は不可です。


生成時にヘッダーファイル名も課題IDに統一されます。たとえば元サンプルの `pd_LCD_S2.h` と `LCD_S2.h` が同じ内容なら、課題IDが `kadai_01` なら `kadai_01.h` だけが残り、Cファイル内の `#include` も自動的に修正されます。独自の補助ヘッダーは削除されません。

## multi_task1_Aとmulti_task2の動作比較

比較対象の元サンプルは、[multi_task1_A](../pd_multi_task1_A/pd_multi_task1_A.c) と [multi_task2](../pd_multi_task2/pd_multi_task2.c) です。

- **multi_task1_A**：MAIN_TASKはループ内の`tslp_tsk(5)`で5msだけ待ち状態になります。一方、SUB_TASKは最初の`wai_sem(SEM_SUB)`で待ち状態になり、MAIN_TASKが全カウントを終えて`sig_sem(SEM_SUB)`を実行するまで解除されません。そのため、MAIN_TASKが時間待ちしている間もSUB_TASKは実行できず、2つのタスクは並列動作しません。MAIN_TASKは「実行中」と「時間待ち」を繰り返し、SUB_TASKはセマフォ待ちのままです。
- **multi_task2**：MAIN_TASKは各カウントの後に`sig_sem(SEM_SUB)`でSUB_TASKを起こし、次のループで`wai_sem(SEM_MAIN)`を実行して待ち状態になります。SUB_TASKは起床してカウントし、`sig_sem(SEM_MAIN)`でMAIN_TASKへ実行権を返します。この受け渡しを繰り返すため、両タスクが交互に動作します。

つまり、`tslp_tsk`による時間待ちだけでは、セマフォで待ち続けている別タスクは実行されません。multi_task2では、セマフォの解放と待ちを組み合わせて、実行可能なタスクを切り替えています。

## multi_task1_A（SUB_TASK先行）

この課題版では、元サンプルの動作を保ちながら、次の部分だけを変更しています。

- `multi_task1_A.cfg`でSUB_TASKの優先度をMAIN_TASKより高くする（優先度の数値を小さくする）
- `SEM_SUB`の初期値を1にし、SUB_TASKが最初から処理を開始できるようにする
- `SEM_MAIN`の初期値を0にし、MAIN_TASKは先頭の`wai_sem(SEM_MAIN)`で待機する
- SUB_TASKが全カウントを終えたところで`sig_sem(SEM_MAIN)`を実行し、MAIN_TASKを起こす

優先度だけを入れ替えると、元のSUB_TASKは最初に`wai_sem(SEM_SUB)`で待ってしまうため、指定された順番になりません。セマフォの初期値と通知先も合わせて変更することで、SUB_TASKの全カウント後にMAIN_TASKのカウントが始まります。

## multi_task3（3タスク並列）

multi_task2のセマフォによる実行権の受け渡しを3タスクに拡張しています。

`SEM_MAIN → SEM_SUB1_A → SEM_SUB2 → SEM_MAIN` の順に実行権を渡し、各タスクが0～100をsyslogへ出力します。想定される出力順は、たとえば次のようになります。

```text
MAIN_TASK = 0
SUB_TASK1_A = 0
SUB_TASK2 = 0
MAIN_TASK = 1
SUB_TASK1_A = 1
SUB_TASK2 = 1
...
```

`multi_task3.h`には`sub_task1_A`と`sub_task2`のプロトタイプ宣言を追加し、`multi_task3.cfg`には2つのタスクと3つのセマフォを定義しています。最後のSUB_TASK2が終了ログを出してからMAIN_TASKを終了させるため、最終カウントも確認できます。

なお、SPIKE Prime Hubは単一CPUで動作するため、3タスクが同じ瞬間にCPU上で実行されるわけではありません。ここでの「並列動作」は、タスクが待ち状態と実行状態を切り替えながら並行して進むマルチタスク動作を指します。

## PID_Line_Tracer

元プログラム：[kouhei1970/pid_exercise](https://github.com/kouhei1970/pid_exercise)

左右2つのカラーセンサーで反射光を読み取り、その差を偏差としてPID制御します。PIDゲイン、基準速度、操作量上限、10msの制御周期、積分制限などは元プログラムの設定を維持しています。

### 機体・ポート構成

正方形走行の[M_S6](M_S6/M_S6.c)と同じ機体構成に合わせています。

| 機器 | 接続ポート |
| --- | --- |
| 左モーター | A（反時計回り） |
| 右モーター | E（時計回り） |
| 左カラーセンサー | B |
| 右カラーセンサー | F |
| 開始・停止用フォースセンサー | C |

**Port F support:** SPIKE-RT v0.2.0 disables F by default. This toolkit enables it in the GitHub Actions build checkout via `.github/scripts/enable-spike-rt-port-f.py`, including the port's UART9 driver. The F debug UART is disabled; USB serial remains configured separately. Port F must be verified on the actual Hub.

モーターは直径88mmのタイヤ、トレッド144mmを使用するM_S6と同じ構成を想定します。PID演算自体は反射光差とモーター回転速度で行うため、車輪寸法は元の制御式には追加していません。

### 元プログラムからの必要な変更

1. モーターC/DをM_S6と同じA/Eへ変更し、回転方向もM_S6に合わせる。
2. A/Eをモーターに使用するため、左カラーセンサーをB、右カラーセンサーをFへ割り当て。
3. モーター初期化をSPIKE-RT v0.2.0で使用している`pup_motor_get_device()`と`pup_motor_setup()`へ変更。
4. Cポートにフォースセンサーを追加し、押下ごとに開始／停止を切り替える（押下・解放とも約20msのチャタリング対策）。
5. 左右反射光の差、PID計算、速度出力の考え方は元プログラムを維持。

起動から約3秒後に合図音が鳴り、Cポートのフォースセンサーの押下を受け付けます。最初の押下で走行開始、いったん離してから次の押下で両モーターをブレーキ停止します。停止後にもう一度押すと再開します。押しっぱなしでは切り替わりません。停止中はPID演算も速度指令もしません。

カラーセンサーはラインの左右に同じ高さ・同じ距離で取り付けてください。目標反射光差は元プログラムと同じ0です。実際のコース、センサー位置、床面に応じて`midpoint`とPIDゲインを調整してください。

## PID_Line_Tracer_V2

[PID_Line_Tracer](PID_Line_Tracer/PID_Line_Tracer.c)を残したまま、標準SPIKE-RT v0.2.0 APIだけで制御周期とPID演算を改善した別バージョンです。モーター・カラーセンサー・フォースセンサーのドライバは変更していません。

### Hardware and ports

| Device | Port |
| --- | --- |
| Left motor | A |
| Right motor | E |
| Left color sensor | B |
| Right color sensor | F |
| Start/stop force sensor | C |

### Mode selection and operation

The Hub display shows one letter for the current selection:

- **L**: line tracing
- **C**: calibration

Use the Hub's LEFT button to select line tracing, RIGHT button to select calibration, and CENTER button to confirm. The force sensor on port C remains the line-tracing start/stop control.

After a successful calibration, the robot stops and returns to the mode-selection screen. Select **L** and confirm to enter line-tracing standby. Press the force sensor on C to start; press it again to stop. Calibration values remain in RAM until power-off.

### Moving calibration across a transverse line

Place the robot on the white surface, facing straight toward a black line that crosses the direction of travel. Both color sensors (B and F) must initially be on white. Select **C** and confirm.

1. The robot samples both white surfaces while stationary.
2. It drives forward at a low speed while synchronizing the left/right wheel encoders.
3. Each sensor independently records its transition from white to black and then back to white.
4. The robot brakes only after both sensors have completed the full white -> black -> white sequence.
5. White and black averages are calculated separately for the left and right sensors. The result is accepted only when both sensors have sufficient black samples and sufficient white/black contrast.

The calibration pass has a maximum travel limit and timeout. The CENTER button cancels the pass. A failed or cancelled attempt does not overwrite a previous successful calibration. If calibration fails, check the initial placement, sensor height, line width, and reflection contrast before retrying.

### Improvements

1. **Periodic control wake-up:** a 10ms cyclic handler releases a binary semaphore. The control task waits on that semaphore instead of adding a relative 10ms delay after every calculation.
2. **Measured elapsed time:** `fch_hrt()` measures the actual interval between control iterations. The PID derivative and integral use this measured interval, with bounds for abnormal delays.
3. **Per-sensor calibration:** the moving calibration measures white and black reflectance independently for sensors B and F. Each sensor is normalized to a common 0..100 scale (black=0, white=100), compensating for sensor-to-sensor differences.
4. **Line-state classification:** normalized readings classify the pair as centered, left sensor over the line, right sensor over the line, both sensors dark (wide line/crossing), or transition. Both-dark holds the previous steering direction and slows down; transition also limits speed.
5. **Encoder odometry:** encoder angle increments from motors A/E are converted to wheel travel using the M_S6 values (88 mm wheel diameter, 144 mm track). Encoder-only X/Y, heading, distance, speed, yaw rate, and curvature are retained as an independent diagnostic reference.
6. **Six-state line-relative EKF:** the state is `[s, e_y, e_psi, omega, b_g, kappa]`: along-track distance, cross-track error, heading error, yaw rate, gyro bias, and local path curvature.
7. **Three sensor observation sources:** raw IMU gyro Z observes `omega + b_g`; differential encoder speed observes `omega`; calibrated B/F reflectance difference provides a gated lateral line observation of `e_y`.
8. **Adaptive observation confidence:** gyro noise is initialized from stationary samples; encoder observation variance increases when gyro/encoder yaw disagree; line observations use different variances for edge, transition, centered, and both-dark states. Both-dark is rejected as ambiguous.
9. **EKF-based steering:** the PID input uses estimated cross-track and heading error. EKF yaw rate provides steering damping and speed adaptation. High estimated uncertainty reduces speed rather than automatically stopping the robot.
10. **Future course-map interface:** the EKF tracks along-route distance and local curvature. A currently inactive map hook can later provide route curvature and target speed without changing the EKF state definition.
11. **Filtered derivative and anti-windup:** a first-order low-pass filter reduces derivative spikes, and conditional integration prevents windup during output saturation.
12. **Start/stop input:** force sensor C toggles line-tracing operation. Three consecutive 10ms samples are required for a press or release, preventing repeated toggles while the sensor is held.

The EKF currently estimates position in a **local line-relative frame**, not global course X/Y coordinates. The along-track coordinate starts at zero for each run and is primarily propagated by wheel speed; cross-track error is corrected by the two color sensors. The line measurement conversion uses the provisional parameter `LINE_EKF_MM_PER_PERCENT=0.10` mm per normalized-reflectance percentage point. This is a tunable approximation because the actual B/F sensor spacing and course line width have not yet been measured. Both-white observations are weak and gated; both-dark observations are rejected. A later course-map module can transform local `(s, e_y)` into global map coordinates and add landmark/map observations.

### Six-state EKF: IMU + encoder + line observations

The fixed 85/15 gyro/encoder blend has been replaced by a six-state Extended Kalman Filter. It uses only the standard SPIKE-RT v0.2.0 APIs; no kernel or device-driver changes are made.

The state vector is:

`x = [s, e_y, e_psi, omega, b_g, kappa]^T`

| State | Meaning | Unit |
| --- | --- | --- |
| `s` | Distance along the current local line/path frame | mm |
| `e_y` | Cross-track offset from the local line | mm |
| `e_psi` | Robot heading error relative to the local line tangent | rad |
| `omega` | Body yaw rate | rad/s |
| `b_g` | Z-gyro bias | rad/s |
| `kappa` | Local path curvature | 1/m |

#### Prediction

The average measured wheel speed provides forward velocity. The current yaw-rate and curvature estimates propagate the local path-relative state:

- `s += v * cos(e_psi) * dt`
- `e_y += v * sin(e_psi) * dt`
- `e_psi += (omega - v * kappa) * dt`

The covariance is propagated with the nonlinear model Jacobian and process-noise terms. Curvature is a slowly varying state, so the filter can follow straight and curved local line sections without hardcoding a specific annual course.

#### Sequential observation updates

1. **IMU gyro Z:** raw gyro Z is modeled as `z_g = omega + b_g + noise`. The startup stationary bias estimate initializes `b_g`; the EKF can then refine it while moving.
2. **Wheel encoders A/E:** measured left/right motor speeds produce an independent differential-drive yaw-rate observation. Its variance is increased when the gyro and encoder disagree, reducing the influence of a possible wheel-slip measurement.
3. **Color sensors B/F:** the calibrated normalized-reflectance difference provides a provisional lateral observation of `e_y`. Edge observations receive higher confidence, transition observations lower confidence, centered both-white observations are weak and gated, and both-dark observations are rejected as ambiguous.

The scalar observation updates use an innovation gate and Joseph-form covariance update. The EKF covariance is retained and its standard deviations are logged; it is also used to reduce the commanded speed when position/heading uncertainty becomes large.

#### Line observation approximation

The current line observation is:

`z_line = (normalized_left - normalized_right) * LINE_EKF_MM_PER_PERCENT`

The initial `LINE_EKF_MM_PER_PERCENT=0.10` is an **engineering starting value**, not a measured property of the robot. It must be tuned using the actual B/F sensor spacing, sensor height, and line width. The current moving white-black-white calibration estimates each sensor's reflectance range; it does not yet calibrate the lateral millimeter-per-reflectance relationship.

#### Course-map integration (future hook)

No ETロボコン2023 route is hardcoded. The current `course_map_get_reference(s)` hook returns `valid=false`. Once a course file is loaded, it can provide local path curvature and a target speed at the EKF along-track coordinate. A map matcher can later add landmark/segment observations to convert local `(s, e_y, e_psi)` into global course coordinates. This keeps the estimator independent of which annual course segment is selected.

#### Diagnostics

At nominal 5 Hz, syslog records line/motion state, raw and normalized reflectance, accepted/rejected line observation status, encoder-only odometry, EKF state, EKF uncertainty, gyro bias estimate, and motor commands. While waiting for the force-sensor start command, idle logs remain available for stationary gyro and hand-rotation checks.

**Limitations:** the EKF is currently local line-relative localization, not global map localization. Along-track position still accumulates encoder error. The lateral line observation is an approximate scalar model until sensor spacing and line width are measured. The curvature state is locally estimated; the route map is not loaded yet. Wheel slip, sensor timing, gyro bias drift, and the actual line/sensor geometry must be checked using real logs before treating the tuning values as final.

### Initial tuning values

The original exercise gains are retained as initial values: `KP=1.0`, `KI=0.1`, and `KD=0.1`. The straight/turn base speeds start at 120/80 deg/s. Calibration speed starts at 60 deg/s. These are initial values, not verified optimum values; tune them on the actual course.

The cyclic handler performs only a semaphore notification. Sensor reads, calibration, PID calculations, and motor commands run in ordinary task context. If a control iteration falls behind, the binary semaphore limits pending notifications to one and an overrun warning is recorded. This improves scheduling behavior but does not guarantee a hard 10ms deadline; verify timing on the actual Hub.

## cyc_task：周期ハンドラの動作と改造内容

元のサンプルは[cyc_task](../pd_cyc_task/pd_cyc_task.c)です。元プログラムの`cyc_task.c`と`cyc_task.cfg`は変更せず、改造版を2つ追加しています。

### 元の周期ハンドラの動作

- `CRE_CYC(TEST_SPIKE_CYC1, ...)`で周期ハンドラを登録し、周期を1,000,000μs（1秒）に設定します。
- `sta_cyc(TEST_SPIKE_CYC1)`で周期ハンドラを開始します。
- 1秒ごとに`test_spike_cychdr()`が呼び出され、カウント値をLCDに表示します。
- 周期ハンドラはMAIN_TASKとは独立して呼び出されます。MAIN_TASKが音の再生時間だけ待っていても、周期ハンドラは動作します。

SPIKE-RT v0.2.0では、`hub_speaker_play_tone(frequency, duration)`は指定時間だけ呼び出し元タスクを待たせてから音を停止します。一方、`SOUND_MANUAL_STOP`を指定すると音を開始してすぐ戻り、`hub_speaker_stop()`で手動停止します（[スピーカーAPI実装](https://github.com/spike-rt/spike-rt/blob/v0.2.0/drivers/spike/hub/speaker.c)）。

### cyc_task_multi：3タスク並列

| タスク | 処理 |
| --- | --- |
| MAIN_TASK | スター・ウォーズのメロディーを繰り返し再生 |
| COUNT_TASK | 1秒ごとにカウントを増やしてLCDに表示 |
| MOVE_TASK | A/Eポートのモータで50cm直進と90度旋回を4回実行 |

**元プログラムからの主な変更**

1. 元のMainのメロディー再生を残し、曲データをTS_S4のスター・ウォーズに変更
2. LCD表示をCOUNT_TASKへ分離
3. 周期ハンドラはLCD APIを直接呼ばず、`isig_sem(SEM_COUNT)`でCOUNT_TASKへ通知
4. M_S6のモータ制御をMOVE_TASKとして追加
5. モータ角度の確認ループに10msの`dly_tsk()`を入れ、他タスクへCPUを譲る

周期ハンドラは非タスクコンテキストで動作するため、ここでは短いセマフォ通知だけを行います。LCD表示は通常のタスク側で処理します。

走行はM_S6と同じく、左モータA・右モータE、直径88mmタイヤ、トレッド144mmを仮定しています。直進時は始動・終了を120°/s、中間を300°/sとする台形速度プロファイルを使い、最初の20%で加速、最後の20%で減速します。10msごとに左右エンコーダ差も確認し、先行側を減速・遅れ側を増速します。左右それぞれが目標角度に達した時点で個別にブレーキをかけます。90度旋回は左車輪を支点に右車輪を逆転させる方式です。タイヤ径や車輪間隔が異なる場合は、プログラム先頭の寸法定数を実機に合わせてください。床面の滑りやモータ負荷による誤差は実機で調整が必要です。

### cyc_task_realtime：蓋の開閉に100ms以内で応答する構成

カラーセンサーをDポートに接続し、蓋を閉じたときにセンサー面が覆われ、開いたときに周囲の光が入る向きに取り付けます。

- MAIN_TASKを高優先度のセンサー監視タスクに変更し、10msごとに環境光を読み取ります。
- 2回連続で開側のしきい値を超えたら開状態にします。
- 2回連続で閉側のしきい値以下になったら、状態を閉に変更してすぐ`hub_speaker_stop()`を呼びます。
- SOUND_TASKは低い優先度で動作し、`SOUND_MANUAL_STOP`を使って音を開始します。音の長さを10ms単位で管理するため、長い音の再生中でもセンサー監視が優先されます。

標準では環境光を使います。反射光で判定したい場合は、ソースの`LID_DETECT_USE_AMBIENT`を0に変更します。環境光の開／閉しきい値は12／8、反射光の初期値は25／18です。これらは仮の値なので、蓋を開けた状態と閉じた状態の実測値に合わせて調整してください。

ソフトウェア上は10ms周期・2回連続確認を目標にしていますが、センサー内部の更新周期や通信遅延もあるため、100ms以内の実機応答は測定して確認してください。

## TS_S4の仕組み：マルチタスクを使わない演奏と押下確認

[TS_S4](TS_S4/TS_S4.c)は、演奏用とセンサー確認用のタスクを別々に作らず、アプリの単一タスク内で処理します。SPIKE-RTのOSやスピーカーのドライバーは利用していますが、アプリ側で追加のタスクや独自の割り込み処理は作成していません。

1. **音を鳴らし始め、すぐに次の処理へ進む。** `hub_speaker_play_tone(音階, SOUND_MANUAL_STOP)`を呼ぶと、音の終了を待たずに処理が戻ります（非ブロッキング再生）。スピーカーは停止や次の音の指示が来るまで鳴り続けるため、その間にセンサーを確認できます。
2. **音の長さを10msずつに分けて待つ。** 音階と長さを配列に保存し、各音について「押下を確認する → `dly_tsk(10000)`で10ms待つ」を繰り返します。待ち時間の合計がその音の長さに達したら、次の音へ進みます。長い音の途中でも押下を確認できるのはこのためです。実際の間隔には処理時間やOSのスケジューリングによる遅れが加わります。
3. **一度離した後の押下で停止する。** 開始時の押しっぱなしを停止操作と誤認しないよう、まず離された状態を3回連続、その後に押された状態を2回連続で確認します。停止条件を満たすと`hub_speaker_stop()`で音を止め、演奏処理を抜けます。
4. **停止してから文字を表示する。** 「Pressed!」の表示は演奏停止後に行います。表示終了まで待つ処理が、演奏中のセンサー確認を妨げないようにしています。

このように、音の出力をスピーカー側に任せ、アプリは短い間隔でセンサー確認と次の音への切り替えを行うことで、単一タスクでも演奏しながら停止操作を受け付けられます。`TS_S4_2`は各音を鳴らし終えてから押下を確認するため、長い音の途中では停止操作を確認できません。

## DC_S2の仕組み：LCD表示中も距離を測定する

[DC_S2](DC_S2/DC_S2.c)では、LCD（ハブの5×5 LED画面）に「Red／Yellow／Blue」を1文字ずつ繰り返し表示し、中央ボタンのライトも同じ色にします。計測エラー時は「Err」を表示してライトを消灯します。距離の数値（mm）はシリアルモニタに出力します。

`hub_display_text()`は、文字列を1文字ずつ表示し終わるまで呼び出し元の処理を待たせます。そこで、表示後すぐに処理が戻る`hub_display_char()`を使い、単一タスクのループ内で文字を切り替えています。表示用の別タスクや独自の割り込み処理は作成していません。

1. **距離測定と色の判定を毎ループ行う。** 250mm未満はRed、250mm以上500mm未満はYellow、500mm以上はBlueと判定し、ライトを更新します。
2. **表示の進み具合を変数で覚える。** 現在の判定、何文字目か、何回ループしたかを記録します。1文字を20回の待機（約200ms）だけ表示し、その後5回の待機（約50ms）だけ消灯して次の文字へ進みます。同じ文字が続く「Yellow」の「ll」や「Err」の「rr」も見分けられます。末尾まで進んだら先頭へ戻ります。
3. **表示中も測定を続け、判定が変われば表示を切り替える。** 文字の表示時間をまとめて待たず、ループ末尾の`dly_tsk(10000)`で10msずつ待ちます。その間も各ループで距離を測定し、新しい判定になった時点で文字列の先頭から表示し直します。

1周にはセンサー取得・ログ出力・表示更新などの処理時間も含まれるため、測定周期や文字の表示時間は上記の値より長くなることがあります。

表示APIの動作は、使用しているSPIKE-RT v0.2.0の[実装](https://github.com/spike-rt/spike-rt/blob/v0.2.0/drivers/spike/hub/display.c)で確認できます。
