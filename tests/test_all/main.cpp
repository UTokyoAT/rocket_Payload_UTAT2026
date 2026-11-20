#include <Arduino.h>
#include <Expander.h>
#include <Sensor.h>
#include <GPS.h>
#include <Encoder.h>
#include <KalmanFilter.h>
#include <Telemetry.h>
#include <Actuator.h>
#include <MotorController.h>
#include <Deployer.h>
#include "secrets.h"

// 打ち上げ〜着地〜誘導の一連の流れを通しで行うテスト（test_judge_launch + test_mission）。
// PlatformIOで env:test-all を選択して書き込む。
//
//   WAIT_CMD ─goal→ GOAL_FIX → READY ─start→ BARO_CAL → LAUNCH_WAIT → ROCKET_CUT
//     → LAND_WAIT → PARA_CUT → WAIT_FIX → STATIC_0 → CAL_RUN → STATIC_2 → GUIDE → POST → DONE
//
//   BARO_CAL     気圧を BARO_CAL_TICKS 周期平均して基準気圧にする（以後の高度はこの基準からの相対高度）
//   LAUNCH_WAIT  相対高度 > LAUNCH_ALT_THRESHOLD_M が LAUNCH_CONFIRM_TICKS 周期続いたら LAUNCH 確定
//   ROCKET_CUT   ロケット分離のニクロム線を BURN_MS 通電
//   LAND_WAIT    気圧が基準値から LAND_DP_HPA 以内に LAND_CONFIRM_TICKS 周期とどまったら着地確定
//                （0.2 hPa ≒ 高度 1.7 m。外れたらその時点の気圧を新しい基準にして数え直す）
//   PARA_CUT     パラシュート分離のニクロム線を BURN_MS 通電
//   WAIT_FIX 以降 test_mission と同じ
//
// ニクロム線の通電は Deployer::deployRocket() / deployParachute() を使わず、setRocket() /
// setParachute() で ON/OFF だけ切り替えて時間はこちらで測る（deploy*() は delay() で3秒止まり、
// その間 IMU・GPS・テレメトリが止まるため）。
//
// 地上局から MQTT の <prefix>/cmd (既定 rocket/cmd) にテキストで指示する。
//   goal              いまいる場所の GPS を N_avg 個平均してゴールに登録する（WAIT_CMD / READY で受付）
//   goal <lat> <lon>  ゴールを座標で直接登録する（WAIT_CMD / READY で受付）
//   start             気圧基準の取得 → LAUNCH 待ちへ（READY でだけ受付）
//   launch            ベンチ確認用。LAUNCH_WAIT で LAUNCH を強制確定する
//   land              ベンチ確認用。LAND_WAIT で着地を強制確定する
//   abort             GOAL_FIX: 登録を取り消す / BARO_CAL・LAUNCH_WAIT: READY に戻る /
//                     それ以降: ニクロム線とモーターを止めて POST へ
// 例: mosquitto_pub -h <PCのIP> -t rocket/cmd -q 1 -m start
//
// phase の番号は test_mission と違うので注意（PHASE_NAMES を参照）。
// 符号規約：方位は真北0・時計回り正、ω も時計回り正（MPU6050 の gz の符号反転）、a は機体前方正。

// --- 定数 ---
static const double ACCEL_FWD_SIGN = +1;       // a = ACCEL_FWD_SIGN × ay（機体前方 = +y の想定） TODO(実測)
static const int ENC_SIGN_R = +1;              // encoder.setDirection() に渡す。test-motor で確認
static const int ENC_SIGN_L = +1;
static const int MOTOR_SIGN_R = +1;            // MotorControlConfig に渡す
static const int MOTOR_SIGN_L = +1;

static const int   BARO_CAL_TICKS = 20;        // [周期] 基準気圧の平均に使う周期数（2 s）
static const float LAUNCH_ALT_THRESHOLD_M = 3.0f;  // [m] test_judge_launch と同じ
static const int   LAUNCH_CONFIRM_TICKS = 5;   // [周期] test_judge_launch と同じ
static const float LAND_DP_HPA = 0.2f;         // [hPa] 着地判定の気圧変化の許容幅（≒ 高度 2 m）
static const int   LAND_CONFIRM_TICKS = 30;    // [周期] 着地判定に必要な連続周期数（3 s）
static const uint32_t BURN_MS = 3000;          // [ms] ニクロム線の通電時間（Deployer の DELAY_ms と同じ）

static const double CAL_DUTY = 0.6;            // [-] キャリブレーション直進のデューティ
static const uint32_t T_CAL_MS = 10000;        // [ms] キャリブレーション直進の時間
static const uint32_t T_SETTLE_MS = 1000;      // [ms] 直進停止後、STATIC_2 に入るまでの待ち時間
static const uint32_t T_MAX_MS = 600000;       // [ms] 誘導開始からのタイムアウト
static const uint32_t T_POST_MS = 5000;        // [ms] 停止後のログ継続時間

static const double KP = 2.0;                  // 方位PID（説明書 6章のシミュレーション結果）
static const double KI = 0.2;
static const double KD = 0.1;

static const uint32_t IMU_PERIOD_MS = 10;      // IMU・気圧の読み取り周期
static const uint32_t TICK_PERIOD_MS = 100;    // 判定・EKF・誘導・ログの周期（EkfConfig::dt と同じ）

static const int TELEMETRY_QUEUE_LEN = 50;     // [周期] 送信待ちの上限（5 s ぶん）。あふれた分は捨てる
static const int LOG_QUEUE_LEN = 8;            // [行] ログの送信待ちの上限
static const int LOG_MSG_LEN = 96;             // [文字] ログ1行の長さ
static const int CMD_QUEUE_LEN = 4;            // [個] 地上局コマンドの受信待ちの上限

static const char CSV_HEADER[] =
    "t_ms,phase,pN,pE,v,theta,ba,bg,P_pN,P_pE,P_theta,d_hat,theta_e,u,dutyL,dutyR,"
    "gnss_upd,cog_upd,y_gN,y_gE,S_gN,S_gE,y_cog,S_cog,y_eR,y_eL,S_eR,S_eL,a,omega,"
    "vR,vL,lat,lon,sog,tick_us,imu_us,late_ms,overruns,tx_drop,"
    "pres,pres_base,alt_rel,launch_cnt,land_ref,land_cnt";

// --- フェーズ ---
enum Phase { WAIT_CMD, GOAL_FIX, READY, BARO_CAL, LAUNCH_WAIT, ROCKET_CUT, LAND_WAIT, PARA_CUT,
             WAIT_FIX, STATIC_0, CAL_RUN, STATIC_2, GUIDE, POST, DONE };  // ログの phase 列はこの番号
static const char* const PHASE_NAMES[] = {"WAIT_CMD", "GOAL_FIX", "READY", "BARO_CAL", "LAUNCH_WAIT",
                                          "ROCKET_CUT", "LAND_WAIT", "PARA_CUT", "WAIT_FIX", "STATIC_0",
                                          "CAL_RUN", "STATIC_2", "GUIDE", "POST", "DONE"};

// 静止区間（STATIC_0 / STATIC_2）の集計。IMU は 100 ms 平均を1サンプルとする
struct StaticAverage {
    int samples = 0, fixes = 0;
    double aSum = 0, omegaSum = 0, omegaSqSum = 0;
    double latSum = 0, lonSum = 0;

    void addImu(double a, double omega) {
        samples++;
        aSum += a;
        omegaSum += omega;
        omegaSqSum += omega * omega;
    }
    void addFix(double lat, double lon) {
        fixes++;
        latSum += lat;
        lonSum += lon;
    }
    double a()     const { return aSum / samples; }
    double omega() const { return omegaSum / samples; }
    double lat()   const { return latSum / fixes; }
    double lon()   const { return lonSum / fixes; }
};

// 最新の GPS fix
struct LatestFix {
    bool   valid = false;   // 一度でも fix を受信したか
    double lat = 0, lon = 0;
    float  alt = NAN;
    double cog_deg = 0, sog = 0;
};

// 100 ms ぶんの IMU・気圧の積算
struct ImuAverage {
    int n = 0;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float p = 0;   // [hPa]
};

// 打ち上げ〜着地の判定状態
struct FlightJudge {
    int    baroCalN = 0;
    double baroCalSum = 0;
    float  baseHpa = NAN;      // [hPa] BARO_CAL で取った基準気圧
    float  altRel = NAN;       // [m] 基準気圧からの相対高度
    int    launchCount = 0;    // LAUNCH 条件を満たした連続周期数
    float  landRefHpa = NAN;   // [hPa] 着地判定の基準気圧
    int    landCount = 0;      // 着地判定の基準から LAND_DP_HPA 以内にいる連続周期数
};

// 周期の処理時間の計測値（CSV と Telemetry に出す）
struct TickTiming {
    uint32_t tick_us = 0;   // [us] 前回の周期の処理時間（runTick 全体）
    uint32_t imu_us = 0;    // [us] この周期の sampleImu の最大時間
    uint32_t late_ms = 0;   // [ms] この周期の開始が予定より遅れた時間
    uint32_t overruns = 0;  // [回] 開始が1周期以上遅れた周期の累計
    uint32_t tx_drop = 0;   // [回] 送信キューがあふれて捨てた周期の累計
};

// 制御ループから送信タスクへ渡す1周期分のデータ
struct TelemetryMsg {
    TelemetryRecord rec;
    int phase;
    double d_hat, theta_e_deg, u;
    double goal_lat, goal_lon;   // 未登録なら NAN（送信時に省略される）
    bool gnss_upd, cog_upd;
    double pres_base, alt_rel;   // 未取得なら NAN
    int launch_cnt, land_cnt;
    TickTiming timing;
};

static Expander  expander;
static Sensor    sensor;
static GPS       gps;
static Encoder   encoder;
static Actuator  actuator;
static Deployer  deployer;
static Telemetry telemetry;
static KalmanFilter    ekf;
static MotorController motor;
static EkfConfig       ekfCfg;

static Phase    phase = WAIT_CMD;
static uint32_t phaseStartMs = 0;
static uint32_t lastImuMs = 0;
static uint32_t lastTickMs = 0;

static StaticAverage static0, static2, goalAvg;
static bool      goalSet = false;      // ゴールが登録済みか
static double    goalLat = NAN, goalLon = NAN;  // [deg] ゴール
static LatestFix latest;
static bool      fixPending = false;   // 前回の周期以降に新しい fix が届いたか
static ImuAverage imuSum;
static FlightJudge judge;
static float     lastPressure = NAN;   // [hPa] 直近の周期の気圧平均
static bool      calStopped = false;   // CAL_RUN の直進を止めて T_SETTLE_MS 待っている間 true
static TickTiming timing;
static QueueHandle_t telemetryQueue = nullptr;
static QueueHandle_t logQueue = nullptr;
static QueueHandle_t cmdQueue = nullptr;

// 送信タスク。ここで MQTT の送信待ちが続いても制御ループは止まらない
static void telemetryTask(void*) {
    for (;;) {
        TelemetryMsg msg;
        if (xQueueReceive(telemetryQueue, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
            telemetry.publishRecord(msg.rec);
            telemetry.sendTest("guidance", {
                {"phase", msg.phase},
                {"d_hat", msg.d_hat},
                {"theta_e_deg", msg.theta_e_deg},
                {"u", msg.u},
                {"goal_lat", msg.goal_lat},
                {"goal_lon", msg.goal_lon},
                {"gnss_upd", msg.gnss_upd},
                {"cog_upd", msg.cog_upd},
                {"tick_us", msg.timing.tick_us},
                {"imu_us", msg.timing.imu_us},
                {"late_ms", msg.timing.late_ms},
                {"overruns", msg.timing.overruns},
                {"tx_drop", msg.timing.tx_drop},
            });
            telemetry.sendTest("flight", {
                {"phase", msg.phase},
                {"pres_base", msg.pres_base},
                {"alt_rel", msg.alt_rel},
                {"launch_cnt", msg.launch_cnt},
                {"land_cnt", msg.land_cnt},
            });
        }
        char line[LOG_MSG_LEN];
        while (xQueueReceive(logQueue, line, 0) == pdTRUE) telemetry.log("%s", line);
    }
}

// telemetry.log() も MQTT の送信待ちで止まるので、制御ループからはキュー経由で出す
static void logEvent(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void logEvent(const char* fmt, ...) {
    char line[LOG_MSG_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    xQueueSend(logQueue, line, 0);
}

static void setPhase(Phase next) {
    phase = next;
    phaseStartMs = millis();
    logEvent("phase %s", PHASE_NAMES[next]);
}

// 両方のニクロム線を止める（abort・通電終了時）
static void stopBurn() {
    deployer.setRocket(false);
    deployer.setParachute(false);
}

// 相対高度は Sensor の起動時キャリブレーションではなく、BARO_CAL で取った基準気圧から計算する
// （起動からロケット搭載・打ち上げまで時間が空き、その間に気圧が変わるため）
static float relativeAltitude(float p) {
    return 44330.0f * (1.0f - powf(p / judge.baseHpa, 0.1903f));
}

// LAUNCH 確定 → ロケット分離の通電開始
static void confirmLaunch(const char* how) {
    logEvent("LAUNCH detected (%s): alt_rel=%.2f m", how, judge.altRel);
    setPhase(ROCKET_CUT);
    if (!deployer.setRocket(true)) logEvent("rocket nichrome ON failed (expander write)");
}

// 着地確定 → パラシュート分離の通電開始
static void confirmLanding(const char* how) {
    logEvent("LANDING detected (%s): p=%.2f hPa alt_rel=%.2f m", how, lastPressure, judge.altRel);
    setPhase(PARA_CUT);
    if (!deployer.setParachute(true)) logEvent("parachute nichrome ON failed (expander write)");
}

// MQTT の内部タスクから呼ばれる。ここでは積むだけにして、処理は制御ループの runTick() で行う
static void onCommand(const char* cmd) {
    char buf[Telemetry::COMMAND_MAX_LEN];
    strlcpy(buf, cmd, sizeof(buf));
    xQueueSend(cmdQueue, buf, 0);
}

static void registerGoal(double lat, double lon, const char* how) {
    goalLat = lat;
    goalLon = lon;
    goalSet = true;
    logEvent("goal registered (%s): %.7f, %.7f", how, lat, lon);
    setPhase(READY);
}

// 地上局コマンドを1つ処理する。受け付けられないものはログに出して捨てる
static void handleCommand(char* cmd) {
    char* verb = strtok(cmd, " \t\r\n");
    if (!verb) return;
    char* arg1 = strtok(nullptr, " \t\r\n");
    char* arg2 = strtok(nullptr, " \t\r\n");
    const bool beforeStart = (phase == WAIT_CMD || phase == READY);

    if (strcmp(verb, "goal") == 0) {
        if (!beforeStart) {
            logEvent("cmd goal rejected in %s", PHASE_NAMES[phase]);
        } else if (!arg1) {
            goalAvg = StaticAverage();
            setPhase(GOAL_FIX);
        } else {
            char *end1, *end2 = nullptr;
            const double lat = strtod(arg1, &end1);
            const double lon = arg2 ? strtod(arg2, &end2) : NAN;
            if (*end1 || !arg2 || *end2 || fabs(lat) > 90 || fabs(lon) > 180) {
                logEvent("cmd goal: bad args (use: goal <lat> <lon>)");
            } else {
                registerGoal(lat, lon, "manual");
            }
        }
    } else if (strcmp(verb, "start") == 0) {
        if (phase != READY || !goalSet) {
            logEvent("cmd start rejected in %s%s", PHASE_NAMES[phase], goalSet ? "" : " (no goal)");
        } else if (!sensor.isBmp280Ready()) {
            logEvent("cmd start rejected: BMP280 not ready");
        } else {
            judge = FlightJudge();
            setPhase(BARO_CAL);
        }
    } else if (strcmp(verb, "launch") == 0) {
        if (phase == LAUNCH_WAIT) confirmLaunch("forced");
        else logEvent("cmd launch rejected in %s", PHASE_NAMES[phase]);
    } else if (strcmp(verb, "land") == 0) {
        if (phase == LAND_WAIT) confirmLanding("forced");
        else logEvent("cmd land rejected in %s", PHASE_NAMES[phase]);
    } else if (strcmp(verb, "abort") == 0) {
        if (phase == GOAL_FIX) {
            logEvent("goal registration cancelled");
            setPhase(goalSet ? READY : WAIT_CMD);
        } else if (phase == BARO_CAL || phase == LAUNCH_WAIT) {
            logEvent("launch wait cancelled");
            setPhase(READY);
        } else if (!beforeStart && phase != POST && phase != DONE) {
            stopBurn();
            motor.stop();
            logEvent("ABORT");
            setPhase(POST);
        }
    } else {
        logEvent("unknown cmd: %s", verb);
    }
}

// getLat() などは TinyGPS の更新フラグを消すので、更新フラグを見て読むのはここだけにする。
// ログや EKF はこの latest を使う。
static void pollGps() {
    gps.update();
    if (gps.locationUpdated()) {
        latest.valid = true;
        latest.lat = gps.getLat();
        latest.lon = gps.getLon();
        latest.alt = gps.getAltitude();
    }
    // 位置は同じ測位の RMC と GGA の両方で更新されるので、新しい fix は RMC でだけ数える
    // （同じ測位を独立な観測として2回 EKF に入れないため）
    if (gps.courseUpdated()) {
        latest.cog_deg = gps.getCourseDeg();
        latest.sog = gps.isCourseValid() ? gps.getSpeedMps() : 0;  // 無効なら COG 更新を確実にスキップさせる
        fixPending = true;
    }
}

// MPU6050 の DLPF 21 Hz に対して 10 Hz で読むと折り返し雑音が入るので、10 ms ごとに読んで積算する
static void sampleImu() {
    sensor.update();
    imuSum.n++;
    imuSum.ax += sensor.getAccelX();
    imuSum.ay += sensor.getAccelY();
    imuSum.az += sensor.getAccelZ();
    imuSum.gx += sensor.getGyroX();
    imuSum.gy += sensor.getGyroY();
    imuSum.gz += sensor.getGyroZ();
    imuSum.p  += sensor.getPressure();
}

// 積算した値を平均して返し、次の周期のために積算をリセットする
static ImuAverage takeImuAverage() {
    ImuAverage avg = imuSum;
    if (avg.n > 0) {
        avg.ax /= avg.n;  avg.ay /= avg.n;  avg.az /= avg.n;
        avg.gx /= avg.n;  avg.gy /= avg.n;  avg.gz /= avg.n;
        avg.p  /= avg.n;
    }
    imuSum = ImuAverage();
    return avg;
}

static void startGuidance() {
    const int n = static0.samples + static2.samples;
    const double omegaSum = static0.omegaSum + static2.omegaSum;
    const double omegaSqSum = static0.omegaSqSum + static2.omegaSqSum;
    const double omegaVar = max(0.0, (omegaSqSum - omegaSum * omegaSum / n) / (n - 1));

    EkfCalibration cal;
    cal.lat_start = static0.lat();
    cal.lon_start = static0.lon();
    cal.lat_init  = static2.lat();
    cal.lon_init  = static2.lon();
    cal.b_g0 = (static0.omega() + static2.omega()) / 2;
    cal.b_a0 = static2.a();
    cal.sigma_bg0 = max(3 * sqrt(omegaVar) / sqrt((double)n), 1e-4);
    cal.sigma_ba0 = max(fabs(static0.a() - static2.a()), 0.05);
    ekf.ekf_setup(ekfCfg, cal);

    double goalN, goalE;
    ekf.latLonToLocal(goalLat, goalLon, goalN, goalE);
    motor.setGoal(goalN, goalE);

    // エンコーダの差分基準をそろえる
    encoder.getRightDeltaAngle();
    encoder.getLeftDeltaAngle();

    setPhase(GUIDE);
}

static void updateEkf(double a, double omega, double vR, double vL, bool hasFix) {
    GnssFix fix;
    fix.lat = latest.lat;
    fix.lon = latest.lon;
    fix.cog_deg = latest.cog_deg;
    fix.sog = latest.sog;
    ekf.ekf_update(a, omega, vR, vL, hasFix ? &fix : nullptr);
}

static void logCsv(double a, double omega, double vR, double vL) {
    const auto& P = ekf.getCovarianceMatrix();
    Serial.printf("%lu,%d,%.3f,%.3f,%.3f,%.2f,%.5f,%.6f,%.6g,%.6g,%.6g,%.3f,%.2f,%.3f,%.3f,%.3f,"
                  "%d,%d,%.3f,%.3f,%.4g,%.4g,%.2f,%.4g,%.4f,%.4f,%.4g,%.4g,%.4f,%.5f,"
                  "%.4f,%.4f,%.7f,%.7f,%.2f,%lu,%lu,%lu,%lu,%lu,"
                  "%.3f,%.3f,%.2f,%d,%.3f,%d\n",
                  (unsigned long)millis(), (int)phase,
                  ekf.getPositionN(), ekf.getPositionE(), ekf.getSpeed(), ekf.getAzimuth() * RAD_TO_DEG,
                  ekf.getAccelBias(), ekf.getGyroBias(),
                  P[KalmanFilter::PN][KalmanFilter::PN], P[KalmanFilter::PE][KalmanFilter::PE],
                  P[KalmanFilter::THETA][KalmanFilter::THETA],
                  motor.getDistance(), motor.getHeadingError() * RAD_TO_DEG, motor.getControl(),
                  motor.getDutyL(), motor.getDutyR(),
                  (int)ekf.gnss_updated, (int)ekf.cog_updated,
                  ekf.y_gnss[0], ekf.y_gnss[1], ekf.S_gnss[0], ekf.S_gnss[1],
                  ekf.y_cog * RAD_TO_DEG, ekf.S_cog,
                  ekf.y_enc[0], ekf.y_enc[1], ekf.S_enc[0], ekf.S_enc[1],
                  a, omega, vR, vL, latest.lat, latest.lon, latest.sog,
                  (unsigned long)timing.tick_us, (unsigned long)timing.imu_us, (unsigned long)timing.late_ms,
                  (unsigned long)timing.overruns, (unsigned long)timing.tx_drop,
                  lastPressure, judge.baseHpa, judge.altRel, judge.launchCount,
                  judge.landRefHpa, judge.landCount);
}

static void sendTelemetry(const ImuAverage& imu) {
    TelemetryMsg msg;
    TelemetryRecord& rec = msg.rec;
    if (latest.valid) {
        rec.pos.lat = latest.lat;
        rec.pos.lon = latest.lon;
        rec.pos.alt = latest.alt;
    }
    rec.imu.ax = imu.ax;  rec.imu.ay = imu.ay;  rec.imu.az = imu.az;
    rec.imu.gx = imu.gx;  rec.imu.gy = imu.gy;  rec.imu.gz = imu.gz;
    // 気圧は判定に使っているのと同じ 100 ms 平均を送る。高度は BARO_CAL 後は判定と同じ基準からの
    // 相対高度にする（それまでは Sensor の起動時キャリブレーション基準。どちらも地上が 0 m）
    rec.baro.pressure_hpa = lastPressure;
    rec.baro.alt_m = isnan(judge.baseHpa) ? sensor.getAltitude() : judge.altRel;
    rec.encoder.right_rev = encoder.getRightRevolutions() / TWO_PI;  // 戻り値は rad なので回転数に直す
    rec.encoder.left_rev  = encoder.getLeftRevolutions() / TWO_PI;
    rec.gnss.fix  = gps.isValid() ? 1 : 0;
    rec.gnss.hdop = gps.hdop();
    rec.gnss.sats = gps.satellites();
    if (ekf.isInitialized()) {
        double azimuthDeg = ekf.getAzimuth() * RAD_TO_DEG;
        if (azimuthDeg < 0) azimuthDeg += 360.0;
        rec.ekf.position_N = ekf.getPositionN();
        rec.ekf.position_E = ekf.getPositionE();
        rec.ekf.speed      = ekf.getSpeed();
        rec.ekf.azimuth    = azimuthDeg;
        rec.ekf.bias_accel = ekf.getAccelBias();
        rec.ekf.bias_gyro  = ekf.getGyroBias();
    }
    rec.motor.duty_l = motor.getDutyL();
    rec.motor.duty_r = motor.getDutyR();

    msg.phase = (int)phase;
    msg.d_hat = motor.getDistance();
    msg.theta_e_deg = motor.getHeadingError() * RAD_TO_DEG;
    msg.u = motor.getControl();
    msg.goal_lat = goalLat;
    msg.goal_lon = goalLon;
    msg.gnss_upd = ekf.gnss_updated;
    msg.cog_upd = ekf.cog_updated;
    msg.pres_base = judge.baseHpa;
    msg.alt_rel = judge.altRel;
    msg.launch_cnt = judge.launchCount;
    msg.land_cnt = judge.landCount;
    msg.timing = timing;
    if (xQueueSend(telemetryQueue, &msg, 0) != pdTRUE) timing.tx_drop++;
}

// 100 ms ごとの処理：フェーズの状態遷移 → CSV・Telemetry
static void runTick() {
    const ImuAverage imu = takeImuAverage();
    const double a = ACCEL_FWD_SIGN * imu.ay;
    const double omega = -imu.gz;  // MPU6050 の gz は反時計回り正なので −1 倍して時計回り正にする
    const double vR = ekf.wheelSpeed(encoder.getRightDeltaAngle());
    const double vL = ekf.wheelSpeed(encoder.getLeftDeltaAngle());
    const bool hasFix = fixPending;
    fixPending = false;
    if (imu.n > 0) lastPressure = imu.p;
    if (!isnan(judge.baseHpa)) judge.altRel = relativeAltitude(lastPressure);

    char cmd[Telemetry::COMMAND_MAX_LEN];
    while (xQueueReceive(cmdQueue, cmd, 0) == pdTRUE) handleCommand(cmd);
    const uint32_t elapsed = millis() - phaseStartMs;

    switch (phase) {
    case WAIT_CMD:  // 地上局の goal / start を待つだけ
    case READY:
        break;

    case GOAL_FIX:
        if (hasFix && gps.isValid()) goalAvg.addFix(latest.lat, latest.lon);
        if (goalAvg.fixes >= ekfCfg.N_avg) {
            char how[24];
            snprintf(how, sizeof(how), "GPS avg of %d", goalAvg.fixes);
            registerGoal(goalAvg.lat(), goalAvg.lon(), how);
        }
        break;

    case BARO_CAL:
        judge.baroCalSum += lastPressure;
        judge.baroCalN++;
        if (judge.baroCalN >= BARO_CAL_TICKS) {
            judge.baseHpa = judge.baroCalSum / judge.baroCalN;
            judge.altRel = relativeAltitude(lastPressure);
            logEvent("baro base = %.2f hPa (avg of %d)", judge.baseHpa, judge.baroCalN);
            setPhase(LAUNCH_WAIT);
        }
        break;

    case LAUNCH_WAIT:  // test_judge_launch と同じ判定
        judge.launchCount = (judge.altRel > LAUNCH_ALT_THRESHOLD_M) ? judge.launchCount + 1 : 0;
        if (judge.launchCount >= LAUNCH_CONFIRM_TICKS) confirmLaunch("baro");
        break;

    case ROCKET_CUT:
        if (elapsed >= BURN_MS) {
            deployer.setRocket(false);
            logEvent("rocket nichrome OFF");
            judge.landRefHpa = lastPressure;
            judge.landCount = 0;
            setPhase(LAND_WAIT);
        }
        break;

    case LAND_WAIT:
        // 基準から LAND_DP_HPA を超えて動いたら、いまの気圧を新しい基準にして数え直す
        if (fabsf(lastPressure - judge.landRefHpa) > LAND_DP_HPA) {
            judge.landRefHpa = lastPressure;
            judge.landCount = 0;
        } else if (++judge.landCount >= LAND_CONFIRM_TICKS) {
            confirmLanding("baro");
        }
        break;

    case PARA_CUT:
        if (elapsed >= BURN_MS) {
            deployer.setParachute(false);
            logEvent("parachute nichrome OFF");
            setPhase(WAIT_FIX);
        }
        break;

    case WAIT_FIX:
        if (gps.isValid()) setPhase(STATIC_0);
        break;

    case STATIC_0:
        static0.addImu(a, omega);
        if (hasFix) static0.addFix(latest.lat, latest.lon);
        if (static0.fixes >= ekfCfg.N_avg) {
            setPhase(CAL_RUN);
            motor.setDuty(CAL_DUTY, CAL_DUTY);
        }
        break;

    case CAL_RUN:
        if (!calStopped) {
            if (elapsed >= T_CAL_MS) {
                motor.setDuty(0, 0);
                calStopped = true;
                phaseStartMs = millis();
            }
        } else if (elapsed >= T_SETTLE_MS) {
            setPhase(STATIC_2);
        }
        break;

    case STATIC_2:
        static2.addImu(a, omega);
        if (hasFix) static2.addFix(latest.lat, latest.lon);
        if (static2.fixes >= ekfCfg.N_avg) startGuidance();
        break;

    case GUIDE:
        updateEkf(a, omega, vR, vL, hasFix);
        motor.updateGuidance(ekf, omega);
        if (motor.isStopped()) {
            setPhase(POST);
        } else if (elapsed >= T_MAX_MS) {
            motor.stop();
            logEvent("TIMEOUT");
            setPhase(POST);
        }
        break;

    case POST:
        if (ekf.isInitialized()) updateEkf(a, omega, vR, vL, hasFix);  // 誘導前の abort では EKF 未初期化
        if (elapsed >= T_POST_MS) setPhase(DONE);
        break;

    case DONE:  // CSV は止めるが、地上局で生存と最終位置が見えるよう Telemetry は送り続ける
        sendTelemetry(imu);
        return;
    }

    logCsv(a, omega, vR, vL);
    sendTelemetry(imu);
}

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    // コマンドは接続直後から届きうるので、キューを作ってから begin() する
    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(TelemetryMsg));
    logQueue = xQueueCreate(LOG_QUEUE_LEN, LOG_MSG_LEN);
    cmdQueue = xQueueCreate(CMD_QUEUE_LEN, Telemetry::COMMAND_MAX_LEN);

    Telemetry::Config telemetryCfg;
    telemetryCfg.ssid     = WIFI_SSID;
    telemetryCfg.password = WIFI_PASSWORD;
    telemetryCfg.mqttHost = MQTT_HOST;
    telemetry.onCommand(onCommand);
    telemetry.begin(telemetryCfg);

    xTaskCreatePinnedToCore(telemetryTask, "telemetry-tx", 6144, nullptr, 1, nullptr, 0);  // Wi-Fi と同じ Core 0

    if (!expander.setup()) logEvent("[ALL] Expander init failed (motor direction / nichrome will not work)");
    actuator.begin(expander);
    deployer.begin(expander);  // enableOutput で OUTPUT+LOW になる（起動直後に通電しない）

    MotorControlConfig motorCfg;
    motorCfg.Kp = KP;
    motorCfg.Ki = KI;
    motorCfg.Kd = KD;
    motorCfg.motorSignL = MOTOR_SIGN_L;
    motorCfg.motorSignR = MOTOR_SIGN_R;
    motor.begin(actuator, motorCfg);

    if (!sensor.begin()) logEvent("[ALL] sensor init failed (BMP280=%d MPU6050=%d)",
                                  sensor.isBmp280Ready(), sensor.isMpu6050Ready());
    gps.begin();
    encoder.begin();
    encoder.setDirection(ENC_SIGN_R, ENC_SIGN_L);

    Serial.println(CSV_HEADER);

    lastImuMs = lastTickMs = phaseStartMs = millis();
    logEvent("phase %s", PHASE_NAMES[phase]);
}

void loop() {
    pollGps();  // どの状態でも毎ループ呼ぶ（DONE でもモーター 0・ニクロム線 OFF のまま計測と送信だけ続ける）

    const uint32_t now = millis();
    if (now - lastImuMs >= IMU_PERIOD_MS) {
        lastImuMs += IMU_PERIOD_MS;
        const uint32_t startUs = micros();
        sampleImu();
        timing.imu_us = max<uint32_t>(timing.imu_us, micros() - startUs);
    }
    if (now - lastTickMs >= TICK_PERIOD_MS) {
        lastTickMs += TICK_PERIOD_MS;  // 処理落ちしても累積遅れしないようにする
        timing.late_ms = millis() - lastTickMs;
        if (timing.late_ms >= TICK_PERIOD_MS) timing.overruns++;
        const uint32_t startUs = micros();
        runTick();
        timing.tick_us = micros() - startUs;
        timing.imu_us = 0;
    }
}
