# Ardupilot-UmemotoLab-ekf-V2

ArduPilot ベースのカスタムファームウェアです。標準の ArduPilot に **AP_Observer（外乱推定・揺れ抑制）機能**を追加したリポジトリです。

---

## AP_Observer とは

ドローンが荷物（ペイロード）を吊り下げて飛行する際、荷物の揺れが機体に周期的な外乱として伝わります。AP_Observer は、この外乱を**拡張カルマンフィルタ（EKF）**で推定し、機体の姿勢を傾けて揺れを能動的に抑制する機能です。

- **センサレス**: ペイロード側にセンサを付けず、機体側の既存 IMU とモータースロットルのみで推定します。
- **周波数自動同定**: 荷物の紐の長さ（＝揺れの周波数）が未知でも、EKF が自動で周波数を推定します。
- **ロバスト性**: 突風・衝撃などの外乱に対して、Innovation Clipping と出力フェード機構で発散を防ぎます。

### 数理モデル

外力の振動成分を**調和振動子**（$\ddot d = -\omega^2 d$）としてモデル化し、各軸ごとに以下の4状態を EKF で推定します。

$$
\mathbf{x}_k = [d,\ \dot{d},\ c,\ \omega]^\top
$$

| 状態 | 意味 |
|------|------|
| $d$ | 振動位置（周期性外乱の正弦波成分） |
| $\dot{d}$ | 振動速度 |
| $c$ | DC バイアス（重心ずれ・定常風などの一定外力） |
| $\omega$ | 角周波数（時変パラメータとして推定） |

観測モデルは $z = d + c$（振動成分＋バイアスの線形和）です。

### 処理フロー

1. モータースロットルと加速度センサから外力 $z$ を構成
2. X・Y軸に対してシンプレクティック・オイラー法で EKF 予測（Z軸はリソース節約のためスキップ）
3. イノベーションクリッピング後、EKF 観測更新（NaN 防御付き）
4. 推定振幅から出力フェード機構でゲイン制御
5. 予測ホライズン先の外力を予測
6. 予測外力から補正オイラー角（ロール・ピッチ）を生成
7. 姿勢制御ループへ補正を適用し、揺れを抑制

### 主要パラメータ（`OBS_` グループ）

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `OBS_CORR_GAIN` | 0.0 | 姿勢補正ゲイン |
| `OBS_MAX_CORR_ANG` | 0.5 | 補正角度の上限 [rad] |
| `OBS_PRED_TIME` | 0.0 | 外力予測ホライズン [s] |
| `OBS_EKF_W_INIT` | 3.7699 | 初期角周波数 [rad/s] |
| `OBS_EKF_W_MIN` / `MAX` | 2.1991 / 5.718 | 角周波数の上下限 [rad/s] |
| `OBS_EKF_Q_*` | - | プロセスノイズ分散 |
| `OBS_EKF_R_MEAS` | 46.0 | 観測ノイズ分散 |
| `OBS_EKF_INN_MAX` | 0.70 | イノベーションクリッピング上限 [N] |
| `OBS_FADE_TH` | 1.0 | 出力フェード振幅閾値 [N] |
| `OBS_FADE_DLY` | 2.0 | フェード開始遅延 [s] |
| `OBS_FADE_IN_T` / `OUT_T` | 0.1 / 1.0 | フェード時定数 [s] |
| `OBS_W_FREEZE` | 0 | ω を初期値に固定（1=有効） |

### RC スイッチ操作

- **CH7**: AP_Observer 制御 ON/OFF（HIGH=ON）
- **CH8**: EKF リセット（HIGH→LOW で発火）

### ログ

- `OBSV`: 時刻、外力、予測外力、推定周波数、補正角、制御状態
- `OBEK`: 各軸の EKF 診断値（共分散、カルマンゲイン等）

---

## このリポジトリで追加・変更した内容

標準の ArduPilot に対して、以下の AP_Observer 機能を移植・追加しました。

### 新規追加

| パス | 内容 |
|------|------|
| `libraries/AP_Observer/` | AP_Observer 本体（`AP_Observer.h` / `AP_Observer.cpp` / `wscript`） |

### 変更ファイル

| パス | 内容 |
|------|------|
| `ArduCopter/Copter.h` | `observer` メンバ・アクセサ・`observer_update()` 宣言 |
| `ArduCopter/Copter.cpp` | 100Hz スケジューラ登録、`observer_update()` 実装 |
| `ArduCopter/Parameters.h` / `.cpp` | `OBS_` パラメータグループ登録 |
| `ArduCopter/defines.h` | `LOG_OBSERVER_MSG` 追加 |
| `ArduCopter/system.cpp` | `observer.init()` 追加 |
| `ArduCopter/UserCode.cpp` | CH7/CH8 による制御 ON/OFF・EKF リセット |
| `ArduCopter/wscript` | `AP_Observer` ライブラリ登録 |
| `libraries/AC_AttitudeControl/` | `set_correction_euler()` 追加、`attitude_controller_run_quat()` に補正適用ロジック |
| `Tools/autotest/` | Observer オートテスト（`test.CopterObserver` / `test.CopterMedium`）追加 |

### ビルド

```sh
# SITL
./waf configure --board sitl
./waf copter

# 実機（Pixhawk6C 等）
./waf configure --board Pixhawk6C
./waf copter
```

### オートテスト

```sh
# Observer 専用テスト（Lite）
./Tools/autotest/autotest.py build.Copter test.CopterObserver

# Observer + コア機能テスト（Medium）
./Tools/autotest/autotest.py build.Copter test.CopterMedium
```

---

本リポジトリは [ArduPilot](https://github.com/ArduPilot/ardupilot) をベースとしており、ライセンスは **GNU General Public License v3** です。
