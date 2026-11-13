# STServo 2.0

STM32とRaspberry Pi 5/Linuxで同じAPIを使う、FT-SCS/STSシリアルバスサーボ用C++11ライブラリです。
1つのバスに対して各操作のIDを指定します。IDごとの事前登録やインスタンスの作り直しは不要です。

```cpp
#include <STServo/stservo.hpp>

// transport は初期化済みの stservo::Transport 派生オブジェクト。
STServo bus(transport, 100);
auto first = bus.ping(1);
auto second = bus.ping(2);
if (first.ok() && first.status_received && first.servo_error == 0) {
    // ID1から有効な正常応答を受信。
}
```

## 構成

| ファイル | 責務 |
|---|---|
| `src/STServo/stservo.hpp` | 共通公開API、純粋仮想 `stservo::Transport`、ID付き `Reply` |
| `src/stservo.cpp` | フレーム生成、チェックサム、受信解析、タイムアウト、複数ID/Sync Write |
| `adapters/posix/stservo_posix.hpp/.cpp` | Raspberry Pi/Linuxのtermios・非ブロッキングI/O・単調時計 |
| `../../stservo/include/stm32_stservo_transport.hpp` と対応する `src/*.cpp` | このワークスペースのSTM32 HALアダプター |
| `examples/raspberry_pi/main.cpp` | 複数IDのPING/ID読取・ID走査CLI。動作指令は送らない |
| `examples/multi_servo.cpp` | 共通APIでの複数ID・速度Sync Writeの例 |

- [Raspberry Pi 5で使う](docs/raspberry-pi.md)
- [新しいプラットフォームへ移植する](docs/porting.md)
- [旧stservo.hppから移行する](docs/migration.md)
- [プロトコルと制約](docs/protocol.md)

## API

| 操作 | 戻り値 | 内容 |
|---|---|---|
| `ping(id)` | `Reply` | 0〜253の指定IDへPING |
| `readRegister(id, address, output, size)` | `Reply` | 連続レジスタ読取 |
| `writeRegister(id, address, data, size, policy)` | `Reply` | WRITE。既定はACKを待たない |
| `setVelocityMode(id, policy)` | `Reply` | STSの速度モード設定 |
| `setPosition(id, position, policy)` | `Reply` | 単回転目標位置（0～4095）の送信 |
| `setVelocity(id, signed_speed, policy)` | `Reply` | 符号付き速度の送信 |
| `syncWriteRegister(address, width, ids, data, count)` | `Result` | 複数IDを1フレームで更新 |
| `syncSetVelocity(ids, velocities, count)` | `Result` | 複数台へそれぞれ異なる速度を同時送信 |

```cpp
const std::uint8_t ids[] = {1, 2};
const std::int32_t speeds[] = {500, -500};
// 各IDの速度モードは別途設定・確認しておく。
auto sent = bus.syncSetVelocity(ids, speeds, 2);
// sent == Result::OK はローカル送信受付。各サーボの受理確認ではない。
```

### 通信結果とサーボのERRORを分ける

`Reply` は `id`, `result`, `status_received`, `servo_error`, `parameter_count` を持ちます。
`ok()` は **通信処理が成功したかだけ** を表します。サーボの正常応答は次の3条件で確認します。

```cpp
reply.ok() && reply.status_received && reply.servo_error == 0
```

ACKなしWRITEとSync Writeは、送信を受け付けてもサーボの正常状態を保証しません。
ACKが必要なら `WriteResponse::Required` を指定し、サーボ側の応答設定と合わせます。
READで非ゼロERRORを受けた場合、出力バッファは変更せずエラー値を返します。
別IDの結果が上書きされるグローバルな `getServoError()` は使いません。

### エコー

動作確認済みの、ジャンパ・はんだジャンパを開放した現在の構成は `EchoMode::None`（既定）を使います。
電気的に送信データが必ず1回戻る構成だけ、コンストラクターへ `EchoMode::LocalEcho` を渡します。
このモードでは、送信と一致する全バイトを1フレーム分消費してからサーボ応答を解析します。
ACKなしWRITE/Sync Writeでも、LocalEcho設定時は自分の送信フレームを消費してから戻ります。
応答を要求したのにエコーだけなら `ECHO_ONLY`、想定したエコーと不一致なら `ECHO_ERROR` です。

PING命令の自己受信は、本物のERROR=1応答と同じバイト列になります。
そのため、ERROR=1を無条件に捨てる処理やエコーの自動推測は行いません。

## ビルド・検証

Linux/Raspberry Pi OS（g++、CMake、make、Python 3を用意）:

```sh
cmake -S . -B build/linux -DCMAKE_BUILD_TYPE=Debug
cmake --build build/linux -j2
ctest --test-dir build/linux --output-on-failure
```

Windowsでも共通ライブラリとプロトコルテストをCMakeでビルドできます。
POSIXアダプター・CLI・PTYテストはUNIX環境のみでビルドします。
STM32はワークスペースルートから `pio run -d stservo` を実行します。

テストは、複数IDの結果分離、正しい送信列、破損/別ID/分割受信、ACK方針、
エコーと本物のERROR=1、Tick周回、最大長、符号付き速度、I/Oエラーを確認します。
Linuxでは実際のtermiosと擬似端末（PTY）でCLIまで実行します。
Linux PTY検証は実機サーボ・Pi 5のUARTでの電気的な動作確認を代替しません。

## Doxygen

公開API・通信アダプターの説明と、生成される画面の見出しは日本語です。入力文字コードはUTF-8です。Doxygenをインストールした環境で:

```sh
doxygen Doxyfile
# または
cmake --build build/linux --target docs
```

Doxygen 1.9.8では日本語翻訳が古い旨の警告が出ます。APIの説明は日本語ですが、自動生成される定型文の一部は英語のままになる場合があります。

生成先は `build/docs/html/index.html` です。README、移植手順、Raspberry Pi手順も含みます。
共通コードにHALヘッダー・OS API・ログ出力・動的メモリ確保は含めていません。

ワークスペース全体を使う場合は、`doxygen Doxyfile.stm32` またはCMakeの `docs-stm32` ターゲットで
STM32アダプターも含めたAPIを生成できます。出力は `build/docs-stm32/html/index.html` です。

## 最小の設定・現在値・目標値の例

[最小コード](examples/raspberry_pi/minimal_control.cpp) は、応答設定と動作モードの読取、
必要時だけのEEPROM書込・再ロック・読戻し確認、トルク制御、現在位置/速度の読取を含みます。
使用例は [最小制御手順](docs/minimal-control.md) を参照してください。

## トルク・位置モード・ID変更

`setTorqueEnabled(id, bool)`、`setPositionMode(id)`、`setId(old_id, new_id)`、
`setEepromLocked(id, bool)` を使用できます。
[使用手順と応答確認](docs/control-setup.md)、[コンパイル可能な使用例](examples/control_setup.cpp) を参照してください。

位置モードの目標は `bus.setPosition(id, 2048)` で指定できます。通常の分解能1では約180°です。
