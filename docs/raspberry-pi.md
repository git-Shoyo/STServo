# Raspberry Pi 5 / Raspberry Pi OS

## 対象と接続

`PosixTransport` はLinuxのUART/USBシリアルデバイスをraw 8N1で開きます。
サーボバスへの半二重変換は、既に動作確認した外付けドライバに任せます。
PiのTX/RXを1本のDATA線へ直接短絡して使う実装ではありません。
GPIOのロジック電圧は3.3Vです。サーボ電源とロジック電源を区別し、共通GNDを接続してください。
このワークスペースでは、Seeed基板のジャンパ・はんだジャンパを開放した構成が動作確認されています。
その実機設定を維持し、異なる基板モードでは回路図に従って入出力方向を確認してください。

## Pi 5のUARTを選ぶ

Pi 5では `/dev/serial0` が専用デバッグUART `/dev/ttyAMA10` を指す場合があります。
40ピンヘッダーのUARTと同一だと仮定せず、実際に使用するデバイスを引数で指定します。
[Raspberry Pi公式UART資料](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts)

GPIO14（物理8番/TX）・GPIO15（物理10番/RX）でUART0を使う場合、
Pi 5用の `uart0-pi5` overlayを利用できます。Raspberry Pi OSの `/boot/firmware/config.txt` に設定し、再起動します。
使用UARTをシリアルコンソールや他プロセスと共用しないでください。
[公式overlay定義](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README)

```ini
[pi5]
dtoverlay=uart0-pi5
[all]
```

デバイス名は環境で確認します。

```sh
readlink -f /dev/serial0
ls -l /dev/ttyAMA* /dev/serial/by-id/ 2>/dev/null
```

GPIO UART0の例は `/dev/ttyAMA0`、USBシリアルを使う場合は該当する `/dev/serial/by-id/...` 等を指定します。
デバイスがない場合はハードウェア/OS設定を確認し、権限エラーはその環境のシリアルデバイス権限に従って対応してください。

## ビルドとCLI

`shared/STServo` をPiへコピーして、そのディレクトリで実行します。
依存はC++11対応g++、CMake、make、テスト用Python 3です。コア/アダプターは外部C++ライブラリを必要としません。

```sh
cmake -S . -B build/linux -DCMAKE_BUILD_TYPE=Release
cmake --build build/linux -j2
ctest --test-dir build/linux --output-on-failure
./build/linux/stservo_ping --device /dev/ttyAMA0 --baud 1000000 --timeout 100 1 2
```

```text
id=1 result=OK status=1 err=0x00
id=1 read_id=OK status=1 err=0x00 value=1
id=2 result=OK status=1 err=0x00
id=2 read_id=OK status=1 err=0x00 value=2
responding=2 queried=2
```

上記出力は正常時の例です。CLIはPINGとIDレジスタ読取しか送らず、動作/設定書き込みはしません。
終了コードは、0=指定IDがすべて正常、1=引数/ポート設定失敗、2=タイムアウト/通信/サーボエラーです。
`--scan` は0〜253を順に調べ、1台でも有効応答があれば0、見つからなければ2です。
走査時の正常状態は各行のERRORとREAD結果で確認してください。

```sh
./build/linux/stservo_ping --device /dev/ttyAMA0 --scan
```

ローカルエコーが必ず1フレーム戻る回路では `--echo` を付けます。
現在のエコーなし構成では付けません。
POSIXアダプターの対応速度は、OSのtermios定数で表せる38400、57600、115200、
および利用可能なら76800、128000、250000、500000、1000000です。
未対応の速度はEINVALで拒否します（任意ボーレートのBOTHER設定は実装していません）。

## アプリケーションへ組み込む

```cpp
#include <STServo/stservo.hpp>
#include "stservo_posix.hpp"

stservo::PosixTransport port;
if (!port.open("/dev/ttyAMA0", 1000000)) {
    // port.errorNumber() に errno。
    return 1;
}
STServo bus(port, 100);
auto id1 = bus.ping(1);
auto id2 = bus.ping(2);
```

CMakeではこのディレクトリを `add_subdirectory` し、アプリを `stservo_posix` にリンクします。
LinuxアダプターはOSの送信バッファへ全フレームが入った時点でwrite成功を返します。
受信は非ブロッキング、待機はsteady_clockと短いsleepで行います。
TTYの元設定はclose/destructorで戻し、FDはRAIIで閉じます。

## 検証範囲

STM32F303K8向けビルド、Linux（WSL Ubuntu）の実termios/PTYを使う複数ID通信テストが対象です。
Pi 5実機や現在接続されているサーボへの書き込み・動作テストは、この変更では実施していません。
