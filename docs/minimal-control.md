# 最小制御例（STS3215 / Raspberry Pi 5）

実装は `examples/raspberry_pi/minimal_control.cpp`、ビルド先は `stservo_minimal` です。
`id`、`goal_position`、`goal_speed` の定数を変更して使います。
単回転・位置分解能1・角度制限0〜4095の設定を想定します。位置2048は約180°です。
速度はrpmではなくレジスタ値で、単位はPhase（0x12）BIT2と機種の設定に依存します。

```sh
cmake --build build/linux -j2
./build/linux/stservo_minimal /dev/ttyAMA0 position
./build/linux/stservo_minimal /dev/ttyAMA0 speed
```

2つのコマンドは別の例です。`position` は位置モード0で位置2048へ、移動速度500で指令します。
`speed` は速度モード1で速度500を指令します。負速度にするには `goal_speed` を負数へ変更します。
この実行例は実際にトルクを有効化して動作指令を送ります。速度モードは終了後も回転を継続します。
停止は、同じPhase設定のまま速度0を送るか、トルクスイッチ0x28へ0を書きます。
後者は位置保持も解除します。現在値の読取は瞬時値で、目標到達を待つ処理ではありません。

| 内容 | レジスタ |
|---|---|
| 応答設定（WRITEのACK有無） | 0x08、1バイト |
| Phase（速度0の扱い・速度単位など） | 0x12、1バイト |
| 動作モード（0=位置、1=速度） | 0x21、1バイト |
| トルク（0=無効、1=有効） | 0x28、1バイト |
| 目標位置 | 0x2A、2バイト |
| 目標速度（位置モードでは移動速度） | 0x2E、2バイト |
| EEPROMロック | 0x37、1バイト |
| 現在位置 | 0x38、2バイト |
| 現在速度 | 0x3A、2バイト |

2バイト値は下位バイトから送受信します。現在位置・現在速度の符号はbit15で、2の補数ではありません。
最小例は4バイトまとめて読み、前半を位置・後半を速度として変換します。
[FEETECHのSTS制御表](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)

変更する設定がある場合だけ、トルクを切った後に0x37へ0を書き、モードとPhaseを書き、0x37へ1を書きます。
設定の各書込みは読戻し確認付きです。速度モードではPhaseの他ビットを保持してBIT3だけ0にします。
これにより速度0を停止として使えます。設定が既に一致する場合、EEPROMの設定を書き直しません。
WRITEのACK有無は0x08から読み、応答方針を合わせます。コアの変更は不要です。

STM32では冒頭の `PosixTransport` の代わりに、既存の `STM32Transport` と初期化済みUARTを使います。
`STServo bus(...)` より後の読取・書込操作は共通です。
