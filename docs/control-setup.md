# トルク・位置モード・ID変更

`STServo bus(transport, 100);` を初期化した後、次のAPIを使用します。STM32・Linux共通です。

| 操作 | API | 書込先 |
|---|---|---|
| トルクON | `bus.setTorqueEnabled(id, true)` | 0x28 = 1 |
| トルクOFF | `bus.setTorqueEnabled(id, false)` | 0x28 = 0 |
| 位置モード | `bus.setPositionMode(id)` | 0x21 = 0 |
| 目標位置 | `bus.setPosition(id, position)` | 0x2A～0x2B |
| ID変更 | `bus.setId(old_id, new_id)` | 0x05 = new_id |
| EEPROM解除 | `bus.setEepromLocked(id, false)` | 0x37 = 0 |
| EEPROM再ロック | `bus.setEepromLocked(id, true)` | 0x37 = 1 |

## 応答と失敗の扱い

既定ではACKを待ちません。`reply.ok()` は送信受付だけを示します。
WRITE応答レベル（0x08）が1なら、`setId()`以外では最後の引数に
`STServo::WriteResponse::Required` を指定できます。
その場合は `r.ok() && r.status_received && r.servo_error == 0` を確認してください。
応答レベル0ならACKは返らないため、既定のACKなし送信とREADでの読戻しを使います。
コンパイル対象の [使用例](../examples/control_setup.cpp) は両方の応答レベルに対応し、
各設定を読み戻して確認します。失敗を無限に再試行しないでください。

## 位置モードの手順

1. `setTorqueEnabled(id, false)` で出力をOFFにし、0x28を読み戻す。
2. `setEepromLocked(id, false)` でEEPROMへの保存を有効にする。
3. `setPositionMode(id)` を実行し、0x21が0か読み戻す。
4. `setEepromLocked(id, true)` で再ロックする。途中で失敗しても再ロックを試みる。
5. `setVelocity(id, speed)` と `setPosition(id, position)` で移動速度・目標位置を設定する。
6. 設定成功を確認してから `setTorqueEnabled(id, true)` で動作を開始する。

`positionModeExample(bus, id)` は手順1～4を実装しています。成功後もトルクはOFFです。
モード関数自体はトルク・EEPROMロック・目標値・角度制限・分解能を変更しません。
通常の分解能1・単回転設定では目標位置0～4095、2048が約180°です。
設定を保存する処理は制御ループの外で実行してください。

## 目標位置の指定

位置モード設定後、`bus.setPosition(id, 2048)` で目標位置を送信します。
引数は0～4095で、範囲外は送信せず `INVALID_ARGUMENT` を返します。
内部で2バイトのリトルエンディアンに変換します。多回転・符号付き目標には `writeRegister()` を使ってください。
この関数自体は動作モード、移動速度、トルク、角度制限、分解能を変更しません。

```cpp
// 位置モード・トルクON・WRITE応答レベル1を設定済みの場合
const auto r = bus.setPosition(1, 2048, STServo::WriteResponse::Required);
if (!r.ok() || !r.status_received || r.servo_error != 0) {
    // 通信失敗またはサーボ異常を処理する。
}
```

動作中も同じ関数で目標を更新でき、毎回トルクを切る必要はありません。
正常ACKは指令への応答であり、到達を意味しません。到達確認には現在位置などを読み取ります。
応答レベル0の場合はACKを要求せず、0x2Aから2バイトを読んで目標レジスタを確認してください。

## ID変更の手順

対象を1台だけ接続して実行してください。IDは0～253です。
`setId()`はブロードキャスト元ID・変更先IDを拒否します。

1. 旧IDでトルクOFFとEEPROM解除を行う。
2. `bus.setId(old_id, new_id)` を実行する。
3. 新ID宛てに0x05をREADし、変更先IDと一致することを確認する。
4. 新ID宛てにEEPROM再ロックを実行し、0x37が1か読み戻す。
5. 以降の通信では新IDを指定する。バスインスタンスの作り直しは不要。

`changeIdExample(bus, 1, 3)` がこの処理を実装します。ID書込ACKは待ちません。
変更前後どちらのIDでACKが返るかに依存せず、新IDのREADをもって成功確認します。
失敗しても既にIDが変わっている場合があります。旧ID・新IDの両方を確認してください。
失敗経路では再ロックを試みますが、通信断時の再ロック成功は保証できません。

レジスタ定義は [FEETECH STS仕様](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/) に基づきます。
