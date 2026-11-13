/** @file control_setup.cpp @brief トルク・位置モード・ID変更の使用例。明示的に呼び出したときだけ実行します。 */
#include "STServo/stservo.hpp"

namespace {
bool readEquals(STServo& bus, std::uint8_t id, std::uint8_t address, std::uint8_t expected) {
    std::uint8_t actual = 0;
    const auto r = bus.readRegister(id, address, &actual, 1);
    return r.ok() && r.status_received && r.servo_error == 0 && actual == expected;
}
bool verify(STServo& bus, const STServo::Reply& sent, std::uint8_t address, std::uint8_t expected) {
    return sent.ok() && readEquals(bus, sent.id, address, expected);
}
}

// 応答レベル0/1の両方で使えるよう、WRITEはACKなし、READで結果確認。
bool torqueExample(STServo& bus, std::uint8_t id, bool enabled) {
    return verify(bus, bus.setTorqueEnabled(id, enabled), 0x28, enabled ? 1 : 0);
}

// トルクOFFのまま位置モードを保存する。目標設定とトルクONは別途行う。
bool positionModeExample(STServo& bus, std::uint8_t id) {
    if (!torqueExample(bus, id, false)) return false;
    const bool unlocked = verify(bus, bus.setEepromLocked(id, false), 0x37, 0);
    const bool configured = unlocked && verify(bus, bus.setPositionMode(id), 0x21, 0);
    const bool locked = verify(bus, bus.setEepromLocked(id, true), 0x37, 1);
    return configured && locked;
}

// 対象サーボ1台だけを接続して実行する。成功後もトルクはOFF。
// 失敗時には旧ID・新IDの両方を調べる。送信結果だけでID変更を確定しない。
bool changeIdExample(STServo& bus, std::uint8_t old_id, std::uint8_t new_id) {
    if (old_id > STServo::MaxId || new_id > STServo::MaxId) return false;
    if (!torqueExample(bus, old_id, false)) return false;
    if (!verify(bus, bus.setEepromLocked(old_id, false), 0x37, 0)) {
        bus.setEepromLocked(old_id, true);
        return false;
    }
    const auto sent = bus.setId(old_id, new_id);
    if (sent.ok() && readEquals(bus, new_id, 0x05, new_id))
        return verify(bus, bus.setEepromLocked(new_id, true), 0x37, 1);
    // 応答喪失時は変更済みの場合もあるため、両IDで再ロックを試みる。
    bus.setEepromLocked(old_id, true);
    bus.setEepromLocked(new_id, true);
    return false;
}

// 位置モード設定済み・WRITE応答レベル1を前提とする。正常ACKは到達確認ではない。
// この例は目標位置を設定するだけで、トルクON/OFFや移動速度は変更しない。
bool positionTargetExample(STServo& bus, std::uint8_t id, std::uint16_t position) {
    const auto r = bus.setPosition(id, position, STServo::WriteResponse::Required);
    return r.ok() && r.status_received && r.servo_error == 0;
}

// 無負荷・静止状態で呼び出す。原点の内部値は2048。成功後もトルクはOFF。
// WRITE応答レベル0/1の両方に対応する。失敗時は自動で再校正しない。
bool originExample(STServo& bus, std::uint8_t id) {
    if (id > STServo::MaxId || !torqueExample(bus, id, false)) return false;
    if (!bus.setOrigin(id).ok()) return false;
    std::uint8_t position[2] = {};
    const auto r = bus.readRegister(id, 0x38, position, 2);
    if (!r.ok() || !r.status_received || r.servo_error != 0 ||
        position[0] != 0 || position[1] != 8) return false;
    // 古い座標系の目標を使わないよう、新しい基準位置を目標に設定する。
    if (!bus.setPosition(id, 2048).ok()) return false;
    std::uint8_t target[2] = {};
    const auto goal = bus.readRegister(id, 0x2a, target, 2);
    return goal.ok() && goal.status_received && goal.servo_error == 0 &&
           target[0] == 0 && target[1] == 8;
}
