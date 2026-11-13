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
