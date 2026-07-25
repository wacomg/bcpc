#!/usr/bin/env python3
"""
BC Series Motor Control CLI
Строго по документации Application Note (стандарт CiA 301)
SDO Read: 0x40, Write 4: 0x23, Write 2: 0x2B, Write 1: 0x2F
Включение: одна команда 0x000F
Скорость: counts/s (не 0.1!)
"""

import can
import struct
import time
import sys
import csv
import argparse
from datetime import datetime


CAN_INTERFACE = 'can0'
DEFAULT_NODE_ID = 3
ENCODER_RES = 131072
GEAR_RATIO = 1.0
DEGREES_PER_REV = 360.0
LOG_FILE = "motor_control_log.csv"

COB_SDO_RX = 0x600
COB_SDO_TX = 0x580
COB_NMT = 0x000

# SDO команды — СТАНДАРТ CiA 301 (из Application Note)
SDO_READ = 0x40
SDO_WRITE_4 = 0x23
SDO_WRITE_2 = 0x2B
SDO_WRITE_1 = 0x2F
SDO_RESPONSE_4 = 0x43
SDO_RESPONSE_2 = 0x4B
SDO_RESPONSE_1 = 0x4F
SDO_ABORT = 0x80

OD_STATUSWORD = 0x6041
OD_CONTROLWORD = 0x6040
OD_MODES_OF_OPERATION = 0x6060
OD_POSITION_ACTUAL_VALUE = 0x6064
OD_TARGET_POSITION = 0x607A
OD_PROFILE_VELOCITY = 0x6081
OD_PROFILE_ACCELERATION = 0x6083
OD_PROFILE_DECELERATION = 0x6084
OD_GENERAL_INPUTS = 0x2190


def setup_can(interface='can0'):
    try:
        bus = can.interface.Bus(channel=interface, interface='socketcan')
        print(f"[OK] Подключено к {interface}")
        return bus
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)


def send_nmt(bus, node_id, command=0x01):
    bus.send(can.Message(arbitration_id=COB_NMT, data=[command, node_id], is_extended_id=False))
    time.sleep(0.05)


def sdo_read(bus, node_id, index, subindex=0, timeout=0.2):
    data = [SDO_READ, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
    bus.send(can.Message(arbitration_id=COB_SDO_RX + node_id, data=data, is_extended_id=False))
    start = time.time()
    while time.time() - start < timeout:
        resp = bus.recv(timeout=0.05)
        if resp and resp.arbitration_id == COB_SDO_TX + node_id:
            cmd = resp.data[0]
            if cmd == SDO_RESPONSE_4:
                return struct.unpack('<I', resp.data[4:8])[0], None
            elif cmd == SDO_RESPONSE_2:
                return struct.unpack('<H', resp.data[4:6])[0], None
            elif cmd == SDO_RESPONSE_1:
                return resp.data[4], None
            elif cmd == SDO_ABORT:
                return None, f"Abort 0x{struct.unpack('<I', resp.data[4:8])[0]:08X}"
    return None, "Timeout"


def sdo_write(bus, node_id, index, subindex, value, size):
    if size == 4:
        data = [SDO_WRITE_4, index & 0xFF, (index >> 8) & 0xFF, subindex,
                value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF]
    elif size == 2:
        data = [SDO_WRITE_2, index & 0xFF, (index >> 8) & 0xFF, subindex,
                value & 0xFF, (value >> 8) & 0xFF, 0, 0]
    else:
        data = [SDO_WRITE_1, index & 0xFF, (index >> 8) & 0xFF, subindex,
                value & 0xFF, 0, 0, 0]

    bus.send(can.Message(arbitration_id=COB_SDO_RX + node_id, data=data, is_extended_id=False))
    start = time.time()
    while time.time() - start < 0.15:
        resp = bus.recv(timeout=0.05)
        if resp and resp.arbitration_id == COB_SDO_TX + node_id:
            if resp.data[0] == 0x60:
                return True, "OK"
            elif resp.data[0] == 0x80:
                return False, f"Abort 0x{struct.unpack('<I', resp.data[4:8])[0]:08X}"
    return False, "Timeout"


def read_statusword(bus, node_id):
    return sdo_read(bus, node_id, OD_STATUSWORD, 0)


def read_position(bus, node_id):
    return sdo_read(bus, node_id, OD_POSITION_ACTUAL_VALUE, 0)


def read_input(bus, node_id, pin):
    value, error = sdo_read(bus, node_id, OD_GENERAL_INPUTS, 0)
    if error:
        return None, error
    return bool(value & (1 << pin)), None


def reset_fault(bus, node_id):
    """Сброс ошибки: NMT Reset Application (81 nodeID)"""
    print("  [*] NMT Reset Application (81 nodeID)")
    send_nmt(bus, node_id, 0x81)
    time.sleep(1.0)
    send_nmt(bus, node_id, 0x01)
    time.sleep(0.5)
    sw, _ = read_statusword(bus, node_id)
    print(f"  Статус после сброса: 0x{sw:04X}" if sw else "  [!] Нет ответа")


def enable_motor(bus, node_id):
    """Включение мотора: Controlword = 0x000F"""
    print("\n[INFO] Запуск мотора...")

    sw, _ = read_statusword(bus, node_id)
    print(f"  Текущий Statusword: 0x{sw:04X}")

    if sw & 0x0008:
        print("  [!] Обнаружен Fault, сброс...")
        reset_fault(bus, node_id)
        sw, _ = read_statusword(bus, node_id)
        print(f"  Statusword после сброса: 0x{sw:04X}")

    send_nmt(bus, node_id, 0x01)
    time.sleep(0.1)

    print("  [1] Режим: Profile Position (0x6060 = 1)")
    sdo_write(bus, node_id, OD_MODES_OF_OPERATION, 0, 1, 1)
    time.sleep(0.1)

    print("  [2] Enable: Controlword = 0x000F")
    ok, msg = sdo_write(bus, node_id, OD_CONTROLWORD, 0, 0x000F, 2)
    if not ok:
        print(f"  [ERROR] Не удалось записать Controlword: {msg}")
        return False
    time.sleep(0.3)

    sw, _ = read_statusword(bus, node_id)
    print(f"  Statusword: 0x{sw:04X}")

    if sw & 0x0008:
        print(f"[ERROR] Привод в Fault! SW=0x{sw:04X}")
        return False

    if (sw & 0x0007) == 0x0007:
        print("[OK] Мотор включён (Operation Enabled)!")
        return True

    print(f"[WARN] Неожиданный статус: 0x{sw:04X}, продолжаем...")
    return True


def disable_motor(bus, node_id):
    """Выключение мотора: Controlword = 0x0000"""
    print("\n[INFO] Выключение мотора (M5)...")
    sdo_write(bus, node_id, OD_CONTROLWORD, 0, 0x0000, 2)
    time.sleep(0.2)
    print("[OK] Мотор выключен")


def move_to_position(bus, node_id, target_degrees, speed_rpm, encoder_res, gear_ratio):
    """
    Перемещение согласно Application Note раздел 5.
    Скорость: counts/s (НЕ 0.1 count/s!)
    """
    scale = encoder_res * gear_ratio / DEGREES_PER_REV
    target_inc = int(target_degrees * scale)

    # Скорость: rpm → counts/s
    # 1 rpm = encoder_res counts / 60 sec
    velocity_raw = int(speed_rpm * encoder_res / 60)
    velocity_raw = max(100, min(velocity_raw, 0x7FFFFFFF))

    # Ускорение: counts/s² (быстрый разгон ~0.2 сек)
    accel_raw = velocity_raw * 10
    accel_raw = max(1000, min(accel_raw, 0x7FFFFFFF))

    print(f"\n[INFO] Перемещение:")
    print(f"  Цель:    {target_degrees:.2f}° = {target_inc} counts")
    print(f"  Скорость: {speed_rpm:.1f} rpm = {velocity_raw} counts/s")
    print(f"  Ускорение: {accel_raw} counts/s²")

    # Проверка: за сколько секунд должен быть один оборот
    if velocity_raw > 0:
        sec_per_rev = encoder_res / velocity_raw
        print(f"  Расчётное время оборота: {sec_per_rev:.1f} сек")

    sdo_write(bus, node_id, OD_PROFILE_VELOCITY, 0, velocity_raw, 4)
    sdo_write(bus, node_id, OD_PROFILE_ACCELERATION, 0, accel_raw, 4)
    sdo_write(bus, node_id, OD_PROFILE_DECELERATION, 0, accel_raw, 4)
    sdo_write(bus, node_id, OD_TARGET_POSITION, 0, target_inc, 4)

    print("  [.] Запуск (Controlword = 0x001F)...")
    sdo_write(bus, node_id, OD_CONTROLWORD, 0, 0x001F, 2)

    print("  [.] Ожидание завершения...")
    start = time.time()
    history = []
    last_print = 0

    while time.time() - start < 60.0:
        sw, _ = read_statusword(bus, node_id)
        pos, _ = read_position(bus, node_id)

        if pos is not None:
            pos_deg = pos / scale
            history.append((time.time(), pos, pos_deg))

        if sw & 0x0008:
            print(f"[ERROR] FAULT! SW=0x{sw:04X}")
            return False, history

        if sw & 0x0400:
            elapsed = time.time() - start
            final_pos, _ = read_position(bus, node_id)
            final_deg = final_pos / scale if final_pos else 0
            print(f"[OK] Достигнуто за {elapsed:.2f}с: {final_deg:.2f}°")
            return True, history

        elapsed = time.time() - start
        if elapsed - last_print >= 0.5 and pos is not None:
            print(f"  [{elapsed:.1f}s] {pos_deg:.2f}°")
            last_print = elapsed

        time.sleep(0.05)

    print("[ERROR] Таймаут 60с")
    return False, history


class MotionLogger:
    def __init__(self, filename, encoder_res, gear_ratio):
        self.encoder_res = encoder_res
        self.gear_ratio = gear_ratio
        self.file = open(filename, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'Timestamp', 'Command', 'Target_deg', 'Speed_rpm',
            'Encoder_Start', 'Encoder_End', 'Angle_Start_deg', 'Angle_End_deg',
            'Duration_sec', 'Status'
        ])
        self.file.flush()

    def log_command(self, command, target_deg, speed_rpm, pos_start, pos_end, duration, status):
        scale = self.encoder_res * self.gear_ratio
        start_deg = pos_start * DEGREES_PER_REV / scale if pos_start else 0
        end_deg = pos_end * DEGREES_PER_REV / scale if pos_end else 0
        self.writer.writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            command, f"{target_deg:.2f}", f"{speed_rpm:.1f}",
            pos_start or "", pos_end or "",
            f"{start_deg:.2f}", f"{end_deg:.2f}",
            f"{duration:.3f}" if duration else "", status
        ])
        self.file.flush()

    def close(self):
        self.file.close()


def parse_gcode(line):
    line = line.strip().upper()
    if not line:
        return None
    if line == 'M5':
        return ('M5', None, None)
    if line.startswith('G1'):
        angle = None
        speed = 10.0
        for part in line[2:].strip().split():
            if part.startswith('X') or part.startswith('A'):
                try:
                    angle = float(part[1:])
                except ValueError:
                    pass
            elif part.startswith('F'):
                try:
                    speed = float(part[1:])
                except ValueError:
                    pass
        if angle is not None:
            return ('G1', angle, speed)
    return None


def main():
    parser = argparse.ArgumentParser(description='BC Series Motor Control CLI')
    parser.add_argument('-n', '--node', type=int, default=DEFAULT_NODE_ID)
    parser.add_argument('-i', '--interface', default=CAN_INTERFACE)
    parser.add_argument('--encoder', type=int, default=ENCODER_RES)
    parser.add_argument('--gear', type=float, default=GEAR_RATIO)
    args = parser.parse_args()

    encoder_res = args.encoder
    gear_ratio = args.gear
    node_id = args.node

    print("=" * 60)
    print("  BC Series Motor Control CLI")
    print("=" * 60)
    print(f"  Node {node_id} | Encoder: {encoder_res} | Gear: {gear_ratio}")
    print("=" * 60)

    bus = setup_can(args.interface)

    send_nmt(bus, node_id, 0x01)
    time.sleep(0.2)

    scale = encoder_res * gear_ratio / DEGREES_PER_REV
    logger = MotionLogger(LOG_FILE, encoder_res, gear_ratio)
    print(f"[INFO] Лог: {LOG_FILE}")

    motor_enabled = False

    try:
        print("\n[INFO] Проверка IN1...")
        in1, err = read_input(bus, node_id, 0)
        if err:
            print(f"[WARN] Ошибка IN1: {err}")
        elif in1:
            print("[ERROR] IN1 = HIGH. Требуется LOW.")
            bus.shutdown()
            logger.close()
            sys.exit(1)
        else:
            print("[OK] IN1 = LOW")

        if not enable_motor(bus, node_id):
            print("[ERROR] Не удалось включить мотор")
            bus.shutdown()
            logger.close()
            sys.exit(1)

        motor_enabled = True

        print("\n" + "=" * 60)
        print("  G1 X<угол> F<rpm> | M5 | POS | HELP")
        print("=" * 60)

        pos, _ = read_position(bus, node_id)
        if pos is not None:
            print(f"\n  Позиция: {pos} counts = {pos / scale:.2f}°")

        while True:
            try:
                cmd_line = input("\n> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[INFO] Выход...")
                break

            if not cmd_line:
                continue

            if cmd_line.upper() == 'HELP':
                print("  G1 X<угол> F<rpm> | M5 | POS")
                continue

            if cmd_line.upper() == 'POS':
                pos, err = read_position(bus, node_id)
                if err:
                    print(f"  [!] {err}")
                else:
                    print(f"  {pos} counts = {pos / scale:.2f}°")
                continue

            parsed = parse_gcode(cmd_line)
            if parsed is None:
                print("  [!] G1 X<угол> F<rpm>")
                continue

            cmd, angle, speed = parsed

            if cmd == 'M5':
                break

            if cmd == 'G1':
                pos_start, _ = read_position(bus, node_id)
                t0 = time.time()
                success, _ = move_to_position(bus, node_id, angle, speed, encoder_res, gear_ratio)
                elapsed = time.time() - t0
                pos_end, _ = read_position(bus, node_id)

                status = "OK" if success else "FAIL"
                logger.log_command(cmd_line, angle, speed, pos_start, pos_end, elapsed, status)

                if pos_start is not None and pos_end is not None:
                    print(f"  Пройдено: {abs(pos_end - pos_start) / scale:.2f}°")

    finally:
        if motor_enabled:
            disable_motor(bus, node_id)
        logger.close()
        bus.shutdown()
        print("[OK] Завершено.")


if __name__ == "__main__":
    main()