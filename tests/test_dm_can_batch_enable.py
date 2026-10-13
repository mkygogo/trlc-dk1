from __future__ import annotations

import pytest
from lerobot_robot_trlc_dk1.motors.DM_Control_Python import DM_CAN


class FakeSerial:
    def __init__(self) -> None:
        self.is_open = False
        self.writes: list[bytes] = []
        self.read_count = 0

    def close(self) -> None:
        self.is_open = False

    def open(self) -> None:
        self.is_open = True

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def read_all(self) -> bytes:
        self.read_count += 1
        return b""


def _control_with_motors(count: int = 6):
    serial = FakeSerial()
    control = DM_CAN.MotorControl(serial)
    motors = [DM_CAN.Motor(DM_CAN.DM_Motor_Type.DM4340, index, 0) for index in range(1, count + 1)]
    for motor in motors:
        control.addMotor(motor)
    return control, motors, serial


def test_enable_many_writes_all_frames_before_single_wait_and_receive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, motors, serial = _control_with_motors()
    events: list[tuple[str, float | None]] = []

    original_write = serial.write
    original_read_all = serial.read_all

    def record_write(data: bytes) -> None:
        events.append(("write", None))
        original_write(data)

    def record_sleep(seconds: float) -> None:
        events.append(("sleep", seconds))

    def record_read_all() -> bytes:
        events.append(("read", None))
        return original_read_all()

    serial.write = record_write
    serial.read_all = record_read_all
    monkeypatch.setattr(DM_CAN, "sleep", record_sleep)

    sent_ids = control.enable_many_unverified(motors, settle_time_s=0.025)

    assert sent_ids == (1, 2, 3, 4, 5, 6)
    assert events == [("write", None)] * 6 + [("sleep", 0.025), ("read", None)]
    assert serial.read_count == 1
    assert [frame[13] | (frame[14] << 8) for frame in serial.writes] == list(sent_ids)
    assert all(frame[21:29] == b"\xff" * 7 + b"\xfc" for frame in serial.writes)


def test_timed_enable_records_last_write_before_wait_and_receive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, motors, serial = _control_with_motors(2)
    events: list[tuple[str, float | None]] = []
    timestamps = iter((10.25, 10.50))

    original_write = serial.write
    original_read_all = serial.read_all

    def record_write(data: bytes) -> None:
        events.append(("write", None))
        original_write(data)

    def record_sleep(seconds: float) -> None:
        events.append(("sleep", seconds))

    def record_read_all() -> bytes:
        events.append(("read", None))
        return original_read_all()

    def record_monotonic() -> float:
        value = next(timestamps)
        events.append(("monotonic", value))
        return value

    serial.write = record_write
    serial.read_all = record_read_all
    monkeypatch.setattr(DM_CAN, "sleep", record_sleep)
    monkeypatch.setattr(DM_CAN, "monotonic", record_monotonic)

    result = control.enable_many_timed_unverified(motors, settle_time_s=0.003)

    assert result == {
        "slave_ids": (1, 2),
        "last_frame_write_monotonic_s": 10.25,
        "receive_complete_monotonic_s": 10.50,
    }
    assert events == [
        ("write", None),
        ("write", None),
        ("monotonic", 10.25),
        ("sleep", 0.003),
        ("read", None),
        ("monotonic", 10.50),
    ]


@pytest.mark.parametrize("settle_time_s", [-0.001, float("nan"), float("inf")])
def test_timed_enable_rejects_invalid_settle_time(settle_time_s: float) -> None:
    control, motors, serial = _control_with_motors(1)

    with pytest.raises(ValueError, match="finite and non-negative"):
        control.enable_many_timed_unverified(motors, settle_time_s=settle_time_s)

    assert serial.writes == []
    assert serial.read_count == 0


@pytest.mark.parametrize("settle_time_s", [-0.001, float("nan"), float("inf")])
def test_enable_many_rejects_invalid_settle_time(settle_time_s: float) -> None:
    control, motors, serial = _control_with_motors(1)

    with pytest.raises(ValueError, match="finite and non-negative"):
        control.enable_many_unverified(motors, settle_time_s=settle_time_s)

    assert serial.writes == []
    assert serial.read_count == 0


def test_enable_many_rejects_empty_duplicate_or_unregistered_groups() -> None:
    control, motors, serial = _control_with_motors(2)

    with pytest.raises(ValueError, match="at least one"):
        control.enable_many_unverified([])
    with pytest.raises(ValueError, match="distinct SlaveIDs"):
        control.enable_many_unverified([motors[0], motors[0]])

    replacement = DM_CAN.Motor(DM_CAN.DM_Motor_Type.DM4340, 1, 0)
    with pytest.raises(ValueError, match="not registered"):
        control.enable_many_unverified([replacement])

    assert serial.writes == []
    assert serial.read_count == 0


def test_disable_many_writes_all_frames_before_single_wait_and_receive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, motors, serial = _control_with_motors()
    events: list[tuple[str, float | None]] = []

    original_write = serial.write
    original_read_all = serial.read_all

    def record_write(data: bytes) -> None:
        events.append(("write", None))
        original_write(data)

    def record_sleep(seconds: float) -> None:
        events.append(("sleep", seconds))

    def record_read_all() -> bytes:
        events.append(("read", None))
        return original_read_all()

    serial.write = record_write
    serial.read_all = record_read_all
    monkeypatch.setattr(DM_CAN, "sleep", record_sleep)

    sent_ids = control.disable_many_unverified(motors, settle_time_s=0.015)

    assert sent_ids == (1, 2, 3, 4, 5, 6)
    assert events == [("write", None)] * 6 + [("sleep", 0.015), ("read", None)]
    assert serial.read_count == 1
    assert [frame[13] | (frame[14] << 8) for frame in serial.writes] == list(sent_ids)
    assert all(frame[21:29] == b"\xff" * 7 + b"\xfd" for frame in serial.writes)


@pytest.mark.parametrize("settle_time_s", [-0.001, float("nan"), float("inf")])
def test_disable_many_rejects_invalid_settle_time(settle_time_s: float) -> None:
    control, motors, serial = _control_with_motors(1)

    with pytest.raises(ValueError, match="finite and non-negative"):
        control.disable_many_unverified(motors, settle_time_s=settle_time_s)

    assert serial.writes == []
    assert serial.read_count == 0


def test_disable_many_rejects_empty_duplicate_or_unregistered_groups() -> None:
    control, motors, serial = _control_with_motors(2)

    with pytest.raises(ValueError, match="at least one"):
        control.disable_many_unverified([])
    with pytest.raises(ValueError, match="distinct SlaveIDs"):
        control.disable_many_unverified([motors[0], motors[0]])

    replacement = DM_CAN.Motor(DM_CAN.DM_Motor_Type.DM4340, 1, 0)
    with pytest.raises(ValueError, match="not registered"):
        control.disable_many_unverified([replacement])

    assert serial.writes == []
    assert serial.read_count == 0


def test_feedback_tracks_status_nibble_sequence_and_timestamp(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, motors, _serial = _control_with_motors(1)
    motor = motors[0]
    timestamps = iter((12.5, 12.75, 13.0))
    monkeypatch.setattr(DM_CAN, "monotonic", lambda: next(timestamps))

    normal = [0x01, 0x80, 0x00, 0x80, 0x08, 0x00, 31, 30]
    enabled = [0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 31, 30]
    fault = [0xB1, 0x80, 0x00, 0x80, 0x08, 0x00, 32, 31]
    control._MotorControl__process_packet(normal, 1, 0x11)

    assert motor.getErrorCode() == 0
    assert motor.getFeedbackStatusCode() == 0
    assert motor.getFeedbackSequence() == 1
    assert motor.getFeedbackMonotonic() == pytest.approx(12.5)
    assert motor.isEnable is False

    control._MotorControl__process_packet(enabled, 1, 0x11)

    assert motor.getFeedbackStatusCode() == 1
    assert motor.getFeedbackSequence() == 2
    assert motor.getFeedbackMonotonic() == pytest.approx(12.75)
    assert motor.isEnable is True

    control._MotorControl__process_packet(fault, 1, 0x11)

    assert motor.getErrorCode() == 0xB
    assert motor.getFeedbackStatusCode() == 0xB
    assert motor.getFeedbackSequence() == 3
    assert motor.getFeedbackMonotonic() == pytest.approx(13.0)
    assert motor.isEnable is False


def test_master_id_zero_feedback_tracks_error_nibble() -> None:
    control, motors, _serial = _control_with_motors(1)
    motor = motors[0]
    fault = [0xD1, 0x80, 0x00, 0x80, 0x08, 0x00, 33, 32]

    control._MotorControl__process_packet(fault, 0, 0x11)

    assert motor.getErrorCode() == 0xD
    assert motor.getFeedbackStatusCode() == 0xD
    assert motor.getFeedbackSequence() == 1
