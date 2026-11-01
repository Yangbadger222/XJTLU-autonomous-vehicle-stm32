from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DELAY_SOURCE = ROOT / "Core/Src/platform/user_delay.c"
TIME_CONSUMERS = (
    ROOT / "Core/Src/app/main.c",
    ROOT / "Core/Src/platform/ps2.c",
    ROOT / "Core/Src/service/Motor_Speed_pid.c",
)


def test_microsecond_delay_does_not_reconfigure_systick() -> None:
    source = DELAY_SOURCE.read_text(encoding="utf-8")

    assert "HAL_SYSTICK_Config" not in source
    assert "DWT->CYCCNT" in source
    assert "DWT_CTRL_CYCCNTENA_Msk" in source


def test_drive_control_uses_the_millisecond_hal_timebase() -> None:
    for source_path in TIME_CONSUMERS:
        source = source_path.read_text(encoding="utf-8")
        assert "HAL_GetTick()" in source
