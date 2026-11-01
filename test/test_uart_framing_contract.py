from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
USART_SOURCE = ROOT / "Core/Src/platform/usart.c"


def function_body(source: str, name: str) -> str:
    match = re.search(
        rf"(?m)^[ \t]*[A-Za-z_][A-Za-z0-9_ \t\*]*\s+{name}\s*\([^)]*\)\s*\{{",
        source,
    )
    assert match, f"{name}() is missing"
    index = match.end()
    depth = 1
    while index < len(source) and depth:
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
        index += 1
    assert depth == 0, f"{name}() body is not balanced"
    return source[match.end() : index - 1]


def test_receive_to_idle_uses_hal_managed_idle_callback() -> None:
    source = USART_SOURCE.read_text(encoding="utf-8", errors="ignore")
    start = function_body(source, "USART1_StartReceiveToIdle")
    handler = function_body(source, "USART1_IRQHandler")
    callback = function_body(source, "HAL_UARTEx_RxEventCallback")

    assert "HAL_UARTEx_ReceiveToIdle_DMA" in start
    assert "DMA_IT_HT" in start
    assert handler.strip() == "HAL_UART_IRQHandler(&huart1);"
    assert "HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE" in callback
    assert "HAL_UART_DMAStop" not in source
    assert "HAL_UART_Receive_DMA" not in source


def test_idle_callback_publishes_a_terminated_variable_length_packet() -> None:
    source = USART_SOURCE.read_text(encoding="utf-8", errors="ignore")
    callback = function_body(source, "HAL_UARTEx_RxEventCallback")

    assert "size > 0u" in callback
    assert "size < UART_RX_BUF_SIZE" in callback
    assert "memcpy(UART1_RX_Buffer, DMA_RX_Buffer, size);" in callback
    assert "UART1_RX_Buffer[size] = '\\0';" in callback
    assert "UART1_RX_Size = size;" in callback
    assert "new_serial_data_received = 1u;" in callback
    assert "(void)USART1_StartReceiveToIdle();" in callback
