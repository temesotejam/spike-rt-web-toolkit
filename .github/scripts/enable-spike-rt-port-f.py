#!/usr/bin/env python3
"""Enable SPIKE Prime Port F in the pinned SPIKE-RT v0.2.0 build checkout.

SPIKE-RT's prime_hub_spike-rt pbdrvconfig intentionally lists only A-E
(PBDRV_CONFIG_HAS_PORT_F=0), while the hardware has A-F. Its upstream
platform.c already has F's UART9/LPF2 wiring under HAS_PORT_F guards.

Changes are local to the disposable Actions checkout. Upstream is untouched.
"""
from pathlib import Path

PLATFORM = Path("spike-rt/external/libpybricks/lib/pbio/platform/prime_hub_spike-rt")

def change(text: str, before: str, after: str, path: Path) -> str:
    hits = text.count(before)
    if hits != 1:
        raise RuntimeError(f"{path}: expected one occurrence of {before!r}, got {hits}")
    return text.replace(before, after, 1)

pbdrv_path = PLATFORM / "pbdrvconfig.h"
text = pbdrv_path.read_text(encoding="utf-8")
for old, new in [
    ("#define PBDRV_CONFIG_COUNTER_LPF2_NUM_DEV           (5) // TODO",
     "#define PBDRV_CONFIG_COUNTER_LPF2_NUM_DEV           (6) // Enable Port F"),
    ("#define PBDRV_CONFIG_IOPORT_LPF2_NUM_PORTS          (5) //TODO",
     "#define PBDRV_CONFIG_IOPORT_LPF2_NUM_PORTS          (6) // Enable Port F"),
    ("#define PBDRV_CONFIG_IOPORT_LPF2_LAST_PORT          PBIO_PORT_ID_E",
     "#define PBDRV_CONFIG_IOPORT_LPF2_LAST_PORT          PBIO_PORT_ID_F"),
    ("#define PBDRV_CONFIG_UART_STM32F4_LL_IRQ_NUM_UART   (5)",
     "#define PBDRV_CONFIG_UART_STM32F4_LL_IRQ_NUM_UART   (6)"),
    ("#define PBDRV_CONFIG_HAS_PORT_F (0) // TODO:",
     "#define PBDRV_CONFIG_HAS_PORT_F (1) // Enable LPF2/UART9 on Port F"),
    ("#define PBDRV_CONFIG_NUM_MOTOR_CONTROLLER   (5)",
     "#define PBDRV_CONFIG_NUM_MOTOR_CONTROLLER   (6)"),
]:
    text = change(text, old, new, pbdrv_path)
pbdrv_path.write_text(text, encoding="utf-8")

pbio_path = PLATFORM / "pbioconfig.h"
text = pbio_path.read_text(encoding="utf-8")
text = change(
    text,
    "#define PBIO_CONFIG_USE_PORT_F_AS_ASP3_DEBUG_UART  (1)",
    "#define PBIO_CONFIG_USE_PORT_F_AS_ASP3_DEBUG_UART  (0)",
    pbio_path,
)
pbio_path.write_text(text, encoding="utf-8")
print("Enabled SPIKE-RT Port F and disabled F debug UART in build checkout.")
