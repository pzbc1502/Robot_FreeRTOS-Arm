"""Check the actual fire-key BSP with fake GPIO; never access hardware."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
GCC = os.environ.get("GCC") or shutil.which("gcc") or (
    r"E:\STM32\STM32Tool\CLion\CLion 2025.2.3\bin\mingw\bin\gcc.exe"
)

HAL = r"""
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
typedef uint16_t bsp_io_port_pin_t;
typedef enum { BSP_IO_LEVEL_LOW, BSP_IO_LEVEL_HIGH } bsp_io_level_t;
typedef int fsp_err_t;
enum {
    FSP_SUCCESS = 0,
    BSP_IO_PORT_00_PIN_00 = 0x0000,
    BSP_IO_PORT_00_PIN_02 = 0x0002,
    BSP_IO_PORT_08_PIN_01 = 0x0801,
    IOPORT_CFG_PORT_DIRECTION_INPUT = 0,
    IOPORT_CFG_PORT_DIRECTION_OUTPUT = 4,
    IOPORT_CFG_PORT_OUTPUT_LOW = 0,
    IOPORT_CFG_PULLUP_ENABLE = 16
};
typedef struct { void *p_ctrl; } ioport_instance_t;
extern const ioport_instance_t g_ioport;
fsp_err_t R_IOPORT_PinCfg(void *, bsp_io_port_pin_t, uint32_t);
fsp_err_t R_IOPORT_PinWrite(void *, bsp_io_port_pin_t, bsp_io_level_t);
fsp_err_t R_IOPORT_PinRead(void *, bsp_io_port_pin_t, bsp_io_level_t *);
#endif
"""

FIXTURE = r"""
#include <stdio.h>
#include "bsp_laser.h"
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #expr); \
    return 1; } } while (0)
const ioport_instance_t g_ioport = {0};
static unsigned cfg_count, write_count;
static bsp_io_port_pin_t cfg_pin[2], read_pin, write_pin;
static uint32_t cfg_value[2];
static bsp_io_level_t input_level, output_level;
static fsp_err_t read_result;
fsp_err_t R_IOPORT_PinCfg(void *ctrl, bsp_io_port_pin_t pin, uint32_t cfg) {
    (void)ctrl;
    if (cfg_count < 2) { cfg_pin[cfg_count] = pin; cfg_value[cfg_count] = cfg; }
    ++cfg_count;
    return FSP_SUCCESS;
}
fsp_err_t R_IOPORT_PinWrite(void *ctrl, bsp_io_port_pin_t pin, bsp_io_level_t level) {
    (void)ctrl; ++write_count; write_pin = pin; output_level = level;
    return FSP_SUCCESS;
}
fsp_err_t R_IOPORT_PinRead(void *ctrl, bsp_io_port_pin_t pin, bsp_io_level_t *level) {
    (void)ctrl; read_pin = pin; *level = input_level;
    return read_result;
}
int main(void) {
    CHECK(LASER_FIRE_KEY_PIN == BSP_IO_PORT_00_PIN_02);
    CHECK(LASER_FIRE_KEY_ACTIVE_LEVEL == BSP_IO_LEVEL_LOW);
    BSP_Laser_Init();
    CHECK(cfg_count == 2 && cfg_pin[0] == BSP_IO_PORT_08_PIN_01);
    CHECK(cfg_value[0] == IOPORT_CFG_PORT_DIRECTION_OUTPUT);
    CHECK(cfg_pin[1] == BSP_IO_PORT_00_PIN_02);
    CHECK(cfg_value[1] == (IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE));
    CHECK(write_count == 1 && write_pin == BSP_IO_PORT_08_PIN_01);
    CHECK(output_level == BSP_IO_LEVEL_LOW);
    input_level = BSP_IO_LEVEL_HIGH;
    CHECK(!BSP_Laser_FireKey_IsPressed() && read_pin == BSP_IO_PORT_00_PIN_02);
    input_level = BSP_IO_LEVEL_LOW;
    CHECK(BSP_Laser_FireKey_IsPressed());
    read_result = 1;
    CHECK(!BSP_Laser_FireKey_IsPressed());
    read_result = FSP_SUCCESS;
    input_level = BSP_IO_LEVEL_HIGH;
    CHECK(!BSP_Laser_FireKey_IsPressed());
    BSP_Laser_On();
    CHECK(write_pin == BSP_IO_PORT_08_PIN_01 && output_level == BSP_IO_LEVEL_HIGH);
    BSP_Laser_Off();
    CHECK(output_level == BSP_IO_LEVEL_LOW);
    puts("P002 active-low fire-key GPIO checks passed (fake IO)");
    return 0;
}
"""


def test_fire_key_gpio() -> None:
    env = os.environ.copy()
    env["PATH"] = str(Path(GCC).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="ra6m5_fire_key_") as directory:
        temp = Path(directory)
        (temp / "hal_data.h").write_text(HAL, encoding="ascii")
        (temp / "fixture.c").write_text(FIXTURE, encoding="ascii")
        executable = temp / "fire_key.exe"
        subprocess.run(
            [GCC, "-std=c11", "-Wall", "-Wextra", "-Werror",
             "-I", str(temp), "-I", str(ROOT / "USER_BSP"),
             str(ROOT / "USER_BSP/bsp_laser.c"), str(temp / "fixture.c"),
             "-o", str(executable)],
            check=True, env=env,
        )
        subprocess.run([str(executable)], check=True, env=env)


def test_startup_configuration() -> None:
    settings = {
        item.attrib["configurationId"]: item.attrib["altId"]
        for item in ET.parse(ROOT / "configuration.xml").findall(
            ".//raPinConfiguration/pincfg/configSetting"
        )
    }
    assert settings["p002"] == "p002.input"
    assert settings["p002.gpio_mode"] == "p002.gpio_mode.gpio_mode_in"
    assert settings["p002.gpio_pupd"] == "p002.gpio_pupd.gpio_pupd_ip_up"
    generated = (ROOT / "ra_gen/pin_data.c").read_text(encoding="utf-8")
    block = re.search(
        r"\.pin\s*=\s*BSP_IO_PORT_00_PIN_02,\s*\.pin_cfg\s*=(.*?)\},",
        generated, re.S,
    )
    assert block is not None
    assert "IOPORT_CFG_PORT_DIRECTION_INPUT" in block[1]
    assert "IOPORT_CFG_PULLUP_ENABLE" in block[1]
    assert "IOPORT_CFG_PORT_DIRECTION_OUTPUT" not in block[1]
    # The retained, unused DHT11 driver must not reclaim the fire-key pin.
    for folder in ("APP", "src"):
        for path in (ROOT / folder).glob("*.c"):
            assert not re.search(r"\bDHT11_\w+\s*\(", path.read_text(encoding="utf-8")), path
    print("P002 startup/configuration and no-DHT-caller checks passed")


if __name__ == "__main__":
    test_fire_key_gpio()
    test_startup_configuration()
