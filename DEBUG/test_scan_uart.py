"""Exercise actual UART parser/queue/TX and CRC C with an offline HAL stub."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from test_scan_workflow import ROOT, GCC, stripped
sys.path.insert(0, str(ROOT / "UpperComputer"))
from ra6m5_protocol import build_scan_control, build_unified_frame


PREAMBLE = r'''
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
typedef int sci_uart_instance_ctrl_t;
typedef int uart_cfg_t;
typedef int fsp_err_t;
typedef int BaseType_t;
typedef void *SemaphoreHandle_t;
sci_uart_instance_ctrl_t robot_jeston_ctrl;
const uart_cfg_t robot_jeston_cfg=0;
static uint32_t now_ms;
static uint8_t tx[64];
static unsigned tx_len, tx_count;
#define LOG(...) ((void)0)
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __DMB() ((void)0)
#define __BKPT(x) assert(0)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define FSP_SUCCESS 0
#define pdTRUE 1
#define pdFALSE 0
#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(x) (x)
#define portYIELD_FROM_ISR(x) ((void)(x))
static uint32_t xTaskGetTickCount(void) { return now_ms; }
static SemaphoreHandle_t xSemaphoreCreateBinary(void) { return (void*)1; }
static int xSemaphoreTake(SemaphoreHandle_t s, unsigned timeout) { (void)s;(void)timeout;return 1; }
static void xSemaphoreGiveFromISR(SemaphoreHandle_t s, BaseType_t *wake) { (void)s;*wake=0; }
static fsp_err_t R_SCI_UART_Open(sci_uart_instance_ctrl_t *c,const uart_cfg_t *cfg) { (void)c;(void)cfg;return 0; }
static fsp_err_t R_SCI_UART_Write(sci_uart_instance_ctrl_t *c,const uint8_t *data,uint32_t n) {
    (void)c;assert(n<=sizeof(tx));memcpy(tx,data,n);tx_len=n;tx_count++;return 0;
}
'''


def build_source():
    vectors = {
        "start": build_scan_control(1, 0, 0x12345678, 20),
        "query": build_scan_control(0, 0, 0, 22),
        "bad_len": build_unified_frame(7, 23, b"\x01"),
        "zero_seq": build_unified_frame(7, 0, b"\x01\x00\x64\x00\x00\x00"),
        "heartbeat": build_unified_frame(1, 1, b"\x00" * 4),
    }
    declarations = "\n".join("static const uint8_t " + name + "[]={" +
                             ",".join(str(x) for x in value) + "};" for name, value in vectors.items())
    return (PREAMBLE + stripped("Middle/crc16.h") + stripped("Middle/jetson_vision.h") +
            stripped("Middle/crc16.c") + stripped("Middle/jetson_vision.c") + declarations + CHECKS)


CHECKS = r'''
static void feed(const uint8_t *data,unsigned n) {
    for(unsigned i=0;i<n;i++)jetson_notify_rx_char_from_isr(data[i]);
    jetson_vision_process();
}
int main(void) {
    jetson_scan_control_t c;
    for(unsigned split=0;split<=sizeof(start);split++) {
        jetson_vision_init();feed(start,split);feed(start+split,sizeof(start)-split);
        assert(jetson_get_scan_control(&c));assert(c.action==1 && c.seq==20 && c.scan_id==0x12345678);
        assert(!jetson_get_scan_control(&c));assert(!jetson_is_link_alive(0));
    }
    jetson_vision_init();uint8_t bad[sizeof(start)];memcpy(bad,start,sizeof(start));bad[13]^=1;
    feed(bad,sizeof(bad));assert(!jetson_get_scan_control(&c));
    feed(start,sizeof(start));feed(query,sizeof(query));
    assert(jetson_get_scan_control(&c) && c.action==1);
    assert(jetson_get_scan_control(&c) && c.action==0);assert(!jetson_get_scan_control(&c));
    feed(bad_len,sizeof(bad_len));assert(tx[3]==0xFE && tx[6]==3);
    feed(zero_seq,sizeof(zero_seq));assert(tx[3]==0xFE && tx[6]==3);
    jetson_vision_init();
    for(unsigned i=0;i<9;i++)feed(start,sizeof(start));
    assert(tx_len==18 && tx[3]==0x82 && tx[6]==1 && tx[10]==0 && tx[11]==5);
    for(unsigned i=0;i<8;i++)assert(jetson_get_scan_control(&c));
    assert(!jetson_get_scan_control(&c));
    now_ms=100;feed(heartbeat,sizeof(heartbeat));assert(jetson_is_link_alive(100));
    now_ms=701;feed(query,sizeof(query));assert(!jetson_is_link_alive(701));
    jetson_send_scan_status(20,8,11,7,1,1,0,0x12345678);
    assert(tx_len==18 && tx[5]==10 && tx[12]==0x78 && tx[15]==0x12);
    assert(crc_modbus(tx+2,14)==((uint16_t)tx[16]|((uint16_t)tx[17]<<8)));
    puts("actual-C UART scan: fragmentation, CRC, FIFO, overflow, heartbeat isolation, TX passed");
}
'''


def main():
    env = os.environ.copy()
    env["PATH"] = str(Path(GCC).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="scan_uart_") as d:
        exe = Path(d) / "check.exe"
        subprocess.run([GCC, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                        "-x", "c", "-", "-o", str(exe)], input=build_source(), text=True,
                       encoding="utf-8", check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == "__main__":
    main()
