"""Compile the actual CAN error capture/callback with PC-only peripheral stubs.

No serial, debugger, motor or firmware-download access.
Run: py -3 DEBUG/test_can_error_diagnostics.py [--gcc path/to/gcc.exe]
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from test_scan_motion import ROOT, declaration, function


def build_source():
    can = (ROOT / "USER_BSP/bsp_can.c").read_text(encoding="utf-8")
    api = (ROOT / "ra/fsp/inc/api/r_can_api.h").read_text(encoding="utf-8")
    header = (ROOT / "USER_BSP/bsp_can.h").read_text(encoding="utf-8")
    types = "\n".join(declaration(api, r"typedef (?:enum|struct) " + name +
                                  r"\s*\{.*?\}\s*" + alias + r";")
                      for name, alias in (
                          ("e_can_event", "can_event_t"),
                          ("st_can_info", "can_info_t"),
                          ("e_can_id_mode", "can_id_mode_t"),
                          ("e_can_frame_type", "can_frame_type_t"),
                          ("st_can_frame", "can_frame_t"),
                          ("st_can_callback_args", "can_callback_args_t")))
    types += declaration(header, r"typedef struct\s*\{.*?\}\s*CAN_RxHeader_Mock_t;")
    context_start = header.index("typedef struct", header.index("CAN_RxHeader_Mock_t;"))
    types += declaration(header[context_start:], r"typedef struct\s*\{.*?\}\s*CAN_Context_t;")
    record = declaration(can, r"typedef struct\s*\{.*?\}\s*BSP_CAN_ErrorRecord_t;")
    globals_ = "\n".join(declaration(can, pattern) for pattern in (
        r"static volatile BSP_CAN_ErrorRecord_t s_can_errors[^;]*;",
        r"static volatile uint32_t s_can_recent_tx_id[^;]*;",
        r"static volatile uint8_t s_can_recent_tx_func[^;]*;"))
    functions = "\n".join(function(can, name) for name in (
        "BSP_CAN_RecordError", "BSP_CAN_ProcessErrors", "BSP_CAN_PrintInfo"))
    functions += function(can.replace("fsp_err_t BSP_CAN_Send", "int BSP_CAN_Send"),
                          "BSP_CAN_Send")
    functions += function(can, "canfd0_callback")
    callback = function(can, "canfd0_callback")
    assert "LOG(" not in callback and "BSP_CAN_PrintInfo(" not in callback
    worker = (ROOT / "src/robot_thread_entry.c").read_text(encoding="utf-8")
    assert "BSP_CAN_ProcessErrors();" in worker
    assert worker.index("BSP_CAN_Init();") < worker.index("BSP_CAN_ProcessErrors();")
    assert "void BSP_CAN_ProcessErrors(void);" in header
    init = function(can, "BSP_CAN_Init")
    assert "g_canfd0_extended_cfg.error_interrupts |= R_CANFD_CFDC_CTR_BEIE_Msk;" in init
    assert init.index("R_CANFD_CFDC_CTR_BEIE_Msk") < init.index("g_canfd0.p_api->open")
    return PREAMBLE + types + record + globals_ + FIXTURE + functions + CHECKS


PREAMBLE = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
typedef int fsp_err_t;
typedef int BaseType_t;
typedef void *SemaphoreHandle_t;
#define CAN_DATA_BUFFER_LENGTH 8
#define FSP_SUCCESS 0
#define FSP_ERR_TIMEOUT 2
#define pdFALSE 0
#define ROBOT_MAX_JOINT_NUM 6
#define ROBOT_STATUS_READY 1u
#define ROBOT_STATUS_SET(status, bit) ((status) |= (bit))
#define portTICK_PERIOD_MS 1u
#define portYIELD_FROM_ISR(woken) ((void)(woken))
#define __NOP() ((void)0)
"""

FIXTURE = r"""
static CAN_Context_t g_can_context;
static struct { struct { uint32_t status; } joints[6]; } g_robot;
static bool g_can_tx_complete;
static int g_canfd0_ctrl, info_result, write_result;
static can_info_t hardware_info;
static uint32_t primask, ipsr, now_ms, log_count, info_count;
static char last_log[1024];
static bool inject_during_log;
void canfd0_callback(can_callback_args_t *args);
static uint32_t __get_PRIMASK(void) { return primask; }
static uint32_t __get_IPSR(void) { return ipsr; }
static void __disable_irq(void) { primask=1; }
static void __set_PRIMASK(uint32_t value) { primask=value; }
static uint32_t xTaskGetTickCountFromISR(void) {
    assert(ipsr!=0 && primask==1); return now_ms;
}
static int R_CANFD_InfoGet(void *ctrl, can_info_t *info) {
    assert(ctrl==&g_canfd0_ctrl); info_count++;
    if(info_result==FSP_SUCCESS) *info=hardware_info;
    return info_result;
}
static void emit(can_event_t event, uint32_t error) {
    uint32_t previous=ipsr; ipsr=16;
    can_callback_args_t args={.event=event, .error=error, .channel=0, .buffer=3};
    canfd0_callback(&args); ipsr=previous;
}
static int R_CANFD_Write(void *ctrl, uint32_t buffer, can_frame_t *frame) {
    assert(ctrl==&g_canfd0_ctrl && buffer==0 && frame->options==0);
    if(write_result==FSP_SUCCESS) emit(CAN_EVENT_TX_COMPLETE,0);
    return write_result;
}
static void xSemaphoreGiveFromISR(void *sem, BaseType_t *woken) {
    (void)sem; (void)woken;
}
static void mock_log(const char *format, ...) {
    assert(ipsr==0 && primask==0); log_count++;
    va_list args; va_start(args,format);
    vsnprintf(last_log,sizeof(last_log),format,args); va_end(args);
    if(inject_during_log) {
        inject_during_log=false; now_ms++; emit(CAN_EVENT_ERR_CHANNEL,0x400u);
    }
}
#define LOG(...) mock_log(__VA_ARGS__)
"""

CHECKS = r"""
static void reset_fixture(void) {
    memset(&g_can_context,0,sizeof(g_can_context)); memset(&g_robot,0,sizeof(g_robot));
    s_can_errors=(BSP_CAN_ErrorRecord_t){0};
    hardware_info=(can_info_t){.status=0x88u,.error_count_transmit=190};
    s_can_recent_tx_id=0; s_can_recent_tx_func=0;
    g_can_tx_complete=inject_during_log=false;
    primask=ipsr=log_count=info_count=0; now_ms=100;
    info_result=write_result=FSP_SUCCESS; last_log[0]='\0';
}
static void receive(uint32_t id, const uint8_t *data, uint8_t dlc) {
    can_callback_args_t args={.event=CAN_EVENT_RX_COMPLETE};
    args.frame.id=id; args.frame.data_length_code=dlc;
    args.frame.id_mode=CAN_ID_MODE_EXTENDED;args.frame.type=CAN_FRAME_TYPE_DATA;
    memcpy(args.frame.data,data,dlc);
    ipsr=16; canfd0_callback(&args); ipsr=0;
}
int main(void) {
    reset_fixture(); BSP_CAN_ProcessErrors();
    assert(log_count==0 && info_count==0);

    reset_fixture(); uint8_t command[]={0x36,0x6B};
    assert(BSP_CAN_Send(0x400,CAN_ID_MODE_EXTENDED,command,2)==FSP_SUCCESS);
    assert(g_can_tx_complete && primask==0); g_can_tx_complete=false;
    emit(CAN_EVENT_ERR_CHANNEL,0x1281u);
    assert(log_count==0 && !g_can_tx_complete && primask==0);
    assert(s_can_errors.error_code==0x1281u && s_can_errors.info.error_code==0);
    assert(s_can_errors.info.error_count_transmit==190 && s_can_errors.info.status==0x88);
    BSP_CAN_ProcessErrors();
    assert(log_count==1 && strstr(last_log,"error=0x00001281"));
    assert(strstr(last_log,"recent_tx=0x00000400 func=0x36"));
    assert(strstr(last_log,"info_valid=1 status=0x00000088 tx_err=190 rx_err=0"));
    assert(s_can_errors.pending_count==0 && s_can_errors.total_count==1);
    BSP_CAN_ProcessErrors(); assert(log_count==1);

    reset_fixture(); emit(CAN_EVENT_ERR_GLOBAL,0x80020000u);
    BSP_CAN_ProcessErrors();
    assert(strstr(last_log,"events=0x00001000 error=0x80020000"));

    reset_fixture(); now_ms=10; emit(CAN_EVENT_ERR_CHANNEL,0x1281u);
    now_ms=15; emit(CAN_EVENT_ERR_GLOBAL,0x20000u);
    BSP_CAN_ProcessErrors();
    assert(log_count==1 && strstr(last_log,"count=2 total=2"));
    assert(strstr(last_log,"events=0x00001100 error=0x00021281 first_ms=10 last_ms=15"));

    reset_fixture();
    const can_event_t legacy[]={CAN_EVENT_ERR_BUS_OFF,CAN_EVENT_ERR_PASSIVE,
        CAN_EVENT_ERR_WARNING,CAN_EVENT_BUS_RECOVERY,CAN_EVENT_ERR_BUS_LOCK};
    for(unsigned i=0;i<sizeof(legacy)/sizeof(legacy[0]);i++) emit(legacy[i],0);
    assert(log_count==0 && s_can_errors.pending_count==5);
    BSP_CAN_ProcessErrors(); assert(log_count==1 && strstr(last_log,"events=0x0000009E"));

    reset_fixture(); emit(CAN_EVENT_FIFO_MESSAGE_LOST,0);
    emit(CAN_EVENT_MAILBOX_MESSAGE_LOST,0); BSP_CAN_ProcessErrors();
    assert(strstr(last_log,"events=0x00004020"));

    reset_fixture();
    for(unsigned i=0;i<1000;i++) { now_ms=i; emit(CAN_EVENT_ERR_CHANNEL,0x1000u); }
    assert(log_count==0); BSP_CAN_ProcessErrors();
    assert(log_count==1 && strstr(last_log,"count=1000 total=1000"));

    reset_fixture(); emit(CAN_EVENT_ERR_CHANNEL,0x1281u);
    inject_during_log=true; BSP_CAN_ProcessErrors();
    assert(log_count==1 && s_can_errors.pending_count==1);
    assert(s_can_errors.error_code==0x400u && s_can_errors.total_count==2);
    BSP_CAN_ProcessErrors();
    assert(log_count==2 && strstr(last_log,"count=1 total=2"));
    assert(strstr(last_log,"error=0x00000400"));

    reset_fixture(); emit(CAN_EVENT_ERR_CHANNEL,0x1281u);
    ipsr=16; BSP_CAN_ProcessErrors(); ipsr=0;
    primask=1; BSP_CAN_ProcessErrors();
    assert(log_count==0 && primask==1 && s_can_errors.pending_count==1);
    emit(CAN_EVENT_ERR_GLOBAL,0x20000u); assert(primask==1);
    primask=0; BSP_CAN_ProcessErrors(); assert(log_count==1 && primask==0);

    reset_fixture(); info_result=1; emit(CAN_EVENT_ERR_CHANNEL,0x1281u);
    BSP_CAN_ProcessErrors();
    assert(strstr(last_log,"info_valid=0 status=0x00000000 tx_err=0 rx_err=0"));
    assert(strstr(last_log,"error=0x00001281"));

    reset_fixture();
    const uint8_t angle[]={0x36,0,0,0,0,1,0x6B}; receive(0x400,angle,7);
    assert(g_can_context.motor_rx_flag[3]==1 && g_can_context.motor_rx_dlc[3]==7);
    const uint8_t stop[]={0xFE,0x02,0x6B}; receive(0x400,stop,3);
    assert(g_can_context.motor_stop_flag[3]==1);
    const uint8_t ready[]={0xFD,0x9F,0x6B}; receive(0x400,ready,3);
    assert(g_robot.joints[3].status==ROBOT_STATUS_READY);
    emit(CAN_EVENT_TX_COMPLETE,0); assert(g_can_tx_complete);
    emit(CAN_EVENT_TX_FIFO_EMPTY,0); BSP_CAN_ProcessErrors(); assert(log_count==0);

    reset_fixture(); now_ms=UINT32_MAX; emit(CAN_EVENT_ERR_CHANNEL,0x1000u);
    now_ms=2; emit(CAN_EVENT_ERR_CHANNEL,0x200u); BSP_CAN_ProcessErrors();
    assert(strstr(last_log,"first_ms=4294967295 last_ms=2"));

    reset_fixture(); write_result=1;
    assert(BSP_CAN_Send(0x400,CAN_ID_MODE_EXTENDED,command,2)==1);
    assert(!g_can_tx_complete && primask==0 && s_can_errors.pending_count==0);

    puts("13 actual-C CAN error diagnostic scenarios passed (no hardware)");
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", default=shutil.which("gcc") or
                        r"E:\STM32\STM32Tool\CLion\CLion 2025.2.3\bin\mingw\bin\gcc.exe")
    args = parser.parse_args()
    env = os.environ.copy()
    env["PATH"] = str(Path(args.gcc).parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="can_error_diagnostics_") as directory:
        exe = Path(directory) / "check.exe"
        subprocess.run([args.gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-x", "c", "-", "-o", str(exe)],
                       input=build_source(), text=True, encoding="utf-8", check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == "__main__":
    main()
