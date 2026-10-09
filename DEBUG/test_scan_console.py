"""Drive the Qt scan controls with fake serial channels; no hardware access."""
import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PySide6.QtWidgets import QApplication, QMessageBox
from UpperComputer import ra6m5_upper_console as ui
from UpperComputer import ra6m5_protocol as wire


class Settings:
    def __init__(self): self.data = {}
    def value(self, key, default=None): return self.data.get(key, default)
    def setValue(self, key, value): self.data[key] = value
    def sync(self): pass


class Serial:
    def __init__(self): self.frames=[]
    def is_open(self): return True
    def write(self, frame, label): self.frames.append(frame)
    def close(self): pass


def main():
    app = QApplication.instance() or QApplication([])
    with tempfile.TemporaryDirectory(prefix="scan_gui_") as directory:
        w=ui.QtUpperConsole(Path(directory))
        w.settings=Settings(); w.jetson=Serial(); w.arm=Serial()
        w.start_heartbeat=lambda:None;w.stop_heartbeat=lambda:None
        w.protocol_mode.setCurrentIndex(w.protocol_mode.findData(ui.JETSON_PROTOCOL_NEW))
        QMessageBox.question=lambda *_args:QMessageBox.Yes
        def reply(event,state,flags=0,action=1,seq=None,error=0):
            payload=bytes([event,state,flags,action,1,error])+w.scan_client.scan_id.to_bytes(4,"little")
            w.on_serial_data("JETSON",wire.build_unified_frame(0x82,seq or w.scan_client.start_seq,payload))
            w._flush_logs()
        def workflow(value,seq,error=0,event=0x20):
            w.on_serial_data("JETSON",wire.build_unified_frame(0x81,seq,bytes([event,value,error])))
            w._flush_logs()
        assert not w.scan_photo_ok.isEnabled()
        w.scan_begin_measure();assert(w.automatic_demo_owner=="scan")
        workflow(2,w.scan_measure_seq);assert w.scan_measure_ready
        # Another round's late safe event must not arm the scan.
        workflow(3,(w.scan_measure_seq%255)+1);assert not w.scan_distance_safe
        w._scan_distance_sample(400);w._scan_distance_sample(400);w._scan_distance_sample(400)
        workflow(3,w.scan_measure_seq);assert w.scan_distance_safe
        w.scan_start(); assert w.scan_client.active
        start_seq=w.scan_client.start_seq
        for _ in range(600):assert w._next_unified_seq()!=start_seq
        reply(1,1);reply(3,3);assert w.scan_photo_ok.isEnabled()
        w.scan_photo(True); assert not w.scan_photo_ok.isEnabled()
        photo_seq=w.scan_client.photo_seq
        reply(1,4,1,2,photo_seq);reply(4,4,1,2,photo_seq)
        reply(5,5,3);reply(6,10,7);assert not w.scan_client.confirmed
        reply(8,11,7);assert w.scan_client.confirmed
        before=len(w.jetson.frames);w.send_target_ctrl(True);w.send_capture_ctrl(3,2)
        assert len(w.jetson.frames)==before
        # Full scan issued no extra serial HOME/SELECT.
        assert [f[6] for f in w.jetson.frames if f[3]==7 and f[6]!=0]==[1,2]
        assert not [f for f in w.jetson.frames if f[3]==4]
        w.scan_finish();assert w.automatic_demo_owner=="scan" and not w.scan_client.confirmed
        reply(2,12,0,0,seq=w._next_unified_seq())
        assert w.automatic_demo_owner=="scan"
        workflow(5,w.scan_finish_seq);assert w.automatic_demo_owner is None
        # Cancellation must not be undone by a delayed final success.
        w.scan_client.start(200,50,0);w.automatic_demo_owner="scan";w.scan_abort()
        abort_seq=w.scan_abort_seq; deadline=w.scan_abort_deadline
        workflow(1,abort_seq,event=0x21);assert w.automatic_demo_owner=="scan"
        w.scan_abort();assert w.scan_abort_seq==abort_seq and w.scan_abort_deadline==deadline
        reply(7,8,error=0x0F);assert w.automatic_demo_owner=="scan"
        reply(8,11,7);assert not w.scan_client.confirmed
        workflow(6,abort_seq+1);assert w.automatic_demo_owner=="scan"
        workflow(6,w.scan_abort_seq);assert w.automatic_demo_owner is None
        w.automatic_demo_owner="scan";w.scan_abort();workflow(7,w.scan_abort_seq,error=6)
        assert w.automatic_demo_owner=="scan" and w.scan_stop_failed
        assert "停止未确认" in w.scan_stage.text()
        before=len(w.jetson.frames);w.scan_begin_measure();assert len(w.jetson.frames)==before
        workflow(6,w.scan_abort_seq);assert w.automatic_demo_owner=="scan"
        w.scan_abort();w.scan_abort_deadline=0;w._flush_logs()
        assert w.scan_stop_failed and w.automatic_demo_owner=="scan"
        workflow(6,w.scan_abort_seq);assert w.automatic_demo_owner=="scan"
        w.scan_abort();workflow(6,w.scan_abort_seq);assert w.automatic_demo_owner is None
        w.scan_client.active=False;w.close();app.processEvents()
    print("Qt scan controls passed with fake serial (no hardware)")


if __name__ == "__main__":
    main()
