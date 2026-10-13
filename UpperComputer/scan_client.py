"""Scan-only client state, usable without Qt or a serial port."""

if __package__:
    from .ra6m5_protocol import Ra6Status, build_scan_control
else:
    from ra6m5_protocol import Ra6Status, build_scan_control


class ScanClient:
    def __init__(self):
        self.scan_id = 0
        self.start_seq = 0
        self.state = 0
        self.flags = 0
        self.active = False
        self.pending = None
        self.next_query = 0.0
        self.confirmed = False
        self.awaiting_query = set()
        self.cancelling = False
        self.stage_deadline = None

    def start(self, scan_id: int, seq: int, now: float) -> bytes:
        if self.active:
            raise ValueError("scan already active")
        frame = build_scan_control(1, 0, scan_id, seq)
        self.__init__()
        self.scan_id, self.start_seq = scan_id, seq
        self.active = True
        self.pending = (frame, seq, 1, now + 0.3, 0)
        self.next_query = now + 1
        self.stage_deadline = now + 5
        return frame

    def query(self, seq: int) -> bytes:
        self.awaiting_query.add(seq)
        return build_scan_control(0, 0, self.scan_id, seq)

    def receive(self, s: Ra6Status, now: float | None = None) -> bool:
        if not self.active or s.msg_type != 0x82 or s.scan_id != self.scan_id:
            return False
        correlated = ((s.request_action == 1 and s.seq == self.start_seq) or
                      (s.request_action == 0 and s.seq in self.awaiting_query))
        if not correlated:
            return False
        if s.event == 1:
            if self.pending and s.seq == self.pending[1] and s.request_action == self.pending[2]:
                self.pending = None
                if s.value == 0:
                    # Lost earlier ACK followed by BUSY is not proof that motion stopped.
                    self.confirmed = False
            return True
        if s.event == 2 and s.request_action == 0:
            self.awaiting_query.discard(s.seq)
        if s.scan_state != self.state and now is not None:
            budget = {0:5, 1:62, 2:32, 5:47, 10:62}.get(s.scan_state)
            self.stage_deadline = None if budget is None else now + budget
        self.state, self.flags = s.scan_state, s.scan_flags
        # Only the final event/current query, never ACK or HOME history, qualifies.
        self.confirmed = (not self.cancelling and s.error == 0 and s.value == 1 and s.scan_state == 11 and
                          s.scan_flags == 6 and (s.event == 8 or s.event == 2))
        if s.scan_state in (8, 9, 12):
            self.active = False
            self.pending = None
            self.confirmed = False
        return True

    def abort(self):
        self.cancelling = True
        self.pending = None
        self.confirmed = False

    def expired(self, now: float) -> bool:
        return (self.active and not self.cancelling and self.stage_deadline is not None and
                now >= self.stage_deadline)

    def poll(self, now: float, next_seq) -> list[bytes]:
        if not self.active:
            return []
        frames = []
        if self.pending and now >= self.pending[3]:
            frame, seq, action, _, tries = self.pending
            if tries < 3:
                frames.append(frame)
                self.pending = (frame, seq, action, now + 0.3, tries + 1)
            else:
                self.pending = None
        if now >= self.next_query:
            # Bound late/unanswered query correlation to one outstanding query.
            self.awaiting_query.clear()
            frames.append(self.query(next_seq()))
            self.next_query = now + 1
        return frames
