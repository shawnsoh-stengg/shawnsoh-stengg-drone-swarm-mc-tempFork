from gcs_test_panel.core.statustext import StatustextAssembler


class Msg:
    def __init__(self, text, id=0, chunk_seq=0, sysid=1, compid=191, severity=6):
        self.text, self.id, self.chunk_seq, self.severity = text, id, chunk_seq, severity
        self._src = (sysid, compid)

    def get_srcSystem(self):
        return self._src[0]

    def get_srcComponent(self):
        return self._src[1]


def test_single_frame_text_passes_through():
    assert StatustextAssembler().feed(Msg("FCC link not alive"), 0.0) == "FCC link not alive"


def test_chunks_join_in_order_on_the_short_last_chunk():
    a = StatustextAssembler()
    assert a.feed(Msg("x" * 50, id=7, chunk_seq=0), 0.0) is None
    assert a.feed(Msg("tail", id=7, chunk_seq=1), 0.1) == "x" * 50 + "tail"


def test_exact_multiple_ends_on_empty_terminator():
    a = StatustextAssembler()
    assert a.feed(Msg("a" * 50, id=3, chunk_seq=0), 0.0) is None
    assert a.feed(Msg("b" * 50, id=3, chunk_seq=1), 0.0) is None
    assert a.feed(Msg("", id=3, chunk_seq=2), 0.0) == "a" * 50 + "b" * 50


def test_interleaved_ids_do_not_mix():
    a = StatustextAssembler()
    assert a.feed(Msg("a" * 50, id=1), 0.0) is None
    assert a.feed(Msg("b" * 50, id=2), 0.0) is None
    assert a.feed(Msg("A", id=1, chunk_seq=1), 0.0) == "a" * 50 + "A"
    assert a.feed(Msg("B", id=2, chunk_seq=1), 0.0) == "b" * 50 + "B"


def test_lost_middle_chunk_shows_a_gap():
    a = StatustextAssembler()
    a.feed(Msg("a" * 50, id=4, chunk_seq=0), 0.0)
    assert a.feed(Msg("end", id=4, chunk_seq=2), 0.0) == "a" * 50 + "…" + "end"


def test_stale_partial_is_dropped():
    a = StatustextAssembler(timeout_s=2.0)
    a.feed(Msg("a" * 50, id=5, chunk_seq=0), 0.0)
    assert a.feed(Msg("late", id=5, chunk_seq=1), 10.0) == "…late"
