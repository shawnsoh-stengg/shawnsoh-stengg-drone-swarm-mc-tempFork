"""
Minimal rosbag2 loop-playback helper for the TP-MC-001 §2.4 mock nodes.

Test fixture bags are a few seconds long, so the whole bag is loaded into
memory up front and replayed on a loop, preserving the original
inter-message timing. This is deliberately not `ros2 bag play` (which is
oriented around a one-shot CLI process, not an in-process, loopable,
multi-topic source a mock node's own timer can drive).
"""
from __future__ import annotations

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


class BagPlayback:

    def __init__(self, bag_path: str):
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'
        )
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

        messages = []
        while reader.has_next():
            topic, data, t = reader.read_next()
            msg_type = get_message(topic_types[topic])
            messages.append((t, topic, deserialize_message(data, msg_type)))
        del reader

        if not messages:
            raise ValueError(f'bag at {bag_path!r} contains no messages')
        messages.sort(key=lambda entry: entry[0])
        t0 = messages[0][0]
        self._messages = [(t - t0, topic, msg) for t, topic, msg in messages]
        self._duration_ns = self._messages[-1][0]
        self._next_index = 0
        self._loop_start_ns = 0

    @property
    def duration_ns(self) -> int:
        return self._duration_ns

    @property
    def message_count(self) -> int:
        return len(self._messages)

    def due(self, elapsed_ns: int):
        """
        Return messages whose scheduled offset has now arrived.

        Call periodically with monotonically increasing elapsed_ns since
        playback started. Returns the list of (topic, msg) whose scheduled
        offset has now arrived, advancing an internal cursor and looping
        back to the start once the bag's duration has elapsed.
        """
        out = []
        rel = elapsed_ns - self._loop_start_ns
        if rel > self._duration_ns and self._next_index >= len(self._messages):
            self._loop_start_ns = elapsed_ns
            self._next_index = 0
            rel = 0
        messages = self._messages
        while self._next_index < len(messages) and messages[self._next_index][0] <= rel:
            _, topic, msg = messages[self._next_index]
            out.append((topic, msg))
            self._next_index += 1
        return out
