from typing import List
from threading import Lock

from tycho_env.utils import GenericMessageSubscriber, print_and_cr, numpify


def record_topic(state, topic: str):
    def callback(msg):
        try:
            value = numpify(msg)
            with state.rostopic_mutex:
                state.latest_ros_data[topic] = value
        except Exception as e:
            print_and_cr(f"ERROR: {str(e)}")
    sub = GenericMessageSubscriber(topic, callback, queue_size=10)
    state.topic_subs[topic] = sub


def add_ros_record_function(state, recorded_topics: List[str]):
    state.topic_subs = {}
    state.rostopic_mutex = Lock()
    state.latest_ros_data = {}

    if len(recorded_topics) > 0:
        state.pre_command_hooks["*"].append(pre_cmd_hook)

        for topic in recorded_topics:
            record_topic(state, topic)


def pre_cmd_hook(state):
    with state.rostopic_mutex:
        state.info.update(state.latest_ros_data)
