from io import TextIOWrapper
from typing import Optional
import os

from queue import Queue
from threading import Lock, Thread
from copy import deepcopy
import time
from pickle import Pickler

from tycho_env.utils import print_and_cr, colors

# Singletons
STATE_QUEUE = Queue()
IS_RECORDING = False # Similar to state.curr_recording but flag for callback
WRITER_LOCK = Lock()
CURR_WRITER: Optional[Pickler] = None
CURR_FILE: Optional[TextIOWrapper] = None

def add_logger_function(state):
    state.handlers['r'] = _record
    state.handlers['R'] = _count_recording
    state.handlers['D'] = _delete_recording
    state.onclose.append(stop_recording)
    state.post_command_hooks["*"].append(post_cmd_callback)

    state.last_recording = None
    state.curr_recording = None

    if "save_record_folder" in state.params:
        state.save_record_folder = state.params["save_record_folder"]
    else:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        state.save_record_folder = os.path.join(dir_path, "..", "..", "recording")
    if not os.path.isdir(state.save_record_folder):
        print_and_cr(f"Creating folder for recordings: {state.save_record_folder}")
        os.mkdir(state.save_record_folder)
    
    Thread(target=recording_worker, daemon=True).start()

def recording_worker():
    while True:
        info = STATE_QUEUE.get()
        try:
            with WRITER_LOCK:
                CURR_WRITER.dump(info)
        finally:
            STATE_QUEUE.task_done()

def post_cmd_callback(state):
    if IS_RECORDING:
        info = deepcopy(state.info)
        STATE_QUEUE.put(info)

def _record(_, state):
    toggle_recording(state)

def _count_recording(_, state):
    n_recordings = len([f for f in os.listdir(state.save_record_folder) if f.endswith(".pkl")])
    print_and_cr(f"Number of recordings: {n_recordings}")

def _delete_recording(_, state):
    delete_last_recording(state)

def delete_last_recording(state):
    if state.curr_recording:
        stop_recording(state)
    if state.last_recording:
        print_and_cr(colors.bg.red + 'Deleting last recording' + colors.reset)
        os.remove(state.last_recording)
        state.last_recording = None

def set_recording(state, enabled: bool):
    if enabled != bool(state.curr_recording):
        toggle_recording(state)

def start_recording(state):
    if state.curr_recording:
        stop_recording(state)
    record_path = os.path.join(state.save_record_folder,
                               time.strftime('%y-%m-%d-%H-%M-%S.pkl', time.localtime()))
    print_and_cr(colors.bg.green + f"Recording to: {record_path}")

    global CURR_WRITER, IS_RECORDING, CURR_FILE
    assert STATE_QUEUE.empty()
    with WRITER_LOCK:
        CURR_FILE = open(record_path, "wb")
        CURR_WRITER = Pickler(CURR_FILE)
    IS_RECORDING = True
    state.curr_recording = record_path
    state.last_recording = record_path

def stop_recording(state):
    if state.curr_recording:
        print_and_cr(colors.bg.lightgrey + "Stop recording" + colors.reset)

        global IS_RECORDING, CURR_WRITER, CURR_FILE
        IS_RECORDING = False
        STATE_QUEUE.join()
        with WRITER_LOCK:
            CURR_FILE.close()
            CURR_FILE = None
            CURR_WRITER = None
        state.curr_recording = None

def toggle_recording(state):
    if state.curr_recording:
        stop_recording(state)
    else:
        start_recording(state)
