from pickle import Unpickler
from typing import Generator, Dict, Any

def read_raw_log(path: str) -> Generator[Dict[Any, list], None, None]:
    """
    Reads a raw log file and yields each entry.

    Args:
        path (str): The path to the raw log file.

    Yields:
        dict: An individual log entry, the value of `state.info`.

    Raises:
        FileNotFoundError: If the specified file path does not exist.

    """
    with open(path, "rb") as f:
        reader = Unpickler(f)
        try:
            while True:
                yield reader.load()
        except EOFError:
            pass
