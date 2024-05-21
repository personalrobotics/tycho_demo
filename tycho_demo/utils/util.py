from pickle import Unpickler
from typing import Generator, Dict, Any

def read_raw_log(path: str) -> Generator[Dict[Any, list], None, None]:
    with open(path, "rb") as f:
        reader = Unpickler(f)
        try:
            while True:
                yield reader.load()
        except EOFError:
            pass
