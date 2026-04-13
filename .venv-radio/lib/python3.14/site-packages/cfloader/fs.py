import pathlib
from typing import Any, List, Union

import cfloader.readers as readers
from cfloader.loader import Loader


def open(path: Union[pathlib.Path, str, dict, readers.Reader]) -> Loader:
    """This methods reads configuration from a path to a file, a dict or a reader
    object and returns a Loader object to read objects from the configuration.

    Args:
        path (Union[pathlib.Path, str, dict, readers.Reader]): A path-like string or Path, dict or a configuration reader object

    Returns:
        Loader: An object that can read objects from the configuration
    """

    if isinstance(path, readers.Reader):
        return Loader(path.read_config())

    reader: readers.Reader  # forward-declaration for types

    if isinstance(path, pathlib.Path) or isinstance(path, str):
        reader = readers.Path(path)

    if isinstance(path, dict):
        reader = readers.Dict(path)

    return Loader(reader.read_config())
