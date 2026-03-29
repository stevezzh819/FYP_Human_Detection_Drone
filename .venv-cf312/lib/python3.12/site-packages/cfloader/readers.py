import json
import pathlib
from abc import ABCMeta, abstractmethod
from typing import Union

PathLike = Union[pathlib.Path, str]


class ConfigFileNotFoundError(FileNotFoundError):
    def __init__(self, filepath, path):
        self.filepath = filepath

        super().__init__(f"configuration file {filepath} not found in archive {path}")


def _get_path(_path: PathLike) -> pathlib.Path:
    if isinstance(_path, str):
        return pathlib.Path(_path)

    return _path


class Reader(metaclass=ABCMeta):
    @abstractmethod
    def read_config(self) -> dict:
        raise NotImplementedError


class Path(Reader):
    path: pathlib.Path

    def __init__(self, path: PathLike):
        self.path = _get_path(path)

    def read_config(self):
        return json.loads(self.path.read_text())


class Dict(Reader):
    def __init__(self, config: dict):
        self.config = config

    def read_config(self):
        return self.config


class Archive(Reader):
    path: pathlib.Path
    config_filepath: str

    def __init__(self, path: PathLike, config_filepath: str = "config.json"):
        self.path = _get_path(path)
        self.config_filepath = config_filepath

    def _read_zip(self):
        import zipfile

        try:
            with zipfile.ZipFile(self.path, "r") as archive:
                with archive.open(self.config_filepath) as config_file:
                    return json.load(config_file)
        except KeyError:
            raise ConfigFileNotFoundError(self.config_filepath, self.path)

    def _read_tar(self, compression: str = ""):
        import tarfile

        with tarfile.open(self.path, ":".join(["r", compression])) as archive:
            try:
                config_fileinfo = archive.getmember(self.config_filepath)
            except KeyError:
                raise ConfigFileNotFoundError(self.config_filepath, self.path)

            config_file = archive.extractfile(config_fileinfo)

            if config_file is None:
                raise ConfigFileNotFoundError(self.config_filepath, self.path)

            return json.loads(config_file.read())

    def read_config(self):
        suffix = "".join(self.path.suffixes).lower()

        if suffix == ".zip":
            return self._read_zip()
        elif suffix == ".tar":
            return self._read_tar()
        elif suffix == ".tar.gz":
            return self._read_tar("gz")
        elif suffix == ".tar.xz":
            return self._read_tar("xz")
        else:
            raise Exception(f"unsupported archive format {suffix}")
