import importlib
import inspect
from copy import deepcopy
from typing import Any, List, Optional
from types import SimpleNamespace

def _get_calling_module(lvl=3, offset=0):
    """Gets the module which called the caller of this function.

    Returns:
        ModuleType: The module object
    """
    # caller = inspect.stack()[lvl+offset]
    # module = inspect.getmodule(caller.frame)
    # return module

    stack = inspect.stack()
    module = SimpleNamespace(__name__=__name__)
    while module.__name__ == __name__:
        module = inspect.getmodule(stack.pop(0).frame)
    
    return module

class Loader:
    _config: Optional[dict] = None
    config: Optional[dict] = None

    def __init__(self, _config: dict):
        self._config = _config
        self.config = deepcopy(_config) # create a copy so we don't override anything

    def _get_value(self, compound_key: str) -> Optional[Any]:
        """Gets the object notated by `compound_key`, which is a dot-separated identifier
        for nested objects in the configuration.

        Args:
            compound_key (str): The path of the object to retrieve

        Returns:
            Optional[Any]: Object at path if found, None otherwise
        """
        keys = compound_key.split(".")

        anchor: Any = self._config

        while len(keys) > 0:
            key = keys.pop(0)

            if not key in anchor:
                return None

            anchor = anchor[key]

        return anchor

    def _load_class(self, key, obj, package):
        # TODO: Refactor this, the package derivation/overloading was 
        # incrementally developed; there might be many edge cases
        # and redundancies or conflicts. Think harder about how "smart"
        # we want it to be and remove unnecessary smartness, which can lead
        # to unexpected behaviour. Here be dragons!

        module: Any = None
        if isinstance(obj, str):
            # if the object is just a string, and as_class = True
            # try to load the class represented by the string
            class_path = obj
            params = {}
        elif isinstance(obj, dict):
            # if not, we need to instantiate the class from the parameters
            # given
            class_path_key = f"{key}_name"

            if class_path_key not in obj:
                raise KeyError(f"please specify {key}_name for instantiation")

            class_path = obj.pop(class_path_key)
            params = obj

            # nested dependencies
            for key, param in params.items():
                if isinstance(param, dict) and "load" in param and param["load"]:
                    if package is None:
                        # NOTE: Danger here, what happens if package is defined in the root call of load?
                        module = _get_calling_module()
                        package = module.__name__

                    # pass it recursively, but derive the package so that they are all loaded
                    params[key] = self.load(
                        key=param["key"],
                        as_class=param.get("as_class", False), # allow loading of dependencies as non-instantiated classes
                        package=package
                    )

        # package loading
        class_parts = class_path.split(".")
        class_name = class_parts.pop()

        if len(class_parts) > 0:
            # maybe the class name in the configuration is absolute? If yes, load it.
            module = importlib.import_module(".".join(class_parts))
        elif package is not None:
            if package.startswith("."):
                # load packages starting with a "." relative to the calling module
                module = _get_calling_module()
                package = module.__package__ + package
            module = importlib.import_module(package)  # type: ignore

        if module is None:
            # if module is not identifiable, try to find the module of the calling
            # scope
            module = _get_calling_module()

        class_instantiator = getattr(module, class_name)

        return class_instantiator(**params)

    def load(
        self, key: str, as_class: bool = False, package: Optional[str] = None
    ) -> Any:
        """Loads an object or an instantiated class

        Args:
            key (str): Compound key representing the path of the object
            as_class (bool, optional): Whether to load this object as an instantiated class or not. Defaults to False.
            package (Optional[str], optional): If `as_class = True`, explicitly state which package to load it from, if None the loader will try its best to derive the package. Defaults to None.

        Raises:
            KeyError: Raised if the class name cannot be derived

        Returns:
            Any: An object or an instantiated class
        """
        obj = self._get_value(key)

        if as_class:
            return self._load_class(key, obj, package)

        return obj
