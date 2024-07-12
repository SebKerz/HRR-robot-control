"""
File I/O helper functions
--------------------------
"""
from pathlib import Path
from types import SimpleNamespace
import json
import pickle
import warnings

__all__ = ["file_exists", "dir_contains", "rmdir", "create_dir", "assert_path",
           "save_json", "load_json", "load_pickle", "save_pickle"]


def file_exists(fname):
    """
    Check if file exists

    Args:
        fname(str or Path)

    Returns:
        bool: True if file exists, else False
    """
    f = Path(fname)
    return f.exists()


def file_nonexistent(fname):
    return not file_exists(fname)


def dir_contains(d, f):
    """
    Check if directory contains file f

    Args:
        d(str or Path):  directory to be checked
        f(str):  file nem to be found

    Returns:
        bool: True if d contains f

    Raises:
        AssertionError: If d does not exist
    """
    tmp_dir = Path(d)
    assert tmp_dir.exists()
    find_results = list(tmp_dir.glob(f"**/{f}"))
    return len(find_results) > 0


def rmdir(path):
    """
    Recursively remove path
    Args:
        path(Path):
    """
    for item in path.iterdir():
        try:
            item.unlink()
        except OSError:
            rmdir(item)


def create_dir(dir_path: str or Path, base_dir: str or Path or None = None) -> bool:
    """
    Create directory. In case the directory exists

    Args:
        dir_path(str or Path):  absolute or relative path of new directory
        base_dir(str or Path or None):  optional base path that is assumed to exist
    Returns:
        bool: True if directory has been successfully created or emptied
    """
    if base_dir is None:
        dir_path = Path(dir_path).expanduser()
    else:
        dir_path = assert_path(base_dir) / Path(dir_path)
    if dir_path.exists():
        in_key = input(f"{dir_path} exists. Erase content?[y/N]")
        if in_key.lower() == "y":
            rmdir(dir_path)
        else:
            return False
    else:
        dir_path.mkdir(parents=True)
    return True


def assert_path(p: str or Path) -> Path:
    p = Path(p).expanduser()
    if not p.exists():
        p.mkdir(parents=True)
    return p


def _gen_fname(fname, ending, force_overwrite=False, keep_name=True):
    """
    Args:
        fname:
        ending:
        force_overwrite:
        keep_name:

    Raises:
        RuntimeError: If file exists, but keep_name is set to True and force_overwrite is False Returns:

    Returns:
        str: file name of altered or original file

    """

    def full_file():
        return path / f"{file_name}.{ending}"

    p_file = Path(fname)
    path = p_file.parent
    file_name = p_file.name.split(".")[0]

    if file_exists(full_file()):
        if keep_name:
            if not force_overwrite:
                raise RuntimeError(f"File {p_file} exists, but neither name change nor overwrite is allowed")
        else:
            try:
                num = int(file_name.split("_")[-1]) + 1
            except ValueError:
                num = 1
            file_name = f"{file_name}_{num:03}"
    return full_file()


def save_json(data, fname, force_overwrite=False, keep_name=True):
    """
    Save data dict to file

    Args:
        data(dict or object):  data to be stored
        fname(str or Path): full path to file to store data at
        force_overwrite(bool):  force data overwrite
        keep_name(bool): If True, file name gets altered if file already exists
    Raises:
        RuntimeError: If file exists, but keep_name is set to True and force_overwrite is False
    """
    with open(_gen_fname(fname, "json", force_overwrite, keep_name), 'w') as f:
        json.dump(data, f)


def load_pickle(fname, to_ns=False):
    """
    Reads a pickled file if possible

    Args:
        fname(str or Path):

    Returns:
        dict: pickle content
        to_ns(bool): converts dict to SimpleNamespace

    Raises:
        AssertionError: if file does not exists
    """
    if file_exists(fname):
        with open(fname, 'rb') as pkl:
            data = pickle.load(pkl)
        if to_ns:
            try:
                return SimpleNamespace(**data)
            except TypeError:
                pass
        return data
    raise AssertionError(f"file {fname} does not exist")


def save_pickle(data, fname, force_overwrite=False, keep_name=True):
    """save data to file"""
    with open(_gen_fname(fname, "pkl", force_overwrite, keep_name), 'wb') as f:
        pickle.dump(data, f)


def load_json(fname):
    """
    Reads a json file if possible

    Args:
        fname(str or Path):

    Returns:
        dict: yaml content

    Raises:
        AssertionError: if file does not exists
    """
    if file_exists(fname):
        with open(fname, 'r') as js_file:
            file_content = js_file.read()
        return json.loads(file_content)
    raise AssertionError(f"file {fname} does not exist")


# 3rd party dependent functions
# YAML
try:
    import yaml

    __all__ += ["save_yaml", "load_yaml"]


    def save_yaml(data, fname, force_overwrite=False, keep_name=True):
        """
        Save data dict to file

        Args:
            data(dict or object):  data to be stored
            fname(str or Path): full path to file to store data at
            force_overwrite(bool):  force data overwrite
            keep_name(bool): If True, file name gets altered if file already exists
        Raises:
            RuntimeError: If file exists, but keep_name is set to True and force_overwrite is False
        """
        with open(_gen_fname(fname, "yaml", force_overwrite, keep_name), 'w') as f:
            yaml.dump(data, f)


    def load_yaml(fname):
        """
        Reads a json file if possible

        Args:
            fname(str or Path):

        Returns:
            dict: yaml content

        Raises:
            AssertionError: if file does not exists
        """

        if file_exists(fname):
            with open(fname, 'r') as js_file:
                file_content = js_file.read()
            return yaml.load(file_content)
        raise AssertionError(f"file {fname} does not exist")

except ModuleNotFoundError:
    warnings.warn("Module yaml no found, please install via:\n\tpip install pyyaml")

# Pandas
try:
    import pandas as pd

    __all__ += ["load_csv", "data2pd"]


    def load_csv(fname):
        return pd.read_csv(fname, index_col=None, comment='#')


    def data2pd(data, **kwargs):
        """
        convert data dict to panda dataframe

        Args:
            data(dict): data to be parsed

        Returns:
            pandas dataframe
        """
        return pd.DataFrame(data, **kwargs)

except ModuleNotFoundError:
    warnings.warn("Pandas module not found, please install via \n\tpip install pandas")

# scipy
try:
    import scipy.io

    __all__ += ["load_mat", "batch_mat2pkl"]


    def load_mat(fname, field_name=None):
        """
        Error prone function that loads a mat file to python

        Args:
            fname(str or Path): path to file that is to be loaded
            field_name(str or None): optional field name to select subcontent from mat file
        Returns:
            np.ndarray: structured array with matlab content

        Raises:
            AssertionError: if file does not exists
        """

        if file_exists(fname):
            mat_data = scipy.io.loadmat(fname, squeeze_me=True)
            if field_name is not None:
                try:
                    mat_data = mat_data[field_name]
                except KeyError:
                    print(f"{field_name} is no valid key for {fname}")
            return mat_data
        raise AssertionError(f"file {fname} does not exist")


    def batch_mat2pkl(mat_path, save_path=None, field_name=None):
        path = assert_path(mat_path)
        if save_path is None:
            save_path = path
        else:
            save_path = assert_path(save_path)
        for f in path.glob("*.mat"):
            save_pickle(load_mat(f, field_name), save_path / f.name)

except ModuleNotFoundError:
    warnings.warn("Scipy is not installed" +
                  " please refer https://docs.scipy.org/doc/scipy/reference/io.html")

if __name__ == "__main__":
    m_path = (Path(__file__) / ".." / ".." / ".." / "data" / "2Dletters_mat").resolve()
    s_path = (Path(__file__) / ".." / ".." / ".." / "data" / "2Dletters").resolve()
    batch_mat2pkl(m_path, s_path, "demos")
