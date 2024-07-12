"""
Python based utility helpers
--------------------------------

"""

def in_ipynb():
    """Checks if current script is executed in ipython notebook
    Returns:
        bool: True if currently in ipython mode
    """
    try:
        cfg = get_ipython().config
        if cfg['IPKernelApp']['parent_appname'] == 'ipython-notebook':
            return True
        else:
            return False
    except NameError:
        return False


def is_ipython():
    return in_ipynb()