import matplotlib.pylab as plt
import seaborn as sns
from .ipy_utils import in_ipynb

__all__ = ["get_axis", "set_2Dplot_properties", "set_ipy_plot"]


def shared_plot_rc_settings():
    return {'xtick.minor.visible': True,
            'ytick.minor.visible': True,
            'legend.framealpha': 0.5}


def set_ipy_plot(ncols=1, nrows=1):
    """
    Set Ipython plot configt

    Args:
        ncols:
        nrows:
    """
    rc_params = shared_plot_rc_settings()
    rc_params['figure.figsize'] = [15 * ncols, 6 * nrows]
    rc_params['legend.markerscale'] = 2.0
    rc_params['font.size'] = 14.0
    rc_params['axes.labelsize'] = 10
    rc_params['axes.titlesize'] = 10
    rc_params["figure.titlesize"] = 24.0

    sns.set_theme(context="notebook",
                  style="darkgrid",
                  font="sans-serif",
                  font_scale=2.0,
                  color_codes=True,
                  rc=rc_params)
    plt.rc('legend', markerscale=1.0, fontsize=22.0)


def set_src_plot(ncols=1, nrows=1):
    """
    Set Ipython plot configt

    Args:
        ncols(int): number of columns
        nrows(int): number of rows

    """
    rc_params = shared_plot_rc_settings()
    rc_params['figure.figsize'] = [18 * ncols, 6 * nrows]
    rc_params['font.size'] = 12.0
    rc_params['axes.labelsize'] = 10
    rc_params['axes.titlesize'] = 10
    rc_params["figure.titlesize"] = 24.0
    rc_params['legend.markerscale'] = 1.0
    rc_params['legend.fontsize'] = 8.0
    sns.set_theme(context="paper",
                  style="darkgrid",
                  font="sans-serif",
                  color_codes=True,
                  rc=rc_params)


def set_2Dplot_properties(ncols=1, nrows=1):
    if in_ipynb():
        set_ipy_plot(ncols, nrows)
    else:
        set_src_plot(ncols, nrows)


def get_axis(title, ncols=1, nrows=1, ax_handle=None):
    if ax_handle is None:
        set_2Dplot_properties()
        fig, ax = plt.subplots(ncols=ncols, nrows=nrows)
        fig.suptitle(title)
        plt.xticks()
        plt.yticks()
    else:
        ax = ax_handle
    return ax
