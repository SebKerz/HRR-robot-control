# HRR-Cobot Meta-package

This is the meta package that is mainly used to
generate the project (python-code-)documentation.

Please refer to [data->README.md](../data/README.md) for additional instructions how
to set up your PC in order to use / generate the documentation

## Generate Sphinx-documentation

in order to run this documentation, please install the additional packages

```bash
conda activate hrr
conda install -c conda-forge nbsphinx sphinx_rtd_theme recommonmark jupytext
```

Afterwards, simply navigate into this directory

```bash
roscd hrr_cobot
```

and run

```bash
make html
```

to generate the sphinx docu, which can then be opened via e.g. firefox

```bash
firefox ./build/doc/html/index.html
```

