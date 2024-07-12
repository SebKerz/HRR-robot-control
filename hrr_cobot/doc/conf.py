# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.abspath('../*/src'))
sys.path.insert(0, os.path.abspath('../*/scripts'))

# -- Project information -----------------------------------------------------
release = "public"
project = "HR-Recycler industrial 'cobot' docu"
copyright = "Copyright (C) 2019-2022 TUM LSR"
author = "v.gabler@tum.de"
version = "0.3.0"
# -- General configuration ---------------------------------------------------
on_rtd = os.environ.get('READTHEDOCS', None) == 'True'
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.todo',
    'sphinxcontrib.napoleon',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx_click.ext',
    'nbsphinx'
]

suppress_warnings = [
    'nbsphinx',
]
nbsphinx_custom_formats = {
    '.pct.py': ['jupytext.reads', {'fmt': 'py:percent'}],
    '.md': ['jupytext.reads', {'fmt': 'Rmd'}]
}
mathjax3_config = {
    'tex': {'tags': 'ams', 'useLabelIds': True},
}
## nb sphinx setup
nbsphinx_execute_arguments = [
    "--InlineBackend.figure_formats={'svg', 'pdf'}",
    "--InlineBackend.rc=figure.dpi=96",
]
autodoc_default_flags = ['members', 'undoc-members', 'private-members', 'special-members',
                         'undoc-members', 'inherited-members', 'show-inheritance']

# napoleon settings
napoleon_include_init_with_doc = True
napoleon_use_ivar = True
napoleon_use_param = False
napoleon_use_rtype = False
source_suffix =  {
    '.rst': 'restructuredtext',
    # '.md': 'markdown'
}
master_doc = 'index'
language = None
exclude_patterns = []
pygments_style = 'sphinx'
add_module_names = False
show_authors = True

# -- Options for HTML output -------------------------------------------------
html_domain_indices = False
html_use_index = True
html_split_index = True
html_show_sourcelink = True
html_help_basename = '{} help'.format(project)
html_theme = 'sphinx_rtd_theme'
html_sidebars = {
    'content':['localtoc.html'],
    'docu/nodes':['localtoc.html'],
}
html_theme_options = {
    'globaltoc_includehidden': "false",
}

# -- Options for LaTeX output ------------------------------------------------
man_pages = [
    (master_doc, project, '{} Documentation'.format(project),
     [author], 1)
]
todo_include_todos = True


