# Configuration file for the Sphinx documentation builder.
#
# Documentation is extracted from docstrings in the compiled C++ extension module.

import os
import sys
import locale

# Set locale to avoid locale errors during build
try:
    locale.setlocale(locale.LC_ALL, 'C.UTF-8')
except locale.Error:
    try:
        locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')
    except locale.Error:
        pass  # Fallback to default

# Add libfranka to library path so the compiled extension can be imported
libfranka_paths = [
    '/usr/local/lib',
]
ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
new_paths = ':'.join([p for p in libfranka_paths if os.path.exists(p)])
if new_paths:
    if ld_library_path:
        os.environ['LD_LIBRARY_PATH'] = f"{new_paths}:{ld_library_path}"
    else:
        os.environ['LD_LIBRARY_PATH'] = new_paths

# Add the parent directory to sys.path to import the module
sys.path.insert(0, os.path.abspath('..'))

# -- Project information -----------------------------------------------------
project = 'pylibfranka'
copyright = '2025, Franka Robotics GmbH'
author = 'Franka Robotics GmbH'

# Get version from _version.py
import pylibfranka
version = pylibfranka.__version__

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.mathjax',
    'sphinx.ext.inheritance_diagram',
    'sphinx.ext.graphviz',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = 'pydata_sphinx_theme'
html_static_path = []

html_theme_options = {
    "logo": {
        "text": f"pylibfranka v{version}",
    },
    "github_url": "https://github.com/frankarobotics/libfranka",
    "navbar_end": ["navbar-icon-links", "search-field"],
    "show_nav_level": 2,
    "navigation_depth": 3,
    "show_toc_level": 2,
    "secondary_sidebar_items": ["page-toc"],
}

html_context = {
    "default_mode": "dark",
    "display_version": True,
}

# Add version to HTML title
html_title = f"pylibfranka {version} documentation"

# -- Extension configuration -------------------------------------------------

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'undoc-members': True,
    'show-inheritance': True,
    'inherited-members': False,
}

autodoc_typehints = 'description'
autodoc_typehints_description_target = 'documented'

# Autosummary settings
autosummary_generate = True

# Napoleon settings to parse @param and @return tags
napoleon_google_docstring = False
napoleon_numpy_docstring = False
napoleon_use_param = True
napoleon_use_rtype = True
napoleon_custom_sections = [('Returns', 'params_style')]

# Intersphinx mapping
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
}

# Inheritance diagram settings
inheritance_graph_attrs = dict(rankdir="TB", size='"8.0, 10.0"', fontsize=14, ratio='auto')
inheritance_node_attrs = dict(shape='box', fontsize=14, height=0.5, color='dodgerblue', style='filled', fillcolor='lightyellow')
