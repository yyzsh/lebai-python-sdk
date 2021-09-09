# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'lebai'
copyright = '2021, kingfree'
author = 'kingfree'

# The full version, including alpha/beta/rc tags

release = '0.7.6'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'recommonmark',
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.napoleon',
    'sphinx_rtd_theme'
]

html_theme = "sphinx_rtd_theme"
# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']
html_theme_options = {

    'navigation_depth': 4,
}

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = 'zh_CN'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# import sphinx_rtd_theme

html_theme = "sphinx_rtd_theme"
# html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
# html_logo = "../_static/logo.png"
# html_theme_options = {
#     "use_edit_page_button": False,
#     "icon_links": [
#         {
#             "name": "GitHub",
#             "url": "https://github.com/lebai-robotics/lebai-python-sdk",
#             "icon": "fab fa-github-square",
#         }
#     ],
# }
# html_context = {
#     # "github_url": "https://github.com", # or your GitHub Enterprise interprise
#     "github_user": "https://github.com/lebai-robotics",
#     "github_repo": "https://github.com/lebai-robotics/lebai-python-sdk",
#     "github_version": "main",
# }
# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_p = ['_static']

import os
import sys

sys.path.insert(0, os.path.abspath('..'))
