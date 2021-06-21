#!/bin/bash

# NOTES
#Requires Py3.7: https://linuxize.com/post/how-to-install-python-3-7-on-ubuntu-18-04/
#Update to 3.7 as default for python3
#python -m pip install --upgrade pip
#pip3 install git+https://github.com/DataDog/mkdocs-click.git
#pip3 install mktheapidocs

# To publish a branch of stretch_docs during dev.
# set
# site_url: https://docs.hello-robot.com/stretch_docs in the yaml
# then from hello-robot.github.io:
# python3 -m mkdocs gh-deploy --config-file ../stretch_docs/mkdocs.yml --remote-branch feature/<branch name>
#will publish to https://docs.hello-robot.com/stretch_docs/

pip3 install mkdocs
pip3 install mkdocs-material
pip3 install mkdocs-simple-plugin
pip3 install install Pygments


# #####################################
#This repository generates the documentation hosted at [docs.hello-robot.com](docs.hello-robot.com).
#The following command will serve the static website with hot-reloading (i.e. your edits are reflected in real time).

# $ python3 -m mkdocs serve

## Deploying

#Next to this repository, clone [hello-robot/hello-robot.github.io](https://github.com/hello-robot/hello-robot.github.io).
# The `deploy.sh` script with automatically build the docs, replace the contents of hello-robot.github.io, and push the new docs to master.
# Github will reflect the changes at [docs.hello-robot.com](docs.hello-robot.com).

#$ ./deploy.sh

