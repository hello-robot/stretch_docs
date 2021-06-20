#!/bin/bash

#Requires Py3.7: https://linuxize.com/post/how-to-install-python-3-7-on-ubuntu-18-04/
#Update to 3.7 as default for python3
#python -m pip install --upgrade pip

pip3 install mkdocs
pip3 install mkdocs-material
pip3 install mkdocs-simple-plugin
pip3 install mktheapidocs
pip3 install install Pygments
pip3 install git+https://github.com/DataDog/mkdocs-click.git

#This repository generates the documentation hosted at [docs.hello-robot.com](docs.hello-robot.com).
#The following command will serve the static website with hot-reloading (i.e. your edits are reflected in real time).

# $ python3 -m mkdocs serve

## Deploying

#Next to this repository, clone [hello-robot/hello-robot.github.io](https://github.com/hello-robot/hello-robot.github.io).
# The `deploy.sh` script with automatically build the docs, replace the contents of hello-robot.github.io, and push the new docs to master.
# Github will reflect the changes at [docs.hello-robot.com](docs.hello-robot.com).

#$ ./deploy.sh

