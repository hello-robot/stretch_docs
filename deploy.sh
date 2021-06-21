#!/bin/bash
#rm -rf ./site
cd ../hello-robot.github.io/
mkdocs gh-deploy --config-file ../stretch_docs/mkdocs.yml --remote-branch master

