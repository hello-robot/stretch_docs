# stretch_docs

This repository generates the documentation hosted at [docs.hello-robot.com](docs.hello-robot.com).

## Setup

```bash
$ python3 -m pip install mkdocs
$ python3 -m pip install mkdocs-bootswatch
$ python3 -m pip install mkdocs-material
```

## Development

The following command will serve the static website with hot-reloading. There are two static websites in this repository. The main documentation is in the root of the repository (`http://127.0.0.1:8000`). The auto-generated API based on `stretch_body` is in the `api/` folder (`http://127.0.0.1:8000/stable/stretch_body/arm`).

```bash
$ cd api/ # optional
$ python3 -m mkdocs serve
```

## Deploying

Next to this repository, clone [hello-robot.github.io](https://github.com/hello-robot/hello-robot.github.io). The deploy script will automatically update the files in this repo with the new static website.

#cd ../hello-robot.github.io/
#mkdocs gh-deploy --config-file ../stretch_docs/mkdocs.yml --remote-branch master
#Sizing images: convert foo.png -resize x100 foo_rs.png
#Local site: 127.0.0.1:8000


