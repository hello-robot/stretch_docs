# stretch_docs

This repository generates the documentation hosted at [docs.hello-robot.com](docs.hello-robot.com).

## Setup

```bash
$ python3 -m pip install mkdocs
$ python3 -m pip install mkdocs-bootswatch
$ python3 -m pip install mkdocs-material
```

Then we're install the `mktheapidocs` plugin. Until a PR gets merged in, the steps are:

```bash
$ git clone https://github.com/binitshah/mktheapidocs.git -b feature/hidden_submodules
$ cd mktheapidocs
$ python3 -m pip install -e .
```

## Development

The following command will serve the static website with hot-reloading. There are two static websites in this repository. The main documentation is in the root of the repository (`http://127.0.0.1:8000`). The auto-generated API based on `stretch_body` is in the `api/` folder (`http://127.0.0.1:8000/stable/stretch_body/arm`).

```bash
$ cd api/ # optional
$ python3 -m mkdocs serve
```

## Building

Before building, `hello-robot-stretch-body` for python3 must be installed.

```bash
$ ./build.sh
```

The generated site will reside in the `site/` folder. Visualize it by using `python -m http.server` inside the `site/` folder and pointing your browser to `0.0.0.0:8000`.

## Deploying

Next to this repository, clone [hello-robot.github.io](https://github.com/hello-robot/hello-robot.github.io). Then, replace the contents of the repository entirely with the contents of `site/`. Commit and push. Github will begin serving the new static documentation.
