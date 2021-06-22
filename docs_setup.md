## Guide to Managing Stretch Docs

### Software Setup

#### Git Setup

```bash
>>$ cd ~/repos
>>$ git clone https://github.com/hello-robot.github.io
>>$ git clone https://github.com/stretch_docs
```

#### Python Setup

The mkdoc-click package requires Python 3.7 or later to be installed. We may want to use this package to generate command line tool documentation (lower priority)

#### Conda Environment

TBD

#### PIP Packages

```bash
python3 -m pip install mkdocs
python3 -m pip install mkdocs-material
python3 -m pip install mkdocs-simple-plugin
python3 -m pip install install Pygments
python3 -m pip install git+https://github.com/DataDog/mkdocs-click.git
```

Other things (AE 6.22.21)

* mkdocstring
  * Got working hand to edit the plugin.py though
* mktheapidocs
  * Didn't work / broken install

### Deploying

The `deploy.sh` script with automatically build the docs, replace the contents of hello-robot.github.io, and push the new docs to master.  Github will reflect the changes at [docs.hello-robot.com](docs.hello-robot.com)

```bash
>>$ cd ~/repos/stretch_docs
>>$ ./deploy.sh
```

### Deploying From a Branch

Significant changes to Stretch Docs should be done in a branch, for example

```bash
>>$ cd ~/repos/stretch_docs
>>$git status
On branch feature/refactor
```

Testing can be done locally via your web browser at site http://127.0.0.1:8000/ 

```bash
>>$ cd ~/repos/stretch_docs
>>$ mkdocs serve
```

**Note: I thought this worked but not 100% sure now, browser refresh makes it hard to tell. Somthing like this worked...**

If you want to deploy the development site to share with others, set the site url in mkdocs.yml to:

```yaml
site_url: https://docs.hello-robot.com/stretch_docs
```

Then

```bash
>>$ cd ~/repos/hello-robot.github.io
>>$ python3 -m mkdocs gh-deploy --config-file ../stretch_docs/mkdocs.yml --remote-branch feature/<branch name>
```

And you can view your site at: https://docs.hello-robot.com/stretch_docs/