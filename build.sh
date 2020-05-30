set -e # exit on error

echo "Building main documentation"
rm -rf site
python3 -m mkdocs -q build

echo "Adding CNAME & more"
cp CNAME site/
cp .nojekyll site/

echo "Building API reference"
cd api
rm -rf site
python3 -m mkdocs -q build
mv site ../site/api

echo ""
echo "Copy /site to hello-robot.github.io and commit/push"