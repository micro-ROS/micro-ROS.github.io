# micro-ROS web site

[![pipeline status](https://gitlab.com/micro-ROS/ci-support/micro-ROS-github-io/badges/master/pipeline.svg)](https://gitlab.com/micro-ROS/ci-support/micro-ROS-github-io/commits/master)

This is the source for the micro-ROS website, available
at [https://micro-ros.github.io/](https://micro-ros.github.io/)

## Editing

See [editing instructions](EDITING-INSTRUCTIONS.md)

## Running locally

To test locally, you need a local version of Jekyll, the site-generation
engine used by GitHub Pages. See [Jekyll Quickstart](https://jekyllrb.com/docs/)
for installation instructions.

After installing Jekyll, install all dependencies by running
```bash
bundle install
```

Then, you may launch Jekyll to build and serve the website continuously by
```bash
bundle exec jekyll serve
```

For the includes of README.md files on the micro-ROS demos (in the tutorials chapter) from the corresponding repositories, please init and update the corresponding git submodules (i.e. `git submodule init ; git submodule update`).

## Testing generated site

To test the generated HTML site, you can use `html-proofer` gem.
This Ruby gem checks and validates the jekyll generated HTML files.
It checks a broad set of points: internal and external links existence (alerting of possible 404 errors), HTML attributes of the images and so on.

To install it, It has been incorporated in the Gemfile so the previous dependency install command would have already installed it.

You can run the following gem to tests the generated site.

```bash
bundle exec jekyll build
bundle exec htmlproofer ./_site
```

A utility script has also been included to run these checks in a CI system smoothly.

```bash
./scripts/cibuild
```

## License

Released under [the Apache Public License 2.0](LICENSE).
