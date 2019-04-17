# micro-ROS web site

This is the source for the micro-ROS website, available
at [https://micro-ros.github.io/](https://micro-ros.github.io/)

## Editing

See [editing instructions](EDITING-INSTRUCTIONS.md)

## Running locally

To test locally, you need a local version of Jekyll, the site-generation
engine used by Github Pages. See [Jekyll Quickstart](https://jekyllrb.com/docs/)
for installation instructions.

After installing Jekyll, install all dependencies by running
```bash
bundle install
```

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
