
# micro-ROS web site

[![pipeline status](https://gitlab.com/micro-ROS/ci-support/micro-ROS-github-io/badges/master/pipeline.svg)](https://gitlab.com/micro-ROS/ci-support/micro-ROS-github-io/commits/master)

This is the source for the micro-ROS website, available
at [https://micro-ros.github.io/](https://micro-ros.github.io/)

## Editing

See [editing instructions](EDITING-INSTRUCTIONS.md)

## License

The content of this repository and the generated website is open-sourced under the Attribution-NoDerivatives 4.0 International (CC BY-ND 4.0) license.
You are free to:

* **Share** — copy and redistribute the material in any medium or format for any purpose, even commercially. The licensor cannot revoke these freedoms as long as you follow the license terms.

Under the following terms:

* **Attribution** — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* **NoDerivatives** — If you remix, transform, or build upon the material, you may not distribute the modified material.
* **No additional restrictions** — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.

See the [LICENSE](LICENSE) file for details.

Please note the following third-party elements and content:

* The website is based on the MIT-licensed template [Jekyll Doc Theme](https://github.com/aksakalli/jekyll-doc-theme) by [Can Güney Aksakalli](https://github.com/aksakalli/) and contributors. The template files and source code can be identified by the commits by his user name [aksakalli](https://github.com/aksakalli/) and by the user names of the contributors in the git history. Starting point for the development of this website was the version as of 23 September 2018, cf. commit [3cc3f49](https://github.com/micro-ROS/micro-ROS.github.io/commit/3cc3f492b80db80d87a310cbdc3895425a09db5e).

* All logos and product names are property of their respective owners. All company names, logos and product names used in this website are for identification purposes only. Their use does not imply endorsement.

For details on the open source components included in the micro-ros.github.io repository, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

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
