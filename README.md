
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
This project uses Jekyll, the static site generator behind GitHub Pages (see [Jekyll Quickstart](https://jekyllrb.com/docs/) for more information).
To preview the site locally, you’ll need to install Ruby, Jekyll, and the project dependencies.

### Prerequisites
Make sure your system has the necessary build tools and libraries for compiling Ruby and Jekyll dependencies:

```bash
sudo apt update
sudo apt install -y \
  libffi-dev \
  libyaml-dev \
  git
```

### Install Ruby via rbenv
We recommend using rbenv to manage Ruby versions locally:

```bash
# Install rbenv
sudo apt install -y rbenv

# Add rbenv to your shell
echo 'export PATH="$HOME/.rbenv/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(rbenv init - bash)"' >> ~/.bashrc
source ~/.bashrc

# Install ruby-build plugin
git clone https://github.com/rbenv/ruby-build.git "$(rbenv root)"/plugins/ruby-build
git -C "$(rbenv root)"/plugins/ruby-build pull

# Install Ruby 3.4.5
rbenv install 3.4.5
rbenv global 3.4.5
```

### Configure UTF-8 Locale
Jekyll and its plugins require UTF-8 encoding to handle special characters correctly:

```bash
export LC_ALL="C.UTF-8"
export LANG="en_US.UTF-8"
export LANGUAGE="en_US.UTF-8"
```

### Install Jekyll and Bundler

```bash
gem install jekyll bundler
```

Using a Bundler version different from the one recorded in the `Gemfile.lock` (currently 2.7.2) may lead to compatibility issues.
You can check your installed Bundler version by running:

```bash
bundler -v
```

### Clone repository and dependencies
For the includes of README.md files on the micro-ROS demos (in the tutorials chapter) from the corresponding repositories, please init and update the corresponding git submodules:

```bash
git clone https://github.com/micro-ROS/micro-ROS.github.io.git
cd micro-ROS.github.io/

# Fetch tutorials and demos included via submodules
git submodule update --init --recursive
```

### Install project dependencies
After installing Jekyll, install all dependencies by running:

```bash
bundle install
```

### Run the Jekyll server
You may launch Jekyll to build and serve the website continuously by running:

```bash
bundle exec jekyll serve
```

By default, the site will be available at http://localhost:4000


After installing Jekyll, install all dependencies by running
```bash
bundle install
```

Then, you may launch Jekyll to build and serve the website continuously by
```bash
bundle exec jekyll serve
```

### (Optional) Regenerate `Gemfile.lock` file

When updating Ruby or Bundler version it is possible that dependencies no longer resolve cleanly, causing `bundle install` to fail.
It can be fixed by regenerating the `Gemfile.lock` file and commiting the changes:

```bash
rm Gemfile.lock

bundle install
```

This will resolve all dependencies defined in your `Gemfile` and generate a new `Gemfile.lock` consistent with your current Bundler version and available gems.

## Testing generated site

To test the generated HTML site, you can use `html-proofer` gem.
This Ruby gem checks and validates the jekyll generated HTML files.
It checks a broad set of points: internal and external links existence (alerting of possible 404 errors), HTML attributes of the images and so on.

To install it, It has been incorporated in the Gemfile so the previous dependency install command would have already installed it.

You can run `bundle exec jekyll build` followed by `bundle exec htmlproofer ./_site` to build and test the generated site.
However, note that a comprehensive configuration is required for the htmlproofer.
Therefore, we strongly suggest to run the utility script at

```bash
./scripts/cibuild
```

which is also used for CI.
