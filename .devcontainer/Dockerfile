#-------------------------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License. See https://go.microsoft.com/fwlink/?linkid=2090316 for license information.
#-------------------------------------------------------------------------------------------------------------
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get -y upgrade \
   && apt-get -y install --reinstall ca-certificates \
   && update-ca-certificates \
   && apt-get -y install --no-install-recommends build-essential locales zlib1g-dev curl git \
   && curl -fsSL https://github.com/rbenv/rbenv-installer/raw/HEAD/bin/rbenv-installer | bash \
   && echo 'eval "$(~/.rbenv/bin/rbenv init - bash)"' >> ~/.bashrc \
   && source ~/.bashrc \
   # Use latest version 2 of ruby
   # WARNING: ruby-2.7.7 is nearing its end of life.
   # It only receives critical security updates, no bug fixes.
   && rbenv install 2.7.7 \
   && rbenv global 2.7.7 \
   && gem install bundler \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

RUN echo "en_US UTF-8" > /etc/locale.gen \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
ENV DEBIAN_FRONTEND=dialog
