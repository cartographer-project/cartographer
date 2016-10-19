#!/bin/bash

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Cache intermediate Docker layers. For a description of how this works, see:
# https://giorgos.sealabs.net/docker-cache-on-travis-and-docker-112.html

set -o errexit
set -o verbose
set -o pipefail

if [ -f ${DOCKER_CACHE_FILE} ]; then
  gunzip -c ${DOCKER_CACHE_FILE} | docker load;
fi
