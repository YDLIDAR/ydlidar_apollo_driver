#!/usr/bin/env bash

###############################################################################
# Copyright 2020 EAIBOT. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

set -e

SDK_VERSION_MAJOR=1
SDK_VERSION_MINOR=0
SDK_VERSION_PATCH=1

cd "$(dirname "${BASH_SOURCE[0]}")"
ARCH=$(uname -m)
if [ "$ARCH" == "aarch64" ]; then
  BUILD=$1
  shift
fi

if [ "$BUILD" == "build" ] || [ "$ARCH" == "x86_64" ]; then
  wget https://github.com/YDLIDAR/YDLidar-SDK/archive/v${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}.tar.gz

  tar xzvf v${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}.tar.gz

  pushd YDLidar-SDK-${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}/
  cd build
  cmake -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF ..
  make -j2
  popd
  rm -rf YDLidar-SDK-${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}
  rm v${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}.tar.gz*
fi

