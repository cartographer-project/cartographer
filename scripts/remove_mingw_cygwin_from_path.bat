rem  Copyright 2018 The Cartographer Authors
rem  
rem  Licensed under the Apache License, Version 2.0 (the "License");
rem  you may not use this file except in compliance with the License.
rem  You may obtain a copy of the License at
rem  
rem       http://www.apache.org/licenses/LICENSE-2.0
rem  
rem  Unless required by applicable law or agreed to in writing, software
rem  distributed under the License is distributed on an "AS IS" BASIS,
rem  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
rem  See the License for the specific language governing permissions and
rem  limitations under the License.

rem  Remove git bash's MinGW/Cygwin stuff from PATH, as it interferes with CMake's
rem  detection of Boost when using ninja.
set PATH=%PATH:C:\tools\mingw64\bin;=%
set PATH=%PATH:C:\Program Files\Git\bin;=%
set PATH=%PATH:C:\Program Files\Git\usr\bin;=%
set PATH=%PATH:C:\Program Files\Git\mingw64\bin;=%