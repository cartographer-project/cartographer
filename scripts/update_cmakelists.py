#!/usr/bin/python
# -*- coding: utf-8 -*-

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
"""A dumb CMakeLists.txt generator that relies on source name conventions."""

import os
from os import path
import re


def MakeRelative(filename, directory):
  absolute_directory = path.realpath(directory)
  filename = path.realpath(filename)
  return path.relpath(filename, absolute_directory)


def MaybeUseCmakeFile(filename):
  cmake_filename = filename + ".cmake"
  return cmake_filename if os.path.exists(cmake_filename) else filename


class Target(object):
  """Container for data for a cmake target."""

  def __init__(self, target_type, name, directory, srcs, hdrs):
    self.type = target_type
    self.name = name
    self.directory = directory
    self.srcs = srcs
    self.hdrs = hdrs
    self.depends = set()
    self.uses = set()

  def __repr__(self):
    return "%s(%s, nsrcs:%s, %s)" % (self.type, self.name, len(self.srcs),
                                     self.depends)

  def Format(self, directory):
    """Formats the target for writing into a CMakeLists.txt file."""
    lines = ["%s(%s" % (self.type, self.name),]

    for use in sorted(self.uses):
      lines.append("  " + use)

    if self.srcs:
      lines.append("  SRCS")
      lines.extend("    " + MakeRelative(s, directory)
                   for s in sorted(self.srcs))

    if self.hdrs:
      lines.append("  HDRS")
      lines.extend("    " + MakeRelative(s, directory)
                   for s in sorted(self.hdrs))

    if self.depends:
      lines.append("  DEPENDS")
      lines.extend("    " + s for s in sorted(self.depends))
    lines.append(")")
    return "\n".join(lines)


def ExtractCartographerIncludes(source):
  """Returns all locally included files."""
  includes = set()
  for line in open(MaybeUseCmakeFile(source)):
    if source.endswith(".proto"):
      match = re.match(r'^import "(cartographer/[^"]+)', line)
    else:
      match = re.match(r'^#include "(cartographer/[^"]+)"', line)
    if match:
      includes.add(match.group(1))
  return includes


def ExtractUses(source):
  """Finds the options for the third_party libraries used."""
  uses = set()
  for line in open(MaybeUseCmakeFile(source)):
    if re.match(r'^#include "Eigen/', line):
      uses.add("USES_EIGEN")
    if re.match(r"^#include <lua.hpp>", line):
      uses.add("USES_LUA")
    if re.match(r'^#include "(ceres|glog)/', line):
      # We abuse Ceres CFLAGS for other Google libraries that we all depend on.
      # The alternative is to ship and maintain our own Find*.cmake files which
      # is not appealing.
      uses.add("USES_CERES")
    if re.match(r"^#include <boost/", line):
      uses.add("USES_BOOST")
    if re.match(r'^#include "webp/', line):
      uses.add("USES_WEBP")
  return uses


def FindSourceFiles(basedir):
  sources = set()
  for (directory, _, filenames) in os.walk(basedir):
    for filename in filenames:
      ext = path.splitext(filename)[-1]
      if ext in [".h", ".cc", ".proto"]:
        sources.add(path.join(directory, filename))
      elif filename.endswith(".h.cmake"):
        sources.add(path.join(directory, path.splitext(filename)[0]))
    yield (directory, sources)


def ReadFileWithoutGoogleTargets(filename):
  if path.exists(filename):
    ignoring = False
    for line in open(filename):
      if line.startswith("google_"):
        ignoring = True
      elif line == ")" and ignoring:
        ignoring = False
      elif not ignoring:
        yield line.rstrip()


def main():
  root_directory = os.path.realpath(
      os.path.join(os.path.dirname(__file__), os.path.pardir, "cartographer"))
  targets_by_src = {}
  targets = []

  base_directory = path.realpath(path.join(root_directory, path.pardir))
  directories = set()

  def AddTarget(target_type, name, directory, srcs, hdrs):
    """Adds a new target to 'targets' and updates 'targets_by_src'."""
    target = Target(target_type, name, directory, srcs, hdrs)
    targets.append(target)
    for s in srcs + hdrs:
      relative_s = MakeRelative(s, base_directory)
      targets_by_src[relative_s] = target
      if s.endswith(".proto"):
        proto_stem = os.path.splitext(relative_s)[0]
        targets_by_src[proto_stem + ".pb.h"] = target
        targets_by_src[proto_stem + ".pb.cc"] = target
    directories.add(directory)

  for (directory, sources) in FindSourceFiles(root_directory):
    module_name = path.relpath(directory,
                               path.realpath(root_directory)).replace("/", "_")
    headers = set(fn for fn in sources if fn.endswith(".h"))
    sources -= headers
    for h in sorted(headers):
      srcs = []
      name = module_name + "_" + path.basename(path.splitext(h)[0])
      cc_file = path.splitext(h)[0] + ".cc"
      if cc_file in sources:
        sources.remove(cc_file)
        srcs.append(cc_file)
      AddTarget("google_library", name, directory, srcs, [h])

    tests = set(fn for fn in sources if fn.endswith("_test.cc"))
    sources -= tests
    for c in sorted(tests):
      name = module_name + "_" + path.basename(path.splitext(c)[0])
      AddTarget("google_test", name, directory, [c], [])

    protos = set(fn for fn in sources if fn.endswith(".proto"))
    sources -= protos
    for c in sorted(protos):
      name = module_name + "_" + path.basename(path.splitext(c)[0])
      AddTarget("google_proto_library", name, directory, [c], [])

    assert (not sources), "Remaining sources without target: %s" % sources

  # Write the CMakeLists.txt files.
  for directory in directories:
    targets_in_directory = [t for t in targets if t.directory == directory]
    for target in targets_in_directory:
      for src in target.srcs + target.hdrs:
        for header in ExtractCartographerIncludes(src):
          dependant = targets_by_src[header]
          if dependant.name == target.name:
            continue
          target.depends.add(dependant.name)
        target.uses.update(ExtractUses(src))

    cmake_file = path.join(directory, "CMakeLists.txt")
    lines = list(ReadFileWithoutGoogleTargets(cmake_file))
    with open(cmake_file, "w") as outfile:
      if lines:
        outfile.write("\n".join(lines))
        outfile.write("\n")
      outfile.write("\n\n".join(
          t.Format(directory) for t in targets_in_directory))
      outfile.write("\n")


if __name__ == "__main__":
  main()
