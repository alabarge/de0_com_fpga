# QCustomPlot (vendored)

Version 2.1.1, downloaded from https://www.qcustomplot.com/release/2.1.1/QCustomPlot-source.tar.gz

`qcustomplot.h`/`qcustomplot.cpp` are unmodified upstream source files, vendored
directly (QCustomPlot's own recommended usage: compile the two files as part
of your own project, no separate library install).

## Licensing - action required before any customer build

This is currently used under **GPLv3** (see `GPL.txt`), which is fine for
internal development but becomes a real copyleft obligation the moment a
build containing it is given to anyone outside this project - this app is
occasionally shared with customers (see Phase 0 of the Qt migration).

**Before any build of `jack-qt` is distributed to a customer**, either:
- Purchase a QCustomPlot commercial license (https://www.qcustomplot.com/index.php/support/license),
  or
- Remove/replace this dependency.

Do not ship a GPL build of this application outside this repository's own
development use until that's resolved.
