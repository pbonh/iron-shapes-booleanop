<!--
SPDX-FileCopyrightText: 2022 Thomas Kramer

SPDX-License-Identifier: CC-BY-SA-4.0
-->

# Boolean operations for `iron-shapes`

This project implements boolean operations on polygons.

The code has been initially forked from https://github.com/21re/rust-geo-booleanop.

Large parts have been rewritten to fit better the needs of electronic design automation tools.
This includes for instance support for integer coordinates. Also future work will include
finding interactions and overlaps between polygons. This can be used for electrical connectivity
checks and netlist extraction from chip layouts.