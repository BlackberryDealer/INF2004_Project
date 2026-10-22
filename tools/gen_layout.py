#!/usr/bin/env python3
"""
gen_layout.py - generate the packed layout from tools/layout.csv.

Owner:  Buddy 1 (communications)
Spec:   CSC28 AI spec, Part D.11 (layout table), D.12 (header format), B1.0.2

Usage (from the repo root, or from build_make/ via "make layout"):

    python3 tools/gen_layout.py tools/layout.csv
    python3 ../tools/gen_layout.py ../tools/layout.csv
    python3 tools/gen_layout.py --check <csv>     # validate only, write nothing

Reads the CSV, validates every row (exit 1 with "layout.csv:<line>: <problem>"
on the first error) and then writes three files, located relative to this
script so the working directory does not matter:

    app_program/app_layout.h       C macros (Part D.12)
    tools/telem_fields.py          field table for the laptop decoder
    docs/data_representation.md    data representation table (Week 13)

Python 3 standard library only. Same CSV in, byte-identical files out.
"""

import csv
import os
import sys

COLUMNS = ("group", "group_bits", "field", "bits", "lsb", "unit", "min",
           "max", "offset", "kind", "class", "owner", "description")
GROUP_BITS_ALLOWED = (0, 8, 11, 16, 32)
KINDS = ("SAT", "RNG", "ENUM", "FLAG", "PAD", "RAW")
KINDS_MINIMAL = ("SAT", "RNG", "ENUM", "FLAG")   # must use the minimal width
CLASSES = ("S", "P", "E")
OWNERS = ("B1", "B2", "B3", "B4", "B5")
MACRO_NAME_MAX = 31
BITS_MIN = 1
BITS_MAX = 31

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TOOLS_DIR)
OUT_HEADER = os.path.join(REPO_DIR, "app_program", "app_layout.h")
OUT_FIELDS = os.path.join(TOOLS_DIR, "telem_fields.py")
OUT_DOC = os.path.join(REPO_DIR, "docs", "data_representation.md")

# Range assumptions referenced by the layout rows (descriptions "(A23)" ...).
# Each one is the stored range of the rows that cite it.
ASSUMPTIONS = (
    ("A23", "One run is at most 32 m (APP_RUN_MAX_MM): encoder positions "
            "fit +/-131071 counts, path length 131071 counts, odometry "
            "32767 mm."),
    ("A24", "Pitch stays within +/-45.0 deg (+/-450 x 0.1 deg)."),
    ("A25", "A hump is at most 127 mm high."),
    ("A26", "Turn rate stays within +/-360 deg/s."),
    ("A27", "Obstacle width and clearances are reported up to 1020 mm "
            "(255 x 4 mm); the top code means clear / out of range."),
    ("A28", "One run lasts at most 1023 s, about 17 min (APP_RUN_MAX_S)."),
)


class LayoutError(Exception):
    """A problem in the CSV, reported with its line number."""

    def __init__(self, line, problem):
        Exception.__init__(self, problem)
        self.line = line
        self.problem = problem


def bits_needed(lo, hi):
    """ceil(log2(hi - lo + 1)), computed exactly on integers."""
    return (hi - lo).bit_length()


def parse_int(row, name, line):
    try:
        return int(row[name])
    except ValueError:
        raise LayoutError(line, "column '%s' is not an integer: '%s'"
                          % (name, row[name]))


def read_rows(csv_path):
    """Returns a list of (line_number, dict) rows, header validated."""
    with open(csv_path, "r", newline="", encoding="utf-8") as handle:
        lines = handle.read().splitlines()

    numbered = []
    for idx, text in enumerate(lines):
        stripped = text.strip()
        if (stripped == "") or stripped.startswith("#"):
            continue
        numbered.append((idx + 1, text))

    if not numbered:
        raise LayoutError(1, "file is empty")

    header_line, header_text = numbered[0]
    header = next(csv.reader([header_text]))
    if tuple(col.strip() for col in header) != COLUMNS:
        raise LayoutError(header_line, "header must be: " + ",".join(COLUMNS))

    rows = []
    for line, text in numbered[1:]:
        cells = next(csv.reader([text]))
        if len(cells) != len(COLUMNS):
            raise LayoutError(line, "expected %d columns, found %d"
                              % (len(COLUMNS), len(cells)))
        row = dict(zip(COLUMNS, (cell.strip() for cell in cells)))
        for name in COLUMNS:
            if row[name] == "":
                raise LayoutError(line, "column '%s' is empty" % name)
        rows.append((line, row))
    return rows


def validate(rows):
    """Checks every rule of B1.0.2 and returns the rows as typed records."""
    records = []
    group_bits_of = {}
    group_used = {}
    group_first_line = {}
    seen = set()

    for line, row in rows:
        rec = dict(row)
        rec["line"] = line
        for name in ("group_bits", "bits", "min", "max", "offset"):
            rec[name] = parse_int(row, name, line)
        lsb = parse_int(row, "lsb", line)
        if lsb < 1:
            raise LayoutError(line, "lsb must be >= 1")
        rec["lsb"] = lsb

        group = rec["group"]
        field = rec["field"]

        if rec["kind"] not in KINDS:
            raise LayoutError(line, "kind '%s' not in %s"
                              % (rec["kind"], "/".join(KINDS)))
        if rec["class"] not in CLASSES:
            raise LayoutError(line, "class '%s' not in %s"
                              % (rec["class"], "/".join(CLASSES)))
        if rec["owner"] not in OWNERS:
            raise LayoutError(line, "owner '%s' not in %s"
                              % (rec["owner"], "/".join(OWNERS)))
        if rec["group_bits"] not in GROUP_BITS_ALLOWED:
            raise LayoutError(line, "group_bits %d not in %s"
                              % (rec["group_bits"],
                                 "/".join(str(b) for b in GROUP_BITS_ALLOWED)))
        if (group, field) in seen:
            raise LayoutError(line, "duplicate field %s.%s" % (group, field))
        seen.add((group, field))

        if group in group_bits_of:
            if group_bits_of[group] != rec["group_bits"]:
                raise LayoutError(line, "group %s has group_bits %d here but "
                                  "%d on line %d"
                                  % (group, rec["group_bits"],
                                     group_bits_of[group],
                                     group_first_line[group]))
        else:
            group_bits_of[group] = rec["group_bits"]
            group_first_line[group] = line
            group_used[group] = 0

        if rec["bits"] < BITS_MIN or rec["bits"] > BITS_MAX:
            raise LayoutError(line, "bits %d not in %d..%d"
                              % (rec["bits"], BITS_MIN, BITS_MAX))
        if rec["min"] > rec["max"]:
            raise LayoutError(line, "min %d > max %d"
                              % (rec["min"], rec["max"]))

        rec["needed"] = bits_needed(rec["min"], rec["max"])

        # Class E rows are documentation only: natural storage, no macros.
        if rec["class"] == "E":
            if rec["group_bits"] not in (8, 16, 32):
                raise LayoutError(line, "class E row needs group_bits "
                                  "8/16/32 (its natural storage width)")
            records.append(rec)
            continue

        if (rec["min"] + rec["offset"]) < 0:
            raise LayoutError(line, "min + offset = %d is negative"
                              % (rec["min"] + rec["offset"]))
        top = (1 << rec["bits"]) - 1
        if (rec["max"] + rec["offset"]) > top:
            raise LayoutError(line, "max + offset = %d does not fit %d bits "
                              "(max %d)" % (rec["max"] + rec["offset"],
                                            rec["bits"], top))
        if rec["kind"] in KINDS_MINIMAL:
            minimal = max(rec["needed"], 1)
            if rec["bits"] != minimal:
                raise LayoutError(line, "%s.%s is %d bits, %d needed: %s"
                                  % (group, field, rec["bits"], minimal,
                                     "wider than needed"
                                     if rec["bits"] > minimal
                                     else "narrower than its range"))

        group_used[group] += rec["bits"]
        limit = rec["group_bits"]
        if (limit > 0) and (group_used[group] > limit):
            raise LayoutError(line, "group %s uses %d bits, more than its %d"
                              % (group, group_used[group], limit))

        if limit > 0:
            field_suffixes = ("POS", "BITS", "MIN", "MAX", "OFFSET", "KIND")
            group_suffixes = ("USED_BITS",)
        else:
            field_suffixes = ("BITS", "MIN", "MAX", "OFFSET", "KIND")
            group_suffixes = ("TOTAL_BITS", "BYTES")
        for suffix in field_suffixes:
            name = "%s_%s_%s" % (group, field, suffix)
            if len(name) > MACRO_NAME_MAX:
                raise LayoutError(line, "macro %s is %d characters (max %d)"
                                  % (name, len(name), MACRO_NAME_MAX))
        for suffix in group_suffixes:
            name = "%s_%s" % (group, suffix)
            if len(name) > MACRO_NAME_MAX:
                raise LayoutError(line, "macro %s is %d characters (max %d)"
                                  % (name, len(name), MACRO_NAME_MAX))
        records.append(rec)

    return records


def groups_in_order(records):
    """Packed (non-E) groups in first-appearance order, with their rows."""
    order = []
    rows_of = {}
    for rec in records:
        if rec["class"] == "E":
            continue
        if rec["group"] not in rows_of:
            order.append(rec["group"])
            rows_of[rec["group"]] = []
        rows_of[rec["group"]].append(rec)
    return [(group, rows_of[group]) for group in order]


def define(name, value):
    """One #define with the value aligned (barr_check skips this file's
       line length: chained _POS values can pass 80 columns)."""
    return "#define %-31s %s\n" % (name, value)


def render_header(records):
    out = []
    out.append("/**\n")
    out.append(" * @file    app_layout.h\n")
    out.append(" * @brief   Packed layout of every shared value: position, "
               "width,\n")
    out.append(" *          stored range and kind of each field.\n")
    out.append(" * @owner   Buddy 1 (generated; change tools/layout.csv, "
               "then make layout)\n")
    out.append(" * @kind    cfg\n")
    out.append(" */\n")
    out.append("\n")
    out.append("/* GENERATED from tools/layout.csv by tools/gen_layout.py. "
               "Do not edit. */\n")
    out.append("\n")
    out.append("#ifndef APP_LAYOUT_H\n")
    out.append("#define APP_LAYOUT_H\n")
    out.append("\n")
    out.append("#include \"app_types.h\"\n")

    for group, rows in groups_in_order(records):
        width = rows[0]["group_bits"]
        out.append("\n")
        if width == 0:
            out.append("/* %s: byte stream (app_bits_write, row order) */\n"
                       % group)
        else:
            out.append("/* %s: %d-bit word */\n" % (group, width))

        prev = None
        for rec in rows:
            stem = "%s_%s" % (group, rec["field"])
            if width > 0:
                if prev is None:
                    out.append(define(stem + "_POS", "(0u)"))
                else:
                    out.append(define(stem + "_POS", "(%s_POS + %s_BITS)"
                                      % (prev, prev)))
            out.append(define(stem + "_BITS", "(%du)" % rec["bits"]))
            out.append(define(stem + "_MIN", "(%d)" % rec["min"]))
            out.append(define(stem + "_MAX", "(%d)" % rec["max"]))
            out.append(define(stem + "_OFFSET", "(%d)" % rec["offset"]))
            out.append(define(stem + "_KIND", "(APP_KIND_%s)" % rec["kind"]))
            prev = stem

        if width > 0:
            out.append(define(group + "_USED_BITS", "(%s_POS + %s_BITS)"
                              % (prev, prev)))
            out.append("#if (%s_USED_BITS > %du)\n" % (group, width))
            out.append("#error \"Group %s no longer fits in %d bits\"\n"
                       % (group, width))
            out.append("#endif\n")
        else:
            total = sum(rec["bits"] for rec in rows)
            out.append(define(group + "_TOTAL_BITS", "(%du)" % total))
            out.append(define(group + "_BYTES", "((%s_TOTAL_BITS + 7u) / 8u)"
                              % group))

    out.append("\n")
    out.append("#endif /* APP_LAYOUT_H */\n")
    out.append("\n")
    out.append("/*** end of file ***/\n")
    return "".join(out)


def render_fields(records):
    out = []
    out.append('"""\n')
    out.append("telem_fields.py - field table of every packed group.\n")
    out.append("\n")
    out.append("GENERATED from tools/layout.csv by tools/gen_layout.py. "
               "Do not edit.\n")
    out.append("\n")
    out.append("GROUPS[group] lists (field, bits, offset, lsb, unit) in CSV "
               "order, which\n")
    out.append("is also the order the firmware writes them (LSB first). "
               "Decoded value =\n")
    out.append("(raw - offset) * lsb, in unit.\n")
    out.append('"""\n')
    out.append("\n")
    out.append("GROUPS = {\n")
    for group, rows in groups_in_order(records):
        out.append("    %r: [\n" % group)
        for rec in rows:
            out.append("        (%r, %d, %d, %d, %r),\n"
                       % (rec["field"], rec["bits"], rec["offset"],
                          rec["lsb"], rec["unit"]))
        out.append("    ],\n")
    out.append("}\n")
    return "".join(out)


def md_cell(text):
    """Escapes a table cell (units such as mm|deg|mm/s contain pipes)."""
    return str(text).replace("|", "\\|")


def render_doc(records):
    out = []
    out.append("# Data representation\n")
    out.append("\n")
    out.append("GENERATED from `tools/layout.csv` by `tools/gen_layout.py`. "
               "Do not edit;\n")
    out.append("change the CSV and run `make layout` in `build_make/`.\n")
    out.append("\n")
    out.append("Stored value = clamp(value, min, max) + offset, in stored LSB "
               "units. *Bits\n")
    out.append("needed* = ceil(log2(max - min + 1)). Class S = shared, "
               "P = private,\n")
    out.append("E = justified exception (natural C type kept; *bits stored* "
               "is that type's\n")
    out.append("width). Kinds: SAT saturates silently at a sentinel; RNG "
               "clamps and sets\n")
    out.append("`APP_FAULT_RANGE_CLAMP`; PAD and RAW are exempt from the "
               "minimal-width rule.\n")

    for owner in OWNERS:
        rows = [rec for rec in records if rec["owner"] == owner]
        if not rows:
            continue
        out.append("\n")
        out.append("## %s\n" % owner)
        out.append("\n")
        out.append("| Field | Group | Range | LSB | Bits needed | Bits stored "
                   "| Kind | Class | Description |\n")
        out.append("|---|---|---|---|---|---|---|---|---|\n")
        for rec in rows:
            stored = rec["group_bits"] if rec["class"] == "E" else rec["bits"]
            rng = "%d..%d × %d %s" % (rec["min"], rec["max"], rec["lsb"],
                                      rec["unit"])
            out.append("| %s | %s | %s | %d | %d | %d | %s | %s | %s |\n"
                       % (md_cell(rec["field"]), md_cell(rec["group"]),
                          md_cell(rng), rec["lsb"], rec["needed"], stored,
                          rec["kind"], rec["class"],
                          md_cell(rec["description"])))

    out.append("\n")
    out.append("## Groups\n")
    out.append("\n")
    out.append("| Group | Used bits | Storage bits |\n")
    out.append("|---|---|---|\n")
    for group, rows in groups_in_order(records):
        used = sum(rec["bits"] for rec in rows)
        width = rows[0]["group_bits"]
        storage = width if width > 0 else ((used + 7) // 8) * 8
        out.append("| %s | %d | %d |\n" % (md_cell(group), used, storage))

    out.append("\n")
    out.append("## Range assumptions\n")
    out.append("\n")
    for ident, text in ASSUMPTIONS:
        out.append("- **%s** %s\n" % (ident, text))
    return "".join(out)


def write_text(path, text):
    with open(path, "w", newline="\n", encoding="utf-8") as handle:
        handle.write(text)


def main(argv):
    args = list(argv[1:])
    check_only = False
    if args and (args[0] == "--check"):
        check_only = True
        args = args[1:]
    if len(args) > 1:
        sys.stderr.write("usage: gen_layout.py [--check] [layout.csv]\n")
        return 2
    csv_path = args[0] if args else os.path.join(TOOLS_DIR, "layout.csv")
    csv_name = os.path.basename(csv_path)

    try:
        records = validate(read_rows(csv_path))
    except LayoutError as err:
        print("%s:%d: %s" % (csv_name, err.line, err.problem))
        return 1
    except OSError as err:
        print("%s: %s" % (csv_path, err.strerror))
        return 1

    if check_only:
        print("%s: OK, %d rows" % (csv_name, len(records)))
        return 0

    write_text(OUT_HEADER, render_header(records))
    write_text(OUT_FIELDS, render_fields(records))
    write_text(OUT_DOC, render_doc(records))

    for group, rows in groups_in_order(records):
        if rows[0]["group_bits"] == 0:
            used = sum(rec["bits"] for rec in rows)
            print("%-10s %3d bits = %2d bytes" % (group, used, (used + 7) // 8))
    print("wrote %s, %s, %s"
          % (os.path.relpath(OUT_HEADER, REPO_DIR),
             os.path.relpath(OUT_FIELDS, REPO_DIR),
             os.path.relpath(OUT_DOC, REPO_DIR)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
