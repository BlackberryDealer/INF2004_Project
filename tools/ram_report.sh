#!/usr/bin/env bash
################################################################################
# ram_report.sh -- Week 13 RAM comparison (spec B4.5). Owner: Buddy 4.
#
# Usage:
#   tools/ram_report.sh <baseline.elf> <packed.elf> \
#       [baseline_stack.txt] [packed_stack.txt] > docs/ram_comparison.md
#
# Section 1: arm-none-eabi-size -A for both images: .data, .bss, .text and
#            totals with deltas.
# Section 2: arm-none-eabi-nm -S --size-sort -t d for both images; symbols
#            whose names match a line in tools/ram_symbols.txt are listed
#            with before/after bytes, the delta and the owner from the
#            line's "# B<n>" comment, sorted by owner.
# Section 3: the app_stack_report() console output of both builds; pass the
#            saved serial logs as optional 3rd and 4th arguments.
#
# POSIX bash + awk only (no Python). Exit 1 when a required file is
# missing or a tool is not on PATH.
################################################################################

set -u

BASELINE=${1:-}
PACKED=${2:-}
STACK_BASE=${3:-}
STACK_PACKED=${4:-}

if [ -z "$BASELINE" ] || [ -z "$PACKED" ]; then
    echo "usage: ram_report.sh <baseline.elf> <packed.elf> \\" >&2
    echo "       [baseline_stack.txt] [packed_stack.txt]" >&2
    exit 1
fi

for file in "$BASELINE" "$PACKED"; do
    if [ ! -f "$file" ]; then
        echo "ram_report: file not found: $file" >&2
        exit 1
    fi
done

command -v arm-none-eabi-size >/dev/null 2>&1 || {
    echo "ram_report: arm-none-eabi-size is not on PATH" >&2
    exit 1
}
command -v arm-none-eabi-nm >/dev/null 2>&1 || {
    echo "ram_report: arm-none-eabi-nm is not on PATH" >&2
    exit 1
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SYMBOLS="$ROOT/tools/ram_symbols.txt"

if [ ! -f "$SYMBOLS" ]; then
    echo "ram_report: missing $SYMBOLS" >&2
    exit 1
fi

TMPDIR_RAM="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_RAM"' EXIT

################################################################################
# Section 1: section sizes
################################################################################

size_row() {
    # Prints "<name> <bytes>" per section from "size -A" output.
    arm-none-eabi-size -A "$1" | awk '
        $1 ~ /^\.(data|bss|text)$/ { printf "%s %d\n", $1, $2 }
        $1 == "Total" { printf "total %d\n", $2 }
    '
}

size_row "$BASELINE" | sort >"$TMPDIR_RAM/size_before.txt"
size_row "$PACKED"  | sort >"$TMPDIR_RAM/size_after.txt"

################################################################################
# Section 2: symbol sizes
################################################################################

nm_sizes() {
    # Prints "<symbol> <bytes> <owner>" for the symbols matching
    # tools/ram_symbols.txt (first matching pattern wins).
    arm-none-eabi-nm -S --size-sort -t d "$1" | awk -v patfile="$SYMBOLS" '
        BEGIN {
            npats = 0
            while ((getline line < patfile) > 0) {
                if (line ~ /^[[:space:]]*(#.*)?$/) { continue }
                n = split(line, part, "#")
                pat = part[1]
                sub(/[[:space:]]+$/, "", pat)
                if (pat == "") { continue }
                pats[npats] = pat
                owner = "?"
                if (n >= 2) {
                    owner = part[2]
                    sub(/^[[:space:]]+/, "", owner)
                }
                owners[npats] = owner
                npats++
            }
        }
        {
            sym = $4
            if (sym == "") { next }
            for (idx = 0; idx < npats; idx++) {
                if (sym ~ pats[idx]) {
                    printf "%s %d %s\n", sym, $2, owners[idx]
                    break
                }
            }
        }
    ' | sort
}

nm_sizes "$BASELINE" >"$TMPDIR_RAM/nm_before.txt"
nm_sizes "$PACKED"  >"$TMPDIR_RAM/nm_after.txt"

awk '
    FNR == NR {
        size_before[$1] = $2
        owner[$1] = $3
        seen[$1] = 1
        next
    }
    {
        sym = $1
        before = (sym in size_before) ? size_before[sym] : 0
        after = $2
        delta = after - before
        if (delta < 0) { delta = 0 - delta }
        sign = (after >= before) ? "+" : "-"
        out[sym] = sprintf("%s | %d | %d | %s%d | %s", sym, before, after,
                           sign, delta, $3)
        seen[sym] = 1
        next
    }
    END {
        for (sym in seen) {
            if (!(sym in out)) {
                before = (sym in size_before) ? size_before[sym] : 0
                out[sym] = sprintf("%s | %d | 0 | -%d | %s", sym, before,
                                   before, owner[sym])
            }
        }
        for (sym in out) { print out[sym] }
    }
' "$TMPDIR_RAM/nm_before.txt" "$TMPDIR_RAM/nm_after.txt" \
    | sort -t'|' -k5,5 -k1,1 >"$TMPDIR_RAM/merged.txt"

################################################################################
# Report
################################################################################

echo "# RAM comparison: baseline vs packed"
echo
echo "Generated by \`tools/ram_report.sh\` (spec B4.5). The baseline image"
echo "stores the same information in natural types; the packed image uses the"
echo "\`tools/layout.csv\` fields. Sections 2 and 3 are the storage comparison;"
echo "the \`.text\` row of section 1 is informational only, because the packed"
echo "build also carries the encode/decode code."
echo
echo "## 1. Section sizes (\`arm-none-eabi-size -A\`)"
echo
echo "| Section | Baseline B | Packed B | Delta |"
echo "|---------|-----------:|---------:|------:|"
awk '
    FNR == NR { before[$1] = $2; next }
    {
        after = $2
        delta = after - before[$1]
        sign = (delta >= 0) ? "+" : "-"
        mag = (delta >= 0) ? delta : (0 - delta)
        printf "| %s | %d | %d | %s%d |\n", $1, before[$1], after, sign, mag
    }
' "$TMPDIR_RAM/size_before.txt" "$TMPDIR_RAM/size_after.txt"
echo
echo "## 2. Symbol sizes (\`arm-none-eabi-nm -S --size-sort -t d\`,"
echo "   filtered by \`tools/ram_symbols.txt\`)"
echo
echo "| Symbol | Before B | After B | Delta | Owner |"
echo "|--------|---------:|--------:|------:|-------|"
if [ -s "$TMPDIR_RAM/merged.txt" ]; then
    awk '{ printf "| %s |\n", $0 }' "$TMPDIR_RAM/merged.txt"
else
    echo "| *(no symbols matched)* | - | - | - | - |"
fi
echo
echo "## 3. Stack free space (paste \`app_stack_report()\` output of both builds)"
echo
if [ -n "$STACK_BASE" ] && [ -f "$STACK_BASE" ]; then
    echo "### Baseline"
    echo
    echo '```'
    cat "$STACK_BASE"
    echo '```'
    echo
else
    echo "Baseline stack log: *not provided (flash the \`w10-baseline\` image,"
    echo "save the \`stack ...\` lines and pass the file as argument 3)*"
    echo
fi
if [ -n "$STACK_PACKED" ] && [ -f "$STACK_PACKED" ]; then
    echo "### Packed"
    echo
    echo '```'
    cat "$STACK_PACKED"
    echo '```'
    echo
else
    echo "Packed stack log: *not provided (flash the \`w11-packed\` image,"
    echo "save the \`stack ...\` lines and pass the file as argument 4)*"
    echo
fi

exit 0
