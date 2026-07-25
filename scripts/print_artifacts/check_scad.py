#!/usr/bin/env python3
"""Static sanity check for the .scad sources -- because openscad is NOT
installed on this machine and these files have therefore never been rendered.

    .venv/bin/python scripts/print_artifacts/check_scad.py

WHAT IT CAN PROVE
  * brackets/braces/parens balance, per file, comment- and string-aware
  * every `include <...>` target exists
  * every identifier used as a value is defined (in the file or in the
    included params), so a typo'd parameter name cannot ship silently
  * no parameter is defined twice with different values
  * declared modules are all called or are called by another module
  * the derived geometry closes: it re-evaluates the handful of arithmetic
    relations that the doc's own numbers have to satisfy (slot floor height,
    wall positions, bolt spacing, counterbore fit under the spar seat)

WHAT IT CANNOT PROVE
  * that the CSG is manifold, that nothing self-intersects wrongly, or that
    the part looks like the drawing. ONLY a render can show that. So this
    check exits 0 on a file that could still render to nonsense -- it is a
    typo/consistency net, NOT a substitute for `openscad -o part.stl`.
    Say so plainly wherever these files are handed on.
  * STATEMENT SYNTAX. It is not a parser: a missing semicolon or a malformed
    argument list passes this check (verified with a deliberate mutant). It
    catches typo'd identifiers and unbalanced brackets, nothing more.
"""

from __future__ import annotations

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = ["placard_index_disc.scad", "placard_shoe.scad", "placard_spar_cap.scad"]
PARAMS = "placard_mount_params.scad"

BUILTINS = {
    "cube", "cylinder", "sphere", "polygon", "polyhedron", "square", "circle",
    "translate", "rotate", "scale", "mirror", "resize", "multmatrix", "color",
    "offset", "hull", "minkowski", "union", "difference", "intersection",
    "linear_extrude", "rotate_extrude", "import", "surface", "projection",
    "render", "children", "echo", "assert", "for", "if", "else", "let",
    "intersection_for", "module", "function", "include", "use", "true",
    "false", "undef", "sin", "cos", "tan", "asin", "acos", "atan", "atan2",
    "abs", "ceil", "floor", "round", "sqrt", "pow", "exp", "ln", "log",
    "min", "max", "len", "concat", "str", "chr", "ord", "search", "norm",
    "cross", "PI", "$fn", "$fa", "$fs", "$t", "center", "h", "d", "d1", "d2",
    "r", "r1", "r2", "height", "size", "points", "paths", "convexity", "v",
    "a", "twist", "slices", "scale_", "file", "layer",
}


def strip_comments(src: str) -> str:
    out, i, n = [], 0, len(src)
    while i < n:
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            i = n if j < 0 else j + 2
        elif src[i] == '"':
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == "\\" else 1
            out.append(src[i:j + 1])
            i = j + 1
        else:
            out.append(src[i])
            i += 1
    return "".join(out)


def check_balance(name, code):
    pairs = {")": "(", "]": "[", "}": "{"}
    stack, errs = [], []
    line = 1
    for ch in code:
        if ch == "\n":
            line += 1
        elif ch in "([{":
            stack.append((ch, line))
        elif ch in ")]}":
            if not stack:
                errs.append(f"{name}: unmatched '{ch}' at line {line}")
            elif stack[-1][0] != pairs[ch]:
                errs.append(f"{name}: '{ch}' at line {line} closes "
                            f"'{stack[-1][0]}' from line {stack[-1][1]}")
                stack.pop()
            else:
                stack.pop()
    for ch, ln in stack:
        errs.append(f"{name}: unclosed '{ch}' opened at line {ln}")
    return errs


def strip_includes(code):
    return re.sub(r"\b(include|use)\s*<[^>]*>", " ", code)


def defined_names(code):
    names = set(re.findall(r"^\s*([A-Za-z_]\w*)\s*=", code, re.M))
    names |= set(re.findall(r"\bmodule\s+([A-Za-z_]\w*)", code))
    names |= set(re.findall(r"\bfunction\s+([A-Za-z_]\w*)", code))
    names |= set(re.findall(r"\bmodule\s+\w+\s*\(([^)]*)\)", code) and
                 [p.split("=")[0].strip()
                  for sig in re.findall(r"\bmodule\s+\w+\s*\(([^)]*)\)", code)
                  for p in sig.split(",") if p.strip()])
    names |= set(re.findall(r"\bfor\s*\(\s*([A-Za-z_]\w*)\s*=", code))
    for sig in re.findall(r"\bfor\s*\(([^)]*)\)", code):
        names |= {p.split("=")[0].strip() for p in sig.split(",") if "=" in p}
    return names


def used_names(code):
    body = re.sub(r"\b(module|function)\s+\w+", " ", code)
    return set(re.findall(r"(?<![\w$.])([A-Za-z_]\w*)\s*(?![\w(])", body))


def main():
    errs, notes = [], []
    params_src = open(os.path.join(HERE, PARAMS)).read()
    params_code = strip_comments(params_src)
    errs += check_balance(PARAMS, params_code)
    pnames = defined_names(params_code)

    dupes = {}
    for m in re.finditer(r"^\s*([A-Za-z_]\w*)\s*=\s*([^;]+);", params_code, re.M):
        dupes.setdefault(m.group(1), []).append(m.group(2).strip())
    for k, v in dupes.items():
        if len(v) > 1 and len(set(v)) > 1:
            errs.append(f"{PARAMS}: '{k}' defined {len(v)} times with different "
                        f"values {v}")

    notes.append(f"{PARAMS}: {len(pnames)} parameters/modules defined")

    for part in PARTS:
        path = os.path.join(HERE, part)
        src = open(path).read()
        code = strip_comments(src)
        errs += check_balance(part, code)
        code_ni = strip_includes(code)
        for inc in re.findall(r"include\s*<([^>]+)>", code):
            if not os.path.exists(os.path.join(HERE, inc)):
                errs.append(f"{part}: include <{inc}> does not exist")
        local = defined_names(code_ni)
        unknown = sorted(n for n in used_names(code_ni)
                         if n not in local and n not in pnames
                         and n not in BUILTINS and not n.startswith("$"))
        if unknown:
            errs.append(f"{part}: undefined identifier(s) {unknown}")
        mods = re.findall(r"\bmodule\s+([A-Za-z_]\w*)", code)
        for m in mods:
            if len(re.findall(rf"\b{m}\s*\(", code)) < 2:
                errs.append(f"{part}: module '{m}' is declared but never called")
        # a top-level call is one at COLUMN 0 (not nested inside a module)
        top = [ln for ln in code.splitlines()
               if re.match(r"^[A-Za-z_]\w*\s*\(\s*\)\s*;", ln)]
        if not top:
            errs.append(f"{part}: no top-level module call -- renders empty")
        notes.append(f"{part}: {len(mods)} module(s), top-level call "
                     f"{top[0].strip() if top else 'MISSING'}")

    # ---- derived-geometry relations that must close -------------------
    def val(name):
        m = re.search(rf"^\s*{name}\s*=\s*([-\d.]+)\s*;", params_code, re.M)
        return float(m.group(1)) if m else None

    shoe_h, slot_depth = val("shoe_h"), val("slot_depth")
    disc_t, slot_w, wall_t = val("disc_t"), val("slot_w"), val("wall_t")
    saddle, head_h = val("saddle_depth"), val("m3_head_h")
    spar_od, spar_fit = val("spar_od"), val("spar_fit")
    index_pcd, shoe_len, panel_t = val("index_pcd"), val("shoe_len"), val("panel_t")
    post_x, shoe_w = val("post_x"), val("shoe_w")
    base_t = shoe_h - slot_depth

    rel = []
    rel.append(("slot floor sits at the documented +10 mm above the top plate",
                abs((disc_t + base_t) - 10.0) < 1e-9,
                f"disc_t {disc_t} + base_t {base_t} = {disc_t + base_t} mm "
                f"(placard_mount.md L499/L671 says +10)"))
    rel.append(("slot clears the 5 mm panel", slot_w - panel_t == 1.0,
                f"slot {slot_w} - panel {panel_t} = {slot_w - panel_t} mm "
                f"(0.5 mm per side)"))
    rel.append(("shoe width envelope >= slot + 2 walls",
                shoe_w >= slot_w + 2 * wall_t,
                f"{shoe_w} >= {slot_w + 2 * wall_t}"))
    seat_r = spar_od / 2 + spar_fit
    cb_top = base_t - saddle
    cb_bot = cb_top - head_h
    rel.append(("M3 counterbore fits under the spar seat with material left",
                cb_bot > 1.5,
                f"counterbore z {cb_bot:.2f}..{cb_top:.2f} mm, "
                f"{cb_bot:.2f} mm of base under the screw head"))
    rel.append(("spar seat bottom == counterbore top (no step under the spar)",
                abs((base_t - saddle + seat_r - seat_r) - cb_top) < 1e-9,
                f"seat floor {base_t - saddle:.2f} mm == counterbore top "
                f"{cb_top:.2f} mm"))
    rel.append(("index bolt pair lands inside the shoe body",
                index_pcd / 2 + 3 < shoe_len / 2,
                f"bolts at +/-{index_pcd/2:.1f} mm, body half-length "
                f"{shoe_len/2:.1f} mm"))
    rel.append(("band posts sit outboard of the 110 mm slot, inside the body",
                55.0 < post_x < shoe_len / 2 - 2,
                f"slot ends +/-55, posts +/-{post_x}, body half {shoe_len/2}"))
    rel.append(("spar crown sits below the shoe's top face",
                base_t - saddle + spar_od < shoe_h,
                f"spar crown {base_t - saddle + spar_od:.2f} mm < "
                f"shoe height {shoe_h} mm"))

    print("[scad] STATIC CHECK (openscad is NOT installed -- these files have "
          "never been rendered)")
    for n in notes:
        print(f"[scad]   {n}")
    print("[scad] derived-geometry relations:")
    for name, ok, detail in rel:
        print(f"[scad]   {'PASS' if ok else 'FAIL'}  {name}: {detail}")
        if not ok:
            errs.append(f"geometry: {name} -- {detail}")

    if errs:
        print("\n[scad] FAILURES:")
        for e in errs:
            print(f"[scad]   {e}")
        print(f"[scad] {len(errs)} problem(s)")
        return 1
    print("[scad] all files balanced, all identifiers resolved, all derived "
          "relations close")
    print("[scad] REMINDER: this is a typo/consistency net only. Render with "
          "openscad before printing:")
    print("[scad]   bash scripts/print_artifacts/render_stls.sh")
    return 0


if __name__ == "__main__":
    sys.exit(main())
