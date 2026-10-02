#!/usr/bin/env python3
"""Render the SysML v2 physical-architecture model (docs/sysml/*.sysml) to diagrams.

WHAT THIS IS
------------
docs/sysml/ holds the hardware of every system in this project -- target drone,
interceptor, Tier-1 seeker rig, ground segment and the engagement context -- as
SysML v2 TEXTUAL notation: parts, typed ports, and typed interfaces (wires,
cables, radio paths, mounts). This script turns that text into interconnection
diagrams (power / data / mechanical views per aircraft) and into the HTML block
that render_mbse.py embeds as the MBSE sheet's physical-architecture view.

It reads only a well-defined SUBSET of SysML v2 (documented in
docs/sysml/README.md) and fails loudly on anything else, so a model edit can
never be silently dropped from the pictures. Full-language validation is a
separate, heavier gate: scripts/sysml/check_sysml_official.sh runs the OMG
reference implementation over the same files.

WHAT IT CHECKS (the reference parser does NOT -- tested 2026-10-02: it accepts a
battery wired backwards and an SD slot wired to a GPS mast without complaint)
  * every interface connects a port of exactly the type its end declares, with
    the right conjugation (supplier -> consumer, host -> peripheral);
  * every linkState is planned / wired / verified, and wired/verified name the
    build-step ids (build_tab steps in docs/project_state.json) that prove them;
  * every @BOM points at exactly one build_tab row, and every build_tab row is
    either drawn (@BOM) or deliberately excluded (@NotModelled) -- the model and
    the build sheet cannot drift apart without this failing.

Part STATUS (in hand / ordered / built / ...) is never stored in the model: it is
read from the contract at render time.

USAGE
-----
    python3 scripts/render_sysml.py           # validate + write docs/sysml/views/*.svg
    python3 scripts/render_sysml.py --check   # fail if the model is invalid or views are stale
    python3 scripts/render_sysml.py --png     # also screenshot each view to PNG (needs Chromium)
"""

from __future__ import annotations

import hashlib
import html as _html
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = ROOT / "docs" / "sysml"
VIEW_DIR = MODEL_DIR / "views"
STATE = ROOT / "docs" / "project_state.json"
MANIFEST = VIEW_DIR / "manifest.json"

# Load order matters only for readability of errors; names are global.
MODEL_FILES = [
    "HardwareLibrary.sysml",
    "Components.sysml",
    "Target.sysml",
    "Interceptor.sysml",
    "SeekerRig.sysml",
    "GroundSegment.sysml",
    "Engagement.sysml",
]

CATEGORY_BASES = {
    "PowerLink": "power",
    "DataLink": "data",
    "RFLink": "rf",
    "MechanicalLink": "mechanical",
    "OpticalLink": "optical",
}
ALL_CATS = frozenset(CATEGORY_BASES.values())
LINK_STATES = ("planned", "wired", "verified")
STEP_ID_RE = re.compile(r"^[a-z]{3}-\d{2}$")


class ModelError(Exception):
    pass


# =============================================================================
# 1. Tokenizer + parser for the documented SysML v2 subset
# =============================================================================
TOKEN_RE = re.compile(
    r"""
     (?P<ws>\s+)
    |(?P<line>//[^\n]*)
    |(?P<block>/\*.*?\*/)
    |(?P<str>"(?:[^"\\]|\\.)*")
    |(?P<num>\d+(?:\.\d+)?)
    |(?P<sym>:>>|:>|::\*\*|::\*|::|[{}\[\];:~=,.@])
    |(?P<id>[A-Za-z_][A-Za-z0-9_]*)
    """,
    re.S | re.X,
)

DEF_KINDS = ("part", "port", "item", "attribute", "enum", "metadata", "interface")


@dataclass
class Tok:
    kind: str
    val: str
    file: str
    line: int


def tokenize(text: str, fname: str) -> list[Tok]:
    out, pos, line = [], 0, 1
    while pos < len(text):
        m = TOKEN_RE.match(text, pos)
        if not m:
            raise ModelError(f"{fname}:{line}: unexpected character {text[pos]!r}")
        kind = m.lastgroup
        val = m.group(kind)
        if kind not in ("ws", "line"):
            out.append(Tok(kind, val, fname, line))
        line += val.count("\n")
        pos = m.end()
    return out


def doc_text(block: str) -> str:
    body = block[2:-2]
    lines = [re.sub(r"^\s*\*\s?", "", ln).strip() for ln in body.splitlines()]
    return re.sub(r"\s+", " ", " ".join(ln for ln in lines if ln)).strip()


@dataclass
class Meta:
    name: str
    attrs: dict
    line: int = 0


@dataclass
class PortU:
    name: str
    type: str
    conj: bool
    attrs: dict = field(default_factory=dict)
    doc: str = ""


@dataclass
class PartU:
    name: str
    type: str
    mult: int | None
    owner: str
    file: str
    line: int
    doc: str = ""
    meta: dict = field(default_factory=dict)


@dataclass
class IfaceU:
    name: str
    type: str
    src: list
    dst: list
    owner: str
    file: str
    line: int
    attrs: dict = field(default_factory=dict)
    doc: str = ""
    meta: dict = field(default_factory=dict)


@dataclass
class Def:
    kind: str
    name: str
    abstract: bool
    supers: list
    file: str
    line: int
    package: str
    doc: str = ""
    attrs: dict = field(default_factory=dict)      # declared or redefined attributes
    ends: list = field(default_factory=list)       # (name, type, conj)
    ports: dict = field(default_factory=dict)      # name -> PortU
    parts: dict = field(default_factory=dict)      # name -> PartU
    interfaces: list = field(default_factory=list)
    literals: list = field(default_factory=list)


@dataclass
class Package:
    name: str
    file: str
    doc: str = ""
    defs: list = field(default_factory=list)
    meta: list = field(default_factory=list)      # package-level @NotModelled etc.


class Parser:
    def __init__(self, toks: list[Tok], fname: str):
        self.t, self.i, self.f = toks, 0, fname

    # ---- token helpers
    def peek(self, k: int = 0) -> Tok | None:
        j = self.i + k
        return self.t[j] if j < len(self.t) else None

    def err(self, msg: str, tok: Tok | None = None):
        tok = tok or self.peek() or (self.t[-1] if self.t else None)
        ln = tok.line if tok else 0
        raise ModelError(f"{self.f}:{ln}: {msg}")

    def next(self) -> Tok:
        tok = self.peek()
        if tok is None:
            self.err("unexpected end of file")
        self.i += 1
        return tok

    def at(self, val: str, k: int = 0) -> bool:
        tok = self.peek(k)
        return tok is not None and tok.val == val and tok.kind in ("sym", "id")

    def expect(self, val: str) -> Tok:
        tok = self.next()
        if tok.val != val:
            self.err(f"expected {val!r}, found {tok.val!r}", tok)
        return tok

    def ident(self) -> str:
        tok = self.next()
        if tok.kind != "id":
            self.err(f"expected a name, found {tok.val!r}", tok)
        return tok.val

    def qname(self) -> str:
        parts = [self.ident()]
        while self.at("::"):
            self.next()
            parts.append(self.ident())
        return "::".join(parts)

    def value(self):
        tok = self.next()
        if tok.kind == "str":
            return bytes(tok.val[1:-1], "utf-8").decode("unicode_escape")
        if tok.kind == "num":
            num = float(tok.val) if "." in tok.val else int(tok.val)
            if self.at("["):
                self.next()
                unit = self.ident()
                self.expect("]")
                return f"{tok.val} {unit}"
            return num
        if tok.kind == "id":
            if tok.val in ("true", "false"):
                return tok.val == "true"
            self.i -= 1
            return self.qname().split("::")[-1]       # enum literal reference
        self.err(f"expected a value, found {tok.val!r}", tok)

    # ---- grammar
    def parse_file(self) -> list[Package]:
        pkgs = []
        while self.peek() is not None:
            self.expect("package")
            name = self.ident()
            pkg = Package(name, self.f)
            self.expect("{")
            self.package_body(pkg)
            pkgs.append(pkg)
        return pkgs

    def skip_import(self):
        if self.at("private") or self.at("public"):
            self.next()
        self.expect("import")
        self.qname()
        if self.at("::*") or self.at("::**"):
            self.next()
        self.expect(";")

    def package_body(self, pkg: Package):
        while not self.at("}"):
            if self.at("doc"):
                self.next()
                pkg.doc = doc_text(self.next().val)
            elif self.at("private") or self.at("public") or self.at("import"):
                self.skip_import()
            elif self.at("@"):
                pkg.meta.append(self.meta_usage())
            elif self.at("abstract") or (self.peek(1) and self.peek(1).val == "def"):
                pkg.defs.append(self.definition(pkg.name))
            else:
                self.err(f"unsupported package member starting {self.peek().val!r} "
                         f"(see docs/sysml/README.md for the accepted subset)")
        self.expect("}")

    def meta_usage(self) -> Meta:
        line = self.expect("@").line
        name = self.qname().split("::")[-1]
        attrs = {}
        if self.at("{"):
            self.next()
            while not self.at("}"):
                if self.at(":>>"):
                    self.next()
                key = self.ident()
                self.expect("=")
                attrs[key] = self.value()
                self.expect(";")
            self.next()
        else:
            self.expect(";")
        return Meta(name, attrs, line)

    def definition(self, pkg: str) -> Def:
        abstract = False
        if self.at("abstract"):
            self.next()
            abstract = True
        kind_tok = self.next()
        if kind_tok.val not in DEF_KINDS:
            self.err(f"unsupported definition kind {kind_tok.val!r}", kind_tok)
        self.expect("def")
        name = self.ident()
        supers = []
        if self.at(":>"):
            self.next()
            supers.append(self.qname().split("::")[-1])
            while self.at(","):
                self.next()
                supers.append(self.qname().split("::")[-1])
        d = Def(kind_tok.val, name, abstract, supers, self.f, kind_tok.line, pkg)
        if self.at(";"):
            self.next()
            return d
        self.expect("{")
        self.def_body(d)
        return d

    def attribute(self, attrs: dict):
        """`attribute name (: T | :> T)? (= v)? ;`  or  `attribute :>> name = v;`"""
        self.expect("attribute")
        if self.at(":>>"):
            self.next()
        name = self.ident()
        if self.at(":") or self.at(":>"):
            self.next()
            self.qname()
        val = None
        if self.at("="):
            self.next()
            val = self.value()
        self.expect(";")
        attrs[name] = val

    def port_usage(self) -> PortU:
        self.expect("port")
        name = self.ident()
        self.expect(":")
        conj = False
        if self.at("~"):
            self.next()
            conj = True
        p = PortU(name, self.qname().split("::")[-1], conj)
        if self.at("{"):
            self.next()
            while not self.at("}"):
                if self.at("doc"):
                    self.next()
                    p.doc = doc_text(self.next().val)
                elif self.at("attribute"):
                    self.attribute(p.attrs)
                else:
                    self.err(f"unsupported port-usage member {self.peek().val!r}")
            self.next()
        else:
            self.expect(";")
        return p

    def chain(self) -> list[str]:
        out = [self.ident()]
        while self.at("."):
            self.next()
            out.append(self.ident())
        return out

    def def_body(self, d: Def):
        while not self.at("}"):
            tok = self.peek()
            if self.at("doc"):
                self.next()
                d.doc = doc_text(self.next().val)
            elif self.at("private") or self.at("public") or self.at("import"):
                self.skip_import()
            elif self.at("abstract") or (self.peek(1) and self.peek(1).val == "def"):
                self.err("nested definitions are not in the supported subset")
            elif self.at("enum"):
                self.next()
                d.literals.append(self.ident())
                self.expect(";")
            elif self.at("end"):
                self.next()
                ename = self.ident()
                self.expect(":")
                conj = False
                if self.at("~"):
                    self.next()
                    conj = True
                d.ends.append((ename, self.qname().split("::")[-1], conj))
                self.expect(";")
            elif self.at("attribute"):
                self.attribute(d.attrs)
            elif tok.val in ("in", "out", "inout"):
                self.next()
                self.expect("item")
                self.ident()
                self.expect(":")
                self.qname()
                self.expect(";")
            elif self.at("port"):
                p = self.port_usage()
                if p.name in d.ports:
                    self.err(f"duplicate port {p.name!r} in {d.name}")
                d.ports[p.name] = p
            elif self.at("part"):
                part = self.part_usage(d.name)
                if part.name in d.parts:
                    self.err(f"duplicate part {part.name!r} in {d.name}")
                d.parts[part.name] = part
            elif self.at("interface"):
                d.interfaces.append(self.interface_usage(d.name))
            else:
                self.err(f"unsupported member {tok.val!r} in {d.kind} def {d.name} "
                         f"(see docs/sysml/README.md for the accepted subset)")
        self.next()

    def part_usage(self, owner: str) -> PartU:
        tok = self.expect("part")
        name = self.ident()
        self.expect(":")
        typ = self.qname().split("::")[-1]
        mult = None
        if self.at("["):
            self.next()
            mult = int(self.next().val)
            self.expect("]")
        p = PartU(name, typ, mult, owner, self.f, tok.line)
        if self.at("{"):
            self.next()
            while not self.at("}"):
                if self.at("doc"):
                    self.next()
                    p.doc = doc_text(self.next().val)
                elif self.at("@"):
                    m = self.meta_usage()
                    p.meta[m.name] = m.attrs
                else:
                    self.err(f"unsupported part-usage member {self.peek().val!r}")
            self.next()
        else:
            self.expect(";")
        return p

    def interface_usage(self, owner: str) -> IfaceU:
        tok = self.expect("interface")
        name = self.ident() if not self.at(":") else ""
        self.expect(":")
        typ = self.qname().split("::")[-1]
        self.expect("connect")
        src = self.chain()
        self.expect("to")
        dst = self.chain()
        iu = IfaceU(name, typ, src, dst, owner, self.f, tok.line)
        if self.at("{"):
            self.next()
            while not self.at("}"):
                if self.at("doc"):
                    self.next()
                    iu.doc = doc_text(self.next().val)
                elif self.at("@"):
                    m = self.meta_usage()
                    iu.meta[m.name] = m.attrs
                elif self.at("attribute"):
                    self.attribute(iu.attrs)
                else:
                    self.err(f"unsupported interface-usage member {self.peek().val!r}")
            self.next()
        else:
            self.expect(";")
        if not iu.name:
            raise ModelError(f"{self.f}:{tok.line}: give every interface usage a name "
                             f"(the diagrams and wire lists print it)")
        return iu


# =============================================================================
# 2. Model: resolution + semantic checks
# =============================================================================
@dataclass
class Model:
    packages: list
    defs: dict
    sources: dict          # file name -> text
    build_rows: dict       # tab -> [row dict]
    step_ids: set
    errors: list = field(default_factory=list)

    # ---------------------------------------------------------------- lookups
    def d(self, name: str) -> Def:
        return self.defs[name]

    def systems(self) -> list[Def]:
        return [x for x in self.defs.values() if x.kind == "part" and x.parts]

    def categories(self, iface_type: str) -> list[str]:
        """Ordered categories of an interface def (declaration order of supers)."""
        out, seen = [], set()

        def walk(n):
            if n in seen or n not in self.defs:
                return
            seen.add(n)
            if n in CATEGORY_BASES and CATEGORY_BASES[n] not in out:
                out.append(CATEGORY_BASES[n])
            for s in self.defs[n].supers:
                walk(s)

        walk(iface_type)
        return out

    def iface_attrs(self, iface_type: str) -> set:
        names, seen = set(), set()

        def walk(n):
            if n in seen or n not in self.defs:
                return
            seen.add(n)
            names.update(self.defs[n].attrs)
            for s in self.defs[n].supers:
                walk(s)

        walk(iface_type)
        return names

    def resolve_chain(self, owner: Def, chain: list[str]):
        """-> (list of PartU along the chain, PortU) or raise ModelError."""
        parts, cur = [], owner
        for i, el in enumerate(chain[:-1]):
            if el not in cur.parts:
                raise ModelError(f"{'.'.join(chain)}: {cur.name} has no part {el!r}")
            pu = cur.parts[el]
            parts.append(pu)
            if pu.type not in self.defs:
                raise ModelError(f"{'.'.join(chain)}: part {el!r} has unknown type {pu.type!r}")
            cur = self.defs[pu.type]
        last = chain[-1]
        if last not in cur.ports:
            raise ModelError(f"{'.'.join(chain)}: {cur.name} has no port {last!r} "
                             f"(ports: {', '.join(cur.ports) or 'none'})")
        if len(chain) < 2:
            raise ModelError(f"{last}: an interface end must be part.port, not a bare name")
        return parts, cur.ports[last]

    def bom_row(self, meta: dict):
        tab, name = meta.get("tab"), meta.get("bomName")
        rows = self.build_rows.get(tab)
        if rows is None:
            raise ModelError(f"@BOM tab {tab!r} is not a build_tab subsystem "
                             f"({', '.join(sorted(self.build_rows))})")
        hits = [r for r in rows if str(name).lower() in r["name"].lower()]
        if len(hits) != 1:
            raise ModelError(f"@BOM bomName {name!r} matches {len(hits)} rows in build_tab "
                             f"'{tab}' (must be exactly 1): "
                             f"{[r['name'] for r in hits] or 'no row'}")
        return hits[0]


def load_contract() -> tuple[dict, dict, set]:
    state = json.loads(STATE.read_text())
    rows, steps = {}, set()
    for sub in state["build_tab"]["subsystems"]:
        rows[sub["id"]] = sub["parts"]
        for st in sub.get("steps", []):
            steps.add(st["id"])
    return state, rows, steps


def load_model(model_dir: Path = MODEL_DIR, contract=None) -> Model:
    if contract is None:
        contract = load_contract()
    _, rows, steps = contract
    pkgs, defs, sources = [], {}, {}
    on_disk = sorted(p.name for p in model_dir.glob("*.sysml"))
    if sorted(MODEL_FILES) != on_disk:
        raise ModelError(f"docs/sysml/*.sysml {on_disk} != MODEL_FILES {sorted(MODEL_FILES)} "
                         f"-- add new files to MODEL_FILES in scripts/render_sysml.py")
    for fn in MODEL_FILES:
        text = (model_dir / fn).read_text()
        sources[fn] = text
        for pkg in Parser(tokenize(text, fn), fn).parse_file():
            pkgs.append(pkg)
            for d in pkg.defs:
                if d.name in defs:
                    raise ModelError(f"{fn}:{d.line}: definition {d.name!r} already defined "
                                     f"in {defs[d.name].file}")
                defs[d.name] = d
    m = Model(pkgs, defs, sources, rows, steps)
    check_model(m)
    return m


def port_compatible(m: Model, port: PortU, end) -> bool:
    _, etype, econj = end
    return port.type == etype and port.conj == econj


def check_model(m: Model) -> None:
    errs = []

    def e(where, msg):
        errs.append(f"{where}: {msg}")

    # --- type references
    for d in m.defs.values():
        for s in d.supers:
            if s not in m.defs:
                e(f"{d.file}:{d.line}", f"{d.name} specialises unknown {s!r}")
        for pu in d.ports.values():
            if pu.type not in m.defs or m.defs[pu.type].kind != "port":
                e(f"{d.file}:{d.line}", f"port {d.name}.{pu.name} typed by non-port-def {pu.type!r}")
        for en, et, _ in d.ends:
            if et not in m.defs or m.defs[et].kind != "port":
                e(f"{d.file}:{d.line}", f"end {d.name}.{en} typed by non-port-def {et!r}")
        if d.kind == "interface" and not d.abstract and len(d.ends) != 2:
            e(f"{d.file}:{d.line}", f"interface def {d.name} must declare exactly 2 ends")
        if d.kind == "interface" and not d.abstract and not m.categories(d.name):
            e(f"{d.file}:{d.line}", f"interface def {d.name} specialises no link category "
                                    f"({', '.join(CATEGORY_BASES)})")

    link_states = set(m.defs["LinkState"].literals) if "LinkState" in m.defs else set()
    if link_states != set(LINK_STATES):
        e("HardwareLibrary.sysml", f"LinkState literals {sorted(link_states)} != {LINK_STATES}")

    bom_hits = {}   # (tab, row name) -> list of referrers

    def note_bom(meta, who, where):
        try:
            row = m.bom_row(meta)
            bom_hits.setdefault((meta["tab"], row["name"]), []).append(who)
        except ModelError as ex:
            e(where, f"{who}: {ex}")

    for sysdef in m.systems():
        cells = {}
        for pu in sysdef.parts.values():
            where = f"{pu.file}:{pu.line}"
            if pu.type not in m.defs or m.defs[pu.type].kind != "part":
                e(where, f"part {sysdef.name}.{pu.name} typed by non-part-def {pu.type!r}")
                continue
            lay = pu.meta.get("Layout")
            if not lay or not isinstance(lay.get("col"), int) or not isinstance(lay.get("row"), int):
                e(where, f"part {pu.name} needs @Layout {{ col = <int>; row = <int>; }}")
            else:
                cell = (lay["col"], lay["row"])
                if cell in cells:
                    e(where, f"parts {cells[cell]!r} and {pu.name!r} share layout cell {cell}")
                cells[cell] = pu.name
            is_subsystem = bool(m.defs[pu.type].parts)
            if "BOM" in pu.meta:
                note_bom(pu.meta["BOM"], f"{sysdef.name}.{pu.name}", where)
            elif "External" not in pu.meta and not is_subsystem:
                e(where, f"part {pu.name} needs @BOM {{ tab; bomName; }} or @External")

        names = set()
        for iu in sysdef.interfaces:
            where = f"{iu.file}:{iu.line}"
            if iu.name in names:
                e(where, f"duplicate interface name {iu.name!r} in {sysdef.name}")
            names.add(iu.name)
            idef = m.defs.get(iu.type)
            if not idef or idef.kind != "interface":
                e(where, f"{iu.name}: {iu.type!r} is not an interface def")
                continue
            if idef.abstract:
                e(where, f"{iu.name}: {iu.type} is abstract -- use a concrete interface def")
                continue
            for side, chain, end in (("from", iu.src, idef.ends[0]), ("to", iu.dst, idef.ends[1])):
                try:
                    _, port = m.resolve_chain(sysdef, chain)
                except ModelError as ex:
                    e(where, f"{iu.name}: {ex}")
                    continue
                if not port_compatible(m, port, end):
                    want = ("~" if end[2] else "") + end[1]
                    got = ("~" if port.conj else "") + port.type
                    e(where, f"{iu.name}: '{side}' end {'.'.join(chain)} is {got}, but "
                             f"{iu.type}.{end[0]} needs {want} -- wrong port, or the wire is "
                             f"drawn backwards (supplier/host goes first)")
            allowed = m.iface_attrs(iu.type)
            for k in iu.attrs:
                if k not in allowed:
                    e(where, f"{iu.name}: attribute {k!r} is not declared on {iu.type}")
            st = iu.attrs.get("linkState")
            if st not in LINK_STATES:
                e(where, f"{iu.name}: linkState must be one of {LINK_STATES}, got {st!r}")
            ev = str(iu.attrs.get("evidence") or "").strip()
            ev_ids = [x.strip() for x in ev.split(",") if x.strip()]
            if st in ("wired", "verified") and not ev_ids:
                e(where, f"{iu.name}: linkState {st} needs evidence = \"<build step id>\"")
            for x in ev_ids:
                if not STEP_ID_RE.match(x) or x not in m.step_ids:
                    e(where, f"{iu.name}: evidence {x!r} is not a build_tab step id "
                             f"in docs/project_state.json")
            if "BOM" in iu.meta:
                note_bom(iu.meta["BOM"], f"{sysdef.name}.{iu.name}", where)

    # --- build-sheet coverage, both directions
    excluded = {}
    for pkg in m.packages:
        for meta in pkg.meta:
            if meta.name != "NotModelled":
                e(pkg.file, f"unsupported package-level annotation @{meta.name}")
                continue
            if not meta.attrs.get("reason"):
                e(f"{pkg.file}:{meta.line}", "@NotModelled needs a reason")
            try:
                row = m.bom_row(meta.attrs)
            except ModelError as ex:
                e(f"{pkg.file}:{meta.line}", str(ex))
                continue
            key = (meta.attrs["tab"], row["name"])
            if key in bom_hits:
                e(f"{pkg.file}:{meta.line}", f"build_tab row {row['name']!r} is BOTH drawn "
                                             f"({bom_hits[key][0]}) and @NotModelled")
            excluded[key] = meta.attrs["reason"]
    for tab, rows in m.build_rows.items():
        for r in rows:
            key = (tab, r["name"])
            if key not in bom_hits and key not in excluded:
                e("build_tab", f"row {r['name']!r} ({tab}) is neither drawn by a @BOM nor "
                               f"listed as @NotModelled -- add it to the model or exclude it "
                               f"with a reason")
    m.coverage = {"drawn": bom_hits, "excluded": excluded}
    if errs:
        raise ModelError("SysML model check failed:\n  - " + "\n  - ".join(errs))


# =============================================================================
# 3. Views + layout + SVG
# =============================================================================
VIEWS = [
    # id, system def, categories, title, one-line purpose
    ("interceptor-power", "InterceptorDrone", {"power"}, "Interceptor — power",
     "Two independent rails: PM02 feeds the flight controller, the Matek BEC feeds the Pi."),
    ("interceptor-data", "InterceptorDrone", {"data"}, "Interceptor — data & signal",
     "Camera into the Pi, MAVLink from the Pi to PX4 on TELEM2, DShot to the ESC."),
    ("interceptor-mech", "InterceptorDrone", {"mechanical"}, "Interceptor — mechanical",
     "What bolts to what: the camera hangs off a printed nose mount, the Pi sits on a tray."),
    ("target-power", "TargetDrone", {"power"}, "Target — power",
     "One 6S pack through the Tekko32 stack; the Kakute's BEC powers GPS, RX and buzzer."),
    ("target-data", "TargetDrone", {"data"}, "Target — data & signal",
     "ArduPilot on the Kakute: CRSF in on UART6, GPS on UART3, DataFlash log to the card."),
    ("target-mech", "TargetDrone", {"mechanical"}, "Target — mechanical",
     "The AprilTag placard rides the top plate, facing the approach."),
    ("seeker-rig", "TripodSeekerRig", set(ALL_CATS), "Tier-1 seeker rig — all links",
     "The interceptor's eye on a tripod: the same camera, Pi and card that later fly."),
    ("ground", "Ground", set(ALL_CATS), "Ground segment — all links",
     "One transmitter per aircraft so each keeps an independent kill."),
    ("engagement", "EngagementContext", set(ALL_CATS), "Engagement context — cross-system links",
     "Everything that crosses between the aircraft, the ground and the sky."),
]

CAT_COLOR = {   # light-mode fallbacks; the page overrides via --sx-* vars
    "power": "#B5452B", "data": "#2F6690", "rf": "#7A4E9C",
    "mechanical": "#6E6A5C", "optical": "#2D7D5A",
}
CAT_LABEL = {"power": "power", "data": "data / signal", "rf": "radio",
             "mechanical": "mechanical", "optical": "optical"}
STATUS_CLASS = {"in hand": "ok", "built": "ok", "ordered": "warn", "print": "acc",
                "must-add": "bad", "owned": "mut", "system": "mut"}

BOX_MIN_W, HEADER, PITCH, PAD = 214, 46, 22, 10
COL_GAP, ROW_GAP, MARGIN = 118, 74, 64
CH_W = 6.1          # px per char, 10 px mono (layout estimate)
LANE = 7


def esc(s) -> str:
    return _html.escape(str(s if s is not None else ""))


def seg_hits_box(p, q, box, pad=3) -> bool:
    x0, y0, x1, y1 = box
    x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
    (ax, ay), (bx, by) = p, q
    if ay == by:
        lo, hi = sorted((ax, bx))
        return y0 < ay < y1 and hi > x0 and lo < x1
    lo, hi = sorted((ay, by))
    return x0 < ax < x1 and hi > y0 and lo < y1


def part_status(m: Model, pu: PartU) -> str:
    if "BOM" in pu.meta:
        return m.bom_row(pu.meta["BOM"])["status"]
    if "External" in pu.meta:
        return "owned"
    return "system"


def view_edges(m: Model, sysdef: Def, cats: set):
    out = []
    for iu in sysdef.interfaces:
        icats = m.categories(iu.type)
        hit = [c for c in icats if c in cats]
        if hit:
            out.append((iu, hit[0]))
    return out


def render_view(m: Model, view) -> tuple[str, list]:
    vid, sysname, cats, title, purpose = view
    sysdef = m.d(sysname)
    edges = view_edges(m, sysdef, cats)
    if not edges:
        raise ModelError(f"view {vid}: no connections of {sorted(cats)} in {sysname}")
    show_all = set(cats) == set(ALL_CATS)
    nodes = [p for p in sysdef.parts.values()
             if show_all or any(p.name in (iu.src[0], iu.dst[0]) for iu, _ in edges)]
    names = {p.name for p in nodes}

    # --- compress the grid
    cols = sorted({p.meta["Layout"]["col"] for p in nodes})
    rows = sorted({p.meta["Layout"]["row"] for p in nodes})
    cell = {p.name: (cols.index(p.meta["Layout"]["col"]), rows.index(p.meta["Layout"]["row"]))
            for p in nodes}

    # --- ports per node and side
    ports = {}      # (node, label) -> dict(side, conj, others[])
    for iu, _ in edges:
        for here, there in ((iu.src, iu.dst), (iu.dst, iu.src)):
            if here[0] not in names:
                continue
            key = (here[0], ".".join(here[1:]))
            _, pu = m.resolve_chain(sysdef, here)
            ports.setdefault(key, {"conj": pu.conj, "others": []})["others"].append(there[0])
    for (node, _), info in ports.items():
        c0 = cell[node][0]
        votes = [("R" if cell[o][0] >= c0 else "L") for o in info["others"]]
        info["side"] = max(sorted(set(votes)), key=votes.count) if votes else "R"

    # --- box sizes
    def label_w(s):
        return len(s) * CH_W

    W = BOX_MIN_W
    for p in nodes:
        lw = max([label_w(l) for (n, l), i in ports.items() if n == p.name and i["side"] == "L"] or [0])
        rw = max([label_w(l) for (n, l), i in ports.items() if n == p.name and i["side"] == "R"] or [0])
        typ = f": {p.type}" + (f" [{p.mult}]" if p.mult else "")
        W = max(W, lw + rw + 44, len(p.name) * 8.2 + 30, len(typ) * CH_W + 96)
    W = int(round(W))
    h_of = {}
    for p in nodes:
        nl = sum(1 for (n, _), i in ports.items() if n == p.name and i["side"] == "L")
        nr = sum(1 for (n, _), i in ports.items() if n == p.name and i["side"] == "R")
        h_of[p.name] = HEADER + max(nl, nr, 1) * PITCH + PAD
    ncols, nrows = len(cols), len(rows)
    row_h = [max([h_of[p.name] for p in nodes if cell[p.name][1] == r] or [60]) for r in range(nrows)]
    top = 64
    col_x = [MARGIN + c * (W + COL_GAP) for c in range(ncols)]
    row_y, y = [], top
    for r in range(nrows):
        row_y.append(y)
        y += row_h[r] + ROW_GAP
    diagram_h = y - ROW_GAP + 40
    width = MARGIN * 2 + ncols * W + (ncols - 1) * COL_GAP

    box = {}
    for p in nodes:
        c, r = cell[p.name]
        box[p.name] = (col_x[c], row_y[r], col_x[c] + W, row_y[r] + h_of[p.name])

    def vch_x(k):
        if k == 0:
            return col_x[0] - MARGIN / 2
        if k == ncols:
            return col_x[-1] + W + MARGIN / 2
        return (col_x[k - 1] + W + col_x[k]) / 2

    def hch_y(j):
        if j == 0:
            return row_y[0] - 26
        if j == nrows:
            return row_y[-1] + row_h[-1] + 24
        return (row_y[j - 1] + row_h[j - 1] + row_y[j]) / 2

    # --- port anchor positions (sorted by counterpart y to cut crossings)
    anchor = {}
    for p in nodes:
        for side in ("L", "R"):
            keys = [k for k, i in ports.items() if k[0] == p.name and i["side"] == side]

            def other_y(k):
                os_ = ports[k]["others"]
                return sum(box[o][1] for o in os_ if o in box) / max(1, len(os_))

            keys.sort(key=lambda k: (other_y(k), k[1]))
            x0, y0, x1, _ = box[p.name]
            for i, k in enumerate(keys):
                anchor[k] = (x0 if side == "L" else x1, y0 + HEADER + PITCH * i + PITCH / 2, side)

    # --- route: every wire gets lanes in the channels it uses, chosen so that no
    # segment runs collinear with another wire's (wires sharing a port may overlap:
    # that is one electrical net fanning out).
    used = {}       # channel key -> set of offsets taken
    drawn = []      # (p, q, port keys) of every placed segment
    boxes = list(box.items())
    OFFS = [0] + [s * LANE * k for k in range(1, 9) for s in (1, -1)]

    def free(a, b, skip):
        return not any(seg_hits_box(a, b, bx) for n, bx in boxes if n not in skip)

    def collinear(a, b, c, d):
        if abs(a[1] - b[1]) < .5 and abs(c[1] - d[1]) < .5 and abs(a[1] - c[1]) < 3:
            lo, hi = sorted((a[0], b[0]))
            lo2, hi2 = sorted((c[0], d[0]))
            return min(hi, hi2) - max(lo, lo2) > 2
        if abs(a[0] - b[0]) < .5 and abs(c[0] - d[0]) < .5 and abs(a[0] - c[0]) < 3:
            lo, hi = sorted((a[1], b[1]))
            lo2, hi2 = sorted((c[1], d[1]))
            return min(hi, hi2) - max(lo, lo2) > 2
        return False

    def clash(pts, keys):
        for i in range(len(pts) - 1):
            for (c, d, k2) in drawn:
                if not (keys & k2) and collinear(pts[i], pts[i + 1], c, d):
                    return True
        return False

    def place(chans, gen, keys):
        """Try lane offsets until the path clashes with nothing; claim them."""
        first = None
        for k in range(len(OFFS)):
            offs = [OFFS[k] for _ in chans]
            if any(o in used.get(ch, set()) for o, ch in zip(offs, chans)):
                continue
            pts = gen(offs)
            if first is None:
                first = (offs, pts)
            if not clash(pts, keys):
                break
        else:
            offs, pts = first
        for o, ch in zip(offs, chans):
            used.setdefault(ch, set()).add(o)
        return pts

    routed = []
    for idx, (iu, cat) in enumerate(edges, 1):
        ka = (iu.src[0], ".".join(iu.src[1:]))
        kb = (iu.dst[0], ".".join(iu.dst[1:]))
        keys = {ka, kb}
        xa, ya, sa = anchor[ka]
        xb, yb, sb = anchor[kb]
        ca, cb = cell[ka[0]][0], cell[kb[0]][0]
        va = ca + 1 if sa == "R" else ca
        vb = cb + 1 if sb == "R" else cb
        A, B = (xa, ya), (xb, yb)
        skip = {ka[0], kb[0]}
        pts = None
        if va == vb:
            pts = place([("v", va)], lambda o: [A, (vch_x(va) + o[0], ya),
                                                (vch_x(va) + o[0], yb), B], keys)
        elif abs(ya - yb) < 0.5 and free(A, B, skip) and not clash([A, B], keys):
            pts = [A, B]
        else:
            for vk in (va, vb):
                x = vch_x(vk)
                cand = [A, (x, ya), (x, yb), B]
                if all(free(cand[i], cand[i + 1], skip) for i in range(3)):
                    pts = place([("v", vk)], lambda o, x=x: [A, (x + o[0], ya),
                                                             (x + o[0], yb), B], keys)
                    break
            if pts is None:
                ra, rb = cell[ka[0]][1], cell[kb[0]][1]
                lo, hi = sorted((ra, rb))
                j = sorted(range(nrows + 1),
                           key=lambda j: (0 if lo < j <= hi else 1,
                                          abs(hch_y(j) - (ya + yb) / 2)))[0]
                pts = place([("v", va), ("v", vb), ("h", j)],
                            lambda o: [A, (vch_x(va) + o[0], ya), (vch_x(va) + o[0], hch_y(j) + o[2]),
                                       (vch_x(vb) + o[1], hch_y(j) + o[2]),
                                       (vch_x(vb) + o[1], yb), B], keys)
        for i in range(len(pts) - 1):
            drawn.append((pts[i], pts[i + 1], keys))
        routed.append((idx, iu, cat, pts))

    # --- pills on the longest segment, nudged off each other
    pills = []
    for idx, iu, cat, pts in routed:
        segs = sorted(((abs(pts[i][0] - pts[i + 1][0]) + abs(pts[i][1] - pts[i + 1][1]), i)
                       for i in range(len(pts) - 1)), reverse=True)
        L, i = segs[0]
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        best = None
        for t in (0.5, 0.35, 0.65, 0.25, 0.75, 0.15, 0.85):
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            if all(abs(px - qx) > 24 or abs(py - qy) > 16 for qx, qy in pills) and \
               not any(seg_hits_box((px - 11, py), (px + 11, py), bx_) for _, bx_ in boxes):
                best = (px, py)
                break
        if best is None:
            best = (ax + (bx - ax) * 0.5, ay + (by - ay) * 0.5)
        pills.append(best)

    # --- wire list (inside the SVG so the PNG stands alone)
    list_top = diagram_h + 30
    wl_lines = []
    for (idx, iu, cat, _), _ in zip(routed, pills):
        wl_lines.append((idx, iu, cat))
    line_h = 30
    legend_y = list_top + len(wl_lines) * line_h + 18
    total_h = legend_y + 36

    o = []
    o.append(f'<svg xmlns="http://www.w3.org/2000/svg" class="sx" viewBox="0 0 {width} {total_h}" '
             f'width="{width}" height="{total_h}" role="img" aria-labelledby="t-{vid}">')
    o.append(f'<title id="t-{vid}">{esc(title)}: SysML v2 interconnection view of {esc(sysname)}</title>')
    o.append(SVG_STYLE)
    o.append(f'<rect class="sx-bg" x="0" y="0" width="{width}" height="{total_h}"/>')
    # SysML diagram frame + name tab
    tab_txt = f"ibd [part def] {sysname} [{'all' if show_all else ' + '.join(sorted(cats))}]"
    tw = len(tab_txt) * 7.5 + 24
    o.append(f'<rect class="sx-frame" x="12" y="12" width="{width - 24}" height="{diagram_h - 12}"/>')
    o.append(f'<path class="sx-frame" d="M12 36 H{tw:.0f} L{tw + 12:.0f} 24 V12"/>')
    o.append(f'<text class="sx-tab" x="22" y="29">{esc(tab_txt)}</text>')
    o.append(f'<text class="sx-purpose" x="{tw + 26:.0f}" y="29">{esc(purpose)}</text>')

    # boxes
    for p in nodes:
        x0, y0, x1, y1 = box[p.name]
        st = part_status(m, p)
        cls = STATUS_CLASS.get(st, "mut")
        o.append(f'<g class="sx-node"><rect class="sx-box" x="{x0}" y="{y0}" '
                 f'width="{x1 - x0}" height="{y1 - y0}" rx="2"/>')
        o.append(f'<text class="sx-name" x="{x0 + 10}" y="{y0 + 18}">{esc(p.name)}</text>')
        typ = f": {p.type}" + (f" [{p.mult}]" if p.mult else "")
        o.append(f'<text class="sx-type" x="{x0 + 10}" y="{y0 + 34}">{esc(typ)}</text>')
        chip = st.upper()
        cw = len(chip) * 6.4 + 12
        o.append(f'<rect class="sx-chip sx-{cls}" x="{x1 - cw - 8:.1f}" y="{y0 + 24}" '
                 f'width="{cw:.1f}" height="15" rx="2"/>')
        o.append(f'<text class="sx-chipt sx-{cls}" x="{x1 - cw / 2 - 8:.1f}" y="{y0 + 35}">{esc(chip)}</text>')
        o.append(f'<line class="sx-rule" x1="{x0}" y1="{y0 + HEADER - 4}" x2="{x1}" y2="{y0 + HEADER - 4}"/>')
        o.append('</g>')

    # wires
    for (idx, iu, cat, pts) in routed:
        st = iu.attrs.get("linkState")
        d = "M" + " L".join(f"{x:.1f} {y:.1f}" for x, y in pts)
        o.append(f'<path class="sx-w sx-c-{cat} sx-s-{st}" d="{d}"/>')

    # port squares + labels
    for (node, label), (x, y, side) in anchor.items():
        info = ports[(node, label)]
        cls = "sx-pc" if info["conj"] else "sx-p"
        o.append(f'<rect class="{cls}" x="{x - 5:.1f}" y="{y - 5:.1f}" width="10" height="10"/>')
        if side == "L":
            o.append(f'<text class="sx-pl" x="{x + 9:.1f}" y="{y + 3.5:.1f}">{esc(label)}</text>')
        else:
            o.append(f'<text class="sx-pl sx-end" x="{x - 9:.1f}" y="{y + 3.5:.1f}">{esc(label)}</text>')

    # pills
    for (idx, iu, cat, _), (px, py) in zip(routed, pills):
        st = iu.attrs.get("linkState")
        o.append(f'<g class="sx-pill sx-c-{cat} sx-s-{st}"><rect x="{px - 11:.1f}" y="{py - 8:.1f}" '
                 f'width="22" height="16" rx="8"/><text x="{px:.1f}" y="{py + 4:.1f}">{idx}</text></g>')

    # wire list
    o.append(f'<text class="sx-h" x="20" y="{list_top - 8}">WIRE LIST — every connection in this view</text>')
    for k, (idx, iu, cat) in enumerate(wl_lines):
        y0 = list_top + k * line_h + 12
        st = iu.attrs.get("linkState")
        ev = iu.attrs.get("evidence") or ""
        o.append(f'<g class="sx-pill sx-c-{cat} sx-s-{st}"><rect x="20" y="{y0 - 12}" width="22" '
                 f'height="16" rx="8"/><text x="31" y="{y0}">{idx}</text></g>')
        frm, to = ".".join(iu.src), ".".join(iu.dst)
        cable = ""
        if "BOM" in iu.meta:
            cable = f" · cable: {m.bom_row(iu.meta['BOM'])['name']}"
        stxt = st.upper() + (f" ({ev})" if ev and st != "planned" else "")
        o.append(f'<text class="sx-wl" x="52" y="{y0}"><tspan class="sx-wn">{esc(iu.name)}</tspan>'
                 f'<tspan class="sx-wm">  {esc(frm)} → {esc(to)}</tspan>'
                 f'<tspan class="sx-wt">  : {esc(iu.type)}</tspan>'
                 f'<tspan class="sx-ws sx-s-{st}">  {esc(stxt)}</tspan></text>')
        o.append(f'<text class="sx-wd" x="52" y="{y0 + 14}">{esc(iu.attrs.get("connector") or "")}'
                 f'{" — " + esc(iu.attrs.get("detail")) if iu.attrs.get("detail") else ""}'
                 f'{esc(cable)}</text>')

    # legend
    lx, ly = 20, legend_y
    o.append(f'<text class="sx-h" x="{lx}" y="{ly - 4}">KEY</text>')
    lx += 40
    used_cats = []
    for _, _, cat, _ in routed:
        if cat not in used_cats:
            used_cats.append(cat)
    for cat in used_cats:
        o.append(f'<path class="sx-w sx-c-{cat} sx-s-verified" d="M{lx} {ly - 8} H{lx + 30}"/>')
        o.append(f'<text class="sx-lg" x="{lx + 36}" y="{ly - 4}">{CAT_LABEL[cat]}</text>')
        lx += 44 + len(CAT_LABEL[cat]) * 6.5
    for st, lab in (("verified", "verified (logged step)"), ("wired", "wired, untested"),
                    ("planned", "planned")):
        o.append(f'<g class="sx-pill sx-c-{used_cats[0]} sx-s-{st}"><rect x="{lx}" y="{ly - 16}" width="22" '
                 f'height="16" rx="8"/><text x="{lx + 11}" y="{ly - 4}">n</text></g>')
        o.append(f'<text class="sx-lg" x="{lx + 28}" y="{ly - 4}">{lab}</text>')
        lx += 40 + len(lab) * 6.2
    o.append(f'<rect class="sx-p" x="{lx}" y="{ly - 14}" width="10" height="10"/>')
    o.append(f'<text class="sx-lg" x="{lx + 16}" y="{ly - 4}">port (supplies / hosts)</text>')
    lx += 170
    o.append(f'<rect class="sx-pc" x="{lx}" y="{ly - 14}" width="10" height="10"/>')
    o.append(f'<text class="sx-lg" x="{lx + 16}" y="{ly - 4}">~port (conjugate: receives)</text>')
    o.append("</svg>")
    return "\n".join(o) + "\n", routed


SVG_STYLE = """<style>
.sx text{font-family:"Helvetica Neue",Helvetica,Arial,"Liberation Sans",sans-serif}
.sx .sx-bg{fill:var(--sx-bg,#FBFAF5)}
.sx .sx-frame{fill:none;stroke:var(--sx-ink2,#8A8475);stroke-width:1.2}
.sx .sx-tab{font-size:12px;font-weight:700;fill:var(--sx-ink,#1C1B17);
  font-family:ui-monospace,"DejaVu Sans Mono",Menlo,Consolas,monospace}
.sx .sx-purpose{font-size:12px;fill:var(--sx-mut,#57534A)}
.sx .sx-box{fill:var(--sx-panel,#FFFFFF);stroke:var(--sx-ink,#1C1B17);stroke-width:1.4}
.sx .sx-rule{stroke:var(--sx-line,#C9C3B1);stroke-width:1}
.sx .sx-name{font-size:13.5px;font-weight:700;fill:var(--sx-ink,#1C1B17)}
.sx .sx-type{font-size:10.5px;fill:var(--sx-mut,#57534A);
  font-family:ui-monospace,"DejaVu Sans Mono",Menlo,Consolas,monospace}
.sx .sx-chip{fill:none;stroke-width:1}
.sx .sx-chipt{font-size:9px;font-weight:700;letter-spacing:.06em;text-anchor:middle}
.sx .sx-ok{stroke:var(--sx-ok,#2D5F3E);fill:var(--sx-ok,#2D5F3E)}
.sx .sx-warn{stroke:var(--sx-warn,#8A6A12);fill:var(--sx-warn,#8A6A12)}
.sx .sx-acc{stroke:var(--sx-acc,#29527A);fill:var(--sx-acc,#29527A)}
.sx .sx-bad{stroke:var(--sx-bad,#A63A22);fill:var(--sx-bad,#A63A22)}
.sx .sx-mut{stroke:var(--sx-mut,#57534A);fill:var(--sx-mut,#57534A)}
.sx rect.sx-chip{fill:none}
.sx .sx-pl{font-size:10px;fill:var(--sx-ink,#1C1B17);
  font-family:ui-monospace,"DejaVu Sans Mono",Menlo,Consolas,monospace}
.sx .sx-end{text-anchor:end}
.sx .sx-p{fill:var(--sx-ink,#1C1B17);stroke:var(--sx-ink,#1C1B17);stroke-width:1.2}
.sx .sx-pc{fill:var(--sx-panel,#FFFFFF);stroke:var(--sx-ink,#1C1B17);stroke-width:1.4}
.sx .sx-w{fill:none;stroke-width:2.2;stroke-linejoin:round}
.sx .sx-w.sx-c-power{stroke:var(--sx-power,#B5452B);stroke-width:3}
.sx .sx-w.sx-c-data{stroke:var(--sx-data,#2F6690)}
.sx .sx-w.sx-c-rf{stroke:var(--sx-rf,#7A4E9C);stroke-dasharray:9 5}
.sx .sx-w.sx-c-mechanical{stroke:var(--sx-mech,#6E6A5C);stroke-dasharray:2 4;stroke-width:2.6}
.sx .sx-w.sx-c-optical{stroke:var(--sx-opt,#2D7D5A);stroke-dasharray:12 4 2 4}
.sx .sx-w.sx-s-planned{stroke-opacity:.5}
.sx .sx-pill rect{stroke-width:1.4}
.sx .sx-pill text{font-size:10px;font-weight:700;text-anchor:middle}
.sx .sx-pill.sx-c-power rect{stroke:var(--sx-power,#B5452B)}
.sx .sx-pill.sx-c-data rect{stroke:var(--sx-data,#2F6690)}
.sx .sx-pill.sx-c-rf rect{stroke:var(--sx-rf,#7A4E9C)}
.sx .sx-pill.sx-c-mechanical rect{stroke:var(--sx-mech,#6E6A5C)}
.sx .sx-pill.sx-c-optical rect{stroke:var(--sx-opt,#2D7D5A)}
.sx .sx-pill.sx-s-verified.sx-c-power rect{fill:var(--sx-power,#B5452B)}
.sx .sx-pill.sx-s-verified.sx-c-data rect{fill:var(--sx-data,#2F6690)}
.sx .sx-pill.sx-s-verified.sx-c-rf rect{fill:var(--sx-rf,#7A4E9C)}
.sx .sx-pill.sx-s-verified.sx-c-mechanical rect{fill:var(--sx-mech,#6E6A5C)}
.sx .sx-pill.sx-s-verified.sx-c-optical rect{fill:var(--sx-opt,#2D7D5A)}
.sx .sx-pill.sx-s-verified text{fill:var(--sx-panel,#FFFFFF)}
.sx .sx-pill.sx-s-wired rect{fill:var(--sx-panel,#FFFFFF);stroke-width:2.6}
.sx .sx-pill.sx-s-wired text{fill:var(--sx-ink,#1C1B17)}
.sx .sx-pill.sx-s-planned rect{fill:var(--sx-panel,#FFFFFF);stroke-dasharray:3 2}
.sx .sx-pill.sx-s-planned text{fill:var(--sx-mut,#57534A)}
.sx .sx-h{font-size:10.5px;font-weight:700;letter-spacing:.12em;fill:var(--sx-mut,#57534A)}
.sx .sx-wl{font-size:11.5px;fill:var(--sx-ink,#1C1B17)}
.sx .sx-wn{font-weight:700}
.sx .sx-wm{font-family:ui-monospace,"DejaVu Sans Mono",Menlo,Consolas,monospace;font-size:10.5px}
.sx .sx-wt{fill:var(--sx-mut,#57534A);font-size:10.5px}
.sx .sx-ws{font-size:9.5px;font-weight:700;letter-spacing:.06em}
.sx .sx-ws.sx-s-verified{fill:var(--sx-ok,#2D5F3E)}
.sx .sx-ws.sx-s-wired{fill:var(--sx-warn,#8A6A12)}
.sx .sx-ws.sx-s-planned{fill:var(--sx-mut,#57534A)}
.sx .sx-wd{font-size:11px;fill:var(--sx-mut,#57534A)}
.sx .sx-lg{font-size:10.5px;fill:var(--sx-mut,#57534A)}
</style>"""


def render_all(m: Model) -> dict:
    """-> {view id: (svg text, routed edges)}; deterministic for a given model + contract."""
    return {v[0]: render_view(m, v) for v in VIEWS}


# =============================================================================
# 4. HTML block for the MBSE sheet
# =============================================================================
SECTION_CSS = """
:root{--sx-bg:#FBFAF5;--sx-panel:#FFFFFF;--sx-ink:#1C1B17;--sx-ink2:#8A8475;--sx-mut:#57534A;
  --sx-line:#C9C3B1;--sx-ok:#2D5F3E;--sx-warn:#8A6A12;--sx-acc:#29527A;--sx-bad:#A63A22;
  --sx-power:#B5452B;--sx-data:#2F6690;--sx-rf:#7A4E9C;--sx-mech:#6E6A5C;--sx-opt:#2D7D5A}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--sx-bg:#1D201B;--sx-panel:#232620;
  --sx-ink:#D6D3C7;--sx-ink2:#6E6C62;--sx-mut:#9A978A;--sx-line:#3A3D36;--sx-ok:#7FA671;
  --sx-warn:#C9A24B;--sx-acc:#7A9EC2;--sx-bad:#D2785A;--sx-power:#E0876B;--sx-data:#7FB2D9;
  --sx-rf:#B794D6;--sx-mech:#A3A193;--sx-opt:#6CC59A}}
:root[data-theme="dark"]{--sx-bg:#1D201B;--sx-panel:#232620;--sx-ink:#D6D3C7;--sx-ink2:#6E6C62;
  --sx-mut:#9A978A;--sx-line:#3A3D36;--sx-ok:#7FA671;--sx-warn:#C9A24B;--sx-acc:#7A9EC2;
  --sx-bad:#D2785A;--sx-power:#E0876B;--sx-data:#7FB2D9;--sx-rf:#B794D6;--sx-mech:#A3A193;--sx-opt:#6CC59A}
.sxwrap{overflow-x:auto;border:1px solid var(--line);border-radius:3px;background:var(--sx-bg);
  margin:12px 0 6px;-webkit-overflow-scrolling:touch}
.sxwrap svg{display:block;width:100%;min-width:820px;height:auto}
h3.sx3{font-size:17px;margin:34px 0 4px;font-weight:600}
.sxcap{font-size:13px;color:var(--muted);margin:2px 0 14px;max-width:76ch;
  font-family:"Helvetica Neue",Helvetica,Arial,sans-serif}
details.sxd{margin:10px 0 18px}
details.sxd summary{cursor:pointer;font-family:"Helvetica Neue",Helvetica,Arial,sans-serif;
  font-size:13px;color:var(--accent);padding:4px 0}
pre.sxsrc{background:var(--panel);border:1px solid var(--line);border-radius:3px;padding:14px;
  overflow-x:auto;font-size:11.5px;line-height:1.5;max-height:520px;
  font-family:ui-monospace,"Cascadia Mono",Consolas,Menlo,"DejaVu Sans Mono",monospace}
pre.sxsrc .k{color:var(--accent);font-weight:600} pre.sxsrc .c{color:var(--muted)}
pre.sxsrc .s{color:var(--ok)}
.sxkeys{display:grid;grid-template-columns:auto 1fr;gap:6px 16px;font-size:13px;
  color:var(--muted);max-width:80ch;margin:12px 0 16px}
.sxkeys dt{font-weight:700;color:var(--ink);white-space:nowrap;
  font-family:"Helvetica Neue",Helvetica,Arial,sans-serif;font-size:12px}
.sxkeys dd{margin:0}
"""

KW_RE = re.compile(r"\b(package|part|port|item|attribute|enum|metadata|interface|def|abstract|"
                   r"connect|to|end|in|out|inout|doc|private|import)\b")


def highlight(src: str) -> str:
    out = []
    for line in src.splitlines():
        if line.strip().startswith("//"):
            out.append(f'<span class="c">{esc(line)}</span>')
            continue
        parts = re.split(r'("(?:[^"\\]|\\.)*")', line)
        seg = []
        for i, p in enumerate(parts):
            if i % 2:
                seg.append(f'<span class="s">{esc(p)}</span>')
            else:
                seg.append(KW_RE.sub(lambda mm: f'<span class="k">{mm.group(0)}</span>', esc(p)))
        out.append("".join(seg))
    return "\n".join(out)


def stats(m: Model, rendered: dict) -> dict:
    links = [iu for s in m.systems() for iu in s.interfaces]
    parts = [p for s in m.systems() for p in s.parts.values() if "BOM" in p.meta]
    by = {k: sum(1 for iu in links if iu.attrs.get("linkState") == k) for k in LINK_STATES}
    open_items = [(s.name, iu) for s in m.systems() for iu in s.interfaces
                  if re.search(r"\bTBD\b|UNRESOLVED|check at", f"{iu.attrs.get('connector')} "
                                                            f"{iu.attrs.get('detail')}")]
    return {"links": len(links), "parts": len(parts), "by_state": by,
            "rows_drawn": len(m.coverage["drawn"]), "rows_excluded": len(m.coverage["excluded"]),
            "open": open_items, "views": len(rendered)}


def parts_table(m: Model, sysdef: Def) -> str:
    rows = []
    for p in sysdef.parts.values():
        st = part_status(m, p)
        row = m.bom_row(p.meta["BOM"])["name"] if "BOM" in p.meta else (
            "owned / external" if "External" in p.meta else "subsystem")
        typ = m.d(p.type)
        doc = p.doc or typ.doc
        rows.append(f"<tr><td class='id'>{esc(p.name)}</td><td class='mono'>{esc(p.type)}"
                    f"{' [' + str(p.mult) + ']' if p.mult else ''}</td>"
                    f"<td><span class='pill sx-st-{STATUS_CLASS.get(st, 'mut')}'>{esc(st)}</span>"
                    f"<div class='cap'>{esc(row)}</div></td><td class='cap'>{esc(doc)}</td></tr>")
    return ("<div class='scroller'><table class='grid'><thead><tr><th>Part</th><th>Type (part def)</th>"
            "<th>Build-sheet status</th><th>What it is</th></tr></thead><tbody>"
            + "".join(rows) + "</tbody></table></div>")


def section_html(m: Model, rendered: dict, vno: str = "View 8") -> str:
    s = stats(m, rendered)
    out = [f'<h2 id="physical"><span class="vno">{esc(vno)}</span>Physical architecture '
           f'(SysML&nbsp;v2)</h2>',
           '<p class="lead">Every board, cable, radio path and mount actually used, written as a '
           'SysML&nbsp;v2 textual model in <code class="mono">docs/sysml/</code> and drawn from it. '
           'Unlike views 1&ndash;7 this model is <i>hand-written text</i> &mdash; but each part&rsquo;s '
           'status is read live from the build sheet in the contract, every wire&rsquo;s plug types '
           'and direction are type-checked, and every build-sheet row must be drawn or explicitly '
           'excluded, so the two cannot quietly disagree.</p>',
           '<div class="stats">',
           f'<div class="stat"><div class="n">{s["parts"]}</div><div class="l">parts on the build sheet, '
           f'drawn</div></div>',
           f'<div class="stat"><div class="n">{s["links"]}</div><div class="l">typed connections<br>'
           f'{s["by_state"]["verified"]} verified · {s["by_state"]["wired"]} wired · '
           f'{s["by_state"]["planned"]} planned</div></div>',
           f'<div class="stat"><div class="n">{s["rows_drawn"]}/{s["rows_drawn"] + s["rows_excluded"]}'
           f'</div><div class="l">build-sheet rows drawn<br>({s["rows_excluded"]} excluded with a '
           f'reason)</div></div>',
           f'<div class="stat alert"><div class="n">{len(s["open"])}</div><div class="l">wiring '
           f'decisions still open<br>(TBD in the model)</div></div>',
           '</div>',
           '<dl class="sxkeys">'
           '<dt>Box</dt><dd>a <b>part</b>: <i>name : Type</i>. The chip is its live build-sheet status.</dd>'
           '<dt>Square</dt><dd>a <b>port</b> (plug, pads, socket). Filled = the side that supplies '
           'power or hosts the bus; hollow (<code class="mono">~</code>, &ldquo;conjugate&rdquo;) = '
           'the side that receives. A UART host wired to a conjugate UART <i>is</i> TX/RX crossed.</dd>'
           '<dt>Line</dt><dd>an <b>interface</b> (wire, cable, radio path, mount), coloured by kind. '
           'Faded = not yet made.</dd>'
           '<dt>Number</dt><dd>its row in the wire list under the diagram. Filled = verified by a '
           'logged build step (named in brackets); thick outline = made but untested; dashed = '
           'planned.</dd></dl>']

    if s["open"]:
        items = "".join(f"<li><b>{esc(sn)}.{esc(iu.name)}</b> &mdash; {esc(iu.attrs.get('detail'))}</li>"
                        for sn, iu in s["open"])
        out.append(f'<div class="banner"><b>Open wiring decisions the model surfaced.</b>'
                   f'<ul style="margin:8px 0 0 18px;padding:0">{items}</ul></div>')

    groups = [
        ("Interceptor", "InterceptorDrone",
         ["interceptor-power", "interceptor-data", "interceptor-mech"]),
        ("Target drone", "TargetDrone", ["target-power", "target-data", "target-mech"]),
        ("Tier-1 seeker rig (tripod)", "TripodSeekerRig", ["seeker-rig"]),
        ("Ground segment", "Ground", ["ground"]),
        ("Engagement context", "EngagementContext", ["engagement"]),
    ]
    vmeta = {v[0]: v for v in VIEWS}
    for gname, sysname, vids in groups:
        sysdef = m.d(sysname)
        out.append(f'<h3 class="sx3">{esc(gname)}</h3>')
        out.append(f'<p class="sxcap">{esc(sysdef.doc)}</p>')
        for vid in vids:
            svg = rendered[vid][0]
            out.append(f'<p class="sxcap"><b>{esc(vmeta[vid][3])}.</b> {esc(vmeta[vid][4])} '
                       f'<span class="mono">docs/sysml/views/{vid}.svg</span></p>')
            out.append(f'<div class="sxwrap">{svg}</div>')
        if any(p.meta.get("BOM") for p in sysdef.parts.values()):
            out.append(f'<details class="sxd"><summary>Parts list &mdash; {esc(gname)} '
                       f'({len(sysdef.parts)} parts)</summary>{parts_table(m, sysdef)}</details>')

    excl = "".join(f"<tr><td>{esc(tab)}</td><td>{esc(name)}</td><td class='cap'>{esc(r)}</td></tr>"
                   for (tab, name), r in sorted(m.coverage["excluded"].items()))
    out.append('<details class="sxd"><summary>Build-sheet rows deliberately not drawn '
               f'({s["rows_excluded"]})</summary><div class="scroller"><table class="grid"><thead><tr>'
               '<th>Tab</th><th>Row</th><th>Why not drawn</th></tr></thead><tbody>'
               + excl + '</tbody></table></div></details>')

    out.append('<h3 class="sx3">The model itself (SysML v2 textual notation)</h3>'
               '<p class="sxcap">Edit these files, not the pictures. '
               '<code class="mono">python3 scripts/render_sysml.py</code> re-draws every view; '
               '<code class="mono">scripts/sysml/check_sysml_official.sh</code> validates the text '
               'against the OMG reference implementation. How-to: '
               '<code class="mono">docs/sysml/README.md</code>.</p>')
    for fn in MODEL_FILES:
        n = m.sources[fn].count("\n")
        out.append(f'<details class="sxd"><summary>{esc(fn)} &mdash; {n} lines</summary>'
                   f'<pre class="sxsrc">{highlight(m.sources[fn])}</pre></details>')
    return "\n".join(out)


def views_digest(rendered: dict) -> str:
    h = hashlib.sha256()
    for vid in sorted(rendered):
        h.update(vid.encode())
        h.update(rendered[vid][0].encode())
    return h.hexdigest()


# =============================================================================
# 5. PNG export (optional; needs a Chromium binary)
# =============================================================================
CHROME_CANDIDATES = [
    os.environ.get("CHROME_BIN", ""),
    "/opt/pw-browsers/chromium_headless_shell-1194/chrome-linux/headless_shell",
    "/opt/pw-browsers/chromium-1194/chrome-linux/chrome",
    "chromium", "chromium-browser", "google-chrome",
]


def find_chrome() -> str | None:
    for c in CHROME_CANDIDATES:
        if not c:
            continue
        p = c if os.path.isabs(c) else shutil.which(c)
        if p and os.path.exists(p):
            return p
    return None


def svg_size(svg: str) -> tuple[int, int]:
    mm = re.search(r'width="(\d+)" height="(\d+)"', svg)
    return int(mm.group(1)), int(mm.group(2))


def export_pngs(rendered: dict) -> dict:
    chrome = find_chrome()
    if not chrome:
        raise ModelError("--png needs Chromium (set CHROME_BIN); none found")
    out = {}
    with tempfile.TemporaryDirectory() as td:
        for vid, (svg, _) in rendered.items():
            w, h = svg_size(svg)
            page = Path(td) / f"{vid}.html"
            page.write_text(f"<!doctype html><meta charset='utf-8'><style>html,body{{margin:0;"
                            f"background:#FBFAF5}}</style>{svg}")
            png = VIEW_DIR / f"{vid}.png"
            subprocess.run([chrome, "--headless", "--no-sandbox", "--disable-gpu",
                            "--hide-scrollbars", f"--window-size={w},{h}",
                            "--force-device-scale-factor=2", f"--screenshot={png}",
                            page.as_uri()], check=True, capture_output=True, timeout=120)
            out[vid] = hashlib.sha256(png.read_bytes()).hexdigest()
    return out


# =============================================================================
# 6. CLI
# =============================================================================
def main() -> int:
    args = sys.argv[1:]
    check = "--check" in args
    try:
        m = load_model()
        rendered = render_all(m)
    except ModelError as ex:
        print(f"FAIL: {ex}", file=sys.stderr)
        return 1
    s = stats(m, rendered)
    summary = (f"{len(rendered)} views, {s['parts']} parts, {s['links']} connections "
               f"({s['by_state']['verified']} verified / {s['by_state']['wired']} wired / "
               f"{s['by_state']['planned']} planned), build-sheet rows {s['rows_drawn']} drawn + "
               f"{s['rows_excluded']} excluded, {len(s['open'])} open wiring decisions")

    manifest = json.loads(MANIFEST.read_text()) if MANIFEST.exists() else {}
    if check:
        bad = []
        for vid, (svg, _) in rendered.items():
            f = VIEW_DIR / f"{vid}.svg"
            if not f.exists() or f.read_text() != svg:
                bad.append(f"{f.relative_to(ROOT)} is stale or missing")
            want = hashlib.sha256(svg.encode()).hexdigest()
            entry = manifest.get(vid, {})
            if entry.get("svg_sha256") != want:
                bad.append(f"PNG for {vid} was rendered from an older SVG -- run "
                           f"scripts/render_sysml.py --png")
            png = VIEW_DIR / f"{vid}.png"
            if not png.exists() or hashlib.sha256(png.read_bytes()).hexdigest() != entry.get("png_sha256"):
                bad.append(f"{png.relative_to(ROOT)} missing or not the one in manifest.json")
        extra = sorted({p.stem for p in VIEW_DIR.glob("*.svg")} - set(rendered))
        bad += [f"docs/sysml/views/{x}.svg is not a current view -- delete it" for x in extra]
        if bad:
            print("FAIL: SysML views out of date:\n  - " + "\n  - ".join(bad) +
                  "\n  run: python3 scripts/render_sysml.py --png", file=sys.stderr)
            return 1
        print(f"OK: SysML model valid and views current -- {summary}")
        return 0

    VIEW_DIR.mkdir(parents=True, exist_ok=True)
    for vid, (svg, _) in rendered.items():
        (VIEW_DIR / f"{vid}.svg").write_text(svg)
    for stale in VIEW_DIR.glob("*.svg"):
        if stale.stem not in rendered:
            stale.unlink()
    if "--png" in args:
        pngs = export_pngs(rendered)
        manifest = {vid: {"svg_sha256": hashlib.sha256(rendered[vid][0].encode()).hexdigest(),
                          "png_sha256": pngs[vid]} for vid in rendered}
        MANIFEST.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"OK: rendered docs/sysml/views/ -- {summary}")
    if "--png" not in args:
        print("    (PNGs not refreshed: run with --png before committing, or --check fails)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
