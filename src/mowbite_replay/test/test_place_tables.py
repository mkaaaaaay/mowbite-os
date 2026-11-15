"""Every journaled call that can carry a place stands in the Anonymize table of its core, so that a copy of a journal
without its place (mowbite_replay anonymize) does not miss it. A call carries a place when a parameter is named like a
coordinate, or a parameter, its result or the config has a struct with such a field, also nested, in a vector,
optional or variant. C++ keeps no names of parameters and fields, so this reads the headers.

A json parameter or result cannot be judged by its type: such a call is either in the table or in JSON_WITHOUT_PLACE
below, with the reason it carries no place.

Run: python3 test_place_tables.py
"""
import glob
import os
import re
import unittest

SRC = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
GEO = {"lat", "latitude", "lon", "lng", "longitude"}
JSON_WITHOUT_PLACE = {
    # schedule.leftovers: enabled and min_m2 only.
    ("schedule", "SetLeftovers"),
}


def strip(text):
    """The text without comments and string literals."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r'"(\\.|[^"\\\n])*"', '""', text)


def matching(text, start, open_char, close_char):
    """The index after the bracket that closes the one at start."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == open_char:
            depth += 1
        elif text[i] == close_char:
            depth -= 1
            if depth == 0:
                return i + 1
    raise ValueError("unbalanced " + open_char)


def split_top(text, sep=","):
    """text split at sep outside of <>, (), {} and []."""
    parts, depth, start = [], 0, 0
    for i, c in enumerate(text):
        if c in "<({[":
            depth += 1
        elif c in ">)}]":
            depth -= 1
        elif c == sep and depth == 0:
            parts.append(text[start:i])
            start = i + 1
    parts.append(text[start:])
    return [p.strip() for p in parts if p.strip()]


def type_and_name(declaration):
    """(type, name) of a parameter or field, without its default."""
    declaration = split_top(declaration, "=")[0] if "=" in declaration else declaration
    declaration = re.sub(r"\{.*\}\s*$", "", declaration).strip()
    match = re.match(r"(.*?)\b(\w+)\s*(\[[^\]]*\])?$", declaration, flags=re.S)
    if not match or not match.group(1).strip():
        return declaration, ""
    return match.group(1).strip(), match.group(2)


def headers():
    return sorted(glob.glob(os.path.join(SRC, "*", "include", "*", "*.h")))


def structs_and_aliases():
    """{struct name: [(field type, field name)]} and {alias: its text} over all headers."""
    structs, aliases = {}, {}
    for path in headers():
        with open(path) as f:
            text = strip(f.read())
        for match in re.finditer(r"\b(?:struct|class)\s+(\w+)\s*(?::[^{;]*)?\{", text):
            body = text[match.end() - 1:matching(text, match.end() - 1, "{", "}")][1:-1]
            fields = structs.setdefault(match.group(1), [])
            depth, statement = 0, ""
            for c in body:
                if c in "{(":
                    depth += 1
                elif c in "})":
                    depth -= 1
                if depth == 0 and c in ";}":
                    item = statement.strip()
                    statement = ""
                    if c == "}":
                        continue
                    item = re.sub(r"^(public|private|protected)\s*:", "", item).strip()
                    if not item or "(" in item.split("=")[0] or re.match(r"(using|typedef|friend|static|enum)\b",
                                                                          item):
                        continue
                    fields.append(type_and_name(item))
                else:
                    statement += c
        for match in re.finditer(r"\busing\s+(\w+)\s*=\s*([^;]*);", text):
            aliases[match.group(1)] = match.group(2)
    return structs, aliases


def geo_types(structs, aliases):
    found = {name for name, fields in structs.items() if any(field in GEO for _, field in fields)}
    while True:
        more = {name for name, fields in structs.items()
                if any(re.search(rf"\b{t}\b", ftype) for ftype, _ in fields for t in found)}
        more |= {name for name, text in aliases.items() if any(re.search(rf"\b{t}\b", text) for t in found)}
        if more <= found:
            return found
        found |= more


def journals():
    """Per core: name, the class, its methods, the config type, the table text and the package include dir."""
    cores = []
    for path in sorted(glob.glob(os.path.join(SRC, "*", "include", "*", "*_journal.h"))):
        with open(path) as f:
            text = strip(f.read())
        with open(path) as f:
            names = re.findall(r"kName\s*=\s*\"(\w+)\"", f.read())
        for traits, kname in zip(re.finditer(r"struct\s+mowbite_journal::CoreTraits<([\w:]+)>\s*\{", text), names):
            body = text[traits.end() - 1:matching(text, traits.end() - 1, "{", "}")]
            listing = re.search(r"MOWBITE_JOURNAL_METHODS\((.*?)\);", body, flags=re.S).group(1)
            methods = re.findall(r"&[\w:]+::(\w+)", listing)
            config = re.search(r"using\s+Config\s*=\s*([\w:]+)\s*;", body).group(1).split("::")[-1]
            table = ""
            anonymize = re.search(r"static\s+void\s+Anonymize\s*\(", body)
            if anonymize:
                start = body.index("{", anonymize.end())
                table = body[start:matching(body, start, "{", "}")]
            cores.append({"name": kname, "class": traits.group(1).split("::")[-1], "methods": methods,
                          "config": config, "table": table, "dir": os.path.dirname(path)})
    return cores


def declarations(core):
    """{method: (result type, [(param type, param name)])} from the class in the headers of its package."""
    for path in sorted(glob.glob(os.path.join(core["dir"], "*.h"))):
        with open(path) as f:
            text = strip(f.read())
        match = re.search(rf"\bclass\s+{core['class']}\b[^;{{]*\{{", text)
        if not match:
            continue
        body = text[match.end() - 1:matching(text, match.end() - 1, "{", "}")]
        found = {}
        for method in core["methods"]:
            decl = re.search(rf"([\w:<>,\s*&]*?)\b{method}\s*\(", body)
            assert decl, f"{core['name']}: no declaration of {method} in {path}"
            params_start = decl.end() - 1
            params = body[params_start + 1:matching(body, params_start, "(", ")") - 1]
            result = re.sub(r"\b(virtual|static|inline|explicit)\b", "", decl.group(1)).strip()
            found[method] = (result, [type_and_name(p) for p in split_top(params)])
        return found
    raise AssertionError(f"no class {core['class']} in {core['dir']}")


def mentions(text, types):
    return any(re.search(rf"\b{t}\b", text) for t in types)


class PlaceTables(unittest.TestCase):
    def setUp(self):
        structs, aliases = structs_and_aliases()
        self.geo = geo_types(structs, aliases)
        self.cores = journals()

    def test_the_reading_finds_the_known_places(self):
        self.assertGreaterEqual(len(self.cores), 6)
        self.assertTrue({"Facts", "WeatherRequest", "Output"} <= self.geo, self.geo)
        logic = next(c for c in self.cores if c["name"] == "logic")
        self.assertEqual(declarations(logic)["OnDatum"][1], [("double", "latitude"), ("double", "longitude")])

    def test_every_call_with_a_place_is_in_the_table(self):
        missing = []
        for core in self.cores:
            covered = set(re.findall(r"\b(?:Args|Result)<&[\w:]+::(\w+)>", core["table"]))
            if mentions(core["config"], self.geo) and not re.search(r"\bConfig\s*\(", core["table"]):
                missing.append(f"{core['name']}: config {core['config']}")
            for method, (result, params) in declarations(core).items():
                place = any(name in GEO for _, name in params) or mentions(result, self.geo) or any(
                    mentions(ptype, self.geo) for ptype, _ in params)
                if place and method not in covered:
                    missing.append(f"{core['name']}: {method}")
                uses_json = mentions(result, {"json"}) or any(mentions(ptype, {"json"}) for ptype, _ in params)
                if uses_json and method not in covered and (core["name"], method) not in JSON_WITHOUT_PLACE:
                    missing.append(f"{core['name']}: {method} takes or gives json: table or JSON_WITHOUT_PLACE")
        self.assertEqual(missing, [], "\n".join(missing))


if __name__ == "__main__":
    unittest.main()
