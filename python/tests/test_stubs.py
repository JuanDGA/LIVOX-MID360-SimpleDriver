# Copyright 2026 Juan David Guevara Arévalo
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import ast
from pathlib import Path

import livox_mid360 as mid

# TimestampType.None is a real attribute. A .pyi cannot declare it: None is a keyword.
_STUB_EXCEPTIONS = {("TimestampType", "None")}


def _class_attrs(node: ast.ClassDef) -> set[str]:
    names: set[str] = set()
    for item in node.body:
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)) and not item.name.startswith("_"):
            names.add(item.name)
        elif isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
            if not item.target.id.startswith("_"):
                names.add(item.target.id)
    return names


def _stub_api(path: Path) -> tuple[set[str], dict[str, set[str]]]:
    tree = ast.parse(path.read_text())
    names: set[str] = set()
    classes: dict[str, set[str]] = {}
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            names.add(node.name)
            classes[node.name] = _class_attrs(node)
        elif isinstance(node, ast.FunctionDef):
            names.add(node.name)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if not node.target.id.startswith("_"):
                names.add(node.target.id)
    return names, classes


def test_stub_covers_public_api():
    stub = Path(__file__).resolve().parents[1] / "livox_mid360.pyi"
    stub_names, stub_classes = _stub_api(stub)

    runtime_names = {name for name in dir(mid) if not name.startswith("_") and name != "livox_mid360"}
    assert runtime_names == stub_names

    for class_name, stub_attrs in stub_classes.items():
        runtime_attrs = {
            name for name in type.__getattribute__(getattr(mid, class_name), "__dict__") if not name.startswith("_")
        }
        for parent, attr in _STUB_EXCEPTIONS:
            if parent == class_name:
                runtime_attrs.discard(attr)
        assert runtime_attrs == stub_attrs, class_name
