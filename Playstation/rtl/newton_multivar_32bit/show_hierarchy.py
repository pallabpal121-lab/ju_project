#!/usr/bin/env python3
"""
Universal RTL Hierarchy & Dependency Tree Analyzer
Works on any Verilog (.v) and SystemVerilog (.sv) codebase.
Supports:
  - Scanning entire directories or specific files
  - Top-to-bottom hierarchy tree with arrow and box-drawing formatting
  - Multiple tree styles: arrow (default), pointer, classic, ascii
  - Bottom-up reverse lookup ("Who instantiates module X?")
  - Safe comment stripping (avoids phantom instantiations in comments)
  - Colorized terminal output with auto-detection
"""

import os
import re
import sys
import argparse

STYLES = {
    'arrow': {
        'branch': '├──> ',
        'last':   '└──> ',
        'pipe':   '│    ',
        'space':  '     ',
        'bullet': '• '
    },
    'pointer': {
        'branch': '├──► ',
        'last':   '└──► ',
        'pipe':   '│    ',
        'space':  '     ',
        'bullet': '▸ '
    },
    'classic': {
        'branch': '├── ',
        'last':   '└── ',
        'pipe':   '│   ',
        'space':  '    ',
        'bullet': '• '
    },
    'ascii': {
        'branch': '|---> ',
        'last':   '\\---> ',
        'pipe':   '|     ',
        'space':  '      ',
        'bullet': '* '
    }
}

COLORS = {
    'reset':   '\033[0m',
    'bold':    '\033[1m',
    'header':  '\033[1;36m', # Cyan
    'root':    '\033[1;35m', # Magenta
    'module':  '\033[1;32m', # Green
    'file':    '\033[90m',   # Gray
    'inst':    '\033[33m',   # Yellow
    'conn':    '\033[36m',   # Cyan
    'warning': '\033[1;31m', # Red
}

def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'//.*$', '', text, flags=re.MULTILINE)
    return text

def collect_rtl_files(target_paths):
    files_to_scan = []
    for path in target_paths:
        if os.path.isfile(path):
            files_to_scan.append(path)
        elif os.path.isdir(path):
            for root, _, files in os.walk(path):
                for f in sorted(files):
                    if f.endswith(('.v', '.sv', '.vh', '.svh')):
                        files_to_scan.append(os.path.join(root, f))
        else:
            print(f"Warning: '{path}' is not a valid file or directory.")
    return sorted(list(set(files_to_scan)))

def parse_rtl_hierarchy(files_to_scan, target_top=None, find_parent=None, style_name='arrow', use_color=None):
    if not files_to_scan:
        print("No Verilog or SystemVerilog files found to analyze.")
        return

    if use_color is None:
        use_color = sys.stdout.isatty()

    c = COLORS if use_color else {k: '' for k in COLORS}

    selected_style = (style_name or 'arrow').lower()
    if selected_style not in STYLES:
        selected_style = 'arrow'

    st = dict(STYLES[selected_style])
    enc = getattr(sys.stdout, 'encoding', '') or 'utf-8'
    if selected_style != 'ascii':
        try:
            (st['branch'] + st['last'] + st['pipe'] + st['bullet']).encode(enc)
        except (UnicodeEncodeError, LookupError):
            if 'utf' in os.environ.get('LANG', '').lower() or 'utf' in os.environ.get('LC_ALL', '').lower():
                try:
                    import io
                    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
                except Exception:
                    st = dict(STYLES['ascii'])
            else:
                st = dict(STYLES['ascii'])

    module_defs = {}      # module_name -> filepath
    file_contents = {}    # filepath -> cleaned text
    instantiations = {}   # parent_module -> list of (child_module, instance_name)
    parents_of = {}       # child_module -> list of (parent_module, instance_name)

    mod_decl_re = re.compile(r'\bmodule\s+([a-zA-Z0-9_]+)\b', re.MULTILINE)

    for fpath in files_to_scan:
        try:
            with open(fpath, 'r', errors='ignore') as fp:
                cleaned = strip_comments(fp.read())
                file_contents[fpath] = cleaned
                for mod_name in mod_decl_re.findall(cleaned):
                    module_defs[mod_name] = fpath
                    instantiations[mod_name] = []
                    parents_of[mod_name] = []
        except Exception as e:
            print(f"Error reading {fpath}: {e}")

    for mod_name, fpath in module_defs.items():
        content = file_contents[fpath]
        block_pattern = rf'\bmodule\s+{mod_name}\b.*?\bendmodule\b'
        blocks = re.findall(block_pattern, content, re.DOTALL)

        for block in blocks:
            for potential_child in module_defs.keys():
                if potential_child == mod_name:
                    continue
                inst_re = rf'\b{potential_child}\b\s*(?:#\s*\([\s\S]*?\)\s*)?([a-zA-Z0-9_]+)\s*\('
                matches = re.findall(inst_re, block)
                for inst_name in matches:
                    if inst_name not in ('always', 'initial', 'assign', 'generate', 'function', 'task', 'begin', 'if', 'else'):
                        instantiations[mod_name].append((potential_child, inst_name))
                        parents_of[potential_child].append((mod_name, inst_name))

    # Reverse query
    if find_parent:
        target = find_parent.strip()
        print(f"\n{c['header']}======================================================={c['reset']}")
        print(f"{c['header']}       REVERSE LOOKUP: Who uses '{target}'?           {c['reset']}")
        print(f"{c['header']}======================================================={c['reset']}\n")
        if target not in module_defs:
            print(f"Module '{target}' was not found in the scanned files.")
            return

        parents = parents_of.get(target, [])
        if not parents:
            print(f"'{target}' is not instantiated by any module (It is a Top-Level module).\n")
        else:
            fname = os.path.basename(module_defs.get(target, ""))
            print(f"Module '{c['module']}{target}{c['reset']}' {c['file']}[{fname}]{c['reset']} is instantiated by:")
            for p_mod, inst in parents:
                pfname = os.path.basename(module_defs.get(p_mod, ""))
                print(f"  {c['conn']}{st['last']}{c['reset']}{c['module']}{p_mod}{c['reset']} {c['file']}[{pfname}]{c['reset']} {c['inst']}(instance: {inst}){c['reset']}")
            print()
        return

    # Forward query
    all_children = set()
    for p, children in instantiations.items():
        for ch, inst_name in children:
            all_children.add(ch)

    top_modules = [m for m in module_defs.keys() if m not in all_children]
    if target_top:
        if target_top in module_defs:
            top_modules = [target_top]
        else:
            print(f"Specified top module '{target_top}' not found. Available modules: {list(module_defs.keys())}")
            return

    print(f"\n{c['header']}======================================================={c['reset']}")
    print(f"{c['header']}           RTL MODULE HIERARCHY TREE                   {c['reset']}")
    print(f"{c['header']}======================================================={c['reset']}\n")

    def print_subtree(mod, prefix, is_last, inst_label="", visited=None):
        if visited is None:
            visited = set()

        fname = os.path.basename(module_defs.get(mod, ""))
        connector = st['last'] if is_last else st['branch']
        label = f"{c['conn']}{connector}{c['reset']}{c['module']}{mod}{c['reset']}  {c['file']}[{fname}]{c['reset']}"
        if inst_label:
            label += f" {c['inst']}(instance: {inst_label}){c['reset']}"

        print(prefix + label)

        if mod in visited:
            continuation = st['space'] if is_last else st['pipe']
            print(prefix + c['conn'] + continuation + st['last'] + c['warning'] + "[recursive / loop]" + c['reset'])
            return

        visited.add(mod)
        child_list = instantiations.get(mod, [])
        new_prefix = prefix + (st['space'] if is_last else st['pipe'])
        for idx, (child_mod, instance_name) in enumerate(child_list):
            last_child = (idx == len(child_list) - 1)
            print_subtree(child_mod, new_prefix, last_child, instance_name, visited.copy())

    for top in top_modules:
        if instantiations.get(top):
            fname = os.path.basename(module_defs.get(top, ""))
            print(f"{c['bold']}[TOP-LEVEL DESIGN ROOT]: {c['module']}{top}{c['reset']}")
            print(f"  {c['bold']}{c['module']}{top}{c['reset']}  {c['file']}[{fname}]{c['reset']}")
            child_list = instantiations.get(top, [])
            for idx, (child_mod, instance_name) in enumerate(child_list):
                last_child = (idx == len(child_list) - 1)
                print_subtree(child_mod, "  ", last_child, instance_name, set([top]))
            print()

    standalone = [m for m in top_modules if not instantiations.get(m)]
    if standalone:
        print(f"{c['bold']}[STANDALONE / UNCONNECTED MODULES]:{c['reset']}")
        for s in standalone:
            fname = os.path.basename(module_defs.get(s, ""))
            print(f"  {c['conn']}{st['bullet']}{c['reset']}{c['module']}{s}{c['reset']}  {c['file']}[{fname}]{c['reset']}")
        print()

def main():
    parser = argparse.ArgumentParser(
        description="Universal RTL Hierarchy Tree Analyzer for Verilog/SystemVerilog."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        default=["."],
        help="Directories or specific .v/.sv files to analyze (default: current directory)."
    )
    parser.add_argument(
        "--top",
        dest="top_module",
        help="Specify a custom top module to root the tree."
    )
    parser.add_argument(
        "--parent",
        dest="find_parent",
        help="Find which parent modules instantiate this child module."
    )
    parser.add_argument(
        "--style",
        dest="style",
        choices=['arrow', 'pointer', 'classic', 'ascii'],
        default='arrow',
        help="Formatting style for hierarchy tree: arrow (default), pointer, classic, ascii."
    )
    parser.add_argument(
        "--color",
        dest="color",
        action="store_true",
        default=None,
        help="Force colorized terminal output."
    )
    parser.add_argument(
        "--no-color",
        dest="color",
        action="store_false",
        help="Disable colorized terminal output."
    )

    args = parser.parse_args()
    files = collect_rtl_files(args.paths)
    parse_rtl_hierarchy(
        files,
        target_top=args.top_module,
        find_parent=args.find_parent,
        style_name=args.style,
        use_color=args.color
    )

if __name__ == "__main__":
    main()
