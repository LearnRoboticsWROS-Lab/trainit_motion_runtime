"""vNext M2.6 AFTER (ADR-0022), M1, the reference half.

The byte copy of the reference normaliser (test/data/m2_6_contract/normalize_eef.py) answers
every row of eef_refusal_mutations.txt exactly as the table says, and dumps the conformance
fixture exactly as eef_conformance.expected.txt. test_m2_6_eef_contract.cpp checks TMR's C++
parser against the SAME table, so the two agree on every row: the table is anchored to the
reference by execution, at every test run, not by the hand that wrote it.

No path outside this repository is read.
"""
import hashlib
import importlib.util
import os
import pathlib
import sys
import traceback

import yaml

DATA = pathlib.Path(os.environ.get(
    'TRAINIT_M26_CONTRACT_DIR',
    pathlib.Path(__file__).resolve().parent / 'data' / 'm2_6_contract'))

PINNED = {
    'eef_conformance.yaml': '873a1d582c4e3947a62026d153bd49ccb02d7b4d38e5cb3a5b4b3a8f5a27a04f',
    'eef_conformance.expected.txt': '1f6bd1d2ac2fb18865ee67a815877483537d0a325fbc9a5400cf2977710d242c',
    'eef_contract_tokens.txt': '55048ea44d466979f57c28a634295a060a79b837fe614a30c49f60572a2bd5de',
    'normalize_eef.py': '9fba00c3981762974d9e916464fd670c8f83964392649c3e0854a94149290a0b',
}


def _reference():
    """Imports the byte copy without writing a bytecode cache beside it (the data stay as copied)."""
    spec = importlib.util.spec_from_file_location('m26_normalize_eef', DATA / 'normalize_eef.py')
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def _rows():
    rows = []
    for line in (DATA / 'eef_refusal_mutations.txt').read_text(encoding='utf-8').splitlines():
        if not line or line.startswith('#'):
            continue
        rid, muts, expected = (f.strip() for f in line.split(' | '))
        rows.append((rid, [] if muts == '(none)' else [m.strip() for m in muts.split(' ; ')], expected))
    return rows


def _apply(params, mutation):
    if mutation.startswith('erase_prefix '):
        prefix = mutation[len('erase_prefix '):]
        for k in [k for k in params if k.startswith(prefix)]:
            del params[k]
    elif mutation.startswith('erase '):
        del params[mutation[len('erase '):]]
    elif mutation.startswith('set '):
        key, value = mutation[len('set '):].split(' = ', 1)
        params[key] = yaml.safe_load(value)
    else:
        raise AssertionError(f'bad mutation {mutation!r}')


def test_the_byte_copies_are_the_pinned_ones():
    for name, sha in PINNED.items():
        assert hashlib.sha256((DATA / name).read_bytes()).hexdigest() == sha, name


def test_the_reference_dumps_the_fixture_as_expected():
    ref = _reference()
    params = ref.load_params(str(DATA / 'eef_conformance.yaml'))
    assert ref.dump(ref.parse(params)) == (DATA / 'eef_conformance.expected.txt').read_text(encoding='utf-8')


def test_the_reference_answers_every_mutation_row_as_the_table_says():
    ref = _reference()
    rows = _rows()
    assert len(rows) >= 100
    wrong = []
    for rid, muts, expected in rows:
        params = ref.load_params(str(DATA / 'eef_conformance.yaml'))
        for m in muts:
            _apply(params, m)
        try:
            contract = ref.parse(params)
            got = 'LEGACY' if not contract else 'ACCEPT ' + hashlib.sha256(
                ref.dump(contract).encode('utf-8')).hexdigest()
        except ref.Refused as e:
            got = f'REFUSED {e}'
        if got != expected:
            wrong.append(f'{rid}: table says {expected!r}, the reference answers {got!r}')
    assert not wrong, '\n'.join(wrong)


def test_every_refusal_site_of_the_reference_is_in_the_table():
    """Every `raise Refused(...)` of the reference is reached by at least one row, by its SITE.

    A row's refusal is attributed to the source line that raised it (the innermost traceback
    frame, in the byte copy), never to its text: several sites raise the same text ('required',
    'a positive integer', ...), so deleting the only row that reaches a site fails here even when
    another site still raises those words.
    """
    ref = _reference()
    path = (DATA / 'normalize_eef.py').resolve()
    source = path.read_text(encoding='utf-8').splitlines()
    sites = {n for n, line in enumerate(source, start=1) if 'raise Refused(' in line}
    reached = {}
    for rid, muts, _ in _rows():
        params = ref.load_params(str(DATA / 'eef_conformance.yaml'))
        for m in muts:
            _apply(params, m)
        try:
            ref.parse(params)
        except ref.Refused as e:
            frame = traceback.extract_tb(e.__traceback__)[-1]
            assert pathlib.Path(frame.filename).resolve() == path, f'{rid}: raised in {frame.filename}'
            reached.setdefault(frame.lineno, []).append(rid)
    assert set(reached) <= sites, f'a refusal raised off a raise site: lines {sorted(set(reached) - sites)}'
    missing = sorted(sites - set(reached))
    assert not missing, 'no row reaches the refusal site(s): ' + '; '.join(
        f'line {n}: {source[n - 1].strip()}' for n in missing)
    assert len(sites) == 45, f'the reference now raises Refused at {len(sites)} sites: extend the table'
