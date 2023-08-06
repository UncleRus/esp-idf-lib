from __future__ import annotations

import os
import yaml
import pydantic
import typing as t
import pathlib as p
from enum import Enum

from .errors import *
from . import const


__all__ = [
    'ThreadSafety',
    'Licenses',
    'Group',
    'Person',
    'Copyright',
    'Component',
    'Metadata',
]


class ThreadSafety(str, Enum):
    NO = 'no'
    YES = 'yes'
    NA = 'n/a'

    @staticmethod
    def from_value(raw: bool | str) -> ThreadSafety:
        if isinstance(raw, bool):
            return ThreadSafety.YES if raw else ThreadSafety.NO
        if raw == 'true':
            return ThreadSafety.YES
        if raw == 'false':
            return ThreadSafety.NO
        return ThreadSafety(raw)


class Licenses(str, Enum):
    ISC = 'ISC'
    MIT = 'MIT'
    BSD3 = 'BSD-3-Clause'

    @staticmethod
    def from_value(raw: str) -> Licenses:
        if raw == 'BSD-3':
            return Licenses.BSD3
        return Licenses(raw)


class Group(pydantic.BaseModel):
    name: str
    description: str


class Person(pydantic.BaseModel):
    name: str
    full_name: str | None = None
    gh_id: str | None = None
    email: str | None = None
    url: str | None = None

    def __init__(self, **kwargs):
        if not kwargs.get('full_name', None):
            kwargs['full_name'] = kwargs['name']
        super().__init__(**kwargs)


class Copyright(pydantic.BaseModel):
    name: Person
    year: int


class Component(pydantic.BaseModel):
    name: str
    description: str
    version: str
    groups: t.List[Group] = []
    code_owners: t.List[Person] = []
    depends: t.List[str] = []
    thread_safe: ThreadSafety = ThreadSafety.NA
    targets: t.List[str] = []
    license: Licenses = Licenses.ISC
    copyrights: t.List[Copyright] = []

    @staticmethod
    def load(m: Metadata, dirname: os.PathLike) -> Component:
        path: p.Path = m.repo_path / const.COMPONENTS_DIR / dirname
        meta_fn: p.Path = path / const.METADATA_FILENAME
        if not meta_fn.is_file:
            raise MetadataNotFoundError(path)

        raw = yaml.safe_load(meta_fn.open('r', encoding=const.ENCODING))

        if not isinstance(raw, dict):
            raise StructureError(str(dirname))

        if set(Component.__fields__.keys()) != set(raw.keys()):
            raise InvalidFieldsError(
                str(dirname), set(Component.__fields__.keys()) - set(raw.keys()),
                set(raw.keys()) - set(Component.__fields__.keys()))

        ctx = str(dirname)
        if ctx != raw['name'].strip():
            raise InvalidNameError(ctx, raw['name'])

        ts = str(raw['thread_safe'])
        try:
            ts = ThreadSafety.from_value(ts.lower())
        except:
            raise InvalidThreadSafetyError(ctx, ts)

        lc = str(raw['license'])
        try:
            lc = Licenses.from_value(lc.upper())
        except:
            raise InvalidLicenseError(ctx, lc)

        res = Component(
            name=ctx,
            description=raw['description'].strip().strip('.').replace('\n', ' '),
            version=raw['version'].strip(),
            groups=m.get_groups(ctx, raw),
            code_owners=m.get_persons(ctx, raw['code_owners']),
            depends=[lib for lib in raw['depends']],
            thread_safe=ts,
            targets=m.get_targets(ctx, raw['targets']),
            license=lc,
            copyrights=m.get_copyrights(ctx, raw['copyrights'])
        )
        return res

    def save(self, m: Metadata, dirname: os.PathLike) -> None:
        meta_fn: p.Path = m.repo_path / const.COMPONENTS_DIR / dirname / const.METADATA_FILENAME
        yaml.safe_dump(self.dict(), meta_fn.open('w', encoding=const.ENCODING))


class Metadata:

    GROUPS_FILENAME: str = 'groups.yml'
    PERSONS_FILENAME: str = 'persons.yml'
    TARGETS_FILENAME: str = 'targets.yml'

    DICT_DIR: str = 'devtools'
    COMPONENTS_DIR: str = 'components'

    def __init__(self, repo_path: os.PathLike):
        self.repo_path: p.Path = p.Path(repo_path)

        self.groups = {}
        self.persons = {}
        self.targets = []

        self.idx_groups = []

        self.load_dictionaries()

    def load_dictionaries(self) -> None:
        path = self.repo_path / p.Path(self.DICT_DIR)
        groups_fn = path / self.GROUPS_FILENAME
        persons_fn = path / self.PERSONS_FILENAME
        targets_fn = path / self.TARGETS_FILENAME

        self.groups = {r['name']: Group(**r) for r in yaml.safe_load(groups_fn.open('r', encoding=const.ENCODING))}
        self.persons = {r['name']: Person(**r) for r in yaml.safe_load(persons_fn.open('r', encoding=const.ENCODING))}
        self.targets = [r['name'] for r in yaml.safe_load(targets_fn.open('r', encoding=const.ENCODING))]

        self.idx_groups = sorted(self.groups.values(), key=lambda x: x.description)

    def _get_group(self, ctx: str, name: str) -> Group:
        if name not in self.groups:
            raise InvalidGroupError(ctx, name);
        return self.groups[name]

    def _get_person(self, ctx: str, name: str) -> Person:
        if name not in self.persons:
            raise InvalidPersonError(ctx, name)
        return self.persons[name]

    def _get_target(self, ctx: str, name: str) -> str:
        if name not in self.targets:
            raise InvalidTargetError(ctx, name)
        return name

    def get_groups(self, ctx: str, meta: dict) -> t.List[Group]:
        return [self._get_group(ctx, g) for g in meta['groups']]

    def get_persons(self, ctx: str, names: str | t.List[str]) -> t.List[Person]:
        if isinstance(names, str):
            names = [names]
        return [self._get_person(ctx, name) for name in names]

    def get_targets(self, ctx: str, names: str | t.List[str]) -> t.List[str]:
        if isinstance(names, str):
            names = [names]
        return [self._get_target(ctx, name) for name in names]

    def get_copyrights(self, ctx: str, meta: list) -> t.List[Copyright]:
        return [Copyright(name=self._get_person(ctx, m['name']), year=m['year']) for m in meta]

    def iter_component_dirs(self) -> t.Generator[p.Path]:
        components_dir = self.repo_path / const.COMPONENTS_DIR
        return components_dir.iterdir()
