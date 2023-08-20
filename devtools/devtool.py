#!/usr/bin/env python3

import argparse
import devtool
import pathlib as p
import typing as t
import jinja2
import sys
import os
import subprocess
import pydantic


class CI(pydantic.BaseModel):

    meta: bool = False
    build: bool = False
    docs: bool = False
    readme: bool = False
    ci: bool = False


class Devtool:

    PROGRAM = 'devtool.py'
    COMMANDS = {
        'check': 'Check components metadata',
        'render': 'Generate documentation based on components metadata',
        'target': 'Find components by target',
        'depends': 'Show components that depend (directly or indirectly) on a specified component',
        'ci': 'Show list of ci jobs to be done',
    }
    README = 'README.md'

    def __init__(self):
        self.meta: devtool.Metadata | None = None
        self.parse_args()

    def parse_args(self):
        parser = argparse.ArgumentParser(
            prog=self.PROGRAM,
            description='esp-idf-lib developer tool',
            usage='usage: devtool.py [-h] {%s} [args]' % ','.join(self.COMMANDS.keys())
        )
        parser.add_argument('command', choices=[cmd for cmd in self.COMMANDS],
                            help='Subcommand to run')
        args = parser.parse_args(sys.argv[1:2])

        repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.meta = devtool.Metadata(repo_path)

        method = 'cmd_%s' % args.command
        if not hasattr(self, method) or not callable(getattr(self, method)):
            raise RuntimeError('Invalid command: method not found')
        getattr(self, method)()

    def iter_components(self) -> t.Generator[devtool.Component, None, None]:
        for d in sorted((str(d.parts[-1]) for d in self.meta.iter_component_dirs() if d.is_dir())):
            yield devtool.Component.load(self.meta, d)

    def cmd_check(self):
        for c in self.iter_components():
            print('Component %s is OK.' % c.name)

    def cmd_render(self):
        # TODO: render docs source
        groups = {}
        authors = {}
        for c in self.iter_components():
            for group in c.groups:
                if group.description not in groups:
                    groups[group.description] = []
                groups[group.description].append(c)
            for cr in c.copyrights:
                if cr.name.full_name not in authors:
                    authors[cr.name.full_name] = {'person': cr.name, 'components': []}
                authors[cr.name.full_name]['components'].append(c)

        env = jinja2.Environment(
            loader=jinja2.PackageLoader('devtool'),
            finalize=lambda x: x if x is not None else '',
            autoescape=False,
        )

        # render README
        tpl = env.get_template(self.README)
        readme_fn = p.Path(self.meta.repo_path) / self.README
        with readme_fn.open('w') as f:
            f.write(tpl.render(groups=groups, authors=authors))

    @staticmethod
    def _find_by_target(target: str, inverse: bool, components: t.Dict[str, devtool.Component]) -> t.List[str]:
        res = []

        for c in components.values():
            if not inverse:
                if target in c.targets:
                    res.append(c.name)
            else:
                if target not in c.targets:
                    res.append(c.name)

        return res

    def cmd_target(self):
        parser = argparse.ArgumentParser(
            prog=self.PROGRAM,
            description=self.COMMANDS['target'],
            usage='devtool.py target [-h] [-x] {%s}' % ','.join(self.meta.targets)
        )
        parser.add_argument('target', choices=self.meta.targets,
                            help='Target that components must support')
        parser.add_argument('-x', action='store_true',
                            help='Inverse operation: find components that do not support the specified target')
        args = parser.parse_args(sys.argv[2:])

        print(' '.join(self._find_by_target(args.target, args.x, {c.name: c for c in self.iter_components()})))

    @staticmethod
    def _find_dependants(dependency: str, components: t.Dict[str, devtool.Component]) \
            -> t.Dict[str, devtool.Component]:
        res = {}

        # direct dependants
        for c in components.values():
            if dependency in c.depends and c.name not in res:
                res[c.name] = c

        # indirect dependants
        while True:
            found = False
            for d in tuple(res.values()):
                for c in components.values():
                    if d.name in c.depends and c.name not in res:
                        res[c.name] = c
                        found = True
            if not found:
                break

        return res

    def cmd_depends(self):
        parser = argparse.ArgumentParser(
            prog=self.PROGRAM,
            description=self.COMMANDS['depends'],
            usage='devtool.py depends [-h] <dependency>'
        )
        parser.add_argument('dependency', help='Dependency name')
        args = parser.parse_args(sys.argv[2:])

        print('\n'.join(self._find_dependants(args.dependency, {c.name: c for c in self.iter_components()}).keys()))

    def cmd_ci(self):
        diff = subprocess.run(
            args=('git', 'diff', '--name-only', 'origin/master..HEAD'),
            cwd=self.meta.repo_path,
            capture_output=True,
            encoding='utf-8',
        )
        diff.check_returncode()

        diff = tuple(l.split('/') for l in diff.stdout.strip().replace('\r', '').split('\n'))
        dirs = set((l[0] for l in diff))
        changed_components = set((l[1] for l in diff if len(l) > 1 and l[0] == 'components'))
        all_components = {c.name: c for c in self.iter_components()}
        components_to_build = set()

        res = CI()
        if self.README in dirs:
            res.readme = True
        if changed_components:
            res.meta = True
            res.build = True
            res.readme = True
        if 'devtools' in dirs or '.github' in dirs:
            res.ci = True
        if 'docs' in dirs:
            res.docs = True
        if res.ci:
            res.meta = True
            res.build = True
            res.readme = True
            res.docs = True
            components_to_build = set(all_components.keys())
        if res.build and not components_to_build:
            for c_name in changed_components:
                if c_name in all_components:
                    components_to_build.add(c_name)
                components_to_build = components_to_build.union(self._find_dependants(c_name, all_components).keys())

        print('_ci_meta=%d' % res.meta)
        print('_ci_readme=%d' % res.readme)
        print('_ci_docs=%d' % res.docs)
        for target in self.meta.targets:
            components_by_target = set(self._find_by_target(target, False, all_components))
            print('_ci_build_%s=%s' % (target, ' '.join(components_to_build.intersection(components_by_target))))


if __name__ == '__main__':
    Devtool()
