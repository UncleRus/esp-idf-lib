# How to contribute to `esp-idf-lib`

## Table of Contents

<!-- vim-markdown-toc GFM -->

* [Possible contributions](#possible-contributions)
    * [Submitting a bug report](#submitting-a-bug-report)
    * [Submitting a fix](#submitting-a-fix)
    * [Writing documentation](#writing-documentation)
    * [Suggesting enhancements](#suggesting-enhancements)
    * [Promoting the project](#promoting-the-project)
    * [Writing code](#writing-code)
* [Development Life Cycle](#development-life-cycle)
    * [Creating an Issue](#creating-an-issue)
    * [Creating a feature branch in your fork and develop](#creating-a-feature-branch-in-your-fork-and-develop)
    * [C Code style](#c-code-style)
    * [`markdown` Code style](#markdown-code-style)
    * [`git` branch name convention](#git-branch-name-convention)
    * [Typical issues you will face in developments](#typical-issues-you-will-face-in-developments)
    * [Writing a commit message](#writing-a-commit-message)
    * [Updating README.md](#updating-readmemd)
    * [Creating a Pull Request](#creating-a-pull-request)
* [Licenses](#licenses)
    * [Acceptable licenses](#acceptable-licenses)
    * [Acceptable license for new code](#acceptable-license-for-new-code)
    * [Unacceptable licenses](#unacceptable-licenses)

<!-- vim-markdown-toc -->

## Possible contributions

If you would like to contribute to `esp-idf-lib`, we would like to say _thank
you_. We appreciate your efforts and contributions. Here is possible
contributions you can make.

* [Submitting a bug report](#submitting-a-bug-report)
* [Submitting a fix](#submitting-a-fix)
* [Writing documentation](#writing-documentation)
* [Suggesting enhancements](#suggesting-enhancements)
* [Promoting the project](#promoting-the-project)
* [Writing code](#writing-code)

### Submitting a bug report

In embedded device development, finding bugs is more difficult than in other
software development. There are too many uncontrollable factors: physical
environment, counterfeit IC chips, deviations in revisions and variations,
difficulties in automations. Even if the bug turned out to be not a bug, such
report is still appreciated as it is another proof that the code works as
expected in a different environment.

Please include how to reproduce the bug in the Issue. The more context, the
better. For example:

* The _full_ error message in text format and the entire code (comment with
  ` ``` ` for short code, use [Gist](https://gist.github.com) for long code)
* The circuit diagram
* Captured signals by an oscilloscope or a signal analyser ([sigrok](https://sigrok.org/))

A question as a bug report is okay but we expect bug reporters to do their
homework. The homework include:

* Reading the data sheets
* Reading [the official documentation of `esp-idf`](https://docs.espressif.com/projects/esp-idf)
  (it's good, really)
* Understanding C language in general

For introductory C tutorials, see:

* [C Tutorial](https://www.tutorialspoint.com/cprogramming/) by
  `Tutorialspoint`
* [C Programming](https://en.wikibooks.org/wiki/C_Programming) by
  `Wikibooks`

### Submitting a fix

If you found a bug and a fix for it, please create a bug report before
creating a Pull Request unless the bug is subtle, typos, or easy to reproduce
and fix. Make sure to read [Development Life Cycle](#development-life-cycle)
as a Pull Request must meet the same standards documented in the section.

A GitHub Actions workflow,
[pr-labeler-action](https://github.com/TimonVS/pr-labeler-action), is used to
label PRs by branch name. Your fix branch should have prefixes defined in
[.github/pr-labeler.yml](.github/pr-labeler.yml). Create a branch with one of
the prefixes. If you are fixing a bug, your branch name should be `bugfix-`.
The rest of branch name should be short, and descriptive. If the fix has
related issues, the branch name should include them.

See also [`git` branch name convention](#git-branch-name-convention).

```console
git checkout -b bugfix-issue-1
```

Change the branch name before creating a PR if your branch name does not
follow the convention.

```console
git branch --move bugfix-issue-1
```

### Writing documentation

Even if you are not an author of the code in the repository, you can write
documentation as a contribution.

Creating and maintaining FAQ entries is one of great examples. Have you
encountered seemingly common issues while using a component? That might help
others.

We encourage code authors to write documentation in the code so that the code
and the documentation is always synced. However, sometimes they are not.
Spotting such mistakes is always appreciated.

Not all contributors are native English speakers. If you are, please let us
know ambiguity in the documentation, wrong usages of terms, and mistakes in
English grammar. For this case, please create a Pull Request (creating an
issue is optional).

Create a branch that documents features, or fixes existing documentations.

```console
git checkout -b doc-foo
```

See also [`git` branch name convention](#git-branch-name-convention).

### Suggesting enhancements

While we are not always able to write a driver for a chip, we still appreciate
a request for new driver. It is more likely to happen when:

* the chip is _cool_
* the chip is easily available
* the chip is affordable

### Promoting the project

If you find the project useful, we are interested in what you did with
`esp-idf-lib`, and _how_ you did it.

* Writing a blog post about your porject with `esp-idf-lib`
* Mentioning the project in SNS

### Writing code

If you can write a driver for new chip, that would be great. Please read
[Development Life Cycle](#development-life-cycle).

## Development Life Cycle

In this section, a typical development life cycle is explained.

### Creating an Issue

If you are working on a new driver, or an existing one, please create an
Issue, and assign the Issue to yourself.

`esp-idf-lib` aims at providing stable drivers for IC chips and general
components. IC chips are stable, in that a chip is manufactured for a long
time, retaining backward compatibilities. A driver for a chip usually requires
minimal maintenance once the driver becomes stable. However, network protocols,
graphics drivers, libraries for external services, are a moving-target.
Standards will change, external services will evolve, user expectations will
change, too. We think that such moving-targets should be maintained in a
dedicated repository. Do you think your code is a moving target? It might be
better to create a your own repository for the driver. If you are not sure,
ask in the Issue.

### Creating a feature branch in your fork and develop

_Feature branch workflow_ is adopted in our development.
[Git Feature Branch Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow)
by `atlassian` explains the workflow in details.

Fork the repository and clone it on your machine.
See [Fork a repo](https://docs.github.com/en/github/getting-started-with-github/fork-a-repo).

Create a feature branch in your fork from the `master` branch.

```console
git checkout master
```

Check out the feature branch. The feature branch name should start with
`feat-` or `feature-`.

```console
git checkout -b feat-implement-foo
````

See also [`git` branch name convention](#git-branch-name-convention).

Write your code. Test the code in your physical test environment.  Commit your
changes and push them to your remote fork on GitHub.

[`components/example`](components/example) has an example component, and
[`examples/example`](examples/example) has an example application for the
`example` component.

```console
git add path/to/files
git commit -v
git push --set-upstream origin feat-implement-foo
````

See also [Writing a commit message](#writing-a-commit-message).

At this point, our CI workflows will run to test the changes. The test
workflows include:

* building the documentation,
* building all examples for all supported targets with all supported
  `esp-idf` versions, and
* linting code and documentation

You can see the test results in `Actions` page on your GitHub fork. To
merge your changes to `master` branch, all the tests must pass.

Make sure you are working on the latest `master` of `esp-idf-lib`. To sync the
`master` in your fork and the latest `master` of `esp-idf-lib`, run:

```console
git checkout master
git fetch upstream
git reset --hard upstream/master
```

If your branch has many commits, consider `git rebase` to reduce the number of
commits. This is especially useful when you are actively developing and the
commit history has many trial-and-error commits.

```console
git checkout feat-implement-foo
git rebase -i master
git push -f
```

Note that `git rebase` rewrites the commit history. You should avoid `git
rebase` after you asked someone to review your code because the reviewer needs
additional steps to ensure the review result is included.

### C Code style

We use a style for source files based on [LLVM Coding Standards](https://llvm.org/docs/CodingStandards.html)
except some cases, notably brace wrapping. Here is a brief list of the styles.

* Use `snake_case`, not `CamelCase`
* Use `SNAKE_CASE` in uppercase for macro name, e.g. `MACRO_NAME`.
* Use spaces. The indent width is four
* Use `\n`, or `LF` for line breaks
* Use `//` for inline comments. Use `/* */` for multi line comments after an
  empty line
* Break before braces in *most cases* (functions, conditionals, control
  statements, etc)
* Always check given arguments
* Always check return code, return value, or `errno`
* Return `esp_err_t` from functions where possible
* Document public functions, data types, and macros in header files
* Use suffix `_t` for `typedef`, e.g. `foo_t`
* Use suffix `_cb_t` for function `typedef`, e.g.`my_function_cb_t`
* Use suffix `_s` for `struct`, e.g. `my_struct_s`
* Wrap numbers in macro definition with parenthesis, e.g. `#define N_FOO (1)`
* Use `#include <foo.h> for headers that are not part of the component, such
  as `string.h`, `esp_log,h`, and `i2cdev.h`. Use `#include "foo.h" when the
  header is private, i.e. the header is the part of the component.

The style should be followed for all new code. In general, code can be
considered "new code" when it makes up about 50% or more of the file(s)
involved. This is enough to break precedents in the existing code and use the
current style guidelines.

See an example source files under [`components/exmaple`](components/example)
and, for complete rules, see [`.clang-format`](.clang-format) and the output
of `clang-format --dump-config`.

New code will be tested in the CI, using `clang-format` (currently `LLVM`
version 10).

To format your code without modifying the code, run:

```console
clang-format10 components/example/example.c
```

To format your code in-place, run:

```console
clang-format10 -i components/example/example.c
```

### `markdown` Code style

We use [the default `markdownlint` rules](https://github.com/markdownlint/markdownlint/blob/master/docs/RULES.md)
with some non-defaults. Our style can be found in [`.mdlstyle.rb`](.mdlstyle.rb).

| Rule                                 | non-default options                            |
| ------------------------------------ | ---------------------------------------------- |
| `MD003` - Header style               | use `#` for headers                            |
| `MD007` - Unordered list indentation | indent with 4 spaces                           |
| `MD013` - Line length                | ignore line length in code blocks and tables   |
| `MD024` - Multiple headers with the same content | `allow_different_nesting` is true  |
| `MD029` - Ordered list item prefix   | `style` is `ordered`, i.e. incremental numbers |

In the CI, we use ruby version of `markdownlint`, or [`mdl`](https://rubygems.org/gems/mdl/)
gem, but [markdownlint for node.js](https://github.com/DavidAnson/markdownlint)
should also work.

To test `markdown` style of a file, you need:

* `ruby` 2.6
* `bundler` 2.x

```console
bundle install
bundle exec mdl path/to/file
```

The output shows path to the file, line number, and the rule.  An example
output is shown below.

```console
examples/led_strip_spi/README.md:30: MD040 Fenced code blocks should have a language specified
```

### `git` branch name convention

We use the following convention for git branch name. Use one of branch name
prefixes when creating a branch.

| Branch name prefix | Description |
|--------------------|-------------|
| `feat-`, and `feature-` | A feature branch that implements feature(s), or add enhancement(s) to existing code |
| `fix-`, and `bugfix-` | A bug fix branch that fixes bug(s). The rest of the branch name should include issue number, such as `fix-issue-1` |
| `ci-` | A branch that implements enhancement(s), or fixes issue(s) in CI |
| `chore-` | A branch that does not affect code or its behavior, such as updating `.gitignore` |
| `doc-`, and `documentation-` | Adding or updating documentation only, such as documenting undocumented features, or fixing existing documentation(s) |

A GitHub Actions workflow automatically labels PRs depending on the branch
name prefixes so that the PR is automatically included in release notes.

The rest of the branch name should be short, and descriptive. If your branch
fixes, implements, or relates to, an Issue, include the Issue number. Say, if
your branch fixes a bug reported Issue ${N}, the branch name should be
`fix-issue-${N}` so that reviewer immediately understand there is a related
Issue with your branch. Replace `${N}` with the Issue number, such as
`fix-issue-123` when the Issue number is 123.

### Typical issues you will face in developments

**Your code assumes a single target, such as `esp32`**. `esp-idf-lib` supports
other targets, notably `esp8266`. Make sure the driver supports various other
targets. If it cannot, such as the peripheral is not available on the target
chip, your code should bailout during the build by using `#error` C
preprocessor macro, and your driver must be excluded from the CI (TODO
document how).

**Your code assumes a single SDK**. `esp-idf-lib` supports `master` and stable
versions of `esp-idf` and `ESP8266 RTOS SDK`. Generally, the SDKs retain
backward compatibilities, but sometimes not. Make sure to use `if` C
preprocessor macro to support different versions. [`esp_idf_lib_helpers`](components/esp_idf_lib_helpers)
component can help you. `ESP8266 RTOS SDK` shares many functions and
libraries, backported from `esp-idf`, but they are not identical. `I2C`
drivers written with [`i2cdev`](components/i2cdev) should work fine on ESP32
and ESP8266, while SPI drivers need serious workarounds to support ESP8266.
[`led_strip_spi`](components/led_strip_spi) attempted to support both, but you
might want to write a different driver for each.

**Your code assumes a single build method, such as `idf.py`**. Although `GNU make`
build method is considered as legacy, it is still a supported build method.
The CI builds your code twice; with `idf.py` and with `GNU make`. Both must be
successful. In ESP8266 RTOS SDK, `idf.py` is lagged behind from the one in
`esp-idf`. For ESP8266 target, the CI builds examples with `GNU make` only.

Check return codes (most of functions in `esp-idf`), return values (e.g.
`malloc(3)`), or `errno` (e.g. some standard C functions).  Propagate the
error by returning it from your function. An example:

```c
#include <esp_err.h>
#include <esp_log.h>

esp_err_t do_something()
{
    esp_err_t err;

    err = foo();
    if (err != ESP_OK)
    {
        ESP_LOGE("bar", "foo(): %s", esp_err_to_name(err));
        goto fail;
    }
fail:
    return err;
}
```

Note that newer `esp-idf` supports useful macros for error handling, such as
`ESP_GOTO_ON_ERROR` (see
[Error Handling](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/error-handling.html)),
but older versions do not have them yet.

Check given arguments in functions, and return an appropriate error from one
of predefined errors (see
[Error Codes Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/error-codes.html)).

### Writing a commit message

When you commit, prefix the first line of your commit message with `foo:`,
where `foo` is the name of component you are working on. Sometimes, it is not
possible because you are working on multiple components, i.e. fixing common
bugs in multiple components. In such cases, use `bugfix:`. Other commonly used
prefix words are:

* `feature:` for features, or improvements, in multiple components
* `ci:` for fixes or improvements in the CI process
* `doc:` for fixes and improvements in the documentation

These prefix words are for conventional purposes. Use common sense and make
the commit message clear so that others can understand what the change is.

The rest of the first line should start with a verb. Examples:

```text
foo: fix typos
```

```text
foo: resolve race condition in bar()
```

The first line should make sense when reading _"When you merge this, it will
`$THE_FIRST_LINE`"_.

The second line of the commit message must be an empty line.

In the rest of the commit message, write details of the change if necessary.
Explain what it does _and_ *why*.  The lines in the commit message should be
limited to 72 characters or less where possible.

Include a reference to an Issue when the commit fixes an Issue.

```text
fixes #$ISSUE_NUMBER
```

When an Issue number or a Pull Request number is prefixed with certain
keywords, the referenced Issue or Pull Request will be closed. See [Linking a
pull request to an issue using a keyword](https://docs.github.com/en/github/managing-your-work-on-github/linking-a-pull-request-to-an-issue#linking-a-pull-request-to-an-issue-using-a-keyword)
for the supported keywords.

### Updating README.md

Each component has a `.eil.yml` file in its component directory. The file is a
metadata file of the component. If you change the file, you need to update the
`README.md` in the project root directory. The `README.md` is generated from
the metadata and a template.

```console
./devtools/devtool.py --repo=. render
```

See also [`Metadata.md`](Metadata.md).

### Creating a Pull Request

To test your code, you need to create a Pull Request. It is not practical to
test code manually because you have to perform many tests. For instance, the
number of tests is all targets (`esp32`, `esp8266`, `esp32s2`, etc) * build
methods (`make` and `idf.py`) * supported `esp-idf` versions. Let the CI do it
for you.

Before creating a Pull Request, make sure:

1. You compiled the code and the build succeeded
1. Functions, macros, data types are documented in the code
1. An example application is provided under [`examples`](examples). In the
   directory, create a directory `${COMPONENT_NAME}/default`. For instance, a
   component `foo` must have `examples/foo/default`. Create an example
   application in that directory.
1. Update [.github/labeler.yml](.github/labeler.yml). The component should
   have a label for it.
1. You compiled the example code and the example application ran on a
   physical device as expected and documented
1. All files are licensed under one of [Acceptable Licenses](#acceptable-licenses)
   by including the license at the top of file
1. One of your commits in the feature branch, or the PR itself, mentions Issue
   number so that the Issue will be automatically closed when the PR is merged

When a PR is created, GitHub Actions workflows will:

* label the PR with various labels, such as type of the PR (bug fix, or
  feature)
* perform necessary tests depending on the changes (build the examples in a
  matrix if the source code is modified, build the documentation if files
  under `docs` are modified)

After the CI processes complete, you will see "All checks have passed" or some
failures in the PR. To merge the PR, all checks must pass. Log is available
from the link, `Details`, in the failed test.

If the PR does not pass the CI, update the branch with a fix. At this point,
`git rebase` may be used. For instance, if a commit has a typo and one of the
test fails because of syntax error, commit a fix of the syntax error and do
`git rebase` to merge the fix into the original commit that has introduced the
syntax error.

```console
git add path/to/file
git commmit -v
git rebase -i master
```

`-i`, or `--interactive`, flag will launch a text editor where the history of
the branch can be edited with commands. The buffer of the editor will look
like:

```text
pick f7f3f6d bugfix: fix issue foo
pick 310154e bugfix: fix a syntax error in f7f3f6d
```

The second commit, `310154e`, should be part of the previous, `f7f3f6d`. To
rewrite the commit history, replace `pick` with `fixup`.

```text
pick f7f3f6d bugfix: fix issue foo
fixup 310154e bugfix: fix a syntax error in f7f3f6d
```

When you save and exit the editor, `git` rewrites the commit history as if
`310154e` was never committed. The commit `310154e` is now part of `f7f3f6d`.
If you don't save or modify the buffer, `git` will not rewrite the commit
history.

`git rebase` can be used to tidy up the commits. To `rebase` or not to
`rebase` depends on the nature of commits. A single commit per PR is preferred,
but is not mandatory.

See also
[7.6 Git Tools - Rewriting History](https://git-scm.com/book/en/v2/Git-Tools-Rewriting-History).

When all the tests pass, ask the code owner to review the PR. The code owner
can be found in `.eli.yml` file in the component directory.  From this point,
you should avoid to `git rebase` your feature branch. Otherwise, the reviewer
would have to review the PR from scratch.

Developers who has write access to the repository will leave comments, ask
rewrites, and merge the PR.

## Licenses

We provide code that can be freely used, copied, modified, and distributed by
anyone and for any purpose.

### Acceptable licenses

We accept permissive licenses such as:

* [ISC](https://spdx.org/licenses/ISC.html) License
* [MIT](https://spdx.org/licenses/MIT.html) License
* [BSD-2-Clause](https://spdx.org/licenses/BSD-2-Clause.html) License

A list of licenses are available at
[SPDX License List](https://spdx.org/licenses/).

### Acceptable license for new code

New code is the one you own (you wrote it from scratch). The preferred
license to be applied to new code is a simplified ISC License. The license
must be included at the top in every files as long as practically possible.
The following is a preferred wording of the license.

```c
/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) YYYY YOUR NAME HERE <user@your.dom.ain>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
 ```

Add `SPDX-License-Identifier: $YOUR_LICENSE` to your license header.
`$YOUR_LICENSE` is a SPDX License Identifier.

* [ISC](https://spdx.org/licenses/ISC.html)
* [BSD-2-Clause](https://spdx.org/licenses/BSD-2-Clause.html)

### Unacceptable licenses

We do NOT accept `copyleft` licenses such as:

* GPL License
* LGPL License
* GNU Affero General Public License (AGPL)

We do NOT accept _long_ licenses. A license is considered as _long_ when
it has more than four clauses.

We do NOT accept protective licenses that have additional restrictions, such
as:

* Apache license version 2 or later
* various so-called _Shareware_ or _Freeware_
