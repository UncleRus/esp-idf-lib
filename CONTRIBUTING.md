# How to contribute to `esp-idf-lib`

## Table of Contents
<!-- vim-markdown-toc GFM -->

* [Possible contributions](#possible-contributions)
    * [Submitting a bug report](#submitting-a-bug-report)
    * [Submitting a fix](#submitting-a-fix)
    * [Writing documentation](#writing-documentation)
    * [Suggesting enhancements](#suggesting-enhancements)
    * [Writing code](#writing-code)
* [Development Life Cycle](#development-life-cycle)
    * [Crating an Issue](#crating-an-issue)
    * [Creating a feature branch in your fork and develop](#creating-a-feature-branch-in-your-fork-and-develop)
    * [Typical issues you will face in developments](#typical-issues-you-will-face-in-developments)
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
* [Writing documentation](#submitting-a-fix)
* [Suggesting enhancements](#suggesting-enhancements)
* Writing code
* Promoting the project

### Submitting a bug report

In embedded device development, finding bugs is more difficult than other
software development. There are too many uncontrollable factors; physical
environment, counterfeit IC chips, deviations in revisions and variations,
difficulties in automations. Even if the bug turned out to be not a bug, such
report is still appreciated as it is another proof that the code works as
expected in a different environment.

Please include how to reproduce the bug in the Issue. The more context, the
better. For example:

- The _full_ error message in text format and the entire code (comment with
  `&#96;&#96;&#96;` for short code, use [Gist](https://gist.github.com) for
  long code)
- The circuit diagram
- Captured signals by an oscilloscope or a signal analyser ([sigrok](https://sigrok.org/))

A question as a bug report is okay but we expect bug reporters to do their
homework. The homework include:

- Reading the data sheets
- Reading [the official documentation of `esp-idf`](https://docs.espressif.com/projects/esp-idf)
  (it's good, really)
- Understanding C language in general (TODO add resources here)

### Submitting a fix

If you found a bug and a fix for it, please create a bug report before
creating a Pull Request unless the bug is subtle, typos, or easy to reproduce
and fix. Make sure to read [Development Life Cycle] as a Pull Request must
meet the same standards documented in the section.

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

### Suggesting enhancements

While we are not always able to write a driver for a chip, we still appreciate
a request for new driver. It is more likely to happen when:

- the chip is _cool_
- the chip is easily available
- the chip is affordable

### Writing code

If you can write a driver for new chip, that would be great. Please read
[Development Life Cycle].

## Development Life Cycle

In this section, a typical development life cycle is explained.

### Crating an Issue

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

Fork the repository and clone it on your machine. See [Fork a repo](https:
//docs.github.com/en/github/getting-started-with-github/fork-a-repo).

Create a feature branch in your fork from the `master` branch.

```console
git checkout master
```
Check out the feature branch.

```cosole
git checkout -b my-feature
````

Write your code. Test the code in your physical test environment.  Commit your
changes and push them to your remote fork on GitHub

```console
git add path/to/files
git commit -v
git push --set-upstream origin my-feature
````

At this point, our CI workflows will run to test the changes. The test
workflows include:

- building the documentation,
- building all examples for all supported targets with all supported
  `esp-idf` versions, and
- linting code and documentation

You can see the test results in `Actions` page on your GitHub fork. To
merge your changes to `master` branch, all the tests must pass.

Make sure you are working on the latest `master` of `esp-idf-lib`. To sync the
`master` in your fork and the latest `master` of `esp-idf-lib`, run:

```cosole
git checkout master
git fetch upstream
git reset --hard upstream/master
```

If your branch has many commits, consider `git rebase` to reduce the number of
commits. This is especially useful when you are actively developing and the
commit history has many trial-and-error commits.

```console
git checkout my-feature
git rebase -i master
git push -f
```

Note that `git rebase` rewrites the commit history. You should avoid `git
rebase` after you asked someone to review your code because the reviewer needs
additional steps to ensure the review result is included.

### Typical issues you will face in developments

Your code assumes a single target, such as `esp32`. `esp-idf-lib` supports
other targets, notably `esp8266`. Make sure the driver supports various other
targets. If it cannot, such as the peripheral is not available on the target
chip, your code should bailout during the build by using `#error` C
preprocessor macro, and your driver must be excluded from the CI (TODO
document how).

Your code assumes a single SDK. `esp-idf-lib` supports `master` and stable
versions of `esp-idf` and `ESP8266 RTOS SDK`. Generally, the SDKs retain
backward compatibilities, but sometimes not. Make sure to use `if` C
preprocessor macro to support different versions. `esp_idf_lib_helpers`
component can help you.  `ESP8266 RTOS SDK` shares many functions and
libraries, backported from `esp-idf`, but they are not identical. `I2C`
drivers written with [`i2cdev`](components/i2cdev) should work fine on ESP32
and ESP8266, while SPI drivers need serious workarounds to support ESP8266.
[`led_strip_spi`](components/led_strip_spi) attempted to support both, but you
might want to write a different driver for each.

### Creating a Pull Request

When your code is ready to be merged, and all the tests have passed in the CI,
create a Pull Request. Before creating a Pull Request, make sure:

1. You compiled the code and the build succeeded
1. You pushed the changes to your remote fork and the CI passed
1. Functions, macros, data types are documented in the code
1. An example application is provided under [`examples`](examples)
1. You compiled the example code and the example application ran on a
   physical device as expected and documented
1. All files are licensed under one of [Acceptable Licenses](#acceptable-licenses)
   by including the license at the top of file
1. One of your commits in the feature branch, or the PR itself, mentions Issue
   number so that the Issue will be automatically closed when the PR is merged

From this point, you should avoid to `git rebase` your feature branch.

Developers who has write access to the repository will leave comments, ask
rewrites, and merge the PR.

## Licenses

We provide code that can be freely used, copied, modified, and distributed by
anyone and for any purpose.

### Acceptable licenses

We accept permissive licenses such as:

- ISC License
- MIT License
- BSD License

### Acceptable license for new code

New code is the one you own (you wrote it from scratch).  The preferred
license to be applied to new code is a simplified ISC License.  The license
must be included at the top in every files as long as practically possible.
The following is a preferred wording of the license.

```c
/*
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

### Unacceptable licenses

We do NOT accept `copyleft` licenses such as:

- GPL License
- LGPL License
- GNU Affero General Public License (AGPL)

We do NOT accept _long_ licenses. A license is considered as _long_ when
it has more than four clauses.

We do NOT accept protective licenses that have additional restrictions, such
as:

- Apache license version 2 or later
- various so-called _Shareware_ or _Freeware_
