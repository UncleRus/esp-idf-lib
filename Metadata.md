# Metadata

## Table of Contents

<!-- vim-markdown-toc GFM -->

* [Purpose](#purpose)
* [Files](#files)
    * [`.eil.yml`](#eilyml)
    * [`persons.yml`](#personsyml)
    * [`groups.yml`](#groupsyml)
    * [`targets.yml`](#targetsyml)
* [Resources](#resources)
    * [Person](#person)
    * [Target](#target)
    * [License](#license)
    * [Copyright](#copyright)
    * [Group](#group)
    * [Metadata](#metadata)
    * [Component](#component)
* [Usages of metadata in the project](#usages-of-metadata-in-the-project)
    * [Validating metadata of components](#validating-metadata-of-components)
    * [Generating `README.md`](#generating-readmemd)
* [Known issues](#known-issues)
    * [conditional `depends`](#conditional-depends)

<!-- vim-markdown-toc -->

This document describes metadata used in the project. The status of the
document is beta.

## Purpose

The purpose of metadata in the project is to automate works in development and
project management, to ensure the project policies, and to extend the project
to support third-party projects.

## Files

### `.eil.yml`

The metadata file of a component.  Each component must have `.eil.yml` in the
root directory of the component. The file format is YAML.

An example path: `components/ads111x/.eil.yml`.

### `persons.yml`

`persons.yml` is a YAML file that contains a list of `Person`s.

### `groups.yml`

`groups.yml` is a YAML file that contains a list of `Group`s.

### `targets.yml`

`targets.yml` is a YAML file that contains a list of `Target`s.

## Resources

Resources defined here represents various objects used in the metadata.

A resource has unique `name` as a primary key.

When referring to a resource in another resource, use `name` as 
value to identify the resource. As a shorthand, you may use the name of a
resource as `String`. In this case, the value is assumed to be `$VALUE`.

When a resource expects a `Person` as a value,

```yaml
foo:
  - trombik
```

This is a shorthand version of the above example:

```yaml
foo: trombik
```

### Person

A `Person` represents a person. `Person` is used to describe a copyrights
holder and a code owner. A `Person` must be defined in `persons.yml` file.

| Name        | Type     | Description                                                                                  | Required |
|-------------|----------|----------------------------------------------------------------------------------------------|----------|
| `name`      | `String` | A unique ID string of the person. Use GitHub account or GitHub project if the person has one | Yes      |
| `full_name` | `String` | Full name of the person or the project                                                       | No       |
| `gh_id`     | `String` | GitHub account name or project name                                                          | No       |
| `email`     | `String` | Email address of the person                                                                  | No       |
| `url`       | `String` | Web site URL                                                                                 | No       |

When any of `gh_id`, `email`, or `website` is not available, `person` must
have a full name because it is used to identify the source of code.

If the person does not have `gh_id`, use the full name for `name`. For example,
when the full name is "Foo Bar", use `name: FooB`.

`Person` should have one or more of optional keys so that one can contact the
person.

Examples:

```yaml
name: trombik
gh_id: trombik
full_name: Tomoyuki Sakurai
email: y@trombik.org
url: https://github.com/trombik
```

```yaml
name: foo
full_name: Foo `bar` buz
# XXX other keys are optional, but strongly recommended.
```

### Target

| Name   | Type     | Description                                          | Required |
|--------|----------|------------------------------------------------------|----------|
| `name` | `String` | Name of the build target in `esp-idf`, or `esp8266`. | Yes      |

An example:

```yaml
name: esp32
```

### License

| Name   | Type     | Description                                                                      | Required |
|--------|----------|----------------------------------------------------------------------------------|----------|
| `name` | `String` | SPDX License Identifier (see [the list of licenses](https://spdx.org/licenses/)) | Yes      |

An example:

```yaml
name: BSD-3
```

### Copyright

| Name     | Type      | Description                                               | Required |
|----------|-----------|-----------------------------------------------------------|----------|
| `author` | `Person`  | Copyrights holder. See also `Person`.                     | No       |
| `name`   | `String`  | The value of `name` of `Person`. A shorthand for `author` | No       |
| `year`   | `Integer` | Registration year of the copyrights                       | Yes      |

`Copyright` must have only one of `author` and `name`, not both.

Examples:

```yaml
name: trombik
year: 2021
```

The above example is a shorthand form of:

```yaml
author:
  name: trombik
year: 2021
```

### Group

A `Group` represents a group of `Component`s. A `Group` must be in
`groups.yml`.

| Name          | Type     | Description              | Required |
|---------------|----------|--------------------------|----------|
| `name`        | `String` | A unique ID of the group | Yes      |
| `description` | `String` | Description of the group | Yes      |

`name` should be short, and memorable. Use `-` as a word separator. It must
not include spaces (`[0-9a-zA-Z-]+` in regular expression).

An example:

```yaml
name: adc-dac
description: ADC/DAC libraries
```

### Metadata

`Metadata` is the content of `.eil.yml`. `Metadata` includes non-empty list of
`Component` under `components` top level key.

An example:

```yaml
---
components:
  - name: foo
  # ... other keys go here ...
```

### Component

| Name          | Type                  | Description                                               | Required |
|---------------|-----------------------|-----------------------------------------------------------|----------|
| `name`        | `String`              | The name of the component. Must be unique.                | Yes      |
| `description` | `String`              | A short description of the component.                     | Yes      |
| `version`     | `String`              | Component version.                                        | Yes      |
| `groups`      | A list of `Group`     | A list of one or more of `Group`                          | No       |
| `code_owners` | A list of `Person`    | A list of one or more of `Person`                         | Yes      |
| `depends`     | A list of `Component` | Zero or more of `component` that the component depends on | No       |
| `thread_safe` | `Strnig`              | One of `yes`, `no`, and `N/A`                             | Yes      |
| `targets`     | A list of `Target`    | One or more of supported `target`                         | Yes      |
| `license`     | `License`             | License used in the component                             | Yes      |
| `copyrights`  | A list of `Copyright` | One or more of copyright holder                           | Yes      |

FIXME `depends` must be a list because some drivers have conditional `REQUIRES`
in `CMakeLists.txt`.

## Usages of metadata in the project

Requirements are:

* `python` >=3.10

After installing requirements, run:

```console
pip install -r devtool/requirements.txt
```

### Validating metadata of components

To validate metadata, run:

```console
python ./devtools/devtool.py check
```


### Generating `README.md`

`README.md` is generated from the metadata and `devtools/devtools/template/README.md`. To update
`README.md`, run the following command at the repository root directory:

```console
python ./devtools/devtool.py render
```

## Known issues

### conditional `depends`

Some `CMakeLists.txt` conditionally sets `REQUIRES`. `depends` does not handle
the following case.

```yaml
# for esp32
depends:
  - driver
  - freertos
  - log
```

```yaml
# for esp8266
depends:
  - esp8266
  - freertos
  - log
```

A possible solution:

```yaml
depends:
  - name: driver
    target:
      - esp32
      - esp32s2
      - esp32c3
  - name: esp8266
    target:
      - esp8266
  - freertos
  - log
```
