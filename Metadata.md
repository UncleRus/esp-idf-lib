# Metadata

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

TBW

### `groups.yml`

TBW

## Person

A `Person` represents a person. `Person` is used to describe a copyrights
holder and, a code owner.

| Name | Type | Description | Required |
|------|------|-------------|----------|
| `name` | `String` | A unique ID string of the person. Use GitHub account or GitHub project if the person has one | Yes |
| `full_name` | `String` | Full name of the person or the project | No |
| `gh_id` | `String` | GitHub account name or project name | No |
| `email` | `String` | Email address of the person | No |
| `website` | `String` | Web site URL | No |

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
website: https://github.com/trombik
```

```yaml
name: foo
full_name: Foo `bar` buz
# XXX other keys are optional, but strongly recommended.
```

## Target

| Name | Type | Description | Required |
|------|------|-------------|----------|
| `name` | `String` | Name of the build target in `esp-idf`, or `esp8266`. | Yes |

An example:

```yaml
name: esp32
```

## License

| Name | Type | Description | Required |
|------|------|-------------|----------|
| `name` | `String` | SPDX License Identifier (see [the list of licenses](https://spdx.org/licenses/)) | Yes |

An example:

```yaml
name: BSD-3-Clause
```

## Copyright

| Name | Type | Description | Required |
|------|------|-------------|----------|
| `name` | `String` | `name` of `person` of the copyrights holder. See also `person`. The name must be available in `persons.yml` | Yes |
| `year` | `Integer` | Registration year of the copyrights | Yes |

Examples:

```yaml
name: trombik
year: 2021
```

## Group

| Name | Type | Description | Required |
|------|------|-------------|----------|
| `name` | `String` | A unique ID of the group | Yes |
| `description` | `String` | Description of the group | Yes |

`name` should be short, and memorable. Use `-` as a word separator. It must
not include spaces (`[0-9a-zA-Z-]+` in regular expression).

An example:

```yaml
name: adc-dac
description: ADC/DAC libraries
```

## Components

`components` includes one or more of `component`. The metadata file must have
`components` as a top level key.

## Component

| Name | Type | Description | Required |
|------|------|-------------|----------|
| `name` | `String` | The name of the component. Must be unique. | Yes |
| `description` | `String` | A short description of the component. | Yes |
| `group` | `String` | The primary group name of the component. | Yes |
| `groups` | A list of `String` | Optional list of `group` | No |
| `code_owners` | A list of `person` | A list of `code_owners` | No |
| `depends` | A list of component | Zero or more of `component` that the component depends on | No |
| `thread_safe` | `Strnig` | One of `yes`, `no`, and `N/A` | No |
| `targets` | A list of `target` | One or more of supported `target` | Yes |
| `licenses` | A list of `license` | One or more of licenses used in the component | Yes |
| `copyrights` | A list of `copyright` | One or more of copyright holder | Yes |

FIXME `depends` must be a list because some drivers have conditional `REQUIRES`
in `CMakeLists.txt`.
