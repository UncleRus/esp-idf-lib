# Workflows

This document describes how GitHub Actions workflows work in the repository.

<!-- vim-markdown-toc GFM -->

* [Overview](#overview)
* [Labeler workflow](#labeler-workflow)
    * [Labels used by the CI](#labels-used-by-the-ci)
    * [The triggering labeler workflow and Personal Access Token](#the-triggering-labeler-workflow-and-personal-access-token)
    * [Creating a Personal Access Token](#creating-a-personal-access-token)
    * [Using a Personal Access Token](#using-a-personal-access-token)
    * [Security considerations](#security-considerations)
* [Action workflow](#action-workflow)
    * [Security considerations](#security-considerations-1)

<!-- vim-markdown-toc -->

## Overview

The workflows consist of _action_ and _labeler_ workflows.

An action workflow is a typical workflow that builds sources, runs lint
programs, or performs other CI tests. Other actions include "Add a comment to
PR", or "Mention code owners when their code is modified by the PR", or
"Create a draft release notes".

Action workflows are triggered by labels added or removed to PRs. This is
achieved by a job in the action workflows that tests if the PR has a specific
label.

The responsibility of an action workflow is to take actions. An action
workflow should not decide when to perform actions, which is a responsibility
of labeler workflows.

A labeler action manages which action workflow to invoke.  `labeler*.yml` add
or remove labels to/from the PR. These workflows control action workflows
that perform actions. For instance, [labeler.yml](labeler.yml) and its
configuration file, [labeler.yml](../labeler.yml), add labels to
the PR by modified files. The labels are used by action workflows, and if the
label is the one defined in the action workflow, the action workflow runs.

The responsibility of a labeler action is to add or remove labels, or when to
invoke action workflow. A labeler action should not take actions, such as
build sources.

## Labeler workflow

labeler workflow is the logic for action workflows. By removing logics (when
a workflow should run) from the workflow, it is easier to see why a workflow
runs or not.

In a labeler workflow, add or remove labels by using
[actions/github-script](https://github.com/actions/github-script), which
enable to use JavaScript code and GitHub REST APIs. It is possible to use
shell instead, but not very practical.

### Labels used by the CI

| Label             | Description                          |
|-------------------|--------------------------------------|
| `area:components` | Code under `components` is modified. |
| `area:components:${NAME}` | Code of a specific component is modified. `${NAME}` is the name of the component. |
| `area:ci` | The CI is modified. |

### The triggering labeler workflow and Personal Access Token

Events triggered by a workflow do not run another workflow.
[Triggering a workflow from a workflow](https://docs.github.com/en/actions/using-workflows/triggering-a-workflow#triggering-a-workflow-from-a-workflow)
says:

> When you use the repository's GITHUB_TOKEN to perform tasks, events
> triggered by the GITHUB_TOKEN will not create a new workflow run. This
> prevents you from accidentally creating recursive workflow runs.

That means `pull_request` event with type `labeled` will not be fired when a
workflow labels a PR (but it will be if you manually label a PR).

To trigger other workflows from a labeler, the labeler workflow needs a
Personal Access Token, or a PAT, instead of `GITHUB_TOKEN`.

> If you do want to trigger a workflow from within a workflow run, you can use
> a personal access token instead of GITHUB_TOKEN to trigger events that
> require a token.

### Creating a Personal Access Token

To create a PAT, visit [Settings](https://github.com/settings/profile) in your
profile, and `Developer settings` > `Personal access token`. Click `Generate
new token`.

Choose `Expiration`. If you choose an expiration date for the token, the token
will expire after GitHub send a notification that reminds you of the
expiration. Obviously, you need to create a new token after the expiration
date.  If you choose `No expiration`, the token will not expire, but GitHub
does not recommend it.

Choose `public_repo` scope for the access token.

Click `Generate token`.

See
[Creating a personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token)
for details.

### Using a Personal Access Token

To use the generated PAT, it must be passed to workflows by a repository
secret. That way, the token is available `${{ secrets.* }}` in workflows
without hard-coding it in public code.

After creating a PAT, create a repository secret. The name should be
`LABELER_TOKEN` (this is the key of repository secrets used in labeler
workflows), and the value should be the token. See
[Managing encrypted secrets for your repository and organization for Codespaces](https://docs.github.com/en/codespaces/managing-codespaces-for-your-organization/managing-encrypted-secrets-for-your-repository-and-organization-for-codespaces)
for details.

```yaml
jobs:
  labeler:
    name: Labeler
    runs-on: ubuntu-latest
    steps:
      - uses: fuxingloh/multi-labeler@v2
        with:
          github-token: ${{secrets.LABELER_TOKEN}}
          config-path: .github/labeler.yml
```

Note that if the labeler merely labels PRs just for human-being, not to run
another workflow, the labeler does not need a PAT. Use `GITHUB_TOKEN` in that
case.

```yaml
jobs:
  labeler:
    name: Labeler
    runs-on: ubuntu-latest
    steps:
      - uses: fuxingloh/multi-labeler@v2
        with:
          github-token: ${{secrets.GITHUB_TOKEN}}
          config-path: .github/labeler.yml
```

### Security considerations

Labeler workflow needs write access to the repository because they modifies
PRs by adding or removing labels. This means labeler workflows have to run in
the privileged context. In that context, the workflow has full access to the
repository and the security token.

On the other hand, PRs are opened by repository members and (possibly
untrusted) third-party members. This means a labeler workflow is run by a
third-party with privileged access to the repository. For this reason, a
labeler workflow should add or remove label only.

A labeler workflow must use `pull_request_target` event as a trigger.

```yaml
---
name: Label PR
on:
  pull_request_target:
```
See
[pull_request_target](https://docs.github.com/en/actions/using-workflows/events-that-trigger-workflows#pull_request_target)
in the official documentation for details.

## Action workflow

Action workflow is, typically, a workflow that performs tests, such as
linting, or building sources. Other actions include "Writing a comment in the
PR".

In an action workflow, see if the PR has a specific label first, and if the PR
has the label, run the actions. Otherwise, skip.

Note that the action should run not only when an label is added (`labeled`),
but also when the PR is updated (`synchronize`). Typically, you need the
following `on` condition in the workflow:

```yaml
---
name: Build examples
on:
  pull_request:
    branches:
      - main
    types:
      - labeled
      - synchronize
```

If an action workflow needs to add a label depending on the outcome of the
test, use [labeler-by-workflow-status.yml](labeler-by-workflow-status.yml)
because, in the `pull_request` context, it is not possible.

See an example at: [FIXME]

### Security considerations

Most of action workflow do not need privileged access to the repository.
Typically, an action workflow should run in unprivileged context. That means
most of action workflows should use `pull_request` event as a trigger.

```yaml
name: An example action workflow
on:
 pull_request:
  branches:
    - master
```

In the unprivileged context, the action workflow does not have write access to
the repository. The action workflow may build untrusted code (i.e. the code
from forks). The action workflow cannot modify the repository (i.e. adding
comment or labels).

Action workflow might need write access to the repository if the action to take
is an action that modifies the repository, such as adding a comment to PRs.
When this is the case, the action workflow should use `pull_request_target`
event.

```yaml
name: An example action workflow with privileged access
on:
 pull_request_target:
```

In this context, some extra privileges are granted. The `pull_request_target`
is designed for adding labels to PRs, or writing comments to PRs. The action
workflow must not build sources, or run untrusted code (i.e. the code from
forks).

See
[pull_request_target](https://docs.github.com/en/actions/using-workflows/events-that-trigger-workflows#pull_request_target)
in the official documentation for details.
