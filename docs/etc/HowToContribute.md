# How to contribute

Thank you for your interest in improving scenario_simulator_v2.
This contributor guidelines will help you get started.

If you have any questions, please feel free to contact the [maintainers](ContactUs.md).

## Before you get started

To avoid duplicate work, please review current [issues](https://github.com/tier4/scenario_simulator_v2/issues) and [pull requests](https://github.com/tier4/scenario_simulator_v2/pulls).

## Contribution workflow

Please take the following steps:

1. Fork this repository.
2. Clone down the repository to your local machine.
3. Create a new branch from the `master`. See [Branch naming rules](#branch-naming-rules) below.
4. Commit your changes to the branch and push the commit to your GitHub repository that was forked in Step 1.
5. Open a Pull Request (PR). Our CI workflows will automatically test your changes. See [Continuous integration](#continuous-integration) below.
6. The maintainers will review your PR. Once the PR is approved, the code will be merged into the `master` branch. See [Code review](#code-review) below.

## Branch naming rules

### Feature development

Each new feature development should be worked on in its own branch.

Please add the `feature/` prefix to your branch name. For example:

```
feature/(name of the feature you are developing)
```

### Bugfix

Please add the `fix/` prefix to your branch name. For example:

```
fix/(name of bug what you are fixing)
```

### Release

Only maintainers create the `release/` branch.

```
release/prepare_(version_tag)
```

The release branches are used only to update the release notes. An example is [here](https://github.com/tier4/scenario_simulator_v2/pull/477).

## Continuous integration

Your changes proposed in your pull request will be tested automatically by the following checks:

| Checks                                                                                                                                                                                                 | Description                                                        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------| ------------------------------------------------------------------ |
| [![ScenarioTest](https://github.com/tier4/scenario_simulator_v2/actions/workflows/ScenarioTest.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/ScenarioTest.yaml)    | Build all packages and run integration tests.                      |
| [![BuildTest](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Build.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Build.yaml)                     | Build each package independently, run linters, and run unit tests. |
| [![Docker](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml)                      | Build a docker image.                                              |
| [![Documentation](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml) | Build the documentation sites.                                     |
| [![SpellCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml)          | Run a spell checker and add warnings to the PR.                    |

If you contribute to the documentation, your changes should pass the checks below:

| Checks                                                                                                                                                                                            | Description                                     |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------- |
| [![Documentation](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml) | Build the documentation sites.                  |
| [![SpellCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml)     | Run a spell checker and add warnings to the PR. |

## Code review

Any changes to the code or documentation are subject to code review. Maintainers will review your pull request.

As a good practice, reply to the reviewer's comment with a link to your changes (e.g., `Fixed in a0b1c2d`).
To keep the commit hashes consistent, **please DO NOT force-push the commit to your pull request during the code review.**
If you want to force-push the commit during the review, please contact the maintainers for approval in advance.

If more than one maintainer approves your pull request and all checks are passed, your pull request will be merged into the `master` branch.
Your contribution will be recorded in the [release note](ReleaseNotes.md).