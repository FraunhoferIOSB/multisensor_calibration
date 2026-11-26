# CONTRIBUTING

We are using the [Feature Branch Workflow](https://guides.github.com/introduction/flow/), and prefer contributions as merge requests.

**There's only one rule: anything in the master branch is always deployable.**

Create a feature branch:

`git checkout -B feat/<feature_name>`

## Recommended workflow

1. **Fork** the repository if your are not a collaborator and have direct access to the repository
2. Checkout new **branch** (branching of `devel`)
3. Make **changes**
4. Run **tests**
5. **Commit** those changes
6. **Push**
7. Create a **pull request** (targeting `devel`)

To run the tests, call the following from the root of the ROS workspace:

```bash
colcon build --packages-up-to multisensor_calibration
colcon test --packages-select multisensor_calibration
colcon test-result --verbose
```

<hr>

## Branching Recommendations

- Try to keep two main branches ```main``` and ```devel```.

- The ```devel```-branch should be the default branch in which all developments are done and from which new branches are created.

- Each new release with a version tag should be merged into ```main```. *Intermediate merges can be made if there is a feature which is to be made available in the default branch prior to an official release.*

<hr>

## Commit Conventions

We aim at concise and yet descriptive commit messages. In this, we suggest the following:

Each top line of a commit should be made up of a type and a subject

`<type>: <subject>`

Allowed types:

*   **feat**: A new feature
*   **enhance**: An enhancement to the current feature set
*   **fix**: A bug fix
*   **docs**: Documentation only changes
*   **style**: Changes that do not affect the meaning of the code (white-space, formatting, missing semi-colons, newline, line endings, etc)
*   **refactor**: A code change that neither fixes a bug or adds a feature
*   **perf**: A code change that improves performance
*   **test**: Adding missing tests
*   **chore**: Changes to the build process or auxiliary tools and libraries such as documentation generation
*   **release**: Committing changes, e.g. to CHANGELOG.rst, for a new release

You can add additional details after a new line to describe the change in detail or automatically close a issue on Github.

Example: 

```
docs: create initial CONTRIBUTING

This closes #73
```
<hr>

# Versioning Conventions

This repo adheres to semantic versioning: <http://semver.org/>.
Given a version number MAJOR.MINOR.PATCH, increment the:

- MAJOR version when you make incompatible API changes,
- MINOR version when you add functionality in a backwards-compatible manner, and
- PATCH version when you make backwards-compatible bug fixes.

Additional labels for pre-release and build metadata are available as extensions to the MAJOR.MINOR.PATCH format

<hr>