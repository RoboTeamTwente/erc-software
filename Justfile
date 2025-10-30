# List available commands
[default]
_list:
    @just --list

# Release a new version
release:
    #!/bin/sh -eux
    version=$(git cliff --bumped-version --with-commit 'chore(version): TODO')
    message="chore(version): $version"
    git cliff --bump -o CHANGELOG.md --with-commit "$message"
    git commit CHANGELOG.md -m "$message"
    git tag "$version"
