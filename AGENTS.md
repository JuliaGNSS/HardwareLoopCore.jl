# Agent instructions

## Commit messages: Conventional Commits (drive the releases)

Releases are automated: every push to `main` runs semantic-release
(`.github/workflows/Release.yml`), which derives the version bump, the
changelog and the JuliaRegistrator call from the commit messages. Every
commit must follow [Conventional Commits](https://www.conventionalcommits.org):

```
type(scope): short imperative summary
```

`feat` → minor release, `fix` / `perf` → patch release; `refactor`, `test`,
`docs`, `build`, `chore` → no release. A breaking change carries a `!` after
the type/scope and a `BREAKING CHANGE:` footer that says what breaks and how
to migrate.
