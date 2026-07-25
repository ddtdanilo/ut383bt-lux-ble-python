# Releasing

1. Update the version in `pyproject.toml`, `src/ut383bt/__init__.py`,
   `CITATION.cff`, and `CHANGELOG.md`.
2. Run the complete checks in `AGENTS.md`.
3. Merge the release preparation through a protected pull request.
4. Create and push an annotated `vX.Y.Z` tag at the verified `main` commit.
5. The release workflow rebuilds and retests the package, then creates the
   GitHub release with wheel and source distribution.
6. Verify release asset digests and perform one optional manual hardware smoke
   test on a supported host.

Do not publish to PyPI until a separate trusted-publishing policy is explicitly
approved and documented.
