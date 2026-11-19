> Agents keep temporary, project-related files (scratch scripts, logs, notes) in `target/.agentic/`.
> Record only traps that are easy to fall into, not module layout. File format changes get no compatibility code.

# Project conventions

* `vendor/` is official code pinned to a release: never edit it or run `git subtree pull`. Use it directly, and port only what cannot be used as is.
* `crates/` holds only product-agnostic foundation APIs. Product logic (virtual machines, robots, storage, UI flows) belongs to the app.
* Static assets used by the desktop UI belong in `apps/desktop/assets`; do not add UI static assets under `apps/desktop/src/assets`.
* Put a crate's boundary with other crates in a comment right under its Cargo `description`.
* In TypeScript, derive types from existing types and function return values; do not hand-write duplicate state or result types when inference can provide them.
* In Vue, bind event resources to the current scope: register them in setup/composables and clean them with `onScopeDispose` directly or through VueUse; never register them during rendering or ref callbacks.

# Commit rules

* Explain *why*; the diff already shows what changed.
* Keep the title short and imperative, using stable, searchable domain, module, or behavior keywords without implementation details.
* The body is optional; when needed, mention only user-visible behavior, important constraints, or compatibility information. Leave function names, file names, and config keys to the diff unless they are important search terms.
