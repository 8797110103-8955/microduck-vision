import { createHash } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, readdirSync, statSync, writeFileSync } from "node:fs";
import { dirname, join, relative, resolve } from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const desktopRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repositoryRoot = resolve(desktopRoot, "..", "..");
const generatedRoot = join(desktopRoot, "generated");
const generatedTypes = join(generatedRoot, "types.ts");
const stampPath = join(generatedRoot, ".types.sha256");
const force = process.argv.includes("--force");

const sourceRoots = [
  join(desktopRoot, "src-tauri", "src"),
  join(desktopRoot, "src-tauri", "build.rs"),
  join(desktopRoot, "src-tauri", "Cargo.toml"),
  join(desktopRoot, "src-tauri", "Cargo.lock"),
  join(desktopRoot, "generate"),
  join(repositoryRoot, "crates"),
  join(repositoryRoot, "vendor", "microduck", "duck-control", "src"),
];

function filesUnder(path) {
  if (!existsSync(path)) return [];
  if (statSync(path).isFile()) return path.endsWith(".rs") || path.endsWith(".toml") ? [path] : [];
  const entries = readdirSync(path, { withFileTypes: true });
  return entries.flatMap((entry) => {
    const child = join(path, entry.name);
    return entry.isDirectory()
      ? filesUnder(child)
      : child.endsWith(".rs") || child.endsWith(".toml")
        ? [child]
        : [];
  });
}

const inputs = [...new Set(sourceRoots.flatMap(filesUnder))].sort();
const hash = createHash("sha256");
for (const input of inputs) {
  hash.update(relative(repositoryRoot, input).replaceAll("\\", "/"));
  hash.update("\0");
  hash.update(readFileSync(input));
  hash.update("\0");
}
const sourceHash = hash.digest("hex");

if (!force && existsSync(generatedTypes) && existsSync(stampPath)) {
  const previousHash = readFileSync(stampPath, "utf8").trim();
  if (previousHash === sourceHash) {
    console.log("generate-types: cached");
    process.exit(0);
  }
}

mkdirSync(generatedRoot, { recursive: true });
const result = spawnSync(
  "cargo",
  ["run", "--manifest-path", "src-tauri/Cargo.toml", "--bin", "generate-types"],
  { cwd: desktopRoot, stdio: "inherit" },
);
if (result.error) throw result.error;
if (result.status !== 0) process.exit(result.status ?? 1);
writeFileSync(stampPath, `${sourceHash}\n`);
console.log("generate-types: generated");
