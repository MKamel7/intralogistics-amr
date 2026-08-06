# Requirements

Requirements live here as YAML with stable IDs, split by kind:

- `FR-nnn` functional
- `PR-nnn` performance
- `SR-nnn` safety
- `IR-nnn` interface

Each carries a rationale and a verification method (analysis, test, demonstration, inspection).
Each test names the requirement IDs it covers, and CI fails on a gap in either direction: a
requirement with no test, or a test claiming an ID that does not exist.

Empty for now. Requirements are written as each phase starts, not retrofitted at the end, because
retrofitted requirements only ever describe what was already built.
