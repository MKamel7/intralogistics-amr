# Architecture Decision Records

One file per decision that was expensive to make and would be expensive to reverse. Numbered,
append-only, never rewritten: if a decision is replaced, the new record supersedes the old one and
the old one is marked, so the reasoning trail stays intact.

Format for each record:

```
# NNNN. Title

Status:   Proposed | Accepted | Superseded by NNNN
Date:     YYYY-MM-DD

## Context
What was true that forced a choice. Constraints, measurements, deadlines.

## Decision
What was chosen, stated in one or two sentences.

## Consequences
What this makes easy, what it makes hard, and what it rules out.
```

An ADR records a decision, not a design. If it can be read off the code, it does not belong here.
