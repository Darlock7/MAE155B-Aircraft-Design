# CodeGen — Physics Code Writing Process

This is the process to follow every time new physics-based MATLAB code is written for this project. No exceptions.

---

## Phase 1 — RECEIVE

Read the prompt and identify:
- What physical phenomenon or engineering system is being modeled
- Which subsystem it belongs to (`aerodynamics`, `propulsion`, `geometry`, `stability`, `energy`, `economics`, `mission`)
- What existing functions in `src/` are already doing related work — read them before proceeding
- Which class slides are relevant — read them before writing any equations
- What inputs, outputs, and operating conditions are expected
- What is missing or ambiguous

If critical inputs or constraints are missing: ask before writing any code.  
If minor gaps exist: state the assumption explicitly and proceed.

---

## Phase 2 — PLAN

Before writing code, define:
- The governing physics: what equations will be implemented and where they come from (cite slide file)
- The function signature: name, inputs, outputs, units
- Where the file lives in the project structure
- What existing functions will be called or extended
- What assumptions are baked in and why they are valid for this project
- How the result will be validated

Do not proceed to Phase 3 until this is clear.

---

## Phase 3 — REFINE

Check the plan against:
- **Existing code** — does this duplicate or conflict with anything already in `src/`?
- **Class material** — do the equations match what was taught? If a different form is used, justify it.
- **Units** — are all quantities in SI? Are unit conversions explicit?
- **Teammate work** — does this interface correctly with functions written by others?
- **Physics sanity** — do the equations reduce to known limits? Are signs and directions consistent?

Fix the plan here, not mid-build.

---

## Phase 4 — BUILD

Write the function. Every new function must:
- Live in the correct `src/` subfolder
- Follow existing naming conventions (e.g., `camelCase`, verb-noun pattern)
- Include a header defining: purpose, inputs (name, unit, description), outputs (name, unit, description), assumptions
- Call existing project functions rather than reimplementing
- Be self-contained and runnable in isolation
- Include a suggested validation test or sanity check at the bottom (commented)

---

## Phase 5 — DONE

After delivering the code, append an `Engineering Check`:
- Assumptions that could be wrong given real operating conditions
- Physical edge cases or failure modes not handled
- Unit or dimensional analysis flags
- What should be validated before this function is trusted in the main pipeline

---

## Quick Reference

```
RECEIVE  →  understand the physics, read existing code and slides
PLAN     →  equations, signature, file location, validation strategy
REFINE   →  check against codebase, class theory, units, teammates
BUILD    →  write clean, modular, well-documented MATLAB
DONE     →  deliver + Engineering Check
```
