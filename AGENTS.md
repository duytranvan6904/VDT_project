# Senior Engineering & Review Protocol - VDT Project

This repository contains critical robotics, simulation, and embedded control software (ROS 2/ROS, SMC Guidance, Finite State Machine, Drone Simulation). All agent contributions must adhere to the following senior-level standards:

## 1. Core Engineering Mindset
- **Safety & Robustness First**: In robotics/control algorithms, edge cases cause physical crashes. Always inspect numeric bounds, division-by-zero, `NaN`/`Inf` in mathematical calculations (trigonometry, matrix inversions, sliding surfaces).
- **Separation of Concerns**: Core mathematical/control algorithms (e.g. `embedded_handover/algorithms/smc_guidance.py`) must remain decoupled from communication frameworks (ROS/simulation wrappers) whenever feasible.
- **Root Cause Analysis (RCA)**: Never apply superficial "monkey patches" or mask symptoms. Understand the underlying state transition or math formulation before proposing fixes.

## 2. Code Review & Refactoring Protocol
When reviewing or modifying code:
1. **Categorize findings by severity**:
   - `[Blocker]`: Architectural defects, race conditions, memory leaks, unhandled mathematical singularities, or breaking API changes.
   - `[Warning]`: Suboptimal real-time performance, missing type hints/docstrings, loose state machine boundaries.
   - `[Suggestion/Nitpick]`: Formatting, naming consistency, minor readability enhancements.
2. **Review Checklist**:
   - State Machine: Are all transitions explicit? Are there deadlock or unhandled timeout states?
   - Concurrency & ROS: Are callbacks thread-safe? Are rate limits and message queues sized appropriately?
   - Real-time Constraints: Avoid heavy allocations, synchronous I/O, or blocking sleeps inside hot control loops.

## 3. Long-term Knowledge & Memory Retention
- When significant architectural decisions, interface agreements, or tricky bug resolutions are reached, persist them into the Knowledge Graph Memory tool and update project handover documentation (`docs/HANDOVER_CHANGES_AND_REMAINING_ISSUES.md`).
- For multi-step architectural or math-heavy planning, leverage the `sequential-thinking` protocol to reason methodically before execution.
