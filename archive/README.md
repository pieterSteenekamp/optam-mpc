# OptAM-MPC Archive

This folder contains snapshots of the project at various development stages.

## Purpose

- Complete development history for all contributors
- Safety net before major pruning or refactoring
- Reference for recovering over-pruned files
- Understanding of project evolution

## Folder Naming Convention

Each archive folder is named: `timestamp-description`

Example: `20260820-123456-before-linear-mpc-refactor`

## How to Create a New Archive

Run from the project root:

```batch
archive-snapshot.bat "description"
```

## Current Archives

1. `1-initial-working-version` - First complete working version
   - Created: 20 August 2026
   - Includes: MPC controller, process models, OPC UA, digital twin, tests
