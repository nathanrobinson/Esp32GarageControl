# Copilot Instructions for Esp32GarageControl

Purpose

- Guidance for GitHub Copilot / coding agents working on this repository.

Key rules

- Ignore all `libraries/` directories editing or making changes.
- Make minimal, targeted edits that preserve existing style and APIs.
- Prefer changes inside the three main projects: `CarRemote`, `DrivewaySensor`, `GarageSensor`.

Repository notes

- This workspace contains three ESP32 projects (projects target ESP32 microcontrollers).
- Build/test steps should be run locally with the user's ESP32 toolchain (Arduino CLI), not in the repo by the agent.

When creating or modifying files

- Keep changes small and focused; do not modify files under any `libraries/` folder.
- Update README.md only when adding project-level instructions or required steps.
- If adding new files, place them in the appropriate project subfolder.

Communication

- If uncertain about a design decision or risk of hardware impact, ask the user before proceeding.

Todo tracking

- Use the workspace todo list mechanism for multi-step tasks and mark items clearly.

Contact

- The repo owner is the active user; request clarification for anything involving hardware or deployment.
