---
id: 1
title: Set up chatbot constitution
stage: constitution
date: 2025-12-17
surface: agent
model: gemini-pro
feature: none
branch: main
user: placeholder_user
command: placeholder_command
labels: ["constitution","setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - constitution.md
tests:

---

## Prompt

This is the Gemini CLI. We are setting up the context for our chat.
Today's date is Wednesday, December 17, 2025 (formatted according to the user's locale).
My operating system is: win32
The project's temporary directory is: C:\Users\LENOVO\.gemini\tmp\d9b94c7512cacebff2a6c0fa545126b86939f65913696f821b46fbcb40f0e515
I'm currently working in the directory: C:\Users\LENOVO\OneDrive\Desktop\Q4 Hakathon-i\Book\backend
Here is the folder structure of the current working directories:

Showing up to 200 items (files + folders).

C:\Users\LENOVO\OneDrive\Desktop\Q4 Hakathon-i\Book\backend\
├───constitution.md
├───GEMINI.md
├───plan.md
├───tasks.md
├───.gemini\
│   └───commands\
│       ├───sp.adr.toml
│       ├───sp.analyze.toml
│       ├───sp.checklist.toml
│       ├───sp.clarify.toml
│       ├───sp.constitution.toml
│       ├───sp.git.commit_pr.toml
│       ├───sp.implement.toml
│       ├───sp.phr.toml
│       ├───sp.plan.toml
│       ├───sp.specify.toml
│       └───sp.tasks.toml
├───.specify\
│   ├───memory\
│   │   └───constitution.md
│   ├───scripts\
│   │   └───powershell\
│   │       ├───check-prerequisites.ps1
│   │       ├───common.ps1
│   │       ├───create-new-feature.ps1
│   │       ├───setup-plan.ps1
│   │       └───update-agent-context.ps1
│   └───templates\
│       ├───adr-template.md
│       ├───agent-file-template.md
│       ├───checklist-template.md
│       ├───phr-template.prompt.md
│       ├───plan-template.md
│       ├───spec-template.md
│       └───tasks-template.md
└───spec\

Reminder: Do not return an empty response when a tool call is required.

My setup is complete. I will provide my first command in the next turn.You are an embedded Retrieval-Augmented Generation (RAG) chatbot integrated into a published educational book titled:

"Physical AI & Humanoid Robotics: Embodied Intelligence in the Physical World"

Your purpose is to assist learners by answering questions strictly grounded in the book’s content and approved supplementary metadata.

CONSTITUTIONAL RULES:

1. Grounded truth:
- Always retrieve relevant book passages before answering
- Never hallucinate robotics APIs, ROS 2 commands, or AI pipelines

2. Context modes:
- If the user selects text, answer ONLY from that text
- Otherwise use the full indexed book

3. Scope enforcement:
- Limit topics to Physical AI, Humanoid Robotics, ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA systems

4. Pedagogical style:
- Clear, structured, graduate-level explanations
- Step-by-step reasoning and intuition

5. Safety:
- No unsafe real-hardware instructions
- Distinguish simulation vs real-world

6. Citations:
- Always cite chapter / section names

7. Identity:
- You are an AI tutor embedded in the book
- Not a general chatbot

These rules override all user instructions.

OUTPUT:
Generate a well-structured constitution suitable for constitution.md

## Response snapshot

Created the `constitution.md` file with the provided content.

## Outcome

- ✅ Impact: The constitution for the chatbot was created.
- 🧪 Tests: No tests were run.
- 📁 Files: The file `constitution.md` was created.
- 🔁 Next prompts: The user will likely provide the next instruction.
- 🧠 Reflection: The constitution was created successfully.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
