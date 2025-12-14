---
id: 001
title: Initial Constitution Creation
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["project-setup", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
[]
---

## Prompt

You are writing an AI-native technical textbook titled:

“Physical AI & Humanoid Robotics — A Complete Coursebook for Embodied Intelligence”

Global Rules & Requirements:
1. Audience: Intermediate CS/AI learners with Python experience.
2. Purpose: Teach Physical AI through a complete pathway:
   - ROS 2 fundamentals
   - Gazebo simulations
   - Unity visualization
   - NVIDIA Isaac Sim & Isaac ROS
   - Vision-Language-Action (VLA)
   - Humanoid robot locomotion & manipulation
   - Conversational Robotics (Whisper, LLM planning, multimodal I/O)

3. Chapter Standards:
   - Every chapter must include:
     • Learning Objectives
     • Clear explanations
     • System diagrams (ASCII or described)
     • Python/ROS 2 code examples
     • Real-world robotics examples
     • Exercises & Quiz
     • A mini hands-on task

4. Writing Style:
   - Clear, structured, beginner-friendly, but technically rigorous.
   - Strong focus on practical robotics.
   - All examples runnable on:
       • Ubuntu 22.04
       • Jetson Orin Nano/NX
       • Isaac Sim workstations

5. Textbook Philosophy:
   - Connect digital AI → embodied intelligence.
   - Teach simulation-first, deploy-later workflow.
   - Build everything toward the final Capstone:
     “Autonomous Humanoid: Voice → Perception → Planning → Action in Simulation.”

6. Do NOT create vague content. Everything must be:
   - Technically correct
   - Industrially relevant
   - Directly applicable to the capstone

7. Output only the final constitution.

## Response snapshot

Created the project constitution based on your provided global rules and requirements. The constitution is located at `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: Constitution established for project guidance.
- 🧪 Tests: N/A
- 📁 Files: 1 file created
- 🔁 Next prompts: User will proceed with other project setup commands or feature development.
- 🧠 Reflection: Successfully established core project principles as per user input and SDD guidelines.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
