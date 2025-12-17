---
id: 0001
title: Initialize Project Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.phr
labels: ["constitution", "project setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
# 🔵 SP.CONSTITUTION
### Physical AI & Humanoid Robotics — AI-Native Textbook (Spec-Driven Master Constitution)
### Author: Muhammad Taha Khan
### Date: December 5, 2025
### Version: 1.0

## 1. Purpose
The purpose of this Constitution is to define the **vision, principles, constraints, and objectives** for building an AI-Native textbook titled:

**“Physical AI & Humanoid Robotics”**

This project is developed for **Hackathon I**, using Spec-Kit Plus, Claude Code, and Docusaurus. The final deliverable is an AI-augmented, interactive, agent-powered textbook deployed publicly and equipped with an advanced RAG chatbot.

This Constitution governs **all subsequent spec files**, agents, tasks, and implementation decisions.

---

## 2. Vision
To build a **world-class, AI-Native, spec-driven robotics textbook**, combining:
- Humanoid robotics fundamentals (ROS2, Gazebo, Isaac Sim, VLA)
- Embodied intelligence principles
- State-of-the-art generative AI
- Interactive learning

This textbook must surpass traditional static course material and serve as:
- A learning resource
- A guidance platform
- An interactive intelligent system
- A reference book
- A future-ready competency development hub

---

## 3. Core Deliverables (from Hackathon PDF)
The project MUST deliver the following:

### ✔ **1. AI/Spec-Driven Book Creation**
- Built with **Docusaurus**
- Written & generated using **Claude Code + Spec-Kit Plus**
- Structured into clear chapters reflecting the course weekly breakdown
- Fully deployed on GitHub Pages or Vercel

### ✔ **2. RAG Chatbot Integrated into the Book**
- Built with:
  - FastAPI backend
  - Qdrant Cloud vector DB
  - Neon Serverless Postgres for user profiles (optional)
  - OpenAI Agents or ChatKit SDK
- Chatbot must:
  - Answer questions about any chapter
  - Cite sources
  - Answer questions strictly from selected text (required)

### ✔ **3. Public GitHub Repo + Deployed Book**
- Public repo
- CI/CD or manual deploy
- Live site link

### ✔ **4. Demo Video (≤90 seconds)**

---

## 4. Bonus Deliverables (optional but recommended)
Each bonus = +50 points.

### ⭐ Bonus A — Reusable Intelligence via Claude Code Sub-Agents
Define domain subagents:
- Chapter Summarizer Agent
- Concept Extractor Agent
- Quiz Generator Agent
- Code Synthesizer Agent
- Visualization Agent

### ⭐ Bonus B — Authentication via Better-Auth
Sign-up form with:
- Software experience
- Hardware experience
- Robotics background

### ⭐ Bonus C — Personalized Chapters
Personalize content based on user profile:
- Beginner
- Intermediate
- Advanced
- Hardware-focused
- Software-focused

### ⭐ Bonus D — Urdu Translation
Translate entire chapters on demand.

---

## 5. System Architecture (High-Level)

### 🔹 Frontend (Deployed Docusaurus Book)
- React-based
- Chapter pages stored in `/docs`
- ChatKit chat widget embedded

### 🔹 Backend (FastAPI Server)
- `/chat` endpoint for RAG
- `/embed` endpoint for embeddings
- `/auth` endpoints (if bonus added)

### 🔹 Vector Database (Qdrant Cloud)
Stores:
- Chapter chunks
- URLs
- Metadata
- Embeddings

### 🔹 Postgres Database (Neon)
Stores (if bonus added):
- user accounts
- personalization profiles
- translation cache

### 🔹 Agents (OpenAI / Claude)
- RAG answer generation
- Chapter personalization
- Translation agent
- Summaries & quizzes generation

---

## 6. Constraints
- Must use **Docusaurus**
- Must deploy to **GitHub Pages/Vercel**
- Must use **Spec-Kit Plus**
- Must use **Qdrant** + **FastAPI**
- Must embed **ChatKit** or Agent Builder UI
- Must complete within hackathon time frame

---

## 7. Output Quality Standards

### ✔ Content Standards
- Accurate, technical, concise
- Align with Physical AI & Humanoid Robotics course modules
- Include diagrams, examples, and code
- Must be AI-native: interactive, queryable

### ✔ Chatbot Standards
- Must answer questions correctly
- Must cite chapter sources
- Must support “selected text only answers”
- Must gracefully handle lack of information
- Must be ultra-fast (< 1.5s retrieval)

### ✔ UI Standards
- Clean and modern
- Mobile-friendly
- Intuitive chatbot interface

---

## 8. Guiding Principles
- **Clarity over complexity**
- **Minimum viable product first**
- **Spec-first, code-second**
- **Iterate with Claude Code**
- **Make the book useful, not verbose**
- **Every chapter must include an example or diagram**

---

## 9. Acceptance Criteria
The project is considered COMPLETE when:
1. Book deployed + accessible
2. Chapter structure complete
3. Chatbot fully functional
4. Selected-text answering works
5. RAG retrieval functional with citations
6. GitHub Repo & README complete
7. Demo video <90 seconds recorded
8. Submission form completed

---

## 10. Governance
- All changes must adhere to this Constitution.
- All specs (SP files) must flow from this Constitution.
- No development outside spec.

---

**End of SP.CONSTITUTION**
```

## Response snapshot

The project constitution was updated to version 1.0, replacing the template with the provided content. It details the project's purpose, vision, deliverables, architecture, constraints, quality standards, principles, acceptance criteria, and governance. Several template files require manual alignment.

## Outcome

- ✅ Impact: Project constitution initialized to v1.0, defining vision, principles, constraints, and objectives.
- 🧪 Tests: none
- 📁 Files: 1 file modified: .specify/memory/constitution.md
- 🔁 Next prompts: Review and update .specify/templates/plan-template.md, .specify/templates/spec-template.md, and .specify/templates/tasks-template.md to align with the new constitution.
- 🧠 Reflection: Ensure proper error handling for missing scripts and graceful fallback to agent-native tools for PHR creation.

## Evaluation notes (flywheel)

- Failure modes observed: The create-phr.sh script was not found, requiring manual PHR creation via agent-native tools.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate why `create-phr.sh` was not found and ensure its presence and executability in the environment.
