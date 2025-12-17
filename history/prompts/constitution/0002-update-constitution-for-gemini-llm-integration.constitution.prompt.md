---
id: 0002
title: Update Constitution for Gemini LLM Integration
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.phr
labels: ["constitution", "gemini", "llm integration"]
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
# SP.CONSTITUTION
## Project: AI-Native Textbook — Physical AI & Humanoid Robotics
## Version: 1.1 (Updated: Gemini LLM Integration)
## Author: Muhammad Taha Khan
## Date: December 5, 2025

---

## 1. Purpose
This Constitution defines the principles, goals, constraints, and architecture governing the AI-Native textbook. It ensures all development follows Spec-Kit Plus methodology and uses the appropriate tools available to the author.

**Important Update:**
The author will use **Google Gemini LLM (via AI Studio)** for all runtime LLM operations because **Claude API is not purchased**.
Claude Code will be used only as a **local development assistant**, NOT as a runtime LLM.

---

## 2. Vision
To build a world-class, AI-powered interactive robotics textbook using:

- **Gemini Pro** for RAG answering
- **Gemini Embeddings** for vector search
- **Claude Code** for coding assistance & spec refinement
- **Spec-Kit Plus** for structured development
- **Docusaurus** for the textbook
- **FastAPI + Qdrant** for backend RAG stack

The final book must deliver an AI-native learning experience.

---

## 3. Core Deliverables
**No change** except LLM replaced with Gemini:

- Deployed Docusaurus book
- RAG chatbot using:
  - FastAPI
  - Qdrant Cloud
  - **Gemini Pro API (Free tier)**
  - Gemini Embeddings
- ChatKit or custom chat UI
- Public GitHub repo
- Demo video

---

## 4. Bonus Deliverables
Same as before, but updated:

- Subagents built with **Claude Code** (local)
- Personalization powered by Gemini
- Urdu translation via Gemini models
- Better-Auth
- Gemini-based summarizers / translators / quiz agents

---

## 5. System Architecture (Updated)

### 🟦 Frontend
Docusaurus + ChatKit UI

### 🟥 Backend
FastAPI
Gemini Pro API for RAG answering
Gemini embedding model:
- `"models/text-embedding-004"` (recommended)

### 🟩 Vector DB
Qdrant Cloud

### 🟨 Optional DB
Neon Serverless Postgres

### 🧠 AI Models
- **Gemini Pro** (text generation / RAG)
- **Gemini Embeddings**
- Claude Code (local IDE only)

---

## 6. Constraints (Updated)

- No Claude API usage
- No Anthropic access
- All LLM inference MUST use Gemini Pro
- All embeddings MUST use Gemini embeddings

---

## 7. Quality Standards
Same as before, but:

- Answers must follow **Gemini format + JSON response handling**
- Embedding dimension ~768 (Gemini’s standard)
- Pricing constraints: stay within **free tier limits**

---

## 8. Acceptance Criteria
Same as previous version.

---

**End of Updated SP.CONSTITUTION**
```

## Response snapshot

The project constitution was updated to version 1.1, incorporating Gemini LLM for all runtime operations and clarifying Claude Code's role as a local development assistant. This involved updates to the purpose, vision, deliverables, system architecture, constraints, and quality standards sections.

## Outcome

- ✅ Impact: Project constitution updated to v1.1, reflecting Gemini LLM integration and updated project details.
- 🧪 Tests: none
- 📁 Files: 1 file modified: .specify/memory/constitution.md
- 🔁 Next prompts: Review and update .specify/templates/plan-template.md, .specify/templates/spec-template.md, and .specify/templates/tasks-template.md to align with the new constitution.
- 🧠 Reflection: The PHR creation process needs to be more robust, especially for handling script failures and ensuring all placeholders are filled correctly.

## Evaluation notes (flywheel)

- Failure modes observed: Manual PHR creation was required due to the `create-phr.sh` script not being found. All placeholders were successfully filled manually.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure the PHR creation script is available and executable, or enhance the agent's internal logic for more seamless fallback to agent-native PHR generation.
