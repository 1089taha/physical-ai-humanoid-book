# Feature Specification: AI-Native Textbook — Physical AI & Humanoid Robotics

**Feature Branch**: `1-ai-robotics-textbook`
**Created**: 2025-12-05
**Status**: Draft
**Input**: User description: "Functional & Module-Level Specification for AI-Native Textbook — Physical AI & Humanoid Robotics, including module-level detail and updated to include Gemini LLM Integration."

---

## Executive Summary
This document specifies the complete functional requirements and module breakdown for the AI-Native textbook. It defines per-module content, tech stack, outputs, acceptance criteria, data models, RAG behavior, UX flows, API contracts, and risk mitigations. The specification is purpose-built to satisfy hackathon core requirements and maximize points from bonus features.

---

## Table of Contents
1. Goals & Success Metrics
2. High-level Product Features
3. Tech Stack & Rationale
4. Module-by-Module Specification (Module 1 → Module 4 + Intro & Capstone)
5. RAG Chatbot Spec & Selected-Text Behavior
6. API & Data Schemas
7. UI / UX Requirements
8. Deployment, CI, and Deliverables
9. Non-Functional Requirements & Performance Targets
10. Risks & Mitigations
11. Acceptance Criteria (per module + global)

---

## Project Structure

```
physical-ai-book/
├─ docs/
│  ├─ 00-intro.md
│  ├─ 01-ros2.md
│  ├─ 02-gazebo.md
│  ├─ 03-isaac.md
│  ├─ 04-vla.md
│  ├─ 05-capstone.md
├─ static/
│  ├─ images/
│  │  ├─ diagrams/
│  │  └─ hardware-checklist.png
├─ examples/
│  ├─ ros2/
│  │  ├─ publisher.py
│  │  └─ subscriber.py
│  ├─ gazebo/
│  │  └─ world.sdf
│  ├─ isaac/
│  └─ vla/
├─ src/
│  ├─ components/
│  │  └─ ChatWidget.jsx
│  ├─ pages/
│  │  └─ chat.js
├─ docusaurus.config.js
├─ package.json
```

---

## 1. Goals & Success Metrics
**Primary Goal:** Deliver a deployed, spec-driven Docusaurus book augmented with a RAG chatbot that can answer questions about the content and answer strictly from selected text when requested.

**Success Metrics (judge-facing):**
- Book deployed and accessible: ✅
- Chatbot answers context queries reliably: ≥ 90% success on test prompts.
- Selected-text answers: 100% fidelity (if answer exists in text, reply from it; otherwise answer “Answer not found in selected text”).
- Load time for pages < 2s (desktop).
- Backend single-query end-to-end < 2s (excluding LLM latency).
- Repo contains README, architecture diagram, and demo video link.

---

## 2. High-level Product Features
- Docusaurus-based book, modular chapters, code samples, diagrams.
- FastAPI backend exposing `/chat` and `/embed` endpoints.
- Qdrant vector store for retrieval.
- Embeddings via Gemini or Claude (configurable).
- ChatKit embed or custom React chat component.
- Selected-text-only mode with strict instruction to LLM.
- Optional: Better-Auth, Neon Postgres for personalization, Urdu translation agent.

---

## 3. Tech Stack & Rationale
- **Docusaurus** — simple, fast, ideal for docs + GitHub Pages. (Requirement)
- **FastAPI** — minimal, async, well-suited for ML microservices.
- **Qdrant Cloud** — free-tier vector DB; easy to integrate.
- **Gemini (or Claude) embeddings + LLM** — best reliability for RAG; choose available provider.
- **Neon Postgres** (optional) — serverless, store user profiles, translation cache.
- **ChatKit/Agent Builder** — easiest embed for judge-visible chat; fallback to simple React chat.
- **Spec-Kit Plus + Claude Code** — content generation and subagents.
- **GitHub Actions** — optional CI for deploy to gh-pages.

---

## 4. Module-by-Module Specification
Each module section below contains: Overview, Learning Outcomes, Minimum Content Items, Code Examples / Artifacts, RAG Metadata Tags, Acceptance Criteria.

### Intro — Course Overview & How to Use the Book
**Overview:** Introduce Physical AI, course structure, lab setup, and hacker’s quick start.
**Learning Outcomes:** Understand course goals, required hardware, and expected deliverables.
**Minimum Content Items:** Course roadmap, hardware checklist (Jetson, RealSense, RTX requirements), cloud vs local tradeoffs, how to use the RAG chatbot.
**Artifacts:** hardware checklist JSON, sample lab setup diagram image (placeholder).
**RAG Tags:** `intro`, `setup`, `lab`, `requirements`
**Acceptance:** Page deploys, basic queries answered (e.g., “What GPU do I need?”)

---

### Module 1 — The Robotic Nervous System (ROS 2)
**Overview:** ROS 2 fundamentals, node architecture, topics, services, actions, URDFs, rclpy integration.
**Learning Outcomes:**
- Build simple ROS 2 nodes in Python (rclpy).
- Define and load URDF models.
- Publish/subscribe to topics; implement services and actions.
**Minimum Content Items:**
- Short conceptual intro + 2 minimal working code snippets: node publisher, node subscriber (rclpy).
- Example URDF snippet for a simple humanoid limb (skeleton).
- Launch file example (YAML or .launch.py).
**Code Examples / Files:**
- `examples/ros2/publisher.py`
- `examples/ros2/subscriber.py`
- `examples/ros2/robot.urdf`
**RAG Tags / Payload Metadata:** `ros2`, `urdf`, `rclpy`, `launch`, `topics`
**Acceptance Criteria:** The content includes working code blocks, and RAG answers to questions like “How do I create a publisher in ROS 2?” must return correct code snippets and cite the chapter.

---

### Module 2 — The Digital Twin (Gazebo & Unity)
**Overview:** Physics simulation, URDF/SDF, Gazebo plugins, sensor simulation, basic Unity integration for high-fidelity rendering.
**Learning Outcomes:**
- Create SDF/URDF-compatible worlds.
- Simulate sensors (LiDAR, Depth, IMU).
- Attach plugins and test robot models.
**Minimum Content Items:**
- Gazebo world example + instructions to run.
- Sensor config: RealSense sample config.
- Troubleshooting tips for physics instability.
**Code / Assets:**
- `examples/gazebo/world.sdf`
- `examples/gazebo/realsense_config.yaml`
**RAG Tags:** `gazebo`, `sdf`, `sensors`, `simulation`
**Acceptance Criteria:** RAG returns step-by-step commands to spawn a robot in Gazebo and config snippets when asked.

---

### Module 3 — The AI-Robot Brain (NVIDIA Isaac™)
**Overview:** Isaac Sim basics, Isaac ROS, VSLAM, photorealistic sim, synthetic data generation.
**Learning Outcomes:**
- Run Isaac Sim scenes and integrate with ROS 2.
- Generate synthetic datasets; basic VSLAM pipeline.
- Use Nav2 for path planning basics.
**Minimum Content Items:**
- Isaac Sim quickstart notes and requirements.
- Example of Isaac ROS node pipeline skeleton.
- Notes about RTX requirement and cloud alternative.
**Artifacts:** `examples/isaac/isaac_launch.sh`
**RAG Tags:** `isaac`, `vslam`, `nav2`, `isaacsim`
**Acceptance Criteria:** RAG gives correct resource links and procedural steps to set up Isaac Sim or cloud alternative.

---

### Module 4 — Vision-Language-Action (VLA)
**Overview:** Integrating LLMs with perception + action: voice-to-action (Whisper), multimodal planning, translating natural language to ROS actions.
**Learning Outcomes:**
- Use Whisper for speech-to-text.
- Build a pipeline: (speech → LLM → planner → ROS action).
- Implement safety checking & action filters.
**Minimum Content Items:**
- Example pipeline with pseudocode.
- Sample prompt templates for planning (system + user instructions).
- Safety constraints list.
**Code / Templates:**
- `examples/vla/pipeline.py` (pseudocode)
- Prompt templates in `examples/vla/prompt_templates.md`
**RAG Tags:** `vla`, `whisper`, `planner`, `safety`
**Acceptance Criteria:** RAG returns a valid prompt template and a short pseudocode pipeline when prompted.

---

### Capstone — Autonomous Humanoid (Integration)
**Overview:** Bring it all together: voice command → plan → perception → navigation → manipulation.
**Deliverables:** A documented end-to-end design, simplified demo instructions (simulated), and checklist for sim-to-real.
**Acceptance Criteria:** RAG can list the end-to-end steps and cite chapter fragments for each step.

---

## 5. RAG Chatbot Specification & Selected-Text Behavior
This section describes how the RAG system should behave, the prompt engineering, and selected-text semantics.

### RAG Pipeline Overview
1. **Ingest**: Convert chapter markdown files → plaintext blocks; chunk (~400–600 tokens).
2. **Embed**: Compute embeddings (OpenAI or Claude).
3. **Store**: Upsert into Qdrant with payload `{id, text, chapter, url, heading, token_count}`.
4. **Query**: At chat time, embed query and search Qdrant for top-k (k = 5).
5. **Construct Prompt**: Build system + context + user prompt. Include citations in the form `[chapter:heading|url|offset]`.
6. **LLM**: Call model to answer and include explicit citations in the response.

### Selected-Text Mode (Strict)
- If frontend sends `selected_text` with the chat request, backend MUST:
  1. Immediately create a prompt that instructs the model:
     `"You MUST ONLY use the provided selected text to answer. If the selected text does not contain sufficient information to answer, you MUST say: 'Answer not found in selected text'."`
  2. Do NOT call Qdrant; the selected_text is the sole context.
  3. Return `sources: [{type: "selected_text", chapter: <chapter-if-available-or-null>, excerpt: <first 150 chars>}]`.

**Edge Cases:** If selected_text is too short to answer (> 20 chars recommended minimum), reply: `"Selected text insufficient to answer. Please provide more context or ask without selecting text."`

### Prompt Template (example)
