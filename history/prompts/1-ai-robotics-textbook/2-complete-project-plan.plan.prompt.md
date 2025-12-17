---
id: 2
title: Complete project plan
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-ai-robotics-textbook
branch: 1-ai-robotics-textbook
user: user
command: /sp.plan
labels: [plan, completion]
links:
  spec: specs/1-ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-ai-robotics-textbook/plan.md
  - specs/1-ai-robotics-textbook/research.md
  - specs/1-ai-robotics-textbook/data-model.md
  - specs/1-ai-robotics-textbook/contracts/openapi.yaml
  - CLAUDE.md
tests:

---

## Prompt

# Project Plan (sp.plan)
## Physical AI & Humanoid Robotics Textbook

---

## Project Timeline Overview

**Start Date:** December 4, 2025
**Deadline:** November 30, 2025, 6:00 PM
**Total Duration:** 26 days
**Presentation:** November 30, 2025, 6:00 PM (Zoom)

---

## Phase Breakdown

### Phase 1: Foundation & Content (Days 1-14)
**Goal:** Complete book structure and generate all chapters

### Phase 2: Integration & Features (Days 15-21)
**Goal:** Build RAG chatbot and implement bonus features

### Phase 3: Polish & Deployment (Days 22-26)
**Goal:** Testing, debugging, and submission preparation

---

## Detailed Daily Schedule

### Week 1: Setup & Core Content (Days 1-7)

#### Day 1 (Dec 4): Project Setup ✅
- [x] Install Docusaurus
- [x] Create project structure
- [x] Set up GitHub repository
- [x] Configure basic documentation
- [x] Create specification files

**Deliverable:** Working Docusaurus site with intro page

---

#### Day 2 (Dec 5): Module 1 Content Generation
**Focus:** ROS 2 Fundamentals (Chapters 1-3)

**Tasks:**
- [ ] Generate Chapter 1: Introduction to Physical AI (2500 words)
- [ ] Generate Chapter 2: Foundations of Physical AI (2500 words)
- [ ] Generate Chapter 3: Introduction to ROS 2 (2500 words)
- [ ] Review and edit for consistency
- [ ] Test all code examples

**Using Claude Code:**
```
Read sp.constitution.md, sp.specify.md, and sp.phr.md.

Generate Chapter 1: Introduction to Physical AI following all specifications.
Save to: docs/module-1-ros2/chapter-01-intro-physical-ai.md
```

**Deliverable:** 3 complete chapters with working examples

---

#### Day 3 (Dec 6): Module 1 Completion
**Focus:** ROS 2 Fundamentals (Chapters 4-5)

**Tasks:**
- [ ] Generate Chapter 4: ROS 2 Communication (2500 words)
- [ ] Generate Chapter 5: URDF and Robot Description (2500 words)
- [ ] Add diagrams descriptions
- [ ] Cross-reference between chapters
- [ ] Update navigation/sidebar

**Deliverable:** Complete Module 1 (5 chapters)

---

#### Day 4 (Dec 7): Module 2 Content
**Focus:** Simulation (Chapters 6-8)

**Tasks:**
- [ ] Generate Chapter 6: Introduction to Simulation (2500 words)
- [ ] Generate Chapter 7: Gazebo Physics Simulation (2500 words)
- [ ] Generate Chapter 8: Unity for Robotics (2500 words)
- [ ] Include URDF/SDF examples
- [ ] Add Gazebo launch file examples

**Deliverable:** Complete Module 2 (3 chapters)

---

#### Day 5 (Dec 8): Module 3 - Part 1
**Focus:** NVIDIA Isaac (Chapters 9-10)

**Tasks:**
- [ ] Generate Chapter 9: NVIDIA Isaac Platform Overview (2500 words)
- [ ] Generate Chapter 10: Isaac ROS for Perception (2500 words)
- [ ] Clarify Isaac Sim vs Isaac ROS
- [ ] Include hardware requirements section
- [ ] Add installation guides

**Deliverable:** First half of Module 3 (2 chapters)

---

#### Day 6 (Dec 9): Module 3 - Part 2
**Focus:** NVIDIA Isaac (Chapters 11-12)

**Tasks:**
- [ ] Generate Chapter 11: Nav2 Path Planning (2500 words)
- [ ] Generate Chapter 12: Reinforcement Learning (2500 words)
- [ ] Include bipedal locomotion examples
- [ ] Add sim-to-real transfer section
- [ ] Review entire module for consistency

**Deliverable:** Complete Module 3 (4 chapters)

---

#### Day 7 (Dec 10): Module 4 - Part 1
**Focus:** VLA (Chapters 13-14)

**Tasks:**
- [ ] Generate Chapter 13: Conversational Robotics (2500 words)
- [ ] Generate Chapter 14: Cognitive Planning with LLMs (2500 words)
- [ ] Include Whisper integration examples
- [ ] Add GPT API code examples
- [ ] Voice-to-action pipeline diagrams

**Deliverable:** First half of Module 4 (2 chapters)

---

### Week 2: Content Completion & Backend (Days 8-14)

#### Day 8 (Dec 11): Module 4 - Completion
**Focus:** VLA (Chapters 15-16)

**Tasks:**
- [ ] Generate Chapter 15: Humanoid Robot Development (2500 words)
- [ ] Generate Chapter 16: Capstone Project Guide (3000 words)
- [ ] Include complete project walkthrough
- [ ] Add evaluation rubric
- [ ] Review entire module

**Deliverable:** Complete Module 4 (4 chapters) + Full book content done!

---

#### Day 9 (Dec 12): Content Review & Polish
**Focus:** Quality assurance across all chapters

**Tasks:**
- [ ] Read through entire book sequentially
- [ ] Fix inconsistent terminology
- [ ] Verify all cross-references work
- [ ] Check all code examples compile
- [ ] Update table of contents
- [ ] Add missing diagrams descriptions

**Deliverable:** Polished, consistent content

---

#### Day 10 (Dec 13): RAG Backend - Architecture
**Focus:** Set up backend infrastructure

**Tasks:**
- [ ] Set up Neon Postgres database
- [ ] Create database schema for users/content
- [ ] Set up Qdrant Cloud account
- [ ] Create FastAPI project structure
- [ ] Configure environment variables
- [ ] Test database connections

**Deliverable:** Backend infrastructure ready

---

#### Day 11 (Dec 14): RAG Backend - Vector Database
**Focus:** Content embedding and storage

**Tasks:**
- [ ] Write script to extract text from all chapters
- [ ] Generate embeddings using OpenAI
- [ ] Upload embeddings to Qdrant
- [ ] Create vector search endpoint
- [ ] Test retrieval accuracy
- [ ] Optimize chunk size

**Deliverable:** Searchable vector database

---

#### Day 12 (Dec 15): RAG Backend - API Endpoints
**Focus:** Core chatbot functionality

**Tasks:**
- [ ] Create `/chat` endpoint (general questions)
- [ ] Create `/chat-selection` endpoint (selected text)
- [ ] Implement RAG pipeline (retrieve → augment → generate)
- [ ] Add conversation history handling
- [ ] Error handling and validation
- [ ] API documentation

**Deliverable:** Working RAG API

---

#### Day 13 (Dec 16): Backend Deployment
**Focus:** Deploy FastAPI backend

**Tasks:**
- [ ] Choose deployment platform (Railway/Render/Vercel)
- [ ] Configure production environment
- [ ] Set up CORS for Docusaurus domain
- [ ] Deploy backend
- [ ] Test API endpoints in production
- [ ] Set up monitoring/logging

**Deliverable:** Production API deployed

---

#### Day 14 (Dec 17): Frontend Chatbot Integration
**Focus:** Embed chatbot in Docusaurus

**Tasks:**
- [ ] Create React chatbot component
- [ ] Implement text selection detection
- [ ] Connect to backend API
- [ ] Style chatbot UI (floating widget)
- [ ] Add loading states
- [ ] Test on all chapters

**Deliverable:** Working embedded chatbot

---

### Week 3: Bonus Features (Days 15-21)

#### Day 15 (Dec 18): Authentication - Setup
**Focus:** Better-Auth integration (+50 bonus points)

**Tasks:**
- [ ] Install Better-Auth
- [ ] Configure authentication providers
- [ ] Create signup/signin pages
- [ ] Design user background questionnaire
- [ ] Set up session management
- [ ] Store user data in Neon

**Deliverable:** Working authentication system

---

#### Day 16 (Dec 19): Authentication - User Profiles
**Focus:** Collect and store user background

**Tasks:**
- [ ] Create signup form with background questions:
  - Programming experience (beginner/intermediate/advanced)
  - Robotics experience (none/some/professional)
  - Python proficiency (beginner/intermediate/expert)
  - Hardware access (simulation only/Jetson/full setup)
- [ ] Store responses in database
- [ ] Create user profile page
- [ ] Test authentication flow

**Deliverable:** Complete user profiling

---

#### Day 17 (Dec 20): Personalization Feature
**Focus:** Content personalization (+50 bonus points)

**Tasks:**
- [ ] Create "Personalize Content" button component
- [ ] Place button at start of each chapter
- [ ] Implement API endpoint to adjust content
- [ ] Based on user background, simplify or deepen explanations
- [ ] Cache personalized versions
- [ ] Test with different user profiles

**Example:**
```
Beginner: More analogies, simpler code
Advanced: Skip basics, add optimization tips
```

**Deliverable:** Working personalization

---

#### Day 18 (Dec 21): Translation Feature
**Focus:** Urdu translation (+50 bonus points)

**Tasks:**
- [ ] Create "Translate to Urdu" button
- [ ] Integrate OpenAI translation API
- [ ] Translate chapter on-demand
- [ ] Cache translations
- [ ] Handle code blocks (keep in English)
- [ ] Test translation quality

**Deliverable:** Working translation feature

---

#### Day 19 (Dec 22): Claude Code Subagents
**Focus:** Reusable intelligence (+50 bonus points)

**Tasks:**
- [ ] Document use of Claude Code in generation
- [ ] Create specialized subagents:
  - Content Generator Agent
  - Code Example Agent
  - Exercise Designer Agent
- [ ] Document agent skills created
- [ ] Show reusability across chapters

**Deliverable:** Subagent documentation

---

#### Day 20 (Dec 23): Feature Integration Testing
**Focus:** Test all features together

**Tasks:**
- [ ] Test authentication flow
- [ ] Test personalization with real users
- [ ] Test translation accuracy
- [ ] Test chatbot with different queries
- [ ] Test selected text feature
- [ ] Performance testing
- [ ] Mobile responsiveness

**Deliverable:** All features working together

---

#### Day 21 (Dec 24): UI/UX Polish
**Focus:** Visual improvements

**Tasks:**
- [ ] Improve chatbot design
- [ ] Add loading animations
- [ ] Improve button styling
- [ ] Add helpful tooltips
- [ ] Improve mobile experience
- [ ] Add dark mode support (if time)

**Deliverable:** Polished user interface

---

### Week 4: Final Polish & Submission (Days 22-26)

#### Day 22 (Dec 25): Deployment & Testing
**Focus:** Deploy to GitHub Pages

**Tasks:**
- [ ] Configure GitHub Pages deployment
- [ ] Set up custom domain (optional)
- [ ] Deploy production version
- [ ] Test all links work
- [ ] Test all features in production
- [ ] Fix any deployment issues

**Deliverable:** Live production site

---

#### Day 23 (Dec 26): Documentation & README
**Focus:** Project documentation

**Tasks:**
- [ ] Write comprehensive README.md
- [ ] Document architecture
- [ ] Create setup instructions
- [ ] Document API endpoints
- [ ] Add screenshots
- [ ] Create project overview

**Deliverable:** Complete project docs

---

#### Day 24 (Dec 27): Demo Video Creation
**Focus:** 90-second demo video

**Tasks:**
- [ ] Write video script (< 90 seconds)
- [ ] Record screen capture showing:
  - Book navigation
  - Chatbot query
  - Text selection + question
  - Authentication
  - Personalization
  - Translation
- [ ] Edit video
- [ ] Add narration (optional: use NotebookLM)
- [ ] Upload to YouTube/Vimeo

**Video Structure:**
```
0:00-0:10 - Intro and book overview
0:10-0:25 - Navigate chapters
0:25-0:40 - Chatbot demo
0:40-0:55 - Bonus features (auth, personalize, translate)
0:55-1:30 - Quick code walkthrough
```

**Deliverable:** Demo video uploaded

---

#### Day 25 (Dec 28): Final Testing & Bug Fixes
**Focus:** Last-minute polish

**Tasks:**
- [ ] Full regression testing
- [ ] Fix any critical bugs
- [ ] Test on different browsers
- [ ] Test on mobile devices
- [ ] Verify all links
- [ ] Check spelling/grammar

**Deliverable:** Bug-free application

---

#### Day 26 (Dec 29): Submission
**Focus:** Submit project

**Tasks:**
- [ ] Fill out submission form
- [ ] Submit GitHub repo link
- [ ] Submit published book URL
- [ ] Submit demo video link
- [ ] Provide WhatsApp number
- [ ] Double-check all submissions
- [ ] Prepare for potential live presentation

**Submission Checklist:**
- [x] GitHub repo is public
- [x] README is comprehensive
- [x] All features working
- [x] Demo video under 90 seconds
- [x] Book deployed and accessible

**Deliverable:** PROJECT SUBMITTED! 🎉

---

## Contingency Planning

### If Behind Schedule

**Priority 1 (Must Have):**
- Complete book content (16 chapters)
- Basic RAG chatbot
- Deployed to GitHub Pages

**Priority 2 (Should Have):**
- Text selection feature
- Polished UI

**Priority 3 (Nice to Have):**
- Authentication
- Personalization
- Translation
- Claude Code Subagents

### Time Buffers
- 2 days built in for unexpected issues
- Can compress feature development if needed
- Can simplify bonus features to basic versions

---

## Daily Checklist Template

**Each Day:**
- [ ] Morning: Review day's goals
- [ ] Work in focused 2-hour blocks
- [ ] Commit code regularly to GitHub
- [ ] Document decisions and changes
- [ ] Evening: Review progress, adjust plan
- [ ] Update this plan if needed

---

## Success Metrics

### Quantitative
- [x] 16+ chapters completed
- [ ] 100% code examples working
- [ ] RAG chatbot >80% accuracy
- [ ] <3s page load time
- [ ] 0 critical bugs at submission

### Qualitative
- [ ] Content is clear and educational
- [ ] Chatbot provides helpful answers
- [ ] UI is intuitive and polished
- [ ] Demo video is compelling
- [ ] Ready for live presentation

---

## Risk Management

### Risk 1: Content Generation Slow
**Mitigation:** Use Claude Code efficiently, batch generate similar chapters

### Risk 2: RAG Accuracy Low
**Mitigation:** Start early, iterate on prompts and retrieval

### Risk 3: Deployment Issues
**Mitigation:** Deploy early, test continuously

### Risk 4: Feature Creep
**Mitigation:** Stick to priorities, bonus features are optional

---

## Resources & Tools

### Development
- Claude Code (content generation)
- VS Code (coding)
- GitHub (version control)
- Docusaurus (framework)

### Backend
- FastAPI (API framework)
- Neon Postgres (database)
- Qdrant Cloud (vector database)
- OpenAI API (embeddings, chat)

### Deployment
- GitHub Pages (hosting)
- Railway/Render (backend hosting)

### Testing
- Browser DevTools
- Postman (API testing)
- Mobile device testing

---

## Communication Plan

### If Invited to Present
- Prepare 5-minute presentation
- Demo key features live
- Have backup plan if live demo fails
- Emphasize unique aspects
- Show technical depth

---

**Project Plan Version:** 1.0
**Last Updated:** December 4, 2025
**Status:** IN PROGRESS - Day 1 Complete ✅

## Response snapshot

The implementation plan has been successfully updated in `D:\coding\AI Driven AI Native Development\hackathon-project\specs\1-ai-robotics-textbook\plan.md`.

The `research.md` file has been created at `D:\coding\AI Driven AI Native Development\hackathon-project\specs\1-ai-robotics-textbook\research.md`.

The `data-model.md` file has been created at `D:\coding\AI Driven AI Native Development\hackathon-project\specs\1-ai-robotics-textbook\data-model.md`.

The `contracts` directory and `openapi.yaml` file have been created at `D:\coding\AI Driven AI Native Development\hackathon-project\specs\1-ai-robotics-textbook\contracts\openapi.yaml`.

The agent context in `CLAUDE.md` has been updated.

**Branch**: `1-ai-robotics-textbook`
**Implementation Plan Path**: `D:\coding\AI Driven AI Native Development\hackathon-project\specs\1-ai-robotics-textbook\plan.md`
**Generated Artifacts**:
- `specs/1-ai-robotics-textbook/research.md`
- `specs/1-ai-robotics-textbook/data-model.md`
- `specs/1-ai-robotics-textbook/contracts/openapi.yaml`

## Outcome

- ✅ Impact: The project planning phase (Phase 0 and Phase 1) was successfully completed, including the creation/update of plan, research, data model, and API contract documents, and the agent context. All documents align with the constitution and spec, and user clarifications were addressed.
- 🧪 Tests: N/A
- 📁 Files: `specs/1-ai-robotics-textbook/plan.md`, `specs/1-ai-robotics-textbook/research.md`, `specs/1-ai-robotics-textbook/data-model.md`, `specs/1-ai-robotics-textbook/contracts/openapi.yaml`, `CLAUDE.md` were created/updated.
- 🔁 Next prompts: Proceed to task generation based on the plan, review and update dependent templates as per the Sync Impact Report from the constitution update, and then exit plan mode.
- 🧠 Reflection: Successfully executed the planning workflow for `/sp.plan`. Handled the user's clarification regarding the use of OpenAI Agent SDK with Gemini API key by ensuring the documentation reflects this nuance. The process was iterative and adaptive to user feedback.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: N/A
