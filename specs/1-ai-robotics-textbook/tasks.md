# SP.TASKS: Task Breakdown & Dependencies
## Project: AI-Native Textbook — Physical AI & Humanoid Robotics

---

## Overview
This document outlines the detailed, dependency-ordered tasks required to implement the AI-Native Textbook for Physical AI & Humanoid Robotics, based on `specs/1-ai-robotics-textbook/spec.md` and `specs/1-ai-robotics-textbook/plan.md`.

---

## 1. Core Content Generation (Phase 1: Foundation & Content)

### Dependencies
- **Constitution:** `.specify/memory/constitution.md` (for content and quality standards)
- **Specification:** `specs/1-ai-robotics-textbook/spec.md` (for module details, learning outcomes, minimum content, artifacts)
- **Plan:** `specs/1-ai-robotics-textbook/plan.md` (for overall timeline and tech context)

### Tasks
- [ ] **Write Intro Module (docs/00-intro.md)**
    - **Description**: Generate the introductory content including course overview, setup guide, hardware checklist.
    - **Acceptance Criteria**:
        - Adheres to content standards (minimum 2000 words, clear learning objectives, key takeaways).
        - Includes hardware checklist JSON and sample lab setup diagram description.
        - Covers how to use the RAG chatbot.
        - Basic queries about setup and requirements are answerable by future RAG system.
    - **Dependencies**: None
    - **Assigned Agent**: Claude Code (Content Generator Agent)

- [ ] **Write ROS 2 Module (docs/01-ros2.md)**
    - **Description**: Generate content for ROS 2 fundamentals, node architecture, topics, services, actions, URDFs, rclpy integration.
    - **Acceptance Criteria**:
        - Adheres to content standards (minimum 2000 words, Python code examples, hands-on exercise, learning objectives, key takeaways).
        - Includes minimal working code snippets for publisher/subscriber, example URDF, launch file example.
        - Code examples are testable and functional (Python 3.10+, ROS 2 Humble).
        - RAG queries for ROS 2 concepts return correct code and citations.
    - **Dependencies**: Intro Module
    - **Assigned Agent**: Claude Code (Content Generator Agent, Code Example Agent)

- [ ] **Write Gazebo Module (docs/02-gazebo.md)**
    - **Description**: Generate content for physics simulation, URDF/SDF, Gazebo plugins, sensor simulation, basic Unity integration.
    - **Acceptance Criteria**:
        - Adheres to content standards.
        - Includes Gazebo world example, sensor config (RealSense sample), troubleshooting tips.
        - Code/asset examples are functional.
        - RAG returns step-by-step commands to spawn a robot in Gazebo and config snippets.
    - **Dependencies**: ROS 2 Module
    - **Assigned Agent**: Claude Code (Content Generator Agent, Code Example Agent)

- [ ] **Write Isaac Module (docs/03-isaac.md)**
    - **Description**: Generate content for Isaac Sim basics, Isaac ROS, VSLAM, photorealistic sim, synthetic data generation.
    - **Acceptance Criteria**:
        - Adheres to content standards.
        - Includes Isaac Sim quickstart notes, Isaac ROS node pipeline skeleton, RTX requirements.
        - RAG gives correct resource links and setup steps for Isaac Sim.
    - **Dependencies**: Gazebo Module
    - **Assigned Agent**: Claude Code (Content Generator Agent)

- [ ] **Write VLA Module (docs/04-vla.md)**
    - **Description**: Generate content for integrating LLMs with perception + action, voice-to-action, multimodal planning.
    - **Acceptance Criteria**:
        - Adheres to content standards.
        - Includes example pipeline (pseudocode), sample prompt templates, safety constraints.
        - RAG returns valid prompt templates and pseudocode pipeline.
    - **Dependencies**: Isaac Module
    - **Assigned Agent**: Claude Code (Code Generator Agent, Code Example Agent)

- [ ] **Write Capstone Module (docs/05-capstone.md)**
    - **Description**: Generate content for bringing all concepts together for an autonomous humanoid integration.
    - **Acceptance Criteria**:
        - Adheres to content standards.
        - Includes documented end-to-end design, simplified demo instructions (simulated), checklist for sim-to-real.
        - RAG can list end-to-end steps and cite chapter fragments.
    - **Dependencies**: All previous content modules
    - **Assigned Agent**: Claude Code (Content Generator Agent)

- [ ] **Add diagrams and images to all modules**
    - **Description**: Integrate diagram descriptions and actual images (e.g., `static/images/diagrams/`, `hardware-checklist.png`).
    - **Acceptance Criteria**:
        - All referenced diagrams/images are present and correctly linked.
        - Image descriptions are clear and concise.
    - **Dependencies**: All content modules written.
    - **Assigned Agent**: Human (for image creation), Claude Code (for linking/description)

- [ ] **Review and Edit All Content**
    - **Description**: Perform a comprehensive review of all generated content for technical accuracy, consistency, pedagogical soundness, and adherence to constitution standards.
    - **Acceptance Criteria**:
        - Minimum 2000 words per chapter verified.
        - Python code examples in every technical chapter tested and functional.
        - Hands-on exercise at chapter end.
        - Learning objectives at chapter start.
        - Key takeaways summary.
        - Consistent terminology throughout.
        - All cross-references are valid.
        - No broken links.
    - **Dependencies**: All content and diagrams added.
    - **Assigned Agent**: Human, Claude Code (for automated checks like link validation)

---

## 2. Backend Development (Phase 2: Integration & Features)

### Dependencies
- **Specification:** `specs/1-ai-robotics-textbook/spec.md` (RAG chatbot spec, API requirements)
- **Plan:** `specs/1-ai-robotics-textbook/plan.md` (Tech stack, performance goals, constraints)
- **Data Model:** `specs/1-ai-robotics-textbook/data-model.md` (ContentBlock, UserProfile, TranslationCacheEntry)
- **API Contracts:** `specs/1-ai-robotics-textbook/contracts/openapi.yaml` (API endpoint definitions)

### Tasks
- [ ] **Create FastAPI Project Structure**
    - **Description**: Initialize the FastAPI project, set up basic directory structure, and configure environment variables.
    - **Acceptance Criteria**:
        - FastAPI application runs locally.
        - `main.py` (or equivalent) is set up.
        - Environment variables for API keys (Gemini, Qdrant) are configured securely.
    - **Dependencies**: None
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Implement `/embed` Endpoint**
    - **Description**: Develop an API endpoint to generate vector embeddings for text content using Gemini `text-embedding-004`.
    - **Acceptance Criteria**:
        - Endpoint `POST /embed` accepts `{"text": "..."}`.
        - Returns `{"embedding": [...]}` (list of floats).
        - Handles errors (e.g., empty text, API issues).
    - **Dependencies**: FastAPI Project Structure
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Implement `/chat` Endpoint (General Queries)**
    - **Description**: Develop the core RAG chatbot endpoint for general questions.
    - **Acceptance Criteria**:
        - Endpoint `POST /chat` accepts `{"query": "...", "history": [...]}`.
        - Queries Qdrant for top-k (k=5) relevant content blocks.
        - Constructs prompt (system + context + user) and calls Gemini Pro.
        - Returns `{"answer": "...", "sources": [...]}` with citations.
        - Handles conversation history.
    - **Dependencies**: FastAPI Project Structure, `/embed` Endpoint (for query embedding), Qdrant Integration
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Implement `/chat-selection` Endpoint (Strict Mode)**
    - **Description**: Develop an API endpoint to answer questions strictly based on selected text.
    - **Acceptance Criteria**:
        - Endpoint `POST /chat-selection` accepts `{"query": "...", "selected_text": "..."}`.
        - Instructs Gemini Pro to use *only* the `selected_text`.
        - Returns `{"answer": "...", "sources": [...]}` (source type "selected_text").
        - Returns "Answer not found in selected text" if unable to answer from selected text.
        - Returns "Selected text insufficient to answer..." for very short inputs.
    - **Dependencies**: FastAPI Project Structure, `/chat` Endpoint (for LLM interaction patterns)
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Implement Citation Format in Responses**
    - **Description**: Ensure all RAG responses from `/chat` include citations in the format `[chapter:heading|url|offset]`.
    - **Acceptance Criteria**:
        - Citations are correctly parsed from retrieved content metadata.
        - Citations are included in the LLM prompt.
        - LLM response contains correctly formatted citations.
    - **Dependencies**: `/chat` Endpoint, Qdrant Integration (payload metadata)
    - **Assigned Agent**: Claude Code (Code Review Agent - for verification)

- [ ] **(Optional) Implement Authentication Endpoints (`/auth/signup`, `/auth/signin`)**
    - **Description**: Develop API endpoints for user registration and login using Better-Auth and store user profiles in Neon Postgres.
    - **Acceptance Criteria**:
        - User can sign up with email, password, and profile questions.
        - User can sign in and receive an authentication token.
        - User data (profile) is securely stored in Neon Postgres.
    - **Dependencies**: FastAPI Project Structure, Neon Postgres Setup
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **(Optional) Implement Personalization Endpoint (`/personalize`)**
    - **Description**: Develop an API endpoint to adjust chapter content based on the user's profile.
    - **Acceptance Criteria**:
        - Endpoint accepts `chapter_id` and `user_id`.
        - Content is modified (simplified/deepened) based on `UserProfile` data.
        - Returns personalized content.
    - **Dependencies**: FastAPI Project Structure, Authentication Endpoints, UserProfile data in Neon Postgres
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **(Optional) Implement Translation Endpoint (`/translate`)**
    - **Description**: Develop an API endpoint to translate text content to Urdu using Gemini API, with caching.
    - **Acceptance Criteria**:
        - Endpoint accepts `text` and `target_language`.
        - Returns translated text.
        - Translations are cached in Neon Postgres to avoid repeated calls.
        - Code blocks are preserved (not translated).
    - **Dependencies**: FastAPI Project Structure, Neon Postgres Setup (for cache)
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Deploy Backend to Cloud Platform**
    - **Description**: Deploy the FastAPI application to a chosen cloud platform (e.g., Railway/Render/Vercel).
    - **Acceptance Criteria**:
        - Backend is accessible via a public URL.
        - All API endpoints are functional in production.
        - CORS is correctly configured for the Docusaurus frontend domain.
        - Basic monitoring/logging is in place.
    - **Dependencies**: All backend API endpoints implemented.
    - **Assigned Agent**: Human (for deployment config), Claude Code (for verification of endpoints)

---

## 3. RAG System Integration (Phase 1: Foundation & Content)

### Dependencies
- **Content:** All content modules written.
- **Backend Embed Endpoint:** `/embed` endpoint is functional.
- **Qdrant Cloud:** Qdrant Cloud instance is set up.

### Tasks
- [ ] **Preprocess Markdown to Plaintext Blocks**
    - **Description**: Write a script or function to convert chapter markdown files into plaintext blocks suitable for embedding.
    - **Acceptance Criteria**:
        - Script successfully parses all `.md` files in `docs/`.
        - Outputs clean text blocks, potentially with metadata.
    - **Dependencies**: All content modules written.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Chunk Text Content**
    - **Description**: Implement a text chunking strategy (e.g., ~400–600 tokens) for the plaintext blocks.
    - **Acceptance Criteria**:
        - Text blocks are chunked into appropriate sizes.
        - Overlapping chunks are considered if necessary for context.
    - **Dependencies**: Preprocess Markdown.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Generate Embeddings (Gemini)**
    - **Description**: Use the `/embed` endpoint to generate embeddings for all chunked content blocks using Gemini `text-embedding-004`.
    - **Acceptance Criteria**:
        - Embeddings are successfully generated for all content chunks.
        - The process is robust to API rate limits (if applicable).
    - **Dependencies**: Chunk Text Content, `/embed` Endpoint (functional).
    - **Assigned Agent**: Claude Code (Code Runner Agent)

- [ ] **Upsert Embeddings into Qdrant**
    - **Description**: Store the generated embeddings and their associated metadata (id, text, chapter, url, heading, token_count) into Qdrant Cloud.
    - **Acceptance Criteria**:
        - All embeddings are successfully ingested into Qdrant.
        - Qdrant collection is correctly configured.
        - Metadata fields are correctly populated.
    - **Dependencies**: Generate Embeddings.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Evaluate Sample RAG Queries**
    - **Description**: Test the RAG system with a set of sample queries to assess retrieval accuracy and LLM response quality.
    - **Acceptance Criteria**:
        - Chatbot answers context queries reliably (≥ 90% success on test prompts).
        - Selected-text answers show 100% fidelity.
        - Citations are accurate.
    - **Dependencies**: `/chat` and `/chat-selection` endpoints, Upsert Embeddings into Qdrant.
    - **Assigned Agent**: Human, Claude Code (for automated testing of RAG accuracy)

---

## 4. Frontend Integration (Phase 2: Integration & Features)

### Dependencies
- **Docusaurus Setup:** Basic Docusaurus site is running.
- **Backend API:** `/chat`, `/chat-selection`, (optional) `/auth`, `/personalize`, `/translate` endpoints are deployed and accessible.

### Tasks
- [ ] **Embed Chat UI Component in Docusaurus**
    - **Description**: Create a React chatbot component and integrate it into the Docusaurus site, potentially as a floating widget.
    - **Acceptance Criteria**:
        - Chatbot widget is visible on Docusaurus pages.
        - Basic UI allows typing and sending messages.
        - Connects to the `/chat` API endpoint.
    - **Dependencies**: Backend `/chat` endpoint.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Implement Selected-Text Detection**
    - **Description**: Add functionality to detect user-selected text on Docusaurus pages and enable a strict-mode query.
    - **Acceptance Criteria**:
        - User selection of text triggers an option to query based on that text.
        - Selected text is sent to the `/chat-selection` API endpoint.
    - **Dependencies**: Embed Chat UI Component, Backend `/chat-selection` endpoint.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **(Optional) Add User Authentication to Frontend**
    - **Description**: Integrate user signup/signin forms and profile page.
    - **Acceptance Criteria**:
        - Users can register and log in via the frontend.
        - User profile questions are presented during signup.
        - Session management is handled client-side (e.g., JWT storage).
    - **Dependencies**: Embed Chat UI Component, Backend Authentication Endpoints.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **(Optional) Add "Personalize Content" Button**
    - **Description**: Implement a button in chapters that triggers content personalization based on user profile.
    - **Acceptance Criteria**:
        - Button is present in chapters.
        - Clicking it sends `chapter_id` and `user_id` to `/personalize` endpoint.
        - Displays personalized content.
    - **Dependencies**: Embed Chat UI Component, Backend Personalization Endpoint, Frontend Authentication.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **(Optional) Add "Translate to Urdu" Button**
    - **Description**: Implement a button to translate the current chapter content to Urdu.
    - **Acceptance Criteria**:
        - Button is present in chapters.
        - Clicking it sends content to `/translate` endpoint.
        - Displays translated content, preserving code blocks.
    - **Dependencies**: Embed Chat UI Component, Backend Translation Endpoint.
    - **Assigned Agent**: Claude Code (Code Generator Agent)

- [ ] **Implement Style Improvements & UI/UX Polish**
    - **Description**: Enhance the overall visual design and user experience of the Docusaurus site and chatbot.
    - **Acceptance Criteria**:
        - Chatbot design is intuitive and visually appealing.
        - Loading states and animations are implemented.
        - Buttons and interactive elements are styled consistently.
        - Mobile responsiveness is ensured.
    - **Dependencies**: All frontend components integrated.
    - **Assigned Agent**: Human (for design review), Claude Code (for CSS/styling suggestions)

---

## 5. Finalization & Deployment (Phase 3: Polish & Deployment)

### Dependencies
- **All Content Modules:** Completed and reviewed.
- **Backend API:** All core and optional endpoints deployed.
- **Frontend UI:** All components integrated and styled.

### Tasks
- [ ] **Write Comprehensive README.md**
    - **Description**: Create a detailed `README.md` for the GitHub repository.
    - **Acceptance Criteria**:
        - Includes project overview, setup instructions, API documentation, screenshots.
        - Explains architecture and features.
    - **Dependencies**: All code and documentation in place.
    - **Assigned Agent**: Claude Code (Documentation Agent)

- [ ] **Add Architecture Diagram**
    - **Description**: Create and integrate an architecture diagram into the `README.md` and/or `static/images/diagrams/`.
    - **Acceptance Criteria**:
        - Diagram visually represents the system components and their interactions (frontend, backend, Qdrant, Neon, Gemini API).
        - Clearly illustrates data flow.
    - **Dependencies**: Overall system design understood.
    - **Assigned Agent**: Human (for design), Claude Code (for markdown integration)

- [ ] **Record 90-second Demo Video**
    - **Description**: Create a concise demo video showcasing the textbook and its features.
    - **Acceptance Criteria**:
        - Video is under 90 seconds.
        - Covers key features: navigation, chatbot query, selected text, (optional) auth/personalization/translation.
        - Clear narration (optional: NotebookLM).
    - **Dependencies**: All features functional and deployed.
    - **Assigned Agent**: Human

- [ ] **Publish GitHub Repository**
    - **Description**: Ensure the GitHub repository is public and correctly configured.
    - **Acceptance Criteria**:
        - Repository is publicly accessible.
        - All necessary files are committed.
    - **Dependencies**: All code and documentation finalized.
    - **Assigned Agent**: Human

- [ ] **Deploy Docusaurus to GitHub Pages**
    - **Description**: Configure and deploy the Docusaurus frontend to GitHub Pages.
    - **Acceptance Criteria**:
        - Book is accessible at its public URL.
        - All links and assets load correctly.
        - Frontend correctly interacts with the deployed backend.
    - **Dependencies**: All frontend code, GitHub repo published.
    - **Assigned Agent**: Human

- [ ] **Fill out Submission Form**
    - **Description**: Complete the hackathon submission form with all required details.
    - **Acceptance Criteria**:
        - All fields in the submission form are accurately filled.
        - Links to repo, book, and video are correct.
    - **Dependencies**: All project deliverables ready.
    - **Assigned Agent**: Human

- [ ] **Final Testing & Bug Fixes**
    - **Description**: Perform a full regression test, fix any critical bugs, and ensure cross-browser/device compatibility.
    - **Acceptance Criteria**:
        - 0 critical bugs at submission.
        - All features function as expected across target environments.
        - Performance goals are met.
    - **Dependencies**: All previous tasks completed.
    - **Assigned Agent**: Human, Claude Code (for automated testing)

---

**End of SP.TASKS**
