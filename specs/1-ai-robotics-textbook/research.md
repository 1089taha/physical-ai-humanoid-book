# Research Findings: AI-Native Textbook — Physical AI & Humanoid Robotics

## 1. Core Technologies and Rationale

### Docusaurus (Frontend Framework)
- **Decision**: Docusaurus 3.x
- **Rationale**: Specified in `spec.md` and `constitution.md` as "simple, fast, ideal for docs + GitHub Pages" and a "Requirement".
- **Best Practices**:
    - Modular content organization for chapters and sections.
    - Utilize Markdown for content, supporting code blocks and diagrams.
    - Configure sidebar for easy navigation.
    - Optimize for fast page loads (<3s).
    - Ensure mobile responsiveness.

### FastAPI (Backend Framework)
- **Decision**: FastAPI (Latest)
- **Rationale**: Specified in `spec.md` and `constitution.md` as "minimal, async, well-suited for ML microservices".
- **Best Practices**:
    - Define clear API contracts for `/chat` and `/embed` endpoints.
    - Implement asynchronous operations for I/O-bound tasks (e.g., Qdrant, LLM calls).
    - Use Pydantic for request/response validation and serialization.
    - Implement robust error handling and logging.
    - Secure API with appropriate authentication (if implementing Better-Auth).

### Qdrant Cloud (Vector Database)
- **Decision**: Qdrant Cloud (free tier)
- **Rationale**: Specified in `spec.md` as a "free-tier vector DB; easy to integrate".
- **Best Practices**:
    - Design appropriate payload schema `{id, text, chapter, url, heading, token_count}` for efficient retrieval.
    - Optimize chunking strategy (e.g., ~400–600 tokens) during ingestion.
    - Implement efficient vector search with `top-k` (k=5) retrieval.
    - Monitor vector database performance and scaling.

### Gemini (LLM & Embeddings)
- **Decision**: Gemini Pro (text-generation) for LLM, Gemini text-embedding-004 for embeddings
- **Rationale**: Specified in `constitution.md` as "All inference must use Gemini Pro" and "All embeddings must use Gemini text-embedding-004". The `spec.md` also mentions "Gemini (or Claude) embeddings + LLM — best reliability for RAG; choose available provider."
- **Best Practices**:
    - Develop precise prompt templates for RAG, including system, context, and user instructions.
    - Implement strict selected-text mode logic to ensure answers are confined to provided text.
    - Manage API keys securely.
    - Monitor LLM usage and costs.

### Neon Postgres (Optional Database)
- **Decision**: Neon Postgres (Serverless free tier)
- **Rationale**: Optional, for user profiles and translation cache (`spec.md`, `constitution.md`).
- **Best Practices**:
    - Design schema for user profiles (programming experience, robotics experience, etc.).
    - Implement caching strategy for translations to reduce API calls.
    - Ensure secure database connections.

## 2. Testing Strategy

- **Unit Testing**: For backend logic, `pytest` is a suitable framework for Python (`Technical Context` in plan.md implies pytest by Python stack).
- **Integration Testing**: API endpoints (`/chat`, `/embed`, authentication, personalization, translation) should be thoroughly tested to ensure correct interaction between services.
- **Frontend Testing**: Functional testing of Docusaurus pages, chatbot UI, and interactive elements.
- **Content Quality**: Manual review for accuracy, consistency, and adherence to content standards (`constitution.md`).
- **RAG Accuracy**: Evaluate chatbot responses against a diverse set of test prompts, especially for context-based and selected-text queries.
- **Performance Testing**: Monitor page load times and API response times against defined SLAs.

## 3. Deployment Considerations

- **Frontend**: GitHub Pages for Docusaurus, leveraging GitHub Actions for CI/CD if optional.
- **Backend**: Cloud platforms like Railway, Render, or Vercel for FastAPI deployment, ensuring CORS configuration and environment variable management.
- **Monitoring/Logging**: Implement comprehensive logging for both frontend and backend, with appropriate monitoring for API health and performance.

## 4. Architectural Decision Records (ADR) - Evaluation

No new architecturally significant decisions were identified beyond what is already stipulated in the `spec.md` and `constitution.md`. The plan primarily details the implementation steps based on existing architectural choices.