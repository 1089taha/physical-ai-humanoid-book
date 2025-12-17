# Data Model: AI-Native Textbook — Physical AI & Humanoid Robotics

This document outlines the key data entities and their structures for the AI-Native textbook project, derived from the feature specification (`spec.md`).

## 1. Content Block (for RAG Vector Store)

This entity represents a chunk of content from the textbook chapters, stored in the Qdrant vector database for retrieval-augmented generation (RAG).

- **Entity Name**: `ContentBlock`
- **Description**: A segment of text from a chapter, along with metadata for retrieval and citation.
- **Fields**:
    - `id` (string, UUID): Unique identifier for the content block. (Primary Key in Qdrant)
    - `text` (string): The actual text content of the block.
    - `embedding` (vector): Numerical representation of the `text` for vector search.
    - `chapter` (string): The name or title of the chapter the block belongs to.
    - `url` (string, optional): The URL path to the chapter or specific section.
    - `heading` (string, optional): The heading under which the text block falls.
    - `token_count` (integer): The number of tokens in the `text` field.
- **Relationships**: None (self-contained for vector search).
- **Validation Rules**:
    - `id` must be unique.
    - `text` should not be empty.
    - `embedding` must be a valid vector of the expected dimension.
    - `token_count` should accurately reflect the token count of `text`.

## 2. User Profile (Optional - for Personalization/Authentication)

This entity stores user-specific information, primarily for personalizing content and managing authentication. This data would typically reside in Neon Postgres.

- **Entity Name**: `UserProfile`
- **Description**: Information about a registered user, used for content tailoring and authentication.
- **Fields**:
    - `id` (string, UUID): Unique identifier for the user. (Primary Key in Neon Postgres)
    - `email` (string): User's email address (unique).
    - `programming_experience` (string, enum: `beginner`, `intermediate`, `advanced`): User's programming skill level.
    - `robotics_experience` (string, enum: `none`, `some`, `professional`): User's experience with robotics.
    - `python_proficiency` (string, enum: `beginner`, `intermediate`, `expert`): User's proficiency in Python.
    - `hardware_access` (string, enum: `simulation only`, `Jetson`, `full setup`): User's access to robotics hardware.
    - `created_at` (timestamp): Date and time when the profile was created.
    - `updated_at` (timestamp): Date and time when the profile was last updated.
- **Relationships**: None (self-contained).
- **Validation Rules**:
    - `id` must be unique.
    - `email` must be unique and a valid email format.
    - Enum fields must adhere to their predefined values.

## 3. Translation Cache Entry (Optional - for Translation Feature)

This entity stores cached translations of content blocks to improve performance and reduce repeated API calls to the translation service. This data would typically reside in Neon Postgres.

- **Entity Name**: `TranslationCacheEntry`
- **Description**: Stores a translated version of a content block for a specific language.
- **Fields**:
    - `original_text_hash` (string): A hash of the original English text content, used as a key for caching.
    - `language` (string): The target language of the translation (e.g., `Urdu`).
    - `translated_text` (string): The translated content.
    - `created_at` (timestamp): Date and time when the translation was cached.
- **Relationships**: None.
- **Validation Rules**:
    - `original_text_hash` and `language` together form a unique composite key.
    - `translated_text` should not be empty.
