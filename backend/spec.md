# Specification: Embedded RAG Chatbot for "Physical AI & Humanoid Robotics"

**Version:** 1.0
**Status:** Proposed
**Author:** Gemini AI Systems Architect

## 1. System Overview

### 1.1. Purpose

This document specifies the requirements for an embedded Retrieval-Augmented Generation (RAG) chatbot. The chatbot will serve as a pedagogical assistant within the digital version of the book, "Physical AI & Humanoid Robotics: Embodied Intelligence in the Physical World," providing learners with accurate, context-aware answers grounded exclusively in the book's content.

### 1.2. Core Paradigm

The system will employ a strict **retrieval-first RAG architecture**. All answers must be synthesized directly from retrieved text chunks from the book. The system is prohibited from answering questions if no relevant context can be found, thereby minimizing hallucination and ensuring high fidelity to the source material.

### 1.3. Interaction Modes

The chatbot will support two primary user interaction modes:
1.  **Global QA:** The user asks a question in a general chat interface. The system searches the entire book.
2.  **Selected-Text QA:** The user highlights a specific passage in the book and asks a question related to it. The system's context is strictly limited to the selected text.

## 2. System Architecture

The architecture is designed to be modular, scalable, and cloud-native, with a clear separation between the backend service and the frontend UI.

### 2.1. Architectural Layers

1.  **Ingestion Layer:** A set of scripts and services responsible for processing source documents (PDF, Markdown), chunking them, and generating embeddings. This layer runs offline or on-demand.
2.  **Storage Layer:** Comprises a vector database for storing text embeddings and a relational database for storing chunk metadata.
3.  **Retrieval Layer:** A backend service (FastAPI) that accepts user queries, embeds them, and retrieves relevant text chunks from the storage layer.
4.  **Generation Layer:** Orchestrates the interaction with the Gemini model. It constructs a detailed prompt containing the user's question and the retrieved context, receives the LLM's response, and ensures citations are present.
5.  **Frontend Layer:** The user-facing UI embedded within the book's web platform. It captures user input, displays the conversation, and handles text selection events.

### 2.2. Technology Stack

| Component | Technology | Rationale |
| :--- | :--- | :--- |
| Backend Framework | FastAPI (Python) | High performance, async support, and excellent for building RESTful APIs. |
| Vector Database | ChromaDB | Open-source, easy to self-host or use in-memory for rapid development. FAISS is a viable alternative for high-performance needs. |
| Metadata Store | PostgreSQL | Robust, reliable, and provides rich querying capabilities for structured metadata. |
| Embeddings Model | Gemini `text-embedding-gecko` (or latest) | State-of-the-art model optimized for semantic retrieval. |
| Generation Model | Gemini 1.5 Pro | Advanced, large context window model suitable for grounded synthesis and following complex instructions. |
| Frontend | (To be defined) | Decoupled from the backend; can be implemented in any modern web framework (e.g., React, Vue). |

## 3. Data Model & Ingestion

### 3.1. Content Chunking Strategy

- **Source Formats:** The ingestion pipeline must handle PDF and Markdown files.
- **Chunking:** Content will be segmented into semantic chunks (paragraphs, list items, table rows).
- **Chunk Size:** Chunks should target a size of **300-500 tokens** to balance context density and retrieval precision.
- **Overlap:** A small overlap of ~50 tokens between chunks will be maintained to preserve contextual continuity.

### 3.2. Metadata Schema

Each text chunk will be associated with a rich set of metadata to enable precise filtering and citation.

| Field | Type | Description | Example |
| :--- | :--- | :--- | :--- |
| `chunk_id` | UUID | Unique identifier for the chunk. | `c4a1b2c3-d4e5-f6a7-b8c9-d0e1f2a3b4c5` |
| `text` | TEXT | The raw text content of the chunk. | "ROS 2 uses a publish-subscribe pattern..." |
| `embedding`| VECTOR | The numerical vector representation of the text. | `[0.12, -0.45, ...]` |
| `book_title`| VARCHAR(255) | Title of the source book. | "Physical AI & Humanoid Robotics" |
| `edition` | VARCHAR(50) | Book edition. | "First Edition" |
| `chapter` | VARCHAR(255) | Chapter title or number. | "Chapter 3: ROS 2 Architecture" |
| `section` | VARCHAR(255) | Section title or number. | "3.2 Nodes and Topics" |
| `page_number`| INTEGER | Page number in the source PDF. | 42 |
| `content_type`| ENUM | Type of content in the chunk. | `text`, `figure_caption`, `table` |
| `is_simulation`| BOOLEAN | True if the content refers to simulation, False for real hardware. | `true` |

## 4. RAG Pipeline Logic

1.  **Query Reception:** The FastAPI backend receives a query from the frontend via a REST endpoint. The request payload indicates whether it is a `global` or `selected-text` query.
2.  **Query Embedding:** The user's question is converted into a vector embedding using the same model as the document chunks.
3.  **Context Retrieval:**
    - If `selected-text`, the provided text is used directly as context.
    - If `global`, the query embedding is used to perform a similarity search against the vector database (ChromaDB) to retrieve the top-k (e.g., k=5) most relevant text chunks.
4.  **Prompt Engineering:** A system prompt is constructed for the Gemini model. This prompt enforces all constitutional rules, including:
    - The agent's persona (AI tutor).
    - The instruction to answer *only* from the provided context.
    - The requirement to cite sources for every statement.
    - The retrieved chunks are clearly demarcated within the prompt.
5.  **Answer Generation:** The complete prompt is sent to the Gemini API. The model synthesizes an answer based on the provided context.
6.  **Citation Validation:** The backend service performs a final check to ensure the generated answer includes citations. If not, it can append them or (in a more advanced implementation) ask the model to regenerate the answer with citations.
7.  **Response Delivery:** The final, validated answer is sent back to the frontend.

## 5. API Endpoints

The core backend service will expose the following RESTful endpoints:

| Endpoint | Method | Description |
| :--- | :--- | :--- |
| `/query` | POST | Handles global QA. Expects a JSON payload with the user's question. |
| `/query-selection`| POST | Handles selected-text QA. Expects the user's question and the selected text. |
| `/health` | GET | A simple health check endpoint that confirms service availability. |

## 6. Functional Requirements

- **FR-1:** The system must accept natural language user questions.
- **FR-2:** The system must correctly distinguish between global and selected-text context modes.
- **FR-3:** The ingestion pipeline must successfully process and chunk both PDF and Markdown files.
- **FR-4:** The system must generate and store a unique embedding for each content chunk.
- **FR-5:** The retrieval system must return the top-k most semantically similar chunks for any given query.
- **FR-6:** All generated answers must be verifiably grounded in the retrieved context chunks.
- **FR-7:** Every answer must include precise citations to the `chapter` and `section` from which the information was sourced.

## 7. Non-Functional Requirements

- **NFR-1 (Low Hallucination):** Hallucination rate must be near zero. The system should refuse to answer rather than invent information.
- **NFR-2 (Determinism):** For a given question and book version, the generated answer should be highly consistent and explainable by referencing the retrieved context.
- **NFR-3 (Latency):** Retrieval of context chunks (p99) must complete in **< 1 second**. The end-to-end response time will be dependent on the LLM's generation speed.
- **NFR-4 (Scalability):** The ingestion pipeline and database architecture must be able to support new book editions and supplementary materials with minimal reconfiguration.

## 8. Safety & Scope Guardrails

- **SS-1 (Scope Enforcement):** If a question is determined to be out of scope (e.g., "What is the weather today?"), the system must respond with a polite refusal, such as: "As an AI tutor for 'Physical AI & Humanoid Robotics,' I can only answer questions related to the book's content."
- **SS-2 (Safety):** The system is strictly forbidden from generating or explaining code or procedures that involve controlling real-world robotic hardware without explicit and verbatim safety warnings from the book.
- **SS-3 (Context Distinction):** The model's system prompt will instruct it to always be clear when discussing simulated vs. real-world environments, using the `is_simulation` flag from the chunk metadata as a guide.
