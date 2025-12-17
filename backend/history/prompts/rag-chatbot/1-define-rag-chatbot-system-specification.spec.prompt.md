---
id: 1
title: Define RAG Chatbot System Specification
stage: spec
date: 2025-12-17
surface: agent
model: gemini-pro
feature: rag-chatbot
branch: main
user: placeholder_user
command: placeholder_command
labels: ["specification", "rag", "architecture"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - spec.md
tests:

---

## Prompt

Define the complete specification for an Integrated RAG Chatbot embedded within a digital course book platform.

SYSTEM OVERVIEW:
- Purpose: Answer user questions about the book "Physical AI & Humanoid Robotics"
- Paradigm: Retrieval-Augmented Generation with strict grounding
- Interaction modes:
  1) Global Book QA
  2) Selected-Text-Only QA

TECH STACK:
- Backend: FastAPI (Python)
- AI Orchestration: OpenAI Agents SDK / Chat Kit SDK
- Embeddings: OpenAI text-embedding model
- Vector DB: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless Postgres
- File ingestion: Markdown, PDF, and structured chapter metadata
- Auth: Token-based (anonymous + optional user accounts)

DATA MODEL:
- Book → Chapters → Modules → Sections → Paragraph chunks
- Each chunk stores:
  - text
  - module name
  - learning objective
  - simulation vs real-world flag
  - page/section reference

RAG PIPELINE:
1. User question received
2. Detect selected-text mode or global mode
3. Embed query
4. Retrieve top-k chunks from Qdrant
5. Apply filtering rules (module, selection constraint)
6. Inject context into system prompt
7. Generate answer with citations

FUNCTIONAL REQUIREMENTS:
- Semantic search across all modules
- Answer strictly from retrieved content
- Explain ROS 2 concepts, simulation pipelines, and VLA workflows
- Support future humanoid robot extensions

NON-FUNCTIONAL REQUIREMENTS:
- Low latency (<2s for retrieval)
- Stateless API
- Scalable ingestion pipeline
- Cloud-native deployment readyCreate a phased implementation plan for the Integrated RAG Chatbot.

PHASE 1: Knowledge Preparation
- Convert book chapters into structured Markdown
- Chunk content by semantic boundaries (≈300–500 tokens)
- Attach metadata (module, simulator, AI stack)

PHASE 2: Vectorization
- Generate embeddings for all chunks
- Store vectors in Qdrant Cloud
- Store metadata and references in Neon Postgres

PHASE 3: Backend API
- Build FastAPI service
- Endpoints:
  /ingest
  /query
  /query-selected-text
  /health

PHASE 4: Agent Design
- Define system + developer prompts
- Create RAG agent using OpenAI Agents SDK
- Add citation enforcement logic

PHASE 5: Book Integration
- Embed chatbot UI inside book reader
- Capture user-selected text
- Toggle “selected-text-only” mode

PHASE 6: Evaluation
- Test module-wise accuracy
- Validate no hallucinations
- Stress test robotics queries

PHASE 7: Deployment
- Deploy backend
- Connect Qdrant + Neon
- Enable monitoring and logsYou are a senior AI systems architect following the Spec-Kit methodology.

TASK: SPECIFY the system requirements for an embedded Retrieval-Augmented Generation (RAG) chatbot.

PROJECT CONTEXT:
- Product: RAG chatbot embedded inside an educational book
- Book Title: "Physical AI & Humanoid Robotics: Embodied Intelligence in the Physical World"
- Audience: Advanced undergraduate and graduate students
- Platform: Web-based, embedded within the book website
- Model: Gemini (latest available)
- Knowledge Sources:
  - Book chapters (PDF / Markdown)
  - Figures, tables, and captions
  - Approved supplementary notes

FUNCTIONAL REQUIREMENTS:
- Accept natural language questions from learners
- Detect selected text vs global book context
- Ingest and chunk PDF and Markdown content
- Generate embeddings for all chunks
- Retrieve top-k relevant chunks per query
- Answer strictly grounded in retrieved content
- Always cite chapter/module/section references

NON-FUNCTIONAL REQUIREMENTS:
- Low hallucination risk
- Deterministic, explainable answers
- Sub-second retrieval latency (excluding LLM response)
- Scalable to future book editions

ARCHITECTURE REQUIREMENTS:
- Retrieval-first pipeline (no answer without retrieval)
- Modular ingestion, retrieval, and generation layers
- Gemini-compatible prompt orchestration
- Clear separation of backend and frontend

DATA & STORAGE:
- Vector database (FAISS or Chroma)
- Metadata schema (chapter, section, page, modality)

SAFETY & SCOPE:
- Refuse out-of-scope questions politely
- No unsafe real-world robot control instructions
- Clear simulation vs real-hardware distinction

OUTPUT FORMAT:
- Structured specification document
- Use headings, bullet points, and tables
- Content must be suitable for `plan.md` or `spec.md`

## Response snapshot

Created the `spec.md` file with the detailed system specification for the RAG chatbot.

## Outcome

- ✅ Impact: A comprehensive specification document for the RAG chatbot was created.
- 🧪 Tests: No tests were run.
- 📁 Files: The file `spec.md` was created.
- 🔁 Next prompts: The user will likely proceed with the implementation plan.
- 🧠 Reflection: The specification was created successfully, following all user requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
