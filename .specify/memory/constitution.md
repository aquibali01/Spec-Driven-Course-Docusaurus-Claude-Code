<!--
Sync Impact Report:
- Version change: N/A -> 1.0.0
- Modified principles: N/A (new constitution)
- Added sections: All principles based on AI-Spec-Driven Book with Embedded RAG Chatbot project
- Removed sections: None
- Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# AI-Spec–Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development
Spec-first development: All features must be defined in specifications before implementation; Specifications must include acceptance criteria, constraints, and testable requirements; No implementation without a corresponding spec.

### II. Technical Accuracy and Reproducibility
Technical accuracy and reproducibility: All technical claims must be verifiable; Code examples must be runnable or well-annotated; All implementations must be reproducible in different environments.

### III. Modular, Reusable Intelligence
Modular, reusable intelligence: Code and intelligence artifacts must be modular and reusable; Clear separation between spec → implementation → explanation; Components should be independently testable and documented.

### IV. Explicit Architectural Decisions
Explicit architectural decisions: All significant architectural decisions must be documented as ADRs; Multiple options must be considered with trade-offs and rationale; Decisions must be measurable and reversible where possible.

### V. No Hallucinated Content
No hallucinated content: All content must be based on verifiable facts; No fabricated or hallucinated information in the book or code; All claims must be supported by evidence or clearly marked as opinion.

### VI. Docusaurus and GitHub Pages Deployment
Docusaurus and GitHub Pages deployment: The book must be published via Docusaurus on GitHub Pages; All content must be in Markdown/MDX format; Deployment must be automated and reliable.

## Additional Constraints
The book must be authored for Software & AI engineers with a fluent reading level. The embedded RAG chatbot must use OpenAI Agents / ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud (Free Tier). The RAG chatbot must answer questions about the book and user-selected text only.

## Development Workflow
All development must follow the Spec-Driven Development process using Spec-Kit Plus. Features must start with specification (spec.md), followed by implementation plan (plan.md), then executable tasks (tasks.md). All changes must reference code precisely and maintain small, testable diffs.

## Governance
This constitution governs all development activities for the AI-Spec-Driven Book project. All PRs and reviews must verify compliance with these principles. Changes to this constitution require explicit approval and documentation of the rationale. The constitution supersedes all other development practices and guidelines.

**Version**: 1.0.0 | **Ratified**: 2025-12-29 | **Last Amended**: 2025-12-29