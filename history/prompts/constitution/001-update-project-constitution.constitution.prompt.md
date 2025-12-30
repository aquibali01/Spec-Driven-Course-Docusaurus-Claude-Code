---
id: 001
title: Update Project Constitution
stage: constitution
date: 2025-12-29
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: Claude Code
command: /sp.constitution
labels: [constitution, setup, governance]
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

/sp.constitution

Project: AI-Spec–Driven Book with Embedded RAG Chatbot

Mission:
Author and deploy a technical book using Spec-Kit Plus and Claude Code, published via Docusaurus on GitHub Pages, with an embedded RAG chatbot that answers questions about the book and user-selected text only.

Core principles:
- Spec-first development
- Technical accuracy and reproducibility
- Modular, reusable intelligence
- Clear separation: spec → implementation → explanation

Deliverables:
1. AI-authored book (Spec-Kit Plus + Claude Code)
2. Docusaurus site deployed to GitHub Pages
3. Embedded RAG chatbot using:
   - OpenAI Agents / ChatKit SDKs
   - FastAPI
   - Neon Serverless Postgres
   - Qdrant Cloud (Free Tier)
4. Bonus: Reusable Claude Code Subagents and Agent Skills

Standards:
- Verifiable technical claims
- Runnable or well-annotated code
- Explicit architectural decisions
- No hallucinated content

Book constraints:
- Format: Markdown/MDX (Docusaurus)
- Audience: Software & AI engineers
- Reading level: Fluent

## Response snapshot

Updated the project constitution file (.specify/memory/constitution.md) with the following content:

1. Created a new constitution for the "AI-Spec–Driven Book with Embedded RAG Chatbot" project
2. Defined six core principles:
   - I. Spec-First Development
   - II. Technical Accuracy and Reproducibility
   - III. Modular, Reusable Intelligence
   - IV. Explicit Architectural Decisions
   - V. No Hallucinated Content
   - VI. Docusaurus and GitHub Pages Deployment
3. Added sections for Additional Constraints, Development Workflow, and Governance
4. Set version to 1.0.0 with ratification date 2025-12-29
5. Included a Sync Impact Report indicating templates that may need updates

## Outcome

- ✅ Impact: Successfully created project constitution with 6 core principles and governance structure
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with new project constitution
- 🔁 Next prompts: Ready to create feature specifications based on this constitution
- 🧠 Reflection: Constitution aligns with user requirements and provides clear governance structure for the AI-driven book project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A