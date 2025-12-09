---
id: 1
title: Create Course Constitution
stage: constitution
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, course, robotics]
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
/sp.constitution

Project: Teaching Physical AI & Humanoid Robotics Course

Core principles:
- Embodied Intelligence: Emphasize the integration of AI systems with physical robots.
- Hands-on Learning: Students must actively apply AI concepts in simulated and real humanoid robotics environments.
- Accuracy and Rigor: All technical explanations and robotics implementations must be precise and verifiable.
- Clarity for Students: Content must be clear, engaging, and suitable for learners with a computer science or engineering background.
- Reproducibility: Experiments, simulations, and robot exercises must be fully replicable.

Key standards:
- All technical claims and robot control algorithms must be traceable to credible sources.
- Citation format: IEEE or APA style for academic rigor.
- Source types: At least 40% from peer-reviewed journals, conference papers, or textbooks; remaining from verified technical blogs or documentation.
- Plagiarism check: 0% tolerance before deployment.
- Writing clarity: Flesch-Kincaid grade 10-12 suitable for advanced undergraduates.
- Code and Simulation Examples: Must be tested and fully functional in common robotics simulators (e.g., Gazebo, PyBullet) or real humanoid platforms.

Constraints:
- Word count: 50,000–70,000 words (for full course book)
- Minimum 30 sources, including textbooks, research papers, and official robotics documentation.
- Format: Docusaurus Markdown, deployable to GitHub Pages.
- Include diagrams, flowcharts, and annotated code snippets for visual clarity.

Success criteria:
- All AI algorithms and robotics exercises are verified and reproducible.
- Students can replicate experiments and control humanoid robots successfully.
- Content passes peer review or instructor validation.
- Zero plagiarism detected.
- Book is fully deployed and navigable on GitHub Pages with functional links and interactive examples.
```

## Response snapshot

Updated the constitution file at `.specify/memory/constitution.md` with project-specific values for the Teaching Physical AI & Humanoid Robotics Course, including all core principles, course standards, development workflow, and governance guidelines.

## Outcome

- ✅ Impact: Created project constitution document with all required principles and standards
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with course-specific constitution
- 🔁 Next prompts: Continue with course development using established constitution
- 🧠 Reflection: Constitution successfully created with all required principles and standards

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A