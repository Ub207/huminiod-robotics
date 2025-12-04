---
id: 0001
title: Setup Docusaurus for Physical AI Book
stage: general
date: 2025-12-05
surface: agent
model: Gemini
feature: none
branch: main
user: Ub207
command: Docusaurus Default Content Removal + Book Setup Prompt
labels: [docusaurus, documentation, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - huminiod-robotics/docs/01-foundations/what-is-physical-ai.md
  - huminiod-robotics/docs/01-foundations/embodied-intelligence.md
  - huminiod-robotics/docs/01-foundations/difference-between-digital-ai-and-physical-ai.md
  - huminiod-robotics/docs/02-math-for-robotics/linear-algebra.md
  - huminiod-robotics/docs/02-math-for-robotics/vectors-matrices-in-robotics.md
  - huminiod-robotics/docs/02-math-for-robotics/calculus-for-movement.md
  - huminiod-robotics/docs/03-robot-hardware/humanoid-anatomy.md
  - huminiod-robotics/docs/03-robot-hardware/sensors-overview.md
  - huminiod-robotics/docs/03-robot-hardware/actuators-motors.md
  - huminiod-robotics/docs/04-robot-software/ros-introduction.md
  - huminiod-robotics/docs/04-robot-software/perception-systems.md
  - huminiod-robotics/docs/04-robot-software/control-systems.md
  - huminiod-robotics/docs/05-ai-for-robots/computer-vision.md
  - huminiod-robotics/docs/05-ai-for-robots/reinforcement-learning.md
  - huminiod-robotics/docs/05-ai-for-robots/robot-decision-making.md
  - huminiod-robotics/docs/06-simulations/gazebo.md
  - huminiod-robotics/docs/06-simulations/pybullet.md
  - huminiod-robotics/docs/06-simulations/digital-twin.md
  - huminiod-robotics/docs/07-real-world-projects/humanoid-walking.md
  - huminiod-robotics/docs/07-real-world-projects/object-grasping.md
  - huminiod-robotics/docs/07-real-world-projects/balance-system.md
  - huminiod-robotics/docs/08-future/ai-and-humans.md
  - huminiod-robotics/docs/08-future/ethics-and-safety.md
  - huminiod-robotics/docs/08-future/future-of-humanoids.md
  - huminiod-robotics/sidebars.ts
  - huminiod-robotics/src/pages/index.tsx
tests:
  - none
---

## Prompt

{"id":"59384","variant":"standard","title":"Docusaurus Default Content Removal + Book Setup Prompt"}You are a Docusaurus technical writer and documentation architect.Goal: I am creating a book-based documentation website named:"Physical AI & Humanoid Robotics – From Digital Brain to Physical Body"Step 1: Remove all default Docusaurus content, including:- docs/intro.md- docs/tutorial-basics/*- docs/tutorial-extras/*- blog folder (if default)- default homepage content in src/pages/index.js or index.tsx- default sidebar items- any demo images/logos/text left from the templateStep 2: Create a clean book-based structure with the following folders inside /docs:01-foundations- what-is-physical-ai.md- embodied-intelligence.md- difference-between-digital-ai-and-physical-ai.md02-math-for-robotics- linear-algebra.md- vectors-matrices-in-robotics.md- calculus-for-movement.md03-robot-hardware- humanoid-anatomy.md- sensors-overview.md- actuators-motors.md04-robot-software- ros-introduction.md- perception-systems.md- control-systems.md05-ai-for-robots- computer-vision.md- reinforcement-learning.md- robot-decision-making.md06-simulations- gazebo.md- pybullet.md- digital-twin.md07-real-world-projects- humanoid-walking.md- object-grasping.md- balance-system.md08-future- ai-and-humans.md- ethics-and-safety.md- future-of-humanoids.mdStep 3: For EACH file, add:- A strong title- A 100–150 word introduction- Bullet points of key concepts- One real-world example- One mini exerciseStep 4: Update sidebars.js to match book flow order:Foundations → Math → Hardware → Software → AI → Simulation → Projects → FutureStep 5: Update homepage to:Title: Physical AI & Humanoid RoboticsSubtitle: Bridging the gap between the digital brain and the physical bodyButton: “Start Learning”Button link: /docs/01-foundations/what-is-physical-aiStep 6: Keep tone:- Academic- Future-focused- Practical- Robotics-engineer style- No childish languageIMPORTANT:Do not keep any default Docusaurus text. Replace everything with book-related content only.Now generate:1. Updated folder/file structure2. New sidebar config3. New homepage content4. Content for first 5 files in full detail

## Response snapshot

I have successfully set up the Docusaurus project for the book "Physical AI & Humanoid Robotics". This involved removing the default content, creating the new book structure, generating content for all the files (with the first 5 in detail), updating the sidebar configuration, and customizing the homepage.

## Outcome

- ✅ Impact: The Docusaurus project is now fully configured and ready for further content development. The structure is clean, logical, and follows the user's specifications.
- 🧪 Tests: none
- 📁 Files: 26 files were created/modified in total.
- 🔁 Next prompts: none
- 🧠 Reflection: The request was very detailed and clear, which made the execution straightforward. The multi-step nature of the request was well-suited for a todo list approach.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script initially failed because a 'green' stage was used without a feature context. This was resolved by using the 'general' stage.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): For similar setup tasks, start with the 'general' stage for PHR creation to avoid context errors.