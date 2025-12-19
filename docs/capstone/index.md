---
title: "Capstone Project"
description: "Build an autonomous humanoid: The culmination of your Physical AI journey"
tags: [capstone, autonomous-humanoid, final-project, portfolio]
slug: /capstone
sidebar_position: 99
---



# Capstone Project: The Autonomous Humanoid

## Your Portfolio Piece

Tesla interviews robotics engineers with one question: *"Show me what you've built."*

This capstone is your answer.

**By the end, you'll have a 2-3 minute video showing:**
1. Your humanoid receiving a voice command: *"Find the red box and bring it here"*
2. Planning a path through obstacles using Nav2
3. Navigating autonomously using VSLAM
4. Identifying the target object with computer vision
5. Grasping and retrieving it successfully

**This isn't a toy project.** It demonstrates the complete robotics AI stack companies desperately need: ROS 2 + Simulation + Perception + VLA.

---

## The Challenge

### Mission Statement
**Build a simulated humanoid robot that executes complex, multi-step manipulation tasks from natural language commands in an unstructured environment.**

### Core Requirements

**Functional Requirements:**

1. Accept voice input (Whisper speech recognition)
2. Generate action plan (GPT-4 task decomposition)
3. Navigate to target (Nav2 + VSLAM)
4. Detect object (YOLO/SAM + CLIP)
5. Manipulate object (ROS 2 MoveIt grasping)
6. Return to user (path planning)
7. Handle failures (replanning with LLM)

**Technical Requirements:**
- Runs in NVIDIA Isaac Sim or Gazebo
- Uses ROS 2 Humble throughout
- Deploys perception to Jetson (optional but recommended)
- Completes task in under 5 minutes simulation time
- Succeeds 80% or more on 5 different test scenarios

**Documentation Requirements:**
- README with architecture diagram
- Setup instructions (someone else can run it)
- Demo video (90 seconds maximum)
- Technical report (3-5 pages) explaining design decisions

---

## Project Phases

### Phase 1: Integration Planning (Week 12)
**Goal**: Design your system architecture

**Deliverables:**
- Architecture diagram showing all components
- Interface definitions (what topics connect modules?)
- Risk assessment (what could fail?)

**Key decisions:**
- Which simulator? (Isaac Sim = photorealistic, Gazebo = faster)
- Which humanoid model? (Unitree G1, custom URDF)
- Which objects to manipulate? (boxes, cups, tools)

### Phase 2: Environment Setup (Week 12)
**Goal**: Build your test world

**Deliverables:**
- Simulated environment with furniture, obstacles, objects
- Spawn locations for robot and target objects
- Lighting and sensor noise for realism

**Tools:**
- Gazebo World Builder or Isaac Sim USD composer
- ROS 2 launch files for reproducible setup

### Phase 3: Perception Pipeline (Week 13)
**Goal**: Make robot see and understand

**Deliverables:**
- VSLAM running (robot knows where it is)
- Object detection working (finds target objects)
- Depth processing active (avoids obstacles)

**Integration:**
- Connect Isaac ROS perception to Nav2 costmaps
- Publish detections as ROS 2 visualization markers

### Phase 4: Navigation Stack (Week 13)
**Goal**: Enable autonomous movement

**Deliverables:**
- Nav2 configured for humanoid footstep planning
- Obstacle avoidance working
- Goal-reaching behavior (gets within 0.5m of target)

**Testing:**
- Place obstacles between robot and goal
- Verify dynamic replanning when obstacles move

### Phase 5: Manipulation (Week 13)
**Goal**: Grasp and carry objects

**Deliverables:**
- MoveIt motion planning configured
- Gripper control working
- Pick-and-place sequence reliable

**Challenges:**
- Humanoid balance during reach (IK solving)
- Grasp force control (don't crush or drop)

### Phase 6: VLA Integration (Week 13)
**Goal**: Voice control end-to-end

**Deliverables:**
- Whisper transcription running
- GPT-4 action planning integrated
- Error recovery (what if object not found?)

**Testing scenarios:**
- "Bring me the red box"
- "Clean up the blue objects"
- "Find the largest item on the table"

### Phase 7: Testing & Refinement (Week 13)
**Goal**: Make it robust

**Deliverables:**
- 5 different test scenarios (varied objects, positions)
- Success rate of 80% or more across all tests
- Failure logs and fixes implemented

### Phase 8: Documentation & Demo (Week 13)
**Goal**: Package for presentation

**Deliverables:**
- 90-second demo video (edited, narrated)
- GitHub repo with clear README
- Technical report documenting approach

---

## Evaluation Rubric

### Technical Implementation (50 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| System Architecture | 10 | Clean separation of concerns, modularity |
| Perception Accuracy | 10 | Correctly identifies objects 90% or more |
| Navigation Success | 10 | Reaches goals without collisions |
| Manipulation Skill | 10 | Grasps and transports reliably |
| VLA Integration | 10 | Voice commands work end-to-end |

### Complexity & Innovation (20 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Task Difficulty | 10 | Simple fetch vs. multi-object sorting |
| Novel Features | 10 | Beyond requirements (multi-robot, learning) |

### Documentation (20 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Code Quality | 5 | Readable, commented, follows conventions |
| Technical Report | 10 | Clear explanations of design choices |
| Demo Video | 5 | Engaging, showcases key features |

### Execution & Reliability (10 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Success Rate | 5 | 80% or more across test scenarios |
| Error Handling | 5 | Graceful failures, recovery attempts |

**Total: 100 points**

**Bonus opportunities (up to 50 points maximum):**
- Deploy to real Jetson hardware (+20)
- Multi-robot coordination (+15)
- Learning from user feedback/RLHF (+15)

---

## Starter Templates

### Architecture Template