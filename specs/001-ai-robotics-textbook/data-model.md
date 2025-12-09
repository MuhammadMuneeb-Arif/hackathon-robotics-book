# Data Model: Physical AI & Humanoid Robotics Textbook

## Overview
This document defines the data models for the Physical AI & Humanoid Robotics textbook system, including entities, relationships, and validation rules based on the feature specification.

## Entity Models

### 1. Textbook Module
**Description**: A major section of the textbook containing lessons on a specific topic area

**Fields**:
- `id` (string): Unique identifier for the module
- `title` (string): Title of the module (e.g., "Module 1: The Robotic Nervous System")
- `description` (string): Brief description of the module content
- `module_number` (integer): Sequential number of the module (1-4)
- `lessons` (array of Lesson): Collection of lessons within the module
- `learning_objectives` (array of string): Learning objectives for the module
- `prerequisites` (array of string): Prerequisites required for this module
- `estimated_duration_hours` (number): Estimated time to complete the module

**Validation Rules**:
- Title must be 10-100 characters
- Module number must be between 1 and 4
- Must have 1 or more lessons
- Learning objectives must be 3-5 items

### 2. Lesson
**Description**: A subsection within a module containing content, exercises, and learning materials

**Fields**:
- `id` (string): Unique identifier for the lesson
- `title` (string): Title of the lesson
- `module_id` (string): Reference to the parent module
- `content` (string): Main content of the lesson (1000-2000 words)
- `lesson_number` (integer): Sequential number within the module
- `learning_objectives` (array of string): Specific learning objectives for the lesson
- `hands_on_exercises` (array of Exercise): Practical exercises for students
- `key_takeaways` (array of string): Key points to remember
- `reflection_questions` (array of string): Questions for student reflection
- `code_examples` (array of CodeExample): Embedded code examples
- `diagrams` (array of Diagram): Visual aids and diagrams
- `citations` (array of Citation): References and sources
- `estimated_duration_minutes` (number): Estimated time to complete the lesson

**Validation Rules**:
- Content must be between 1000-2000 words
- Must have 1-5 hands-on exercises
- Must have 3-7 key takeaways
- Must have 2-4 reflection questions
- Must have 1 or more citations

### 3. Citation
**Description**: A reference to external sources that validates technical claims

**Fields**:
- `id` (string): Unique identifier for the citation
- `title` (string): Title of the referenced work
- `authors` (array of string): Authors of the work
- `publication_date` (string): Date in YYYY-MM-DD format
- `source_type` (string): Type of source (e.g., "journal", "conference", "book", "documentation", "website")
- `url` (string): URL to the source (optional for print sources)
- `doi` (string): Digital Object Identifier (for peer-reviewed sources)
- `apa_citation` (string): Properly formatted APA 7th edition citation
- `verification_status` (string): Status of fact-checking ("verified", "pending", "unverified")
- `peer_reviewed` (boolean): Whether the source is peer-reviewed

**Validation Rules**:
- APA citation must follow 7th edition format
- Peer-reviewed status must be verified
- URL must be valid if provided
- At least 50% of citations must be peer-reviewed

### 4. Exercise
**Description**: A hands-on activity for students to apply concepts learned

**Fields**:
- `id` (string): Unique identifier for the exercise
- `title` (string): Title of the exercise
- `lesson_id` (string): Reference to the parent lesson
- `description` (string): Detailed description of the exercise
- `difficulty_level` (string): "beginner", "intermediate", or "advanced"
- `estimated_completion_time` (number): Time in minutes
- `prerequisites` (array of string): Requirements to complete the exercise
- `instructions` (array of string): Step-by-step instructions
- `expected_outcome` (string): What students should achieve
- `verification_steps` (array of string): How to verify completion
- `simulation_environment` (string): Required simulation platform (e.g., "Gazebo", "Isaac Sim", "ROS 2")

**Validation Rules**:
- Must have clear, actionable instructions
- Estimated time must be between 15-120 minutes
- Must specify required simulation environment
- Must include verification steps

### 5. CodeExample
**Description**: A code snippet demonstrating technical concepts

**Fields**:
- `id` (string): Unique identifier for the code example
- `lesson_id` (string): Reference to the parent lesson
- `language` (string): Programming language (e.g., "python", "ros", "bash", "yaml")
- `code` (string): The actual code snippet
- `description` (string): Explanation of what the code does
- `usage_context` (string): Where and how the code is used
- `tested_environment` (string): Environment where code was tested
- `verification_status` (string): Whether code was verified to work

**Validation Rules**:
- Code must be syntactically correct
- Must be tested in specified environment
- Must have clear description of functionality

### 6. Diagram
**Description**: A visual representation to aid understanding

**Fields**:
- `id` (string): Unique identifier for the diagram
- `lesson_id` (string): Reference to the parent lesson
- `title` (string): Title of the diagram
- `description` (string): Explanation of the diagram
- `file_path` (string): Path to the diagram file
- `alt_text` (string): Alternative text for accessibility
- `type` (string): Type of diagram (e.g., "flowchart", "architecture", "process", "schematic")

**Validation Rules**:
- Must have alt text for accessibility
- File must exist at specified path
- Type must be one of the allowed values

### 7. DocusaurusDocument
**Description**: A Markdown file with frontmatter that conforms to Docusaurus requirements

**Fields**:
- `id` (string): Unique identifier for the document
- `file_path` (string): Path where the document will be stored
- `title` (string): Title for the document
- `description` (string): Brief description
- `sidebar_label` (string): Label for sidebar navigation
- `slug` (string): URL-friendly slug
- `tags` (array of string): Tags for categorization
- `markdown_content` (string): The actual Markdown content
- `frontmatter` (object): Additional metadata

**Validation Rules**:
- File path must follow Docusaurus conventions
- Must include required frontmatter fields
- Content must be valid Markdown

## Relationships

### Module and Lesson
- One Module contains many Lessons
- Each Lesson belongs to exactly one Module
- Relationship: Module (1) → (0..n) Lesson

### Lesson and Citation
- One Lesson contains many Citations
- Each Citation is associated with one or more Lessons
- Relationship: Lesson (1) → (0..n) Citation

### Lesson and Exercise
- One Lesson contains many Exercises
- Each Exercise belongs to exactly one Lesson
- Relationship: Lesson (1) → (0..n) Exercise

### Lesson and CodeExample
- One Lesson contains many CodeExamples
- Each CodeExample belongs to exactly one Lesson
- Relationship: Lesson (1) → (0..n) CodeExample

### Lesson and Diagram
- One Lesson contains many Diagrams
- Each Diagram belongs to exactly one Lesson
- Relationship: Lesson (1) → (0..n) Diagram

## State Transitions

### Lesson State Model
- `draft`: Initial state, content being created
- `review`: Content ready for review
- `revised`: Content has been reviewed and revised
- `published`: Content is ready for inclusion in textbook
- `archived`: Content is no longer used

### Citation State Model
- `proposed`: Citation identified but not verified
- `verified`: Citation fact-checked and confirmed
- `invalid`: Citation found to be incorrect or unreliable

## Validation Rules Summary

1. **Content Length**: Lessons must be 1000-2000 words
2. **Academic Standards**: Minimum 50% of citations must be peer-reviewed
3. **Academic Integrity**: Zero plagiarism requirement
4. **Readability**: Content must maintain Flesch-Kincaid grade level 10-12
5. **Practical Application**: All concepts must include hands-on exercises
6. **Reproducibility**: All exercises must have verification steps
7. **Citation Format**: All citations must follow APA 7th edition format
8. **Technical Accuracy**: All code examples must be tested and verified