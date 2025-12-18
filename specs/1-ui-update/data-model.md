# Data Model: UI Update for Docusaurus Robotics Book

## Overview
This UI update feature does not introduce new data models or modify existing data structures. All changes are limited to presentation layer (CSS/JS) enhancements.

## Current Data Models (Unchanged)
- Existing Docusaurus content models (MDX, Markdown)
- Navigation structure defined in docusaurus.config.ts
- Existing component props and interfaces

## UI State Extensions

### Theme State Management
**Component**: ThemeProvider/ThemeToggle
**Purpose**: Manage light/dark mode state
**Structure**:
```typescript
interface ThemeState {
  currentTheme: 'light' | 'dark';
  themePreference: 'light' | 'dark' | 'system';
  updateTheme: (theme: 'light' | 'dark') => void;
  toggleTheme: () => void;
}
```

### Animation State Management
**Component**: Animated components
**Purpose**: Control animation triggers and states
**Structure**:
```typescript
interface AnimationState {
  isVisible: boolean;
  animationTriggered: boolean;
  setVisibility: (visible: boolean) => void;
}
```

## Component Props Updates

### Enhanced Button Component Props
**Component**: CustomButton
**Purpose**: Extend existing button functionality with new styling options
**Structure**:
```typescript
interface ButtonProps {
  variant?: 'primary' | 'secondary' | 'outline' | 'icon';
  size?: 'small' | 'medium' | 'large';
  animationType?: 'lift' | 'pulse' | 'none';
  icon?: React.ReactNode;
  disabled?: boolean;
}
```

### Content Block Component Props
**Component**: Specialized content blocks (.robotics-code-block, .robotics-exercise, etc.)
**Purpose**: Handle new styling and interaction patterns
**Structure**:
```typescript
interface ContentBlockProps {
  type: 'code-block' | 'diagram' | 'lesson-objectives' | 'exercise' | 'takeaway' | 'reflection';
  children: React.ReactNode;
  collapsible?: boolean;
  initiallyCollapsed?: boolean;
  animationOnView?: boolean;
}
```

## CSS Custom Properties (Variables)

### Color Palette
**Scope**: Global
**Purpose**: Enable dynamic theming
**Structure**:
```css
:root {
  /* Light theme defaults */
  --ifm-color-primary: #your-primary-color;
  --ifm-color-content: #292929;
  --ifm-background-color: #ffffff;
  --ifm-navbar-shadow: 0 2px 8px rgba(0,0,0,0.1);
}

[data-theme='dark'] {
  /* Dark theme overrides */
  --ifm-color-primary: #your-dark-primary-color;
  --ifm-color-content: #e6e6e6;
  --ifm-background-color: #121212;
  --ifm-navbar-shadow: 0 2px 8px rgba(0,0,0,0.3);
}
```

### Animation Variables
**Scope**: Global
**Purpose**: Standardize animation timing and easing
**Structure**:
```css
:root {
  --ifm-animation-duration-fast: 150ms;
  --ifm-animation-duration-normal: 250ms;
  --ifm-animation-duration-slow: 350ms;
  --ifm-animation-easing: cubic-bezier(0.4, 0, 0.2, 1);
}
```

## Theme Storage Schema

### Local Storage Structure
**Purpose**: Persist user theme preference
**Format**:
```json
{
  "theme": {
    "selectedTheme": "light|dark",
    "lastUpdated": "timestamp",
    "autoDetected": true|false
  }
}
```

## Validation Rules

### Accessibility Compliance
- All color combinations must meet WCAG 2.1 AA contrast ratios
- Interactive elements must have visible focus indicators
- Animation duration must respect user's reduced motion preference

### Performance Constraints
- CSS bundle size increase < 50KB
- Animation performance maintained at 60fps
- No blocking of critical rendering path

## Relationships

### Component Hierarchy
```
ThemeProvider (top-level)
├── Navbar (inherits theme)
├── HomepageHeader (inherits theme, has animations)
├── ContentBlocks (inherits theme, has animations)
└── Buttons (inherits theme, has animations)
```

### Style Cascade
1. Docusaurus default styles (foundation)
2. Custom CSS overrides (branding/ui enhancements)
3. Theme-specific overrides (light/dark mode)
4. Component-specific styles (unique interactions)
5. Animation states (dynamic effects)

## State Transitions

### Theme Transition States
- `initial`: System default or last stored preference
- `user-selected`: After user manually selects theme
- `system-change`: When OS theme changes (if system preference enabled)

### Animation Lifecycle States
- `idle`: Component loaded, animation ready to trigger
- `triggered`: Animation sequence initiated
- `active`: Animation in progress
- `complete`: Animation finished, final state