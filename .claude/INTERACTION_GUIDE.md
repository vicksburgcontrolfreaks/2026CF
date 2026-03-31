# Claude Code Interaction Guide

This document defines how Claude Code should interact with this team.

## Communication Style
- Be concise and direct
- Short commit messages unless detailed explanation requested
- Don't create documentation files unless explicitly asked
- Prefer consolidated documentation over multiple files

## Technical Approach
- Ask about broader goals before implementing if request seems to treat symptoms vs root cause
- Suggest simpler alternatives when appropriate
- Explain tradeoffs between different solutions
- Flag potential issues early (performance, maintainability, edge cases)
- Only make changes explicitly requested - don't proactively refactor or "improve" code

## Educational Context
- Team has basic but capable programming experience
- Explain WHY solutions work, not just WHAT to do
- Point out patterns in codebase for consistency
- Balance being helpful without being patronizing
- Quick working solutions often better than over-engineered ones
- Flag technical debt when it might cause problems later

## When to Push Back
- If request might break existing functionality
- If there's a much simpler way to achieve the goal
- If approach doesn't match existing codebase patterns
- If proactive changes weren't requested (e.g., changing constants, adding features)

## Git Workflow Standards
- **Main branch**: Only tested and proven code - do NOT make changes directly on main
- **Feature branches**: All development work happens on branches
- **Competition branch**: Used for code changes during competitions
  - When code is deployed to robot during competition, commit with summary of changes and match number
  - Example: "Adjust shooter RPM for match 42 - increased by 200 to compensate for battery voltage"
- Always create a new branch for feature work unless user explicitly requests working on existing branch

## Competition Context
- This is FRC robot code - practical and working beats perfect
- Camera failure scenarios need fallback options
- Changes should be competition-tested when possible

## Documentation Workflow
When code changes are made:
1. Consider if GitHub Pages documentation needs updating
2. Relevant docs: driver-controls.md, shooter-testing.md, pid-tuning.md, shooter-system.md, shoot-while-driving.md
3. After merging to main, remind user to run sync script: `.\sync-docs-to-pages.bat`
4. Documentation site: https://vicksburgcontrolfreaks.github.io/2026CF/

### Manual GitHub Pages Sync (if script fails)
If sync-docs-to-pages.bat doesn't work or you need to publish from a feature branch:
```bash
# 1. Switch to gh-pages branch
git checkout gh-pages

# 2. Copy documentation files from source branch
git checkout <source-branch> -- <FILE1>.md <FILE2>.md

# 3. Create Jekyll-formatted versions with front matter
echo "---
layout: default
title: Page Title
---
" > output-file.md && cat SOURCE_FILE.md >> output-file.md

# 4. Remove source files (if they don't match output names)
rm SOURCE_FILE.md

# 5. Stage, commit, and push
git add output-file.md
git commit -m "Sync docs from <source-branch>"
git push origin gh-pages

# 6. Handle local changes if needed
git stash  # if checkout fails due to local changes

# 7. Return to original branch
git checkout <source-branch>
```

Example that worked for dynamic-shooting-test branch:
```bash
git checkout gh-pages
git checkout dynamic-shooting-test -- TRAJECTORY_TEST_PROCEDURE.md DYNAMIC_SHOOTING_TEST_WORKFLOW.md
echo "---\nlayout: default\ntitle: Trajectory Testing\n---\n" > trajectory-testing.md && cat TRAJECTORY_TEST_PROCEDURE.md >> trajectory-testing.md
echo "---\nlayout: default\ntitle: Test Workflow\n---\n" > test-workflow.md && cat DYNAMIC_SHOOTING_TEST_WORKFLOW.md >> test-workflow.md
rm TRAJECTORY_TEST_PROCEDURE.md DYNAMIC_SHOOTING_TEST_WORKFLOW.md
git reset  # if accidentally staged wrong files
git add trajectory-testing.md test-workflow.md
git commit -m "Add docs from dynamic-shooting-test branch"
git push origin gh-pages
git stash  # if local changes block checkout
git checkout dynamic-shooting-test
```

## Conversation Compaction
When conversations get long and need compaction:
1. These interaction preferences are preserved in this file
2. Critical technical context in CHANGELOG.md
3. Read both files after compaction to restore context
