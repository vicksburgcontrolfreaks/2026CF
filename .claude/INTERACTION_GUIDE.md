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

## Conversation Compaction
When conversations get long and need compaction:
1. These interaction preferences are preserved in this file
2. Critical technical context in CHANGELOG.md
3. Read both files after compaction to restore context
