#!/bin/bash
# Script to sync documentation updates from current branch to gh-pages

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Syncing documentation to gh-pages...${NC}"

# Save current branch
CURRENT_BRANCH=$(git branch --show-current)
echo "Current branch: $CURRENT_BRANCH"

# Check if there are uncommitted changes
if [[ -n $(git status -s) ]]; then
    echo -e "${YELLOW}Warning: You have uncommitted changes. Consider committing them first.${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
fi

# Switch to gh-pages
echo "Switching to gh-pages branch..."
git checkout gh-pages

# Copy documentation files from the original branch
echo "Copying documentation files from $CURRENT_BRANCH..."

# Copy root markdown files
git checkout $CURRENT_BRANCH -- \
    SHOOTER_TEST_USAGE.md \
    PID_TUNING_GUIDE.md \
    SHOOTER_SYSTEM_DOCUMENTATION.md \
    SHOOT_WHILE_DRIVING.md

# Rename and add Jekyll front matter
echo "Processing files for GitHub Pages..."

# Copy SHOOTER_TEST_USAGE.md to shooter-testing.md
cp SHOOTER_TEST_USAGE.md shooter-testing.md
sed -i '1i ---\nlayout: default\ntitle: Shooter Test System\n---\n' shooter-testing.md

# Copy PID_TUNING_GUIDE.md to pid-tuning.md
cp PID_TUNING_GUIDE.md pid-tuning.md
sed -i '1i ---\nlayout: default\ntitle: PID Tuning Guide\n---\n' pid-tuning.md

# Copy SHOOTER_SYSTEM_DOCUMENTATION.md to shooter-system.md
cp SHOOTER_SYSTEM_DOCUMENTATION.md shooter-system.md
sed -i '1i ---\nlayout: default\ntitle: Shooter System\n---\n' shooter-system.md

# Copy SHOOT_WHILE_DRIVING.md to shoot-while-driving.md
cp SHOOT_WHILE_DRIVING.md shoot-while-driving.md
sed -i '1i ---\nlayout: default\ntitle: Shoot While Driving\n---\n' shoot-while-driving.md

# Remove the originals (we don't need them in gh-pages)
rm SHOOTER_TEST_USAGE.md PID_TUNING_GUIDE.md SHOOTER_SYSTEM_DOCUMENTATION.md SHOOT_WHILE_DRIVING.md

# Stage the changes
echo "Staging changes..."
git add shooter-testing.md pid-tuning.md shooter-system.md shoot-while-driving.md

# Check if there are any changes to commit
if git diff --staged --quiet; then
    echo -e "${GREEN}No documentation changes to sync.${NC}"
else
    # Commit the changes
    echo "Committing changes..."
    git commit -m "Sync documentation from $CURRENT_BRANCH

Updated documentation files:
- Shooter Test Usage
- PID Tuning Guide
- Shooter System Documentation
- Shoot While Driving

Synced from branch: $CURRENT_BRANCH"

    # Push to origin
    echo "Pushing to origin/gh-pages..."
    git push origin gh-pages

    echo -e "${GREEN}Documentation synced successfully!${NC}"
fi

# Switch back to original branch
echo "Switching back to $CURRENT_BRANCH..."
git checkout $CURRENT_BRANCH

echo -e "${GREEN}Done! Your GitHub Pages site will update in a few minutes.${NC}"
echo "View your site at: https://vicksburgcontrolfreaks.github.io/2026CF/"
