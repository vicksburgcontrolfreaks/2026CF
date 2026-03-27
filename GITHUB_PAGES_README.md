# GitHub Pages Setup Guide

## What is GitHub Pages?

GitHub Pages hosts a documentation website directly from your repository. Your site will be available at:
**https://vicksburgcontrolfreaks.github.io/2026CF/**

## Initial Setup (One-Time)

### Step 1: Enable GitHub Pages on GitHub.com

1. Go to your repository: https://github.com/vicksburgcontrolfreaks/2026CF
2. Click **Settings** (top right)
3. Click **Pages** (left sidebar)
4. Under "Source":
   - Select branch: **gh-pages**
   - Select folder: **/ (root)**
5. Click **Save**
6. Wait 1-2 minutes, then visit: https://vicksburgcontrolfreaks.github.io/2026CF/

### Step 2: Verify Site is Live

Visit https://vicksburgcontrolfreaks.github.io/2026CF/ and you should see your documentation homepage!

## Documentation Structure

### Pages Available:

- **Homepage** (`index.md`): Overview and quick links
- **Driver Controls** (`driver-controls.md`): Controller button mappings and operating procedures
- **Shooter Testing** (`shooter-testing.md`): Running characterization tests with AdvantageScope
- **PID Tuning** (`pid-tuning.md`): Real-time PID tuning guide
- **Shooter System** (`shooter-system.md`): Technical system documentation
- **Shoot While Driving** (`shoot-while-driving.md`): Auto-aim system details

### Branch Structure:

- **main**: Production robot code
- **Shooter-Tuning**: Active development branch
- **gh-pages**: Documentation website (separate from code)

## Updating Documentation

### When You Update Docs on main or Shooter-Tuning:

Documentation files in your code branches:
- `SHOOTER_TEST_USAGE.md`
- `PID_TUNING_GUIDE.md`
- `SHOOTER_SYSTEM_DOCUMENTATION.md`
- `SHOOT_WHILE_DRIVING.md`

**After merging to main**, sync them to gh-pages:

#### Option 1: Using the Script (Recommended)

**On Windows:**
```bash
.\sync-docs-to-pages.bat
```

**On Mac/Linux:**
```bash
chmod +x sync-docs-to-pages.sh
./sync-docs-to-pages.sh
```

The script will:
1. Switch to `gh-pages` branch
2. Copy documentation files from your current branch
3. Add Jekyll front matter
4. Commit and push changes
5. Switch back to your original branch

#### Option 2: Manual Sync

```bash
# Save current branch
git checkout gh-pages

# Copy markdown files from main
git checkout main -- SHOOTER_TEST_USAGE.md PID_TUNING_GUIDE.md SHOOTER_SYSTEM_DOCUMENTATION.md SHOOT_WHILE_DRIVING.md

# Update the Jekyll versions manually (add front matter, rename files)
# ... (see script for details)

# Commit and push
git add shooter-testing.md pid-tuning.md shooter-system.md shoot-while-driving.md
git commit -m "Update docs from main"
git push origin gh-pages

# Switch back
git checkout main
```

### When to Update:

- **After merging Shooter-Tuning to main**: Run the sync script
- **After significant doc changes**: Run the sync script
- **Before competitions**: Ensure docs are up-to-date for drivers

### Reminder System:

**⚠️ REMINDER**: When you commit code to `main` branch that includes documentation changes, remember to run the sync script to update the GitHub Pages site!

## Editing Documentation

### Editing Existing Pages

1. Edit markdown files on your working branch (main or Shooter-Tuning)
2. Commit changes
3. When ready to publish, run the sync script

### Adding New Pages

1. Create new `.md` file on `main` branch
2. Add content with Jekyll front matter:
   ```yaml
   ---
   layout: default
   title: Your Page Title
   ---

   # Your Content Here
   ```
3. Update `index.md` to link to the new page
4. Commit changes
5. Checkout `gh-pages` branch
6. Add the new page there too (with front matter)
7. Update navigation in `_config.yml` if needed
8. Commit and push

### Jekyll Front Matter

All pages need this at the top:
```yaml
---
layout: default
title: Page Title
---
```

This tells Jekyll how to render the page.

## Troubleshooting

### Site not updating after push?
- Wait 2-5 minutes for GitHub to rebuild
- Check Actions tab on GitHub for build errors
- Verify you pushed to `gh-pages` branch, not `main`

### Page shows weird formatting?
- Check Jekyll front matter is present
- Verify markdown syntax
- Test locally with Jekyll (optional, see below)

### Changes not showing up?
- Hard refresh browser (Ctrl+F5 / Cmd+Shift+R)
- Check if you pushed to the right branch
- Verify file is committed and pushed

### Script fails?
- Make sure you're on a clean working tree (commit changes first)
- Check that `gh-pages` branch exists
- Verify you have push permissions

## Testing Locally (Optional)

To preview the site locally before pushing:

1. Install Jekyll: https://jekyllrb.com/docs/installation/
2. Run in repository root:
   ```bash
   jekyll serve
   ```
3. Visit http://localhost:4000/2026CF/

## Customization

### Changing Theme

Edit `_config.yml`:
```yaml
theme: jekyll-theme-minimal  # Change this line
```

Available themes: https://pages.github.com/themes/

### Adding Images

1. Create `assets/images/` folder in `gh-pages` branch
2. Add images there
3. Reference in markdown: `![Alt text](assets/images/image.png)`

### Custom Domain (Optional)

1. Buy domain name
2. In GitHub Pages settings, add custom domain
3. Configure DNS with your provider

## Best Practices

1. **Keep docs up-to-date**: Sync after every main merge
2. **Review before pushing**: Preview locally if unsure
3. **Driver-friendly**: Write for drivers, not just programmers
4. **Use clear titles**: Make navigation obvious
5. **Include examples**: Show real controller inputs, screenshots
6. **Test instructions**: Have a driver review control docs

## Workflow Summary

```
1. Edit docs on Shooter-Tuning branch
   ↓
2. Test and verify on robot
   ↓
3. Merge Shooter-Tuning → main (via PR)
   ↓
4. Run sync-docs-to-pages script
   ↓
5. Site updates automatically on GitHub
   ↓
6. Team can view at vicksburgcontrolfreaks.github.io/2026CF
```

## Resources

- **Jekyll Documentation**: https://jekyllrb.com/docs/
- **GitHub Pages Docs**: https://docs.github.com/en/pages
- **Markdown Guide**: https://www.markdownguide.org/
- **Your Site**: https://vicksburgcontrolfreaks.github.io/2026CF/

---

**Questions?** Contact the programming team lead.

**Last Updated**: 2026-03-27
