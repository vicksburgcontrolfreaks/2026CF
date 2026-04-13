# Shooter Tuning Data Collection - Google Sheets Setup

## Instructions

1. **Create a new Google Sheet** at https://sheets.google.com
2. **Name it:** "2026CF Shooter Tuning - 62° Launch Angle"
3. **Set up the Apps Script:**
   - Click **Extensions** → **Apps Script**
   - Delete any default code
   - Paste the code from **Section A** below
   - Click **Save** (💾 icon)
   - Click **Run** → Select `setupSheet`
   - **Authorize** the script when prompted
4. **Return to your spreadsheet** - it will now have formatted sheets and calculations

---

## Section A: Apps Script Code

Copy this entire code block into the Apps Script editor:

```javascript
/**
 * 2026CF Shooter Tuning Data Collection
 * For 68° → 62° Launch Angle Retuning
 */

function onOpen() {
  const ui = SpreadsheetApp.getUi();
  ui.createMenu('🎯 Tuning Tools')
    .addItem('📊 Calculate Statistics', 'calculateStats')
    .addItem('📈 Generate Summary', 'generateSummary')
    .addItem('🔄 Reset Data', 'resetData')
    .addItem('📋 Export for Code', 'exportForCode')
    .addToUi();
}

function setupSheet() {
  const ss = SpreadsheetApp.getActiveSpreadsheet();

  // Create or get sheets
  let dataSheet = ss.getSheetByName('Test Data');
  if (!dataSheet) {
    dataSheet = ss.insertSheet('Test Data');
  }

  let summarySheet = ss.getSheetByName('Summary');
  if (!summarySheet) {
    summarySheet = ss.insertSheet('Summary');
  }

  let codeSheet = ss.getSheetByName('Code Export');
  if (!codeSheet) {
    codeSheet = ss.insertSheet('Code Export');
  }

  setupDataSheet(dataSheet);
  setupSummarySheet(summarySheet);
  setupCodeSheet(codeSheet);

  SpreadsheetApp.getUi().alert('✓ Shooter Tuning Spreadsheet Setup Complete!\n\nGo to "Test Data" tab to start recording.');
}

function setupDataSheet(sheet) {
  sheet.clear();

  // Title
  sheet.getRange('A1:I1').merge();
  sheet.getRange('A1').setValue('2026CF Shooter Tuning Data - 62° Launch Angle')
    .setFontSize(16)
    .setFontWeight('bold')
    .setBackground('#28a745')
    .setFontColor('#ffffff')
    .setHorizontalAlignment('center');

  // Info box
  sheet.getRange('A2:I2').merge();
  sheet.getRange('A2').setValue('Expected: 8-12% LOWER RPM than 68° baseline | Physics: sin(124°)≈0.829 vs sin(136°)≈0.695')
    .setFontSize(10)
    .setBackground('#d4edda')
    .setHorizontalAlignment('center');

  // Headers
  const headers = [
    'Test #',
    'Date/Time',
    'Measured Distance (m)',
    'Vision Distance (m)',
    'Starting RPM',
    'Final Working RPM',
    'RPM Change from Start',
    'Shots to Converge',
    'Notes'
  ];

  sheet.getRange('A4:I4').setValues([headers])
    .setFontWeight('bold')
    .setBackground('#3498db')
    .setFontColor('#ffffff')
    .setHorizontalAlignment('center')
    .setVerticalAlignment('middle');

  // Format columns
  sheet.setColumnWidth(1, 60);   // Test #
  sheet.setColumnWidth(2, 140);  // Date/Time
  sheet.setColumnWidth(3, 140);  // Measured Distance
  sheet.setColumnWidth(4, 140);  // Vision Distance
  sheet.setColumnWidth(5, 120);  // Starting RPM
  sheet.setColumnWidth(6, 140);  // Final Working RPM
  sheet.setColumnWidth(7, 140);  // RPM Change
  sheet.setColumnWidth(8, 140);  // Shots to Converge
  sheet.setColumnWidth(9, 300);  // Notes

  // Add initial rows with formulas
  for (let i = 5; i <= 10; i++) {
    sheet.getRange(`A${i}`).setValue(i - 4); // Test number
    sheet.getRange(`B${i}`).setFormula(`=IF(C${i}<>"", NOW(), "")`).setNumberFormat('M/d/yyyy h:mm');
    sheet.getRange(`G${i}`).setFormula(`=IF(AND(E${i}<>"", F${i}<>""), F${i}-E${i}, "")`); // RPM Change
  }

  // Add data validation for distance (must be positive)
  const distanceRange = sheet.getRange('C5:D10');
  const distanceRule = SpreadsheetApp.newDataValidation()
    .requireNumberGreaterThan(0)
    .setAllowInvalid(false)
    .setHelpText('Distance must be greater than 0 meters')
    .build();
  distanceRange.setDataValidation(distanceRule);

  // Add data validation for RPM (must be positive)
  const rpmRange = sheet.getRange('E5:F10');
  const rpmRule = SpreadsheetApp.newDataValidation()
    .requireNumberGreaterThan(0)
    .setAllowInvalid(false)
    .setHelpText('RPM must be greater than 0')
    .build();
  rpmRange.setDataValidation(rpmRule);

  // Conditional formatting for RPM change
  const rpmChangeRange = sheet.getRange('G5:G10');
  const decreaseRule = SpreadsheetApp.newConditionalFormatRule()
    .whenNumberLessThan(0)
    .setBackground('#d4edda')
    .setRanges([rpmChangeRange])
    .build();
  const increaseRule = SpreadsheetApp.newConditionalFormatRule()
    .whenNumberGreaterThan(0)
    .setBackground('#fff3cd')
    .setRanges([rpmChangeRange])
    .build();
  sheet.setConditionalFormatRules([decreaseRule, increaseRule]);

  // Instructions
  sheet.getRange('A12:I12').merge();
  sheet.getRange('A12').setValue('📝 Instructions: Fill in Measured Distance, Vision Distance, Starting RPM, Final Working RPM, Shots to Converge, and Notes for each test.')
    .setFontStyle('italic')
    .setBackground('#f8f9fa')
    .setWrap(true);

  // Old baseline reference
  sheet.getRange('A14').setValue('68° Baseline Reference:')
    .setFontWeight('bold')
    .setBackground('#e9ecef');
  sheet.getRange('A15').setValue('Distance (m)');
  sheet.getRange('B15').setValue('RPM @ 68°');
  sheet.getRange('C15').setValue('Expected @ 62°');

  const baseline = [
    [1.75, 2950, '~2600-2700'],
    [2.69, 3450, '~3050-3170'],
    [3.3, 4000, '~3520-3680'],
    [4.8, 4500, '~3960-4140']
  ];

  sheet.getRange('A16:C19').setValues(baseline);
  sheet.getRange('A15:C19').setBackground('#f8f9fa');
  sheet.getRange('A15:C15').setFontWeight('bold');

  // Freeze header rows
  sheet.setFrozenRows(4);
}

function setupSummarySheet(sheet) {
  sheet.clear();

  // Title
  sheet.getRange('A1:E1').merge();
  sheet.getRange('A1').setValue('📊 Tuning Summary & Statistics')
    .setFontSize(16)
    .setFontWeight('bold')
    .setBackground('#3498db')
    .setFontColor('#ffffff')
    .setHorizontalAlignment('center');

  sheet.getRange('A3').setValue('Click "🎯 Tuning Tools" → "📈 Generate Summary" to populate this sheet.')
    .setFontStyle('italic');
}

function setupCodeSheet(sheet) {
  sheet.clear();

  // Title
  sheet.getRange('A1:C1').merge();
  sheet.getRange('A1').setValue('📋 Code Export - Copy to ShooterConstants.java')
    .setFontSize(16)
    .setFontWeight('bold')
    .setBackground('#28a745')
    .setFontColor('#ffffff')
    .setHorizontalAlignment('center');

  sheet.getRange('A3').setValue('Click "🎯 Tuning Tools" → "📋 Export for Code" to generate Java code.')
    .setFontStyle('italic');
}

function calculateStats() {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  const dataSheet = ss.getSheetByName('Test Data');

  if (!dataSheet) {
    SpreadsheetApp.getUi().alert('Error: Test Data sheet not found. Run "Setup Sheet" first.');
    return;
  }

  // Get data range (rows 5-10, columns C-H)
  const data = dataSheet.getRange('C5:H10').getValues();

  // Filter out empty rows
  const validData = data.filter(row => row[0] !== '' && row[3] !== '');

  if (validData.length === 0) {
    SpreadsheetApp.getUi().alert('No data found. Please enter test results first.');
    return;
  }

  // Calculate statistics
  let totalRPMChange = 0;
  let totalShots = 0;

  validData.forEach(row => {
    const startRPM = row[2];
    const finalRPM = row[3];
    const shots = row[5];

    if (startRPM && finalRPM) {
      totalRPMChange += (finalRPM - startRPM);
    }
    if (shots) {
      totalShots += shots;
    }
  });

  const avgRPMChange = totalRPMChange / validData.length;
  const avgShots = totalShots / validData.length;

  SpreadsheetApp.getUi().alert(
    `Statistics:\n\n` +
    `Tests Completed: ${validData.length}\n` +
    `Avg RPM Adjustment: ${avgRPMChange.toFixed(0)} RPM\n` +
    `Avg Shots to Converge: ${avgShots.toFixed(1)}\n` +
    `Total Test Shots: ${totalShots}\n\n` +
    `Click "Generate Summary" for detailed report.`
  );
}

function generateSummary() {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  const dataSheet = ss.getSheetByName('Test Data');
  const summarySheet = ss.getSheetByName('Summary');

  if (!dataSheet || !summarySheet) {
    SpreadsheetApp.getUi().alert('Error: Required sheets not found. Run "Setup Sheet" first.');
    return;
  }

  summarySheet.clear();
  setupSummarySheet(summarySheet);

  // Get data
  const data = dataSheet.getRange('C5:H10').getValues();
  const validData = data.filter(row => row[0] !== '' && row[3] !== '');

  if (validData.length === 0) {
    summarySheet.getRange('A3').setValue('No data to summarize yet. Complete some tests first!');
    return;
  }

  // Summary table
  let row = 3;
  summarySheet.getRange(`A${row}`).setValue('Distance (m)')
    .setFontWeight('bold')
    .setBackground('#3498db')
    .setFontColor('#ffffff');
  summarySheet.getRange(`B${row}`).setValue('Final RPM')
    .setFontWeight('bold')
    .setBackground('#3498db')
    .setFontColor('#ffffff');
  summarySheet.getRange(`C${row}`).setValue('Old RPM (68°)')
    .setFontWeight('bold')
    .setBackground('#3498db')
    .setFontColor('#ffffff');
  summarySheet.getRange(`D${row}`).setValue('% Change')
    .setFontWeight('bold')
    .setBackground('#3498db')
    .setFontColor('#ffffff');
  summarySheet.getRange(`E${row}`).setValue('Shots')
    .setFontWeight('bold')
    .setBackground('#3498db')
    .setFontColor('#ffffff');

  row++;

  // Baseline reference
  const baseline = {
    1.75: 2950,
    2.69: 3450,
    3.3: 4000,
    4.8: 4500
  };

  validData.forEach(testRow => {
    const distance = testRow[1]; // Vision distance
    const finalRPM = testRow[3];
    const shots = testRow[5];

    // Find closest baseline
    let closestDist = 1.75;
    let minDiff = Math.abs(distance - 1.75);

    Object.keys(baseline).forEach(dist => {
      const diff = Math.abs(distance - parseFloat(dist));
      if (diff < minDiff) {
        minDiff = diff;
        closestDist = parseFloat(dist);
      }
    });

    const oldRPM = baseline[closestDist];
    const percentChange = ((finalRPM - oldRPM) / oldRPM * 100).toFixed(1);

    summarySheet.getRange(`A${row}`).setValue(distance);
    summarySheet.getRange(`B${row}`).setValue(finalRPM);
    summarySheet.getRange(`C${row}`).setValue(oldRPM);
    summarySheet.getRange(`D${row}`).setValue(`${percentChange}%`);
    summarySheet.getRange(`E${row}`).setValue(shots || '');

    // Color code percent change
    const changeCell = summarySheet.getRange(`D${row}`);
    if (parseFloat(percentChange) < -5) {
      changeCell.setBackground('#d4edda'); // Green - good reduction
    } else if (parseFloat(percentChange) > 0) {
      changeCell.setBackground('#fff3cd'); // Yellow - increase (unexpected)
    }

    row++;
  });

  // Statistics
  row += 2;
  summarySheet.getRange(`A${row}`).setValue('📊 Overall Statistics:')
    .setFontWeight('bold')
    .setFontSize(14);
  row++;

  let totalRPMChange = 0;
  let totalShots = 0;

  validData.forEach(testRow => {
    const startRPM = testRow[2];
    const finalRPM = testRow[3];
    const shots = testRow[5];

    if (startRPM && finalRPM) {
      totalRPMChange += (finalRPM - startRPM);
    }
    if (shots) {
      totalShots += shots;
    }
  });

  const avgRPMChange = totalRPMChange / validData.length;
  const avgShots = totalShots / validData.length;

  summarySheet.getRange(`A${row}`).setValue('Tests Completed:');
  summarySheet.getRange(`B${row}`).setValue(validData.length);
  row++;

  summarySheet.getRange(`A${row}`).setValue('Total Test Shots:');
  summarySheet.getRange(`B${row}`).setValue(totalShots);
  row++;

  summarySheet.getRange(`A${row}`).setValue('Avg Shots to Converge:');
  summarySheet.getRange(`B${row}`).setValue(avgShots.toFixed(1));
  row++;

  summarySheet.getRange(`A${row}`).setValue('Avg RPM Adjustment:');
  summarySheet.getRange(`B${row}`).setValue(`${avgRPMChange.toFixed(0)} RPM`);

  SpreadsheetApp.getUi().alert('✓ Summary generated successfully!');
}

function exportForCode() {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  const dataSheet = ss.getSheetByName('Test Data');
  const codeSheet = ss.getSheetByName('Code Export');

  if (!dataSheet || !codeSheet) {
    SpreadsheetApp.getUi().alert('Error: Required sheets not found.');
    return;
  }

  codeSheet.clear();
  setupCodeSheet(codeSheet);

  // Get data
  const data = dataSheet.getRange('C5:H10').getValues();
  const validData = data.filter(row => row[0] !== '' && row[3] !== '');

  if (validData.length === 0) {
    codeSheet.getRange('A3').setValue('No data to export yet. Complete some tests first!');
    return;
  }

  // Sort by distance
  validData.sort((a, b) => a[1] - b[1]);

  // Generate Java code
  let row = 3;
  codeSheet.getRange(`A${row}`).setValue('// File: src/main/java/frc/robot/constants/ShooterConstants.java')
    .setFontFamily('Courier New')
    .setBackground('#2c3e50')
    .setFontColor('#ecf0f1');
  row += 2;

  codeSheet.getRange(`A${row}`).setValue('// NEW velocity table (62° launch angle - tuned ' + new Date().toLocaleDateString() + '):')
    .setFontFamily('Courier New');
  row++;

  codeSheet.getRange(`A${row}`).setValue('public static final double[][] kShooterVelocityTable = {')
    .setFontFamily('Courier New')
    .setFontWeight('bold');
  row++;

  validData.forEach((testRow, index) => {
    const distance = testRow[1].toFixed(2); // Vision distance
    const rpm = Math.round(testRow[3]); // Final RPM
    const comma = index < validData.length - 1 ? ',' : '';

    codeSheet.getRange(`A${row}`).setValue(`  {${distance}, ${rpm}}${comma}`)
      .setFontFamily('Courier New');
    row++;
  });

  codeSheet.getRange(`A${row}`).setValue('};')
    .setFontFamily('Courier New')
    .setFontWeight('bold');
  row += 2;

  codeSheet.getRange(`A${row}`).setValue('📋 Copy the code above and paste into ShooterConstants.java')
    .setFontStyle('italic')
    .setBackground('#d4edda');
  row++;

  codeSheet.getRange(`A${row}`).setValue('⚠️ Remember to build, deploy, disable Test Mode, and validate!')
    .setFontStyle('italic')
    .setBackground('#fff3cd');

  // Auto-resize
  codeSheet.autoResizeColumn(1);

  SpreadsheetApp.getUi().alert('✓ Java code generated!\n\nCopy from the "Code Export" sheet and paste into ShooterConstants.java');
}

function resetData() {
  const ui = SpreadsheetApp.getUi();
  const response = ui.alert(
    '⚠️ Reset Data',
    'This will clear ALL test data. Are you sure?',
    ui.ButtonSet.YES_NO
  );

  if (response == ui.Button.YES) {
    const ss = SpreadsheetApp.getActiveSpreadsheet();
    const dataSheet = ss.getSheetByName('Test Data');

    if (dataSheet) {
      dataSheet.getRange('C5:I10').clear();

      // Re-add formulas
      for (let i = 5; i <= 10; i++) {
        dataSheet.getRange(`B${i}`).setFormula(`=IF(C${i}<>"", NOW(), "")`).setNumberFormat('M/d/yyyy h:mm');
        dataSheet.getRange(`G${i}`).setFormula(`=IF(AND(E${i}<>"", F${i}<>""), F${i}-E${i}, "")`);
      }

      ui.alert('✓ Data cleared successfully!');
    }
  }
}
```

---

## Section B: Quick Start Guide

### During Testing:

1. **Open your Google Sheet** on a laptop/tablet at the field
2. **Go to "Test Data" tab**
3. **For each test position:**
   - Enter **Measured Distance** (tape measure)
   - Enter **Vision Distance** (from `Shooter/Distance to Target`)
   - Enter **Starting RPM** (your initial guess)
   - Enter **Final Working RPM** (after converging to 3 successful shots)
   - Enter **Shots to Converge** (how many test shots it took)
   - Enter **Notes** (battery voltage, any issues, etc.)

4. **Green cells** in "RPM Change" = good (velocity decreased as expected)
5. **Yellow cells** = unexpected (velocity increased - check your data)

### After Testing:

1. Click **🎯 Tuning Tools** → **📈 Generate Summary**
   - See statistics and percent changes

2. Click **🎯 Tuning Tools** → **📋 Export for Code**
   - Get ready-to-paste Java code for `ShooterConstants.java`

---

## Section C: Features

✓ **Auto-calculation** of RPM changes
✓ **Date/time stamps** for each test
✓ **Data validation** (prevents negative distances/RPMs)
✓ **Color coding** (green = good decrease, yellow = unexpected increase)
✓ **Statistics** (avg shots, avg RPM change, total tests)
✓ **Java code generator** (ready to paste into your code)
✓ **68° baseline reference** (for comparison)
✓ **Reset function** (clear all data to start fresh)

---

## Section D: Tips

- **Share the sheet** with your team for collaborative data entry
- **Use on mobile** for quick data entry at the field (works in Google Sheets app)
- **Keep it open** during testing for real-time tracking
- **Take screenshots** of the summary for documentation
- **Export as CSV** for backup: File → Download → CSV

---

## Troubleshooting

**"Authorization required":** Click "Review Permissions" → Choose your Google account → "Allow"
**Menu not showing:** Close and reopen the sheet, or run `onOpen()` manually from Apps Script
**Formulas not working:** Make sure you ran `setupSheet` from Apps Script editor
