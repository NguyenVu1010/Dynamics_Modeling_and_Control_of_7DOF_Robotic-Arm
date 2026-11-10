# cobot_studio: Configuration tab redesign

Date: 2026-11-10. Status: draft for owner review.
Builds on: `2026-11-02-cobot-studio-design.md` (ADR-021), the Configuration tab of phase A.

## 1. Why

The owner finds the flow of cobot_studio acceptable, but the look poor and the Configuration
tab hard to use, in three places: parameter input, noise, and the buttons around them. Earlier
feedback (2026-11-07) said display fields look like input fields.

The current tab is a form (base pair, omega, advanced gains, HAL timeout, four tolerances,
speed, waypoints, duration, seed), then a row of five equal buttons (Open, Save, Export for
cobot_bench, Reset to base, Apply to Gazebo), then one `QTreeWidget` of about 50 catalog keys.
Each per-joint key is a row of seven `QDoubleSpinBox` cells showing 9 decimals
(`0.000001745 rad`), and every row has a `reset` button. Keys are shown raw
(`sensor.q_noise_std_rad`). Nothing shows which values differ from the base pair.

On 2026-11-10 the owner looked at three mockups and chose option B (section 3). The
description in this document was played back to the owner and confirmed.

## 2. Goals, non-goals, anti-goals

Goals:
- A Configuration tab where finding a parameter, reading its value and seeing what changed
  takes no effort.
- Noise can be set to a common level in one click.
- A per-joint parameter is edited as one value unless the user asks for seven.
- One clear primary action, Apply to Gazebo. File actions are secondary.
- A consistent light theme for the whole application.

Non-goals (this round):
- Restructuring the Control, Tracking and Verification tabs. They only get the theme.
- Qt 6, Qlementine, ElaWidgetTools, frameless windows, docking panels.
- Changing parameter values, limits, the catalog's key set, the C8 or C13 owner items, or what
  Apply, Export, Open, Save and Sweep do.

Anti-goals (a failure even if it looks good):
- A capability of the current tab is lost. Every catalog key stays editable. The other
  behaviours stay as they are:
  - Gazebo-unsupported keys are greyed with "not supported on Gazebo" while connected;
  - omega and per-joint kp/kd are mutually exclusive;
  - switching the base pair asks before discarding changes;
  - Open, Save, Export, Reset and Sweep work as now.
- A stored value changes because of display rounding. A value the user did not edit is
  materialised bit-for-bit as the base or override value.
- Editing a shared "all 7 joints" field silently overwrites joints that held different values.
- The tab becomes slower to start or to switch base pairs in a way a user notices (now
  instant).

## 3. Layout (owner's choice: option B)

```
+--------------------------------------------------------------------------+
| Configuration  [budget v]  (3 changes)            Open  Save  Export bench |  top bar
+----------------+---------------------------------------------------------+
| Controller   1 |  Noise level  [Off | Profile | x2 | x5 | Custom]        |
| Sensor/noise 2 |  +-----------------------------------------------------+ |
| Actuator       |  | Position noise q             3.50 urad, all joints | |  card
| Friction       |  | Encoder noise std.  sensor.q_noise_std_rad   [Per joint >] [undo] |
| Model & load   |  +-----------------------------------------------------+ |
| Transmission   |  | Velocity noise q'            1.00 mrad/s, all joints  | |
| Timing & comm  |  ...                                                    |
| Run            |                                                         |
+----------------+---------------------------------------------------------+
| [Reset to base]                     Gazebo: connected   [Apply to Gazebo] |  bottom bar
+--------------------------------------------------------------------------+
```

- **Top bar:**
  - the base pair combo;
  - a change counter ("3 changes", hidden at 0);
  - Open, Save and Export for cobot_bench as flat secondary buttons.
- **Left:** a list of the catalog groups plus a "Run" page. The order and names are in 3.1.
  A group with changes shows its count.
- **Right:** a scrollable page of cards for the selected group, one card per catalog key.
- **Bottom bar, always visible:**
  - left: Reset to base;
  - right: the bridge state text and Apply to Gazebo, the only primary-styled button.
  - Apply's enabled rule is unchanged.

### 3.1 Pages

| Page | Content |
|---|---|
| Controller | omega, Advanced gains toggle (reveals per-joint kp, kd), HAL timeout, soft margin, friction compensation, deflection compensation |
| Sensor / noise | noise level selector, then the Sensor keys |
| Actuator | Actuator keys (bus keys as one "Supply bus" card is fine) |
| Friction | Friction keys |
| Model & load | Model keys |
| Transmission | Transmission keys |
| Timing & comm | Timing and Comm keys (all bench-only) |
| Run | speed slider, four tolerances, waypoint file, duration, seed |

The Run page holds settings of a run, not of the hardware. Moving them out of the always-visible
form is the builder's call made here, played back to the owner as a guess and not objected to.

### 3.2 A card

- **Title:** a readable name. **Hint line:** a one-line description, followed by the raw key in
  small grey monospace, which stays visible and searchable.
- **Value:** shown in a display unit chosen so that typical values read as 0.1 to 1000. For
  example rad noise is shown in µrad, rad/s noise in mrad/s, N m stays N m. Three significant
  digits are shown; the full value is in the tooltip.
- **Editor:** each value is one spin box in the display unit (builder's call, made here). It
  shows three significant digits while it does not have focus, and enough precision that any
  value a user types round-trips while it has focus. There is no separate read-only label.
  Values in this tab are inputs, so they look like inputs. The owner's "display fields look
  like inputs" point is about result displays, which are outside this round. Editing converts back to the YAML unit. An unedited cell
  never writes back a rounded value (anti-goal).
- **Changed values:** a value that differs from the base pair has a highlighted (amber)
  background, and the card shows an undo button that removes that key's override. Cards
  without changes show no undo button.
- **Bool and choice keys:** a bool is a switch (a checkable `QAbstractButton`). A choice is a
  segmented button group or a combo box.
- **Gazebo-unsupported keys while connected:** the card's editors are disabled, and the card
  and its editors carry the tooltip "not supported on Gazebo".

### 3.3 Per-joint keys

- **Collapsed (default):**
  - When all seven values are equal, the card shows one field labelled "all joints". Editing
    it sets all seven.
  - When values differ, the card shows "differs by joint" with the range (min to max) and
    opens the seven fields itself. The shared field is not offered until the values are equal
    again.
- **Expanded:** "Per joint >" toggles a J1..J7 row of fields, each with its own amber state.
- `model.payload_offset_m` (x, y, z, not joints) is always three fields labelled x, y, z.

### 3.4 Noise level

A segmented selector at the top of the Sensor / noise page: Off, Profile, x2, x5, Custom.

- **Scaled keys:**
  - `sensor.q_noise_std_rad`
  - `sensor.q_offset_std_rad`
  - `sensor.qd_noise_std_rad_s`
  - `actuator.torque_scale_std`
  - `timing.period_jitter_std_us`
- **Choosing a level** sets every scaled key to the base value times k (k = 0, 1, 2, 5) and
  every joint likewise. k = 1 removes those overrides.
- **Custom** cannot be clicked. It is lit when the current values are not one exact multiple
  of the base for all scaled keys, for example after a hand edit.
- **Shown level:** recomputed from the values on every change, open, reset and base switch.
- **Off** with a base value of 0 stays 0. A base of 0 with x2 stays 0, so such a key does not
  force Custom.
- **Gazebo:** the selector scales all five keys even while Gazebo is connected, including the
  bench-only `timing.period_jitter_std_us` (builder's call, made here). Custom detection
  includes all five, and the change counter counts the jitter key like any other override.
  This is because the same configuration is also exported for cobot_bench.
- **Limits:** no clamping. The five keys have no catalog maximum. A value the loaders reject
  is reported by Apply, Export or Run, as today (builder's call, made here).

### 3.5 Theme

- **Source:** the light theme of BreezeStyleSheets (MIT), applied to the whole application at
  start-up.
- **Vendoring:** the generated stylesheet and its icon resources are kept under
  `third_party/breeze/` with the licence. This avoids a Python step and a network fetch in the
  build.
- **Cards, the amber change state and the primary button:** a small project QSS on top of
  Breeze.
- **QtCharts:** not covered by the theme; charts keep their current light chart theme.
- **Tests:** they run on the `offscreen` platform and must not depend on the theme.

## 4. Contracts kept and changed

- **Object names that stay** (tests and the window-wide lookup rely on them):
  - Configuration tab widgets: `configTab`, `baseCombo`, `omegaSpin`, `advancedGains`,
    `halTimeoutSpin`, `goalPosSpin`, `goalRotSpin`, `pathPosSpin`, `pathRotSpin`,
    `speedSlider`, `waypointsEdit`, `durationSpin`, `seedSpin`;
  - buttons: `openButton`, `saveButton`, `exportButton`, `resetButton`, `applyGazeboButton`.
  - Their widget types and limits do not change: omega [10, 200], HAL timeout [1, 50], speed
    1 to 100 with a tooltip.
- **New names, replacing `paramTree`:**
  - `groupList`: the left list, one row per page of 3.1 (eight rows, Timing and Comm share one).
  - `paramPages`: the stacked pages.
  - `param:<key>`: one card per catalog key except omega and HAL timeout. The card's editors
    are its children.
  - `perJoint:<key>`: the expand toggle.
  - `noiseLevel`: the selector; its buttons are `noiseOff`, `noiseProfile`, `noiseX2`,
    `noiseX5` and `noiseCustom`.
  - `changeCount`: the counter.
- **Frozen tests:** `tests/component/studio_app` and `studio_control` test `paramTree` (group
  count, greyed rows). They are updated to the new names in `test(` commits under the
  `test-change` label. The behaviour they check stays the same.
- **Tab titles and the other tabs:** unchanged.

## 5. Delivery

One feature per PR, merged on green CI:

1. **Theme and catalog presentation.**
   - Vendored Breeze light and the project QSS, applied in `main.cpp`.
   - The catalog gains, per key, a readable name, a one-line description, a display unit and
     its scale.
   - Includes a unit test that every catalog key has them and that display/YAML conversion
     round-trips.
2. **Configuration tab layout.**
   - Group list, pages, cards, top and bottom bars, the Run page.
   - Shared and per-joint fields, amber state and undo, Gazebo greying, change counter.
   - Frozen test updates under `test-change`.
3. **Noise level selector**, with component tests of 3.4.
4. **Docs.** The studio guide section and screenshots of the new tab in `docs/media`.

## 6. Left to the builder

- Widget choices: `QListWidget` or `QListView` for the group list, a segmented control from a
  `QButtonGroup` of checkable `QToolButton`s, card spacing, colours within the Breeze
  palette, icons.
- The display unit per key, following 3.2.
- Wording of names and descriptions in the catalog, kept short. English (section 7).
- How the change counter is computed, as long as it equals the number of overridden keys.

## 7. Language

New labels and descriptions are English, through `tr()`, like the rest of the application.
The owner decided this on 2026-11-10. The mockups were in Vietnamese only for the
discussion.
