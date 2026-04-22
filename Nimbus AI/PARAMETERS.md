# Aircraft Design Parameters

This file is the single source of truth for all fixed design parameters, measured values, and data file references. Update it as new information becomes available. Claude reads this before any physics analysis or code work.

All values sourced from `main.m` unless otherwise noted.

---

## Mission Requirements

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Payload mass | 800 | g | Wp_g |
| Payload weight | 7.848 | N | Wp = (800/1000) × 9.81 |
| Payload volume (actual) | 0.002 | m³ | Vp |
| Payload volume (reference) | 0.001 | m³ | Vp_ref — penalty baseline |
| Volume scaling ratio (VPS) | 2.0 | — | Vp / Vp_ref |
| Cruise speed | 20 | m/s | V_cruise |
| Cruise range | 18000 | m | R_cruise |
| Climb altitude | 120 | m | delta_h |
| Flight time (measured, 1 lap) | 61 | s | Tf_measured |
| Number of competition laps | 3 | — | mission.nLaps |
| Competition score scaling | ×20 | — | Ef and Tf scaled ×20 for scoring |

---

## Competition / Field Geometry

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Runway length | 138.35 | m | mission.runwayLength_m |
| Lap length target | 407.103 | m | mission.lapLengthTarget_m |
| Straight length | 140.0 | m | mission.straightLength_m |
| Liftoff fraction of runway | 0.85 | — | mission.liftoffFrac |
| Touchdown fraction | 0.333 | — | mission.touchdownFrac |
| Cruise altitude | 30 | m | mission.h_cruise |
| Competition site | SeaWorld, San Diego | — | June flight conditions |

---

## Aircraft Weight Budget

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Propulsion system weight | 2.43341 | N | Wprop — measured |
| Empty weight fraction (baseline) | 0.450 | — | fe |
| Empty weight fraction (max cap) | 0.60 | — | fe_max |
| L/D penalty slope | 0.02 | — | kd per unit VPS beyond reference |
| Empty-wt penalty slope | 0.0375 | — | ke = fe/12 per unit VPS |
| Propulsion efficiency | 0.75 | — | eta_p |
| Energy reserve factor | 1.15 | — | reserve_factor |

---

## Aerodynamics

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Aspect ratio (AR) | 8.5 | — | CAD design variable |
| Wing taper ratio | 0.85 | — | wingTapper |
| Wing sweep (c/4) | 30 | deg | wingSweep |
| Oswald efficiency (e) | 0.80 | — | first-pass |
| Parasite drag (CD0) | 0.030 | — | first-pass; updated by drag build-up |
| CLmax | 1.8 | — | first-pass; updated by drag build-up |
| L/D (baseline) | 20 | — | LD; penalized by VPS |
| Root airfoil | e222.dat | — | Eppler 222 |
| Tip airfoil | e230.dat | — | Eppler 230 |
| Reference AoA (spanwise estimate) | 6.5 | deg | spanIn.alpha_ref_deg |
| Static margin target | 10–20 | % MAC | stabIn.SM_target_min/max |

---

## Mission Profile (Climb / Descent)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Climb speed | 14.0 | m/s | mission.V_climb_mps |
| Climb angle | 11.0 | deg | mission.gamma_climb_deg |
| Descent speed | 25.0 | m/s | mission.V_descent_mps |
| Descent angle | 12.0 | deg | mission.gamma_descent_deg |
| Turn load factor | 1.6 | — | mission.n_turn |

---

## Propulsion

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Motor KV | 1100 | RPM/V | propIn.KV |
| Motor resistance (Rm) | 0.073 | Ω | propIn.Rm |
| No-load current (I0) | 0.9 | A | propIn.I0 |
| Battery voltage | 11.1 | V | 3S LiPo |
| Max current | 35 | A | propIn.I_max |
| Propeller | 10×4.5MR | — | APC propeller |
| Propeller diameter | 10 | in | propIn.D_in |
| Propeller pitch | 4.5 | in | propIn.pitch_in |

---

## Geometry (CAD-Derived)

All values from OnShape CAD model unless noted.

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Wing root LE x-location | 0.0822 | m | Imported OnShape 4/21/2026 |
| Wing root y-location | 0.145 | m | Imported OnShape 4/20/2026 |
| Body length (Lf) | 0.5567 | m | Drag build-up input |
| Body max width (Wf) | 0.1664 | m | Drag build-up input |
| Body max height (Hf) | 0.1015 | m | Drag build-up input |
| Wing wetted area | 0.50845 | m² | Swet_wing — from CAD |
| Fuselage wetted area | 0.20136 | m² | Swet_fuse — from CAD |
| Fin wetted area (total) | 0.10996 | m² | Swet_fin — from CAD |
| Thickness-to-chord ratio (t/c) | 0.12 | — | tc — representative |
| x/c at max thickness | 0.30 | — | xc |
| Interference factor (wing) | 1.10 | — | Q_wing |
| Interference factor (fuse) | 1.10 | — | Q_fuse |
| Interference factor (fin) | 1.10 | — | Q_fin |

---

## Mass Properties

| Component | Mass | Unit | Notes |
|-----------|------|------|-------|
| Fuselage (CAD) | 0.605 | kg | cadMass.fuselageOnly — replace with final CAD |
| Fuselage CG | [0.2699, 0.0, 0.0209] | m | cadMass.fuselageOnly.cg_m |
| Wing structure (both halves) | 0.392 | kg | First-pass placeholder |
| Vertical structure (both fins) | 0.040 | kg | First-pass placeholder |
| Main motor | 0.085 | kg | |
| Main propeller | 0.020 | kg | |
| ESC | 0.051 | kg | |
| Battery | 0.300 | kg | At x = 0.33 m — movable for CG tuning |
| Receiver | 0.015 | kg | |
| Each servo (×5) | 0.009 | kg | Wing ×2, back wing ×1, vert stab ×1, cargo ×1 |

---

## Atmosphere & Environment

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Air density (ρ) | 1.19 | kg/m³ | June, near SeaWorld San Diego |
| Dynamic viscosity (μ) | 1.789×10⁻⁵ | kg/(m·s) | Standard sea level |
| Mach number (cruise) | ~0.06 | — | V=20 m/s, incompressible flow |

---

## Structures (First-Pass)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Allowable stress (carbon fiber) | 200 | MPa | sigma_allow |
| Young's modulus (carbon fiber) | 70 | GPa | E |
| Carbon fiber density | 1600 | kg/m³ | rho_cf |
| Safety factor | 2.0 | — | SF |
| Selected spar diameter | 10 | mm | d_selected — solid circular CF spar |
| Landing drop height (assumed) | 0.3 | m | h_drop |

---

## V-n Diagram

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Positive limit load factor | 3.8 | — | n_pos_limit |
| Negative limit load factor | -1.5 | — | n_neg_limit |
| Gust velocity at Vc | 30 | ft/s | Ude_Vc_fps — first-pass |
| Gust velocity at Vd | 15 | ft/s | Ude_Vd_fps — first-pass |
| Dive speed | 1.25 × Vc | m/s | First-pass assumption |

---

## Vertical Stabilizer

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Configuration | Twin fin | — | isTwin = true |
| Sizing method | Tail volume coeff | — | sizeMode = 'tailVolumeCoeff' |
| Vertical tail volume coeff (c_v) | 0.04 | — | Total system |
| Fin aspect ratio | 2.0 | — | AR_v |
| Fin taper ratio | 0.60 | — | taper_v |
| Fin quarter-chord sweep | 30.0 | deg | sweep_c4_v_deg |
| Fin airfoil | NACA0010 | — | |
| Top area fraction | 0.66 | — | topFrac |

---

## Data Files

Excel files and datasets stored in `data/`. Add entries here as files are added.

| File | Location | Description |
|------|----------|-------------|
| all_prop_surrogates.mat | `data/propellers/` | APC propeller surrogate model (used by propulsionAnalysis) |
| e222.dat | root of repo | Eppler 222 airfoil coordinates |
| e230.dat | root of repo | Eppler 230 airfoil coordinates |

---

## Computed Outputs (from last main.m run — 2026-04-22)

| Parameter | Value | Unit |
|-----------|-------|------|
| Gross weight (Wg) | 20.061 | N |
| Gross weight | 2044.98 | g |
| Empty weight (We) | 9.780 | N |
| Wing area (S_ref) | 0.2083 | m² |
| Full span (b) | 1.3307 | m |
| Root chord | 0.1692 | m |
| Tip chord | 0.1439 | m |
| MAC | 0.1569 | m |
| Wing loading (W/S) design | 96.29 | N/m² |
| Thrust/weight design | 0.6055 | — |
| Required thrust | 12.147 | N |
| Static thrust (propulsion) | 17.766 | N |
| Climb energy (E_climb) | 3209.81 | J |
| Cruise energy (E_cruise) | 24555.02 | J |
| Total mission energy (E_f) | 27764.82 | J = 7.71 Wh |
| Design energy w/ reserve | 31929.55 | J = 8.87 Wh |
| Profit score J | 0.000472 | $/s = 1.70 $/hr |
| Root Cl_alpha | 0.12129 | per deg |
| Root alphaL0 | -2.751 | deg |
| Root Cm0 | -0.08234 | — |
| Root Cl_max | 1.208 | — |
| Tip Cl_alpha | 0.13210 | per deg |
| Tip alphaL0 | 0.393 | deg |
| Tip Cm0 | 0.02243 | — |
| Tip Cl_max | 0.797 | — |
| Aerodynamic twist | -3.145 | deg |
| Required tip geometric twist | -5.029 | deg |
| Single-fin area | 0.0276 | m² |
| Total vertical area | 0.0552 | m² |
| Fin span | 0.2350 | m |

---

## Known Issues / Flags

- `aeroPolarAircraft.m` **does not exist in the repo** — `main.m` crashes at the drag build-up section (line 871). This function needs to be created in `src/aerodynamics/` or located and moved from MATLAB Drive.
- `energyCalc.m` was previously missing but is now found after running `run_project.m` — confirmed working. — it lives on MATLAB Drive (`/Users/JuanManuelSanchez/MATLAB-Drive/MAE 155B`). Running `main.m` will fail at the energy calculation step until this is moved into `src/energy/`.
- Fuselage CAD mass and inertia tensor are marked as placeholders — replace with final OnShape export.
- Wing and vertical structure masses are first-pass estimates — update from CAD or structural sizing.
- Neutral point uses approximate wing AC only (`useApproxNP = true`) — replace with AVL result before trusting stability margins.
