"""
Tensile Test Post-Processing
- Compliance correction via lookup table
- Stress-strain calculation
- E-modulus, tensile strength, elongation at break
- Step loss detection flag
- Multi-specimen overlay and statistics

Commands:
  plot        - Show force-displacement and stress-strain plots
  results     - Print calculated results
  save        - Save corrected data and results
  set         - Change parameters (e.g. "set width 5.12")
  recalc      - Recalculate after parameter change
  add         - Load another specimen for comparison
  overlay     - Plot all loaded specimens in one chart
  stats       - Print mean ± SD across all loaded specimens
  help        - Show commands
  quit        - Exit
"""

import csv
import os
import numpy as np
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    PLOTTING = True
except ImportError:
    PLOTTING = False
    print("Warning: matplotlib not available, plotting disabled")

COLORS = ['#2196F3', '#F44336', '#4CAF50', '#FF9800', '#9C27B0', '#00BCD4',
          '#795548', '#E91E63', '#009688', '#FF5722']


# === COMPLIANCE LOOKUP ===

def load_lookup(path="compliance_lookup.csv"):
    """Load compliance lookup table (force_N -> system_displacement_mm)"""
    f_lookup, d_lookup = [], []
    with open(path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        next(reader)  # header
        for row in reader:
            f_lookup.append(float(row[0]))
            d_lookup.append(float(row[1]))
    return np.array(f_lookup), np.array(d_lookup)


def compliance_correction(force, displacement, f_lookup, d_lookup):
    """
    Correct displacement for system compliance.
    d_corrected = d_measured - lookup(F)
    """
    system_disp = np.interp(force, f_lookup, d_lookup)
    return displacement - system_disp


# === DATA LOADING ===

def load_test_csv(path):
    """Load tensile test CSV file (compatible with/without step_loss column)"""
    time_s, steps, disp, force_raw, force_N = [], [], [], [], []
    accel_x, accel_y, accel_z, endstop, step_loss = [], [], [], [], []
    
    with open(path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        header = next(reader)
        for row in reader:
            time_s.append(float(row[0]))
            steps.append(int(row[1]))
            disp.append(float(row[2]))
            force_raw.append(int(row[3]))
            force_N.append(float(row[4]))
            accel_x.append(int(row[5]))
            accel_y.append(int(row[6]))
            accel_z.append(int(row[7]))
            endstop.append(int(row[8]))
            if len(row) > 9:
                step_loss.append(int(row[9]))
            else:
                step_loss.append(0)
    
    return {
        'time_s': np.array(time_s),
        'steps': np.array(steps),
        'displacement_mm': np.array(disp),
        'force_raw': np.array(force_raw),
        'force_N': np.array(force_N),
        'accel_x': np.array(accel_x),
        'accel_y': np.array(accel_y),
        'accel_z': np.array(accel_z),
        'endstop': np.array(endstop),
        'step_loss': np.array(step_loss),
    }


# === ANALYSIS ===

def analyze(data, f_lookup, d_lookup, width_mm, thickness_mm, grip_mm):
    """Run full analysis, return results dict"""
    
    force = data['force_N'].copy()
    disp = data['displacement_mm'].copy()
    step_loss = data.get('step_loss', np.zeros(len(force))).copy()
    
    area_mm2 = width_mm * thickness_mm
    
    # 1. Compliance correction
    disp_corr = compliance_correction(force, disp, f_lookup, d_lookup)
    
    # 2. Trim below preload and zero
    #    Removes unreliable settling region and tare offset effects
    PRELOAD_N = 10
    idx_preload = np.argmax(force >= PRELOAD_N)
    
    force = force[idx_preload:]
    disp = disp[idx_preload:]
    disp_corr = disp_corr[idx_preload:]
    step_loss = step_loss[idx_preload:]
    
    disp_corr = disp_corr - disp_corr[0]
    
    # 3. Stress and strain
    stress_MPa = force / area_mm2
    strain = disp_corr / grip_mm
    strain_pct = strain * 100
    
    # 4. Tensile strength (max stress)
    idx_max_stress = np.argmax(stress_MPa)
    tensile_strength = stress_MPa[idx_max_stress]
    force_at_max = force[idx_max_stress]
    
    # 5. E-modulus (linear fit between 0.05% and 0.25% strain)
    mask_emod = (strain_pct >= 0.05) & (strain_pct <= 0.25)
    if mask_emod.sum() >= 2:
        coeffs = np.polyfit(strain[mask_emod], stress_MPa[mask_emod], 1)
        e_modulus_MPa = coeffs[0]
        fit = np.polyval(coeffs, strain[mask_emod])
        ss_res = np.sum((stress_MPa[mask_emod] - fit) ** 2)
        ss_tot = np.sum((stress_MPa[mask_emod] - stress_MPa[mask_emod].mean()) ** 2)
        e_mod_r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
        e_mod_points = mask_emod.sum()
    else:
        e_modulus_MPa = float('nan')
        e_mod_r2 = float('nan')
        e_mod_points = 0
    
    # 6. Elongation at break
    elongation_at_break_pct = strain_pct[-1]
    
    return {
        'force_N': force,
        'displacement_mm': disp,
        'displacement_corr_mm': disp_corr,
        'stress_MPa': stress_MPa,
        'strain': strain,
        'strain_pct': strain_pct,
        'width_mm': width_mm,
        'thickness_mm': thickness_mm,
        'area_mm2': area_mm2,
        'grip_mm': grip_mm,
        'tensile_strength_MPa': tensile_strength,
        'force_at_max_N': force_at_max,
        'e_modulus_MPa': e_modulus_MPa,
        'e_modulus_GPa': e_modulus_MPa / 1000,
        'e_mod_r2': e_mod_r2,
        'e_mod_points': e_mod_points,
        'elongation_at_break_pct': elongation_at_break_pct,
        'step_loss': step_loss,
        'n_points': len(force),
    }


# === OUTPUT ===

def print_results(results, filepath):
    """Print analysis results"""
    print(f"\n{'='*50}")
    print(f"  TENSILE TEST ANALYSIS")
    print(f"{'='*50}")
    print(f"  File: {os.path.basename(filepath)}")
    print(f"  Data points: {results['n_points']}")
    print(f"")
    print(f"  Specimen:")
    print(f"    Width:       {results['width_mm']:.2f} mm")
    print(f"    Thickness:   {results['thickness_mm']:.2f} mm")
    print(f"    Area:        {results['area_mm2']:.2f} mm²")
    print(f"    Grip sep.:   {results['grip_mm']:.1f} mm")
    print(f"")
    print(f"  Results:")
    print(f"    Tensile strength:  {results['tensile_strength_MPa']:.1f} MPa  ({results['force_at_max_N']:.0f} N)")
    
    if not np.isnan(results['e_modulus_MPa']):
        print(f"    E-modulus:         {results['e_modulus_GPa']:.2f} GPa  (R²={results['e_mod_r2']:.4f}, {results['e_mod_points']} pts)")
    else:
        print(f"    E-modulus:         N/A (not enough data in 0.05-0.25% strain)")
    
    print(f"    Elong. at break:   {results['elongation_at_break_pct']:.2f}%")
    
    if np.any(results['step_loss']):
        idx_sl = np.argmax(results['step_loss'])
        print(f"    Step Loss:         DETECTED at {results['force_N'][idx_sl]:.0f} N / {results['strain_pct'][idx_sl]:.2f}% strain")
    else:
        print(f"    Step Loss:         none")
    
    print(f"{'='*50}\n")


def plot_results(results, filepath):
    """Show analysis plots"""
    if not PLOTTING:
        print("Plotting not available.")
        return
    
    plt.close('all')
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Tensile Test Analysis – {os.path.basename(filepath)}', fontsize=13)
    
    force = results['force_N']
    disp = results['displacement_mm']
    disp_corr = results['displacement_corr_mm']
    stress = results['stress_MPa']
    strain_pct = results['strain_pct']
    
    # 1: Force-Displacement raw vs corrected
    axes[0, 0].plot(disp, force, 'b-', linewidth=0.8, alpha=0.5, label='Raw')
    axes[0, 0].plot(disp_corr, force, 'r-', linewidth=1, label='Corrected')
    axes[0, 0].set_xlabel('Displacement [mm]')
    axes[0, 0].set_ylabel('Force [N]')
    axes[0, 0].set_title('Force-Displacement')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # 2: Stress-Strain full
    axes[0, 1].plot(strain_pct, stress, 'b-', linewidth=1)
    axes[0, 1].set_xlabel('Strain [%]')
    axes[0, 1].set_ylabel('Stress [MPa]')
    axes[0, 1].set_title('Stress-Strain Curve')
    axes[0, 1].grid(True)
    
    idx_max = np.argmax(stress)
    axes[0, 1].plot(strain_pct[idx_max], stress[idx_max], 'rv', markersize=10,
                     label=f'UTS = {results["tensile_strength_MPa"]:.1f} MPa')
    axes[0, 1].legend()
    
    # 3: E-modulus region (0-0.5% strain)
    mask = (strain_pct >= 0) & (strain_pct <= 0.5)
    if mask.sum() > 0:
        axes[1, 0].plot(strain_pct[mask], stress[mask], 'b-', linewidth=1, label='Data')
        
        if not np.isnan(results['e_modulus_MPa']):
            mask_fit = (strain_pct >= 0.05) & (strain_pct <= 0.25)
            if mask_fit.sum() > 0:
                coeffs = np.polyfit(results['strain'][mask_fit], stress[mask_fit], 1)
                strain_line = np.linspace(0, 0.5, 100)
                stress_line = np.polyval(coeffs, strain_line / 100)
                axes[1, 0].plot(strain_line, stress_line, 'r--', linewidth=1.5,
                                 label=f'E = {results["e_modulus_GPa"]:.2f} GPa')
            
            axes[1, 0].axvspan(0.05, 0.25, alpha=0.15, color='green', label='E-mod region')
        
        axes[1, 0].set_xlabel('Strain [%]')
        axes[1, 0].set_ylabel('Stress [MPa]')
        axes[1, 0].set_title('E-Modulus Region')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
    
    # 4: Compliance correction effect
    correction = disp - disp_corr
    axes[1, 1].plot(force, correction, 'g-', linewidth=1)
    axes[1, 1].set_xlabel('Force [N]')
    axes[1, 1].set_ylabel('Compliance Correction [mm]')
    axes[1, 1].set_title('Subtracted System Displacement')
    axes[1, 1].grid(True)
    
    # Step loss marker
    if np.any(results['step_loss']):
        idx_sl = np.argmax(results['step_loss'])
        sl_disp = disp_corr[idx_sl]
        sl_strain = strain_pct[idx_sl]
        sl_force = force[idx_sl]
        
        axes[0, 0].axvline(x=sl_disp, color='red', linestyle='--', linewidth=1.5,
                            label=f'Step Loss ({sl_force:.0f} N)')
        axes[0, 0].legend()
        axes[0, 1].axvline(x=sl_strain, color='red', linestyle='--', linewidth=1.5,
                            label=f'Step Loss ({sl_strain:.2f}%)')
        axes[0, 1].legend()
    
    plt.tight_layout()
    plt.show()
    print("Plot displayed.")


def save_results(results, filepath):
    """Save corrected CSV and results summary"""
    base = os.path.splitext(os.path.basename(filepath))[0]
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # 1. Corrected CSV
    csv_out = f"{base}_analyzed.csv"
    with open(csv_out, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerow(['displacement_mm', 'displacement_corr_mm', 'force_N',
                          'stress_MPa', 'strain_pct', 'step_loss'])
        for i in range(results['n_points']):
            writer.writerow([
                f"{results['displacement_mm'][i]:.4f}",
                f"{results['displacement_corr_mm'][i]:.4f}",
                f"{results['force_N'][i]:.3f}",
                f"{results['stress_MPa'][i]:.3f}",
                f"{results['strain_pct'][i]:.4f}",
                int(results['step_loss'][i]),
            ])
    print(f"Saved: {csv_out}")
    
    # 2. Results summary
    txt_out = f"{base}_results.txt"
    with open(txt_out, 'w') as f:
        f.write(f"Tensile Test Analysis\n")
        f.write(f"{'='*50}\n")
        f.write(f"Source file:        {os.path.basename(filepath)}\n")
        f.write(f"Analysis date:      {timestamp}\n")
        f.write(f"Data points:        {results['n_points']}\n\n")
        f.write(f"Specimen:\n")
        f.write(f"  Width:            {results['width_mm']:.2f} mm\n")
        f.write(f"  Thickness:        {results['thickness_mm']:.2f} mm\n")
        f.write(f"  Area:             {results['area_mm2']:.2f} mm²\n")
        f.write(f"  Grip separation:  {results['grip_mm']:.1f} mm\n\n")
        f.write(f"Results:\n")
        f.write(f"  Tensile strength: {results['tensile_strength_MPa']:.1f} MPa ({results['force_at_max_N']:.0f} N)\n")
        if not np.isnan(results['e_modulus_MPa']):
            f.write(f"  E-modulus:        {results['e_modulus_GPa']:.2f} GPa (R²={results['e_mod_r2']:.4f})\n")
        else:
            f.write(f"  E-modulus:        N/A\n")
        f.write(f"  Elong. at break:  {results['elongation_at_break_pct']:.2f}%\n")
        if np.any(results['step_loss']):
            idx_sl = np.argmax(results['step_loss'])
            f.write(f"  Step Loss:        DETECTED at {results['force_N'][idx_sl]:.0f} N\n")
        else:
            f.write(f"  Step Loss:        none\n")
    print(f"Saved: {txt_out}")


# === MAIN ===

def main():
    print("\n  Tensile Test Post-Processing")
    print("  " + "=" * 40)
    
    filepath = input("  File: ").strip()
    if not filepath:
        print("No file specified.")
        return
    
    width = float(input("  Width [mm]: ").strip())
    thickness = float(input("  Thickness [mm]: ").strip())
    grip = float(input("  Grip separation [mm]: ").strip())
    
    lookup_path = "compliance_lookup.csv"
    
    print(f"\nLoading {filepath}...")
    data = load_test_csv(filepath)
    print(f"  {len(data['force_N'])} data points loaded.")
    
    print(f"Loading compliance lookup: {lookup_path}...")
    f_lookup, d_lookup = load_lookup(lookup_path)
    print(f"  {len(f_lookup)} lookup points loaded.")
    
    results = analyze(data, f_lookup, d_lookup, width, thickness, grip)
    print_results(results, filepath)
    
    # Comparison list
    all_results = [(os.path.basename(filepath), results)]
    
    while True:
        try:
            cmd = input("> ").strip().lower()
            
            if cmd in ("quit", "exit"):
                break
            elif cmd == "plot":
                plot_results(results, filepath)
            elif cmd == "results":
                print_results(results, filepath)
            elif cmd == "save":
                save_results(results, filepath)
            elif cmd == "add":
                fp = input("  File: ").strip()
                w = float(input("  Width [mm]: ").strip())
                t = float(input("  Thickness [mm]: ").strip())
                g = float(input("  Grip separation [mm]: ").strip())
                d = load_test_csv(fp)
                r = analyze(d, f_lookup, d_lookup, w, t, g)
                all_results.append((os.path.basename(fp), r))
                print_results(r, fp)
                print(f"  [{len(all_results)} specimens loaded]")
            elif cmd == "overlay":
                if not PLOTTING:
                    print("Plotting not available.")
                else:
                    plt.close('all')
                    fig, ax = plt.subplots(figsize=(12, 7))
                    for i, (label, r) in enumerate(all_results):
                        ax.plot(r['strain_pct'], r['stress_MPa'],
                                color=COLORS[i % len(COLORS)], linewidth=1.2, label=label)
                    ax.set_xlabel('Strain [%]')
                    ax.set_ylabel('Stress [MPa]')
                    ax.set_title('Stress-Strain Overlay')
                    ax.legend(fontsize=8)
                    ax.grid(True, alpha=0.3)
                    ax.set_xlim(left=0)
                    ax.set_ylim(bottom=0)
                    plt.tight_layout()
                    plt.show()
                    print("Plot displayed.")
            elif cmd == "stats":
                if len(all_results) < 2:
                    print("Need at least 2 specimens. Use 'add' first.")
                else:
                    uts = [r['tensile_strength_MPa'] for _, r in all_results]
                    emod = [r['e_modulus_GPa'] for _, r in all_results]
                    elong = [r['elongation_at_break_pct'] for _, r in all_results]
                    print(f"\n{'='*50}")
                    print(f"  STATISTICS ({len(all_results)} specimens)")
                    print(f"{'='*50}")
                    for label, r in all_results:
                        print(f"  {label}: UTS={r['tensile_strength_MPa']:.1f}  E={r['e_modulus_GPa']:.2f}  ε={r['elongation_at_break_pct']:.1f}%")
                    print(f"  {'-'*46}")
                    print(f"  UTS:        {np.mean(uts):.1f} ± {np.std(uts, ddof=1):.1f} MPa")
                    print(f"  E-modulus:  {np.mean(emod):.2f} ± {np.std(emod, ddof=1):.2f} GPa")
                    print(f"  Elongation: {np.mean(elong):.1f} ± {np.std(elong, ddof=1):.1f}%")
                    print(f"{'='*50}\n")
            elif cmd.startswith("set "):
                parts = cmd.split()
                if len(parts) == 3:
                    param, val = parts[1], float(parts[2])
                    if param == "width":
                        width = val
                    elif param == "thickness":
                        thickness = val
                    elif param == "grip":
                        grip = val
                    else:
                        print(f"Unknown parameter: {param}")
                        continue
                    print(f"{param} = {val}")
                    results = analyze(data, f_lookup, d_lookup, width, thickness, grip)
                    print_results(results, filepath)
                else:
                    print('Usage: set <width|thickness|grip> <value>')
            elif cmd == "recalc":
                results = analyze(data, f_lookup, d_lookup, width, thickness, grip)
                print_results(results, filepath)
            elif cmd == "help":
                print("""
Commands:
  plot        - Show plots (current specimen)
  results     - Print results (current specimen)
  save        - Save corrected CSV and results
  set X Y     - Change parameter (width/thickness/grip)
  recalc      - Recalculate with current parameters
  add         - Load another specimen for comparison
  overlay     - Plot all specimens in one chart
  stats       - Print mean ± SD across all specimens
  help        - This help
  quit        - Exit
""")
            elif cmd == "":
                pass
            else:
                print(f"Unknown command: {cmd}. Type 'help'.")
        
        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except EOFError:
            break


if __name__ == "__main__":
    main()
