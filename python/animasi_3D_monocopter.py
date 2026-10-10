import numpy as np
import matplotlib
matplotlib.use('Agg')  # non-interactive, save to file
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.gridspec import GridSpec
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import os
import sys

print("=== Menginisialisasi animasi_3D_monocopter_v2 ===")

# --- PATH CONFIGURATION ---
HERE = os.path.dirname(os.path.abspath(__file__)) if '__file__' in locals() else os.getcwd()

# Cek lokasi file hasil_simulasi_full.npz secara adaptif
search_paths = [
    os.path.join(HERE, 'hasil-simulasi'),
    '/workspace/artifacts/hasil-simulasi',
    '/workspace/scratch/hasil-simulasi',
    '/workspace/out/hasil-simulasi',
    os.path.join(HERE, '..', 'artifacts', 'hasil-simulasi'),
    os.path.join(HERE, '..', 'scratch', 'hasil-simulasi')
]

npz_file_path = None
for path in search_paths:
    test_path = os.path.join(path, 'hasil_simulasi_full.npz')
    if os.path.exists(test_path):
        npz_file_path = test_path
        OUTDIR = path
        break

if npz_file_path is None:
    OUTDIR = '/workspace/artifacts/hasil-simulasi'
    npz_file_path = os.path.join(OUTDIR, 'hasil_simulasi_full.npz')

# Load parameters
try:
    from monocopter_params_v4 import *
except ImportError:
    try:
        from monocopter_params import *
    except ImportError:
        # Fallback parameters if not found
        dt = 0.005
        servo_min = -0.5236
        servo_max = 0.5236
        yaw_ff = 0.1077

if not os.path.exists(npz_file_path):
    raise FileNotFoundError(f"File data simulasi tidak ditemukan di {npz_file_path}. Silakan jalankan simulasi_full_system_v2.py terlebih dahulu!")

print(f"Memuat data simulasi dari: {npz_file_path}")
data = np.load(npz_file_path)

t_vec     = data['time']
phi_deg   = data.get('phi_deg', np.zeros_like(t_vec))
theta_deg = data.get('theta_deg', np.zeros_like(t_vec))
yaw_deg   = data.get('yaw_deg', np.zeros_like(t_vec))
alt_m     = data.get('altitude_m', np.zeros_like(t_vec))
phi_est   = data.get('phi_est', np.zeros_like(t_vec))
theta_est = data.get('theta_est', np.zeros_like(t_vec))
thr_pct   = data.get('throttle_pct', np.zeros_like(t_vec))
ref_roll  = data.get('ref_roll', np.zeros_like(t_vec))
ref_alt   = data.get('ref_alt', np.zeros_like(t_vec))
ref_yaw   = data.get('ref_yaw', np.zeros_like(t_vec))
x_pos     = data.get('x_pos', np.zeros_like(t_vec))
y_pos     = data.get('y_pos', np.zeros_like(t_vec))
servo_deg = data.get('servo_deg', np.zeros((4, len(t_vec))))

N = len(t_vec)
phi_rad   = np.deg2rad(phi_deg)
theta_rad = np.deg2rad(theta_deg)
psi_rad   = np.deg2rad(yaw_deg)

# --- ROTATION MATRIX ---
def rot_matrix(phi, theta, psi):
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi), np.sin(psi)
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Ry = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

# --- ROCKET GEOMETRY BUILDER ---
def make_rocket_faces(R, pos, scale=1.0, tl=0.5, servo_k=None):
    rb = 0.04 * scale
    hb = 0.20 * scale
    rn = 0.04 * scale
    hn = 0.08 * scale
    rp = 0.10 * scale
    ns = 16
    th_a = np.linspace(0, 2 * np.pi, ns + 1)
    
    def tw(vl):
        return (R @ vl.T).T + pos

    # Body cylinder (bf)
    bf = []
    for i in range(ns):
        th1 = th_a[i]
        th2 = th_a[i+1]
        p1 = np.array([rb * np.cos(th1), rb * np.sin(th1), -hb/2])
        p2 = np.array([rb * np.cos(th2), rb * np.sin(th2), -hb/2])
        p3 = np.array([rb * np.cos(th2), rb * np.sin(th2), hb/2])
        p4 = np.array([rb * np.cos(th1), rb * np.sin(th1), hb/2])
        bf.append(tw(np.array([p1, p2, p3, p4])))

    # Nose cone (nf)
    nf = []
    for i in range(ns):
        th1 = th_a[i]
        th2 = th_a[i+1]
        p1 = np.array([rn * np.cos(th1), rn * np.sin(th1), hb/2])
        p2 = np.array([rn * np.cos(th2), rn * np.sin(th2), hb/2])
        p3 = np.array([0, 0, hb/2 + hn])
        nf.append(tw(np.array([p1, p2, p3])))

    # Propeller disk (pf)
    pf = []
    p_pts = []
    for th in th_a:
        p_pts.append([rp * np.cos(th), rp * np.sin(th), hb/2 + 0.005])
    pf.append(tw(np.array(p_pts)))

    # 4 Servo fins (with deflection if available)
    fins_list = []
    for j in range(4):
        ang = j * np.pi / 2
        # Local fin coordinates
        p1 = np.array([rb * np.cos(ang), rb * np.sin(ang), -hb/2])
        p2 = np.array([(rb + 0.055 * scale) * np.cos(ang), (rb + 0.055 * scale) * np.sin(ang), -hb/2])
        p3 = np.array([(rb + 0.055 * scale) * np.cos(ang), (rb + 0.055 * scale) * np.sin(ang), -hb/2 + 0.045 * scale])
        p4 = np.array([rb * np.cos(ang), rb * np.sin(ang), -hb/2 + 0.045 * scale])
        fin_local = np.array([p1, p2, p3, p4])
        
        # Apply servo deflection
        if servo_k is not None:
            # Radial rotation axis for fin
            axis = np.array([np.cos(ang), np.sin(ang), 0.0])
            fin_local = deflect_fin(fin_local, np.deg2rad(servo_k[j]), axis)
            
        fins_list.append(tw(fin_local))

    # Flame thrust
    ff = []
    if tl > 0.15:
        fl = 0.07 * scale * tl
        rf = rb * 0.55
        for i in range(ns):
            th1 = th_a[i]
            th2 = th_a[i+1]
            p1 = np.array([rf * np.cos(th1), rf * np.sin(th1), -hb/2])
            p2 = np.array([rf * np.cos(th2), rf * np.sin(th2), -hb/2])
            p3 = np.array([0, 0, -hb/2 - fl])
            ff.append(tw(np.array([p1, p2, p3])))

    return bf, nf, pf, fins_list, ff

def deflect_fin(fin_local, ang, axis):
    c, s = np.cos(ang), np.sin(ang)
    u = axis / np.linalg.norm(axis)
    ux, uy, uz = u
    Rr = np.array([
        [c + ux*ux*(1-c),    ux*uy*(1-c) - uz*s,  ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),    uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s,  c + uz*uz*(1-c)]
    ])
    ctr = fin_local.mean(axis=0)
    return (Rr @ (fin_local - ctr).T).T + ctr

# --- SETUP FIGURE ---
print("Mempersiapkan gambar dan tata letak...")
fig = plt.figure(figsize=(18, 11), facecolor='#f5f7fa')
fig.suptitle('MONOCHOPTER FLIGHT SIMULATOR — 3D Professional', fontsize=14, color='#1a237e', fontweight='bold', y=0.98)

gs = GridSpec(3, 4, figure=fig, left=0.03, right=0.97, top=0.94, bottom=0.06, hspace=0.40, wspace=0.35)

# Main 3D view
ax3d = fig.add_subplot(gs[:, :3], projection='3d')
ax3d.set_facecolor('#f0f3f8')
for pane in [ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor('#c8cfe0')
ax3d.grid(True, color='#c8cfe0', linewidth=0.5)
ax3d.set_xlabel('X (m)', color='#37474f', fontsize=9, labelpad=8)
ax3d.set_ylabel('Y (m)', color='#37474f', fontsize=9, labelpad=8)
ax3d.set_zlabel('Z / Altitude (m)', color='#37474f', fontsize=9, labelpad=8)
ax3d.tick_params(colors='#445577', labelsize=7)
ax3d.set_title('3D Flight Trajectory', color='#1a237e', fontsize=10, pad=8)

# Side panels
ax_roll = fig.add_subplot(gs[0, 3])
ax_alt  = fig.add_subplot(gs[1, 3])
ax_thr  = fig.add_subplot(gs[2, 3])

for ax, title, ylabel, ylim in [
    (ax_roll, 'Roll Angle', 'deg', (-18, 18)),
    (ax_alt,  'Altitude',   'm',   (-0.2, 3.5)),
    (ax_thr,  'Throttle',   '%',   (0, 100)),
]:
    ax.set_facecolor('#eef0f5')
    ax.tick_params(colors='#37474f', labelsize=7)
    ax.set_title(title, color='#1a237e', fontsize=8, pad=3)
    ax.set_ylabel(ylabel, color='#37474f', fontsize=7)
    ax.set_xlabel('t (s)', color='#37474f', fontsize=7)
    ax.spines['bottom'].set_color('#c8cfe0')
    ax.spines['top'].set_color('#c8cfe0')
    ax.spines['left'].set_color('#c8cfe0')
    ax.spines['right'].set_color('#c8cfe0')
    ax.grid(True, color='#c8cfe0', linewidth=0.5)
    ax.set_xlim(0, float(t_vec[-1]))
    ax.set_ylim(*ylim)

# Pre-plot reference traces
ax_roll.plot(t_vec, ref_roll, '--', color='#bf360c', alpha=0.5, linewidth=1, label='Ref')
ax_alt.plot(t_vec, ref_alt,   '--', color='#01579b', alpha=0.5, linewidth=1, label='Ref')
ax_thr.axhline(y=76.7, color='#1b5e20', alpha=0.5, linewidth=0.8, linestyle='--', label='Hover')

# Full traces (dim background)
ax_roll.plot(t_vec, phi_deg, color='#ff7043', alpha=0.2, linewidth=0.8)
ax_alt.plot(t_vec, alt_m,    color='#42a5f5', alpha=0.2, linewidth=0.8)
ax_thr.plot(t_vec, thr_pct,  color='#66bb6a', alpha=0.2, linewidth=0.8)

for ax in [ax_roll, ax_alt, ax_thr]:
    ax.legend(fontsize=6, loc='upper right', framealpha=0.3, labelcolor='#1a237e', facecolor='#eef0f5', edgecolor='#c8cfe0')

# Ground plane
x_range = [min(x_pos)-0.5, max(x_pos)+0.5]
y_range = [min(y_pos)-0.5, max(y_pos)+0.5]
xg = np.linspace(x_range[0], x_range[1], 8)
yg = np.linspace(y_range[0], y_range[1], 8)
XG, YG = np.meshgrid(xg, yg)
ZG = np.zeros_like(XG)
ax3d.plot_surface(XG, YG, ZG, alpha=0.08, color='#90caf9', linewidth=0.3, edgecolor='#c8cfe0')

# Grid lines on ground
for xi in xg:
    ax3d.plot([xi,xi], [y_range[0],y_range[1]], [0,0], color='#b0bec5', lw=0.4, alpha=0.6)
for yi in yg:
    ax3d.plot([x_range[0],x_range[1]], [yi,yi], [0,0], color='#b0bec5', lw=0.4, alpha=0.6)

# Axis arrows on ground
ax3d.quiver(x_range[0], y_range[0], 0, 0.4, 0, 0, color='#ff6655', arrow_length_ratio=0.3, lw=1.2)
ax3d.quiver(x_range[0], y_range[0], 0, 0, 0.4, 0, color='#55ff66', arrow_length_ratio=0.3, lw=1.2)
ax3d.text(x_range[0]+0.45, y_range[0], 0, 'X', color='#c62828', fontsize=7)
ax3d.text(x_range[0], y_range[0]+0.45, 0, 'Y', color='#1b5e20', fontsize=7)

# Full trajectory trace (dim line)
ax3d.plot(x_pos, y_pos, alt_m, color='#b0bec5', linewidth=0.8, alpha=0.5, zorder=1)

# Waypoint markers
scenario_data = data.get('scenario', np.array([[2.0, 0, 0, 0, 3.0], [17.0, 0, 0, 30, 3.0]]))
wp_colors = ['#ffeb3b','#ff9800','#4caf50','#2196f3','#e91e63']
for i, sc in enumerate(scenario_data):
    if len(sc) > 4 and sc[4] > 0.1:
        c = wp_colors[i % len(wp_colors)]
        ax3d.scatter([0], [0], [sc[4]], s=40, c=c, marker='*', zorder=5, alpha=0.7)
        ax3d.text(0.05, 0.05, sc[4], f'WP{i}', color=c, fontsize=6, alpha=0.7)

# --- ANIMATION OBJECTS ---
trail_line, = ax3d.plot([], [], [], color='#0277bd', linewidth=1.5, alpha=0.7, zorder=3)

# Poly collections for rocket parts
rocket_polys = {}
part_configs = {
    'body':      {'color': (0.25, 0.35, 0.55), 'alpha': 0.85},
    'nose':      {'color': (0.85, 0.22, 0.15), 'alpha': 0.9},
    'prop_disk': {'color': (0.6, 0.9, 1.0),    'alpha': 0.2},
    'fin0':      {'color': (0.9, 0.6, 0.1),    'alpha': 0.85}, # S1 orange
    'fin1':      {'color': (0.1, 0.7, 0.3),    'alpha': 0.85}, # S2 green
    'fin2':      {'color': (0.9, 0.6, 0.1),    'alpha': 0.85}, # S3 orange
    'fin3':      {'color': (0.1, 0.7, 0.3),    'alpha': 0.85}, # S4 green
    'flame':     {'color': (1.0, 0.5, 0.05),   'alpha': 0.6},
}

for pname, cfg in part_configs.items():
    pc = Poly3DCollection([], alpha=cfg['alpha'])
    pc.set_facecolor(cfg['color'])
    pc.set_edgecolor('none')
    if pname == 'prop_disk':
        pc.set_edgecolor('b')
        pc.set_linewidth(0.4)
    else:
        pc.set_edgecolor('k')
        pc.set_linewidth(0.4)
    ax3d.add_collection3d(pc)
    rocket_polys[pname] = pc

# Live traces lines
live_roll_line, = ax_roll.plot([], [], color='#ff7043', linewidth=1.5, label='Actual')
live_alt_line,  = ax_alt.plot([],  [], color='#42a5f5', linewidth=1.5, label='Actual')
live_thr_line,  = ax_thr.plot([],  [], color='#66bb6a', linewidth=1.5, label='Actual')

# Live time & info cursor texts
time_text = ax3d.text2D(0.02, 0.95, '', transform=ax3d.transAxes, color='#1a237e', fontsize=9, fontfamily='monospace', fontweight='bold')
phase_text = ax3d.text2D(0.02, 0.90, '', transform=ax3d.transAxes, color='#e65100', fontsize=8, fontweight='bold')
alt_text   = ax3d.text2D(0.02, 0.85, '', transform=ax3d.transAxes, color='#1b5e20', fontsize=8, fontweight='bold')

def get_phase(t):
    if t < 2.0:  return "● GROUND IDLE"
    if t < 5.0:  return "↑ TAKEOFF (RAMP)"
    if t < 7.0:  return "◆ STEADY HOVER"
    if t < 10.0: return "↷ ROLL STEP MANEUVER (12°)"
    if t < 12.0: return "◆ LEVEL RECOVERY (ROLL)"
    if t < 15.0: return "↑ PITCH STEP MANEUVER (10°)"
    if t < 17.0: return "◆ LEVEL RECOVERY (PITCH)"
    if t < 19.5: return "↻ YAW TURN STEP (30°)"
    if t < 21.0: return "◆ HOLD HEADING YAW"
    if t < 22.0: return "◆ YAW RETURN HOME"
    if t < 24.0: return "↓ DESCENT STAGE 1"
    if t < 25.5: return "↓ DESCENT STAGE 2"
    return "✓ LANDING & SHUT-OFF"

# Axis limits for 3D trajectory
ax3d.set_xlim(x_range[0], x_range[1])
ax3d.set_ylim(y_range[0], y_range[1])
ax3d.set_zlim(0, alt_m.max() + 0.6)
ax3d.view_init(elev=20, azim=215)

TRAIL_LEN = 150
SKIP = max(1, N // 200)   # target ~200 frames untuk kelancaran rendering

print(f"Total langkah data: {N}, skip frame={SKIP}, estimasi frame animasi: {N//SKIP}")

# --- UPDATE FUNCTION FOR FUNCANIMATION ---
def update(frame):
    k = min(frame * SKIP, N - 1)
    t_cur = t_vec[k]
    
    # 1. Update live text
    time_text.set_text(f"Time: {t_cur:.2f} s")
    phase_text.set_text(f"Phase: {get_phase(t_cur)}")
    alt_text.set_text(f"Altitude: {alt_m[k]:.2f} m")
    
    # 2. Update dynamic trail path
    start_idx = max(0, k - TRAIL_LEN)
    trail_line.set_data(x_pos[start_idx:k+1], y_pos[start_idx:k+1])
    trail_line.set_3d_properties(alt_m[start_idx:k+1])
    
    # 3. Hitung pose 3D wahana
    pos_k = np.array([x_pos[k], y_pos[k], alt_m[k]])
    R_k   = rot_matrix(phi_rad[k], theta_rad[k], psi_rad[k])
    tl_k  = thr_pct[k] / 100.0  # thrust level
    
    # Ambil sudut servo saat ini (4 x 1)
    servo_k = servo_deg[:, k]
    
    # Bangkitkan geometri poligon
    bf, nf, pf, fins, ff = make_rocket_faces(R_k, pos_k, scale=0.9, tl=tl_k, servo_k=servo_k)
    
    # Update Polygons
    rocket_polys['body'].set_verts(bf)
    rocket_polys['nose'].set_verts(nf)
    rocket_polys['prop_disk'].set_verts(pf)
    
    # 4 sirip individu
    rocket_polys['fin0'].set_verts([fins[0]])
    rocket_polys['fin1'].set_verts([fins[1]])
    rocket_polys['fin2'].set_verts([fins[2]])
    rocket_polys['fin3'].set_verts([fins[3]])
    
    # Thrust flame poligon
    rocket_polys['flame'].set_verts(ff)
    
    # 4. Update grafis panel samping kanan
    live_roll_line.set_data(t_vec[:k+1], phi_deg[:k+1])
    live_alt_line.set_data(t_vec[:k+1], alt_m[:k+1])
    live_thr_line.set_data(t_vec[:k+1], thr_pct[:k+1])
    
    artists = [
        trail_line, time_text, phase_text, alt_text,
        rocket_polys['body'], rocket_polys['nose'], rocket_polys['prop_disk'],
        rocket_polys['fin0'], rocket_polys['fin1'], rocket_polys['fin2'], rocket_polys['fin3'],
        rocket_polys['flame'], live_roll_line, live_alt_line, live_thr_line
    ]
    return artists

frame_count = N // SKIP
print(f"Mulai kompilasi {frame_count} frame animasi...")

ani = animation.FuncAnimation(
    fig, update, frames=frame_count, interval=40, blit=False
)

# Legends
legend_elements = [
    Line2D([0],[0], color='#0277bd',  lw=2,  label='Flight path (Trail)'),
    mpatches.Patch(color=(0.25,0.35,0.55), alpha=0.85, label='Rocket body'),
    mpatches.Patch(color=(0.85,0.22,0.15), alpha=0.9,  label='Nose cone (Red)'),
    mpatches.Patch(color=(0.6,0.9,1.0),    alpha=0.6,  label='Propeller disk'),
    mpatches.Patch(color=(0.9,0.6,0.1),    alpha=0.85, label='Servo S1/S3 (Roll)'),
    mpatches.Patch(color=(0.1,0.7,0.3),    alpha=0.85, label='Servo S2/S4 (Pitch)'),
    mpatches.Patch(color=(1.0,0.5,0.05),   alpha=0.6,  label='Thrust Flame'),
]
ax3d.legend(handles=legend_elements, loc='upper left', fontsize=7, framealpha=0.4, facecolor='#f5f7fa', edgecolor='#c8cfe0', labelcolor='#1a237e')

# --- SAVE ANIMATION ---
print("Menyimpan animasi ke MP4... Proses ini membutuhkan ffmpeg.")
mp4_output_path = os.path.join(HERE, 'animasi_3D_monochopter_light.mp4')

Writer = animation.FFMpegWriter(fps=25, bitrate=1800, extra_args=['-vcodec','libx264','-pix_fmt','yuv420p'])
try:
    ani.save(mp4_output_path, writer=Writer, dpi=120)
    print(f"[SUKSES] MP4 animasi professional berhasil disimpan ke: {mp4_output_path}")
except Exception as e:
    print(f"[PERINGATAN] ffmpeg gagal menyimpan MP4 ({e}), mencoba fallback ke GIF...")
    gif_output_path = os.path.join(HERE, 'animasi_3D_monochopter_light.gif')
    Writer2 = animation.PillowWriter(fps=15)
    ani.save(gif_output_path, writer=Writer2, dpi=80)
    print(f"[SUKSES] Animasi fallback berhasil disimpan ke GIF: {gif_output_path}")

plt.close('all')
print("=== PROSES RENDER ANIMASI SELESAI ===")
