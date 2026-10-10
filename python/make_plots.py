"""
Static multi-panel visualization of monochopter simulation
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection

import os
HERE   = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, 'hasil-simulasi')
PLOTDIR = os.path.join(HERE, 'plots')
os.makedirs(PLOTDIR, exist_ok=True)


# ── Load data ──────────────────────────────────────────────
data     = np.load(os.path.join(OUTDIR, 'hasil_simulasi_full.npz'))
t        = data['time']
phi_d    = data['phi_deg']
theta_d  = data['theta_deg']
yaw_d    = data['yaw_deg']
alt      = data['altitude_m']
phi_est  = data['phi_est']
thr      = data['throttle_pct']
ref_roll = data['ref_roll']
ref_alt  = data['ref_alt']
ref_yaw  = data['ref_yaw']
servo    = data['servo_deg']
xp       = data['x_pos']
yp       = data['y_pos']
scenario = data['scenario']
N        = len(t)

phi_r  = np.deg2rad(phi_d)
tht_r  = np.deg2rad(theta_d)
psi_r  = np.deg2rad(yaw_d)

# ── Colour palette ─────────────────────────────────────────
BG   = '#ffffff'
PAN  = '#ffffff'
GRD  = '#c8cfe0'
C1   = '#0277bd'   # blue
C2   = '#d84315'   # deep orange
C3   = '#2e7d32'   # dark green
C4   = '#6a1b9a'   # deep purple
C5   = '#f57f17'   # dark amber
C6   = '#00838f'   # dark teal
REF_C = '#37474f'  # dark grey dashed

def rot_zyx(ph, th, ps):
    cp,sp = np.cos(ph),np.sin(ph)
    ct,st = np.cos(th),np.sin(th)
    cy,sy = np.cos(ps),np.sin(ps)
    Rx = np.array([[1,0,0],[0,cp,-sp],[0,sp,cp]])
    Ry = np.array([[ct,0,st],[0,1,0],[-st,0,ct]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz@Ry@Rx

def rocket_faces(R, pos, scale=1.0, tl=0.5):
    """Return dict of face-lists for each part."""
    rb=0.04*scale; hb=0.20*scale
    rn=0.04*scale; hn=0.08*scale
    rp=0.10*scale
    ns=20
    th_a = np.linspace(0,2*np.pi,ns+1)
    def tw(vl):
        return (R @ vl.T).T + pos

    # Body
    bf=[]
    for i in range(ns):
        a0,a1=th_a[i],th_a[i+1]
        vl=np.array([[rb*np.cos(a0),rb*np.sin(a0),0],
                     [rb*np.cos(a1),rb*np.sin(a1),0],
                     [rb*np.cos(a1),rb*np.sin(a1),hb],
                     [rb*np.cos(a0),rb*np.sin(a0),hb]])
        bf.append(tw(vl))

    # Nose
    nf=[]
    for i in range(ns):
        a0,a1=th_a[i],th_a[i+1]
        vl=np.array([[rn*np.cos(a0),rn*np.sin(a0),hb],
                     [rn*np.cos(a1),rn*np.sin(a1),hb],
                     [0,0,hb+hn]])
        nf.append(tw(vl))

    # Prop disk
    pf=[]
    pz=hb-0.005
    for i in range(ns):
        a0,a1=th_a[i],th_a[i+1]
        vl=np.array([[rp*np.cos(a0),rp*np.sin(a0),pz],
                     [rp*np.cos(a1),rp*np.sin(a1),pz],
                     [0,0,pz]])
        pf.append(tw(vl))

    # 4 fins
    fins=[]
    for fa in [0, np.pi/2, np.pi, 3*np.pi/2]:
        ca,sa=np.cos(fa),np.sin(fa)
        ri,ro=rb,rb+0.055*scale; fh=0.045*scale
        vl=np.array([[ri*ca,ri*sa,0],
                     [ro*ca,ro*sa,0],
                     [ro*ca,ro*sa,fh],
                     [ri*ca,ri*sa,fh]])
        fins.append(tw(vl))

    # Flame
    ff=[]
    if tl>0.35:
        fl=0.07*scale*tl
        for i in range(ns):
            a0,a1=th_a[i],th_a[i+1]
            rf=rb*0.55
            vl=np.array([[rf*np.cos(a0),rf*np.sin(a0),0],
                         [rf*np.cos(a1),rf*np.sin(a1),0],
                         [0,0,-fl]])
            ff.append(tw(vl))
    return bf,nf,pf,fins,ff


# ══════════════════════════════════════════════════════════════
# FIGURE 1  –  3D Flight View  (multiple snapshots)
# ══════════════════════════════════════════════════════════════
print("Building Figure 1: 3D overview...")
fig1 = plt.figure(figsize=(18,11), facecolor=BG)
fig1.suptitle('MONOCHOPTER — 3D Flight Simulation', color='#1a237e',
              fontsize=15, fontweight='bold', y=0.98)

ax3 = fig1.add_axes([0.02, 0.08, 0.60, 0.86], projection='3d')
ax3.set_facecolor('#ffffff')
for pane in [ax3.xaxis.pane, ax3.yaxis.pane, ax3.zaxis.pane]:
    pane.fill=False; pane.set_edgecolor(GRD)
ax3.grid(True, color=GRD, lw=0.4)
ax3.set_xlabel('X (m)', color='#37474f', fontsize=9, labelpad=6)
ax3.set_ylabel('Y (m)', color='#37474f', fontsize=9, labelpad=6)
ax3.set_zlabel('Alt (m)', color='#37474f', fontsize=9, labelpad=6)
ax3.tick_params(colors='#445577', labelsize=7)
ax3.set_title('Full Trajectory + Rocket Snapshots', color='#1a237e',
              fontsize=10, pad=6)

# Axis limits
xmn,xmx = xp.min()-0.4, xp.max()+0.4
ymn,ymx = yp.min()-0.4, yp.max()+0.4
zmx     = alt.max()+0.6
ax3.set_xlim(xmn,xmx); ax3.set_ylim(ymn,ymx); ax3.set_zlim(0,zmx)
ax3.view_init(elev=25, azim=210)

# Ground plane
xg,yg = np.meshgrid(np.linspace(xmn,xmx,8), np.linspace(ymn,ymx,8))
ax3.plot_surface(xg,yg,np.zeros_like(xg),
                 alpha=0.07, color='#90caf9', edgecolor=GRD)
for xi in np.linspace(xmn,xmx,7):
    ax3.plot([xi,xi],[ymn,ymx],[0,0], color=GRD, lw=0.4, alpha=0.7)
for yi in np.linspace(ymn,ymx,7):
    ax3.plot([xmn,xmx],[yi,yi],[0,0], color=GRD, lw=0.4, alpha=0.7)

# Coloured trajectory by altitude
pts  = np.array([xp,yp,alt]).T.reshape(-1,1,3)
segs = np.concatenate([pts[:-1],pts[1:]], axis=1)
# Draw as individual line segments coloured by altitude
cmap = plt.cm.plasma
norm = plt.Normalize(vmin=0, vmax=zmx)
for i in range(0, N-1, 4):
    c = cmap(norm(alt[i]))
    ax3.plot([xp[i],xp[i+1]],[yp[i],yp[i+1]],[alt[i],alt[i+1]],
             color=c, lw=1.6, alpha=0.8)

# Rocket snapshots at key moments
snap_times = [1.0, 3.0, 7.0, 11.0, 15.0, 20.5, 23.5]
snap_labels= ['Takeoff','Hover','Roll','Pitch','Yaw','Descent','Landing']
snap_cols  = [C3, C1, C2, C4, C5, C6, '#ff6699']
for st, sl, sc_col in zip(snap_times, snap_labels, snap_cols):
    ki = int(np.argmin(np.abs(t - st)))
    pos_k = np.array([xp[ki], yp[ki], alt[ki]])
    R_k   = rot_zyx(phi_r[ki], tht_r[ki], psi_r[ki])
    tl_k  = thr[ki]/100.0
    bf,nf,pf,fins,ff = rocket_faces(R_k, pos_k, scale=0.9, tl=tl_k)

    body_pc = Poly3DCollection(bf, alpha=0.82)
    body_pc.set_facecolor((0.22,0.32,0.52)); body_pc.set_edgecolor('none')
    ax3.add_collection3d(body_pc)

    nose_pc = Poly3DCollection(nf, alpha=0.9)
    nose_pc.set_facecolor((0.82,0.18,0.12)); nose_pc.set_edgecolor('none')
    ax3.add_collection3d(nose_pc)

    prop_pc = Poly3DCollection(pf, alpha=0.15+0.15*tl_k)
    prop_pc.set_facecolor((0.5,0.88,1.0)); prop_pc.set_edgecolor('none')
    ax3.add_collection3d(prop_pc)

    fc_list = [(0.9,0.6,0.1),(0.1,0.75,0.3),(0.9,0.6,0.1),(0.1,0.75,0.3)]
    for fv, fc in zip(fins, fc_list):
        fpc = Poly3DCollection([fv], alpha=0.85)
        fpc.set_facecolor(fc); fpc.set_edgecolor('none')
        ax3.add_collection3d(fpc)

    if ff:
        flame_pc = Poly3DCollection(ff, alpha=0.45)
        flame_pc.set_facecolor((1.0, 0.35+0.35*tl_k, 0.05))
        flame_pc.set_edgecolor('none')
        ax3.add_collection3d(flame_pc)

    # Thrust arrow
    thrust_dir = R_k @ np.array([0,0,0.28])
    ax3.quiver(pos_k[0],pos_k[1],pos_k[2],
               thrust_dir[0],thrust_dir[1],thrust_dir[2],
               color=sc_col, alpha=0.7, arrow_length_ratio=0.3, lw=1.6)

    ax3.text(pos_k[0]+0.05, pos_k[1]+0.05, pos_k[2]+0.10,
             f'{sl}\nt={st:.0f}s', color=sc_col, fontsize=6.5,
             fontweight='bold')

# Waypoints
wp_col = '#ffeb3b'
for row in scenario:
    if row[4] > 0.1:
        ax3.scatter([0],[0],[row[4]], s=35, c=wp_col, marker='*',
                    zorder=6, alpha=0.6)

# Phase bands in background colour strip
phase_labels2 = [(0,'Gnd'),(1.5,'T/O'),(4,'Hover'),(6,'Roll'),
                 (10,'Hover'),(12,'Pitch'),(15,'Hover'),
                 (14,'Yaw'),(17.5,'Hover'),(19,'Desc'),(22,'Land')]

# Right info panels
gs2 = GridSpec(4,1, figure=fig1, left=0.65, right=0.97,
               top=0.94, bottom=0.08, hspace=0.50)
ax_r  = fig1.add_subplot(gs2[0])
ax_p  = fig1.add_subplot(gs2[1])
ax_a  = fig1.add_subplot(gs2[2])
ax_th = fig1.add_subplot(gs2[3])

panel_style = dict(facecolor=PAN)
for ax,lbl,dat,ref_d,col,ylim in [
    (ax_r,  'Roll (deg)',    phi_d,  ref_roll, C2, (-22,22)),
    (ax_p,  'Pitch (deg)',   theta_d,None,     C4, (-18,18)),
    (ax_a,  'Altitude (m)',  alt,    ref_alt,  C1, (-0.1,zmx)),
    (ax_th, 'Throttle (%)',  thr,    None,     C3, (0,100)),
]:
    ax.set_facecolor(PAN)
    ax.grid(True, color=GRD, lw=0.4, alpha=0.8)
    ax.tick_params(colors='#8899bb', labelsize=7)
    ax.set_ylabel(lbl, color='#1a237e', fontsize=7)
    ax.set_xlabel('t (s)', color='#37474f', fontsize=6)
    ax.spines[:].set_color(GRD)
    ax.set_xlim(0, float(t[-1]))
    ax.set_ylim(*ylim)
    ax.plot(t, dat, color=col, lw=1.2)
    if ref_d is not None:
        ax.plot(t, ref_d, '--', color=REF_C, alpha=0.4, lw=0.9, label='ref')
        ax.legend(fontsize=6, framealpha=0.3, facecolor=PAN,
                  labelcolor='#1a237e', edgecolor=GRD, loc='upper right')
    for row in scenario:
        ax.axvline(x=row[0], color='#90a4ae', lw=0.5, alpha=0.7)

# Scenario annotations on roll plot
for row,lbl in zip(scenario, ['G','T/O','H','Roll','H','Pitch','H','Yaw','H','H','Desc','Pre','Land']):
    ax_r.text(row[0]+0.1, 19, lbl, color='#607d8b', fontsize=5, rotation=90, va='top')

# Legend
legend_els = [
    Line2D([0],[0], color=C3, lw=2, label='Takeoff'),
    Line2D([0],[0], color=C1, lw=2, label='Hover'),
    Line2D([0],[0], color=C2, lw=2, label='Roll maneuver'),
    Line2D([0],[0], color=C4, lw=2, label='Pitch maneuver'),
    Line2D([0],[0], color=C5, lw=2, label='Yaw turn'),
    Line2D([0],[0], color=C6, lw=2, label='Descent'),
    mpatches.Patch(color=(0.22,0.32,0.52), alpha=0.9, label='Rocket body'),
    mpatches.Patch(color=(0.82,0.18,0.12), alpha=0.9, label='Nose cone'),
    mpatches.Patch(color=(0.5,0.88,1.0),   alpha=0.6, label='Propeller disk'),
    mpatches.Patch(color=(0.9,0.6,0.1),    alpha=0.9, label='Servo fin'),
    Line2D([0],[0], color='#ffcc00', lw=2, label='Thrust vector'),
]
ax3.legend(handles=legend_els, loc='upper left',
           fontsize=6.5, framealpha=0.4,
           facecolor=BG, edgecolor='#90a4ae',
           labelcolor='#1a237e', bbox_to_anchor=(0.0,0.98))

fig1.savefig(os.path.join(PLOTDIR, 'fig1_3d_flight.png'), dpi=150,
             facecolor=BG, bbox_inches='tight')
plt.close(fig1)
print("Fig1 saved.")

# ══════════════════════════════════════════════════════════════
# FIGURE 2  –  Performance Dashboard
# ══════════════════════════════════════════════════════════════
print("Building Figure 2: performance dashboard...")
fig2 = plt.figure(figsize=(18,12), facecolor=BG)
fig2.suptitle('MONOCHOPTER — Control & Performance Dashboard',
              color='#1a237e', fontsize=14, fontweight='bold', y=0.98)

gs3 = GridSpec(3, 4, figure=fig2, left=0.05, right=0.97,
               top=0.93, bottom=0.06, hspace=0.52, wspace=0.38)

# 1. XY Trajectory with altitude colour
ax_xy = fig2.add_subplot(gs3[0,0])
ax_xy.set_facecolor(PAN)
ax_xy.grid(True, color=GRD, lw=0.4)
ax_xy.spines[:].set_color(GRD)
ax_xy.tick_params(colors='#8899bb', labelsize=7)
ax_xy.set_title('XY Trajectory (colour=alt)', color='#1a237e', fontsize=9)
ax_xy.set_xlabel('X (m)', color='#37474f', fontsize=7)
ax_xy.set_ylabel('Y (m)', color='#37474f', fontsize=7)
pts2 = np.array([xp,yp]).T.reshape(-1,1,2)
segs2 = np.concatenate([pts2[:-1],pts2[1:]], axis=1)
from matplotlib.collections import LineCollection as LC2
lc2 = LC2(segs2, cmap='plasma', linewidth=2.0)
lc2.set_array(alt); lc2.set_norm(plt.Normalize(0, zmx))
ax_xy.add_collection(lc2)
ax_xy.set_xlim(xp.min()-0.2, xp.max()+0.2)
ax_xy.set_ylim(yp.min()-0.2, yp.max()+0.2)
ax_xy.plot(xp[0],yp[0],'go', ms=8, label='Start', zorder=5)
ax_xy.plot(xp[-1],yp[-1],'r*', ms=10, label='End', zorder=5)
ax_xy.legend(fontsize=7, framealpha=0.4, facecolor=PAN,
             labelcolor='#1a237e', edgecolor=GRD)
cb = plt.colorbar(lc2, ax=ax_xy, pad=0.02)
cb.ax.tick_params(colors='#37474f', labelsize=6)
cb.set_label('Alt (m)', color='#37474f', fontsize=6)

# 2. Altitude vs ref
ax2b = fig2.add_subplot(gs3[0,1])
ax2b.set_facecolor(PAN); ax2b.grid(True, color=GRD, lw=0.4)
ax2b.spines[:].set_color(GRD); ax2b.tick_params(colors='#8899bb', labelsize=7)
ax2b.set_title('Altitude Control', color='#1a237e', fontsize=9)
ax2b.fill_between(t, alt, alpha=0.15, color=C1)
ax2b.plot(t, alt, color=C1, lw=1.5, label='Altitude')
ax2b.plot(t, ref_alt, '--', color=REF_C, lw=1.0, alpha=0.6, label='Setpoint')
ax2b.set_xlabel('t (s)', color='#37474f', fontsize=7)
ax2b.set_ylabel('m', color='#37474f', fontsize=7)
ax2b.legend(fontsize=7, framealpha=0.3, facecolor=PAN, labelcolor='#1a237e', edgecolor=GRD)

# 3. Roll: actual vs estimated vs reference
ax2c = fig2.add_subplot(gs3[0,2])
ax2c.set_facecolor(PAN); ax2c.grid(True, color=GRD, lw=0.4)
ax2c.spines[:].set_color(GRD); ax2c.tick_params(colors='#8899bb', labelsize=7)
ax2c.set_title('Roll: Actual vs Kalman Est', color='#1a237e', fontsize=9)
ax2c.plot(t, phi_d,   color=C2, lw=1.4, label='True φ')
ax2c.plot(t, phi_est, color=C5, lw=0.9, alpha=0.85, label='Kalman Est')
ax2c.plot(t, ref_roll,'--', color=REF_C, lw=0.9, alpha=0.5, label='Setpoint')
ax2c.set_xlabel('t (s)', color='#37474f', fontsize=7)
ax2c.set_ylabel('deg', color='#37474f', fontsize=7)
ax2c.legend(fontsize=7, framealpha=0.3, facecolor=PAN, labelcolor='#1a237e', edgecolor=GRD)

# 4. Pitch & Yaw
ax2d = fig2.add_subplot(gs3[0,3])
ax2d.set_facecolor(PAN); ax2d.grid(True, color=GRD, lw=0.4)
ax2d.spines[:].set_color(GRD); ax2d.tick_params(colors='#8899bb', labelsize=7)
ax2d.set_title('Pitch & Yaw', color='#1a237e', fontsize=9)
ax2d.plot(t, theta_d, color=C4, lw=1.3, label='Pitch θ')
ax2d.plot(t, yaw_d,   color=C6, lw=1.3, label='Yaw ψ')
ax2d.plot(t, ref_yaw,'--', color='#546e7a', lw=0.8, alpha=0.5, label='Yaw ref')
ax2d.set_xlabel('t (s)', color='#37474f', fontsize=7)
ax2d.set_ylabel('deg', color='#37474f', fontsize=7)
ax2d.legend(fontsize=7, framealpha=0.3, facecolor=PAN, labelcolor='#1a237e', edgecolor=GRD)

# 5. Servo deflections
ax2e = fig2.add_subplot(gs3[1,0:2])
ax2e.set_facecolor(PAN); ax2e.grid(True, color=GRD, lw=0.4)
ax2e.spines[:].set_color(GRD); ax2e.tick_params(colors='#8899bb', labelsize=7)
ax2e.set_title('Servo Deflections (all 4)', color='#1a237e', fontsize=9)
s_cols = [C2, C3, C4, C5]
s_lbl  = ['S1 (Roll+Yaw)','S2 (Pitch+Yaw)','S3 (-Roll+Yaw)','S4 (-Pitch+Yaw)']
for i,(sc,sl) in enumerate(zip(s_cols,s_lbl)):
    ax2e.plot(t, servo[i,:], color=sc, lw=0.9, alpha=0.85, label=sl)
ax2e.axhline(y=30,  color='#ff3333', lw=0.8, ls='--', alpha=0.5)
ax2e.axhline(y=-30, color='#ff3333', lw=0.8, ls='--', alpha=0.5)
ax2e.set_xlabel('t (s)', color='#37474f', fontsize=7)
ax2e.set_ylabel('deg', color='#37474f', fontsize=7)
ax2e.legend(fontsize=6.5, framealpha=0.3, facecolor=PAN, labelcolor='#1a237e',
            edgecolor=GRD, ncol=2)

# 6. Throttle
ax2f = fig2.add_subplot(gs3[1,2:4])
ax2f.set_facecolor(PAN); ax2f.grid(True, color=GRD, lw=0.4)
ax2f.spines[:].set_color(GRD); ax2f.tick_params(colors='#8899bb', labelsize=7)
ax2f.set_title('Throttle Command', color='#1a237e', fontsize=9)
ax2f.fill_between(t, thr, alpha=0.2, color=C3)
ax2f.plot(t, thr, color=C3, lw=1.4)
ax2f.axhline(y=77, color=REF_C, lw=0.8, ls='--', alpha=0.4, label='Hover (77%)')
ax2f.set_ylim(0,100)
ax2f.set_xlabel('t (s)', color='#37474f', fontsize=7)
ax2f.set_ylabel('%', color='#37474f', fontsize=7)
ax2f.legend(fontsize=7, framealpha=0.3, facecolor=PAN, labelcolor='#1a237e', edgecolor=GRD)

# Phase band on throttle
phase_ts = [(0,1.5,'Gnd'),(1.5,4,'T/O'),(4,6,'Hover'),(6,8.5,'Roll'),
            (10,12,'H'),(12,15,'Pitch'),(15,17,'H'),(17,21,'Yaw'),
            (17.5,19,'H'),(19,21,'Desc'),(21,24,'Land')]
ph_cols = ['#e8f5e9','#e3f2fd','#ede7f6','#fce4ec','#ede7f6','#f3e5f5',
           '#ede7f6','#fff8e1','#ede7f6','#e0f7fa','#e0f7fa']
for (ts,te,pl),pbc in zip(phase_ts, ph_cols):
    ax2f.axvspan(ts, te, alpha=0.25, color=pbc)
    ax2f.text((ts+te)/2, 95, pl, color='#607d8b', fontsize=5.5,
              ha='center', va='top')

# 7. Angular rates
ax2g = fig2.add_subplot(gs3[2,0:2])
ax2g.set_facecolor(PAN); ax2g.grid(True, color=GRD, lw=0.4)
ax2g.spines[:].set_color(GRD); ax2g.tick_params(colors='#8899bb', labelsize=7)
ax2g.set_title('Angular Rates', color='#1a237e', fontsize=9)
ax2g.plot(t, np.rad2deg(data['dphi']),   color=C2, lw=1.0, label='ṗ roll rate')
ax2g.plot(t, np.rad2deg(data['dtheta']), color=C4, lw=1.0, label='q pitch rate')
ax2g.plot(t, np.rad2deg(data['dpsi']),   color=C6, lw=1.0, label='r yaw rate')
ax2g.set_xlabel('t (s)', color='#37474f', fontsize=7)
ax2g.set_ylabel('deg/s', color='#37474f', fontsize=7)
ax2g.legend(fontsize=7, framealpha=0.3, facecolor=PAN, labelcolor='#1a237e', edgecolor=GRD)

# 8. Vertical velocity
ax2h = fig2.add_subplot(gs3[2,2:4])
ax2h.set_facecolor(PAN); ax2h.grid(True, color=GRD, lw=0.4)
ax2h.spines[:].set_color(GRD); ax2h.tick_params(colors='#8899bb', labelsize=7)
ax2h.set_title('Vertical Velocity & Error', color='#1a237e', fontsize=9)
ax2h.plot(t, data['dz'], color=C1, lw=1.3, label='Vz (m/s)')
ax2h.fill_between(t, 0, data['dz'], where=data['dz']>0,
                   alpha=0.15, color=C3, label='Climb')
ax2h.fill_between(t, 0, data['dz'], where=data['dz']<0,
                   alpha=0.15, color=C2, label='Descend')
ax2h_r = ax2h.twinx()
alt_err = np.abs(alt - ref_alt)
ax2h_r.plot(t, alt_err, color=C5, lw=0.8, alpha=0.7, label='Alt Error')
ax2h_r.set_ylabel('|err| m', color=C5, fontsize=7)
ax2h_r.tick_params(colors=C5, labelsize=7)
ax2h.set_xlabel('t (s)', color='#37474f', fontsize=7)
ax2h.set_ylabel('m/s', color='#37474f', fontsize=7)
ax2h.legend(fontsize=7, framealpha=0.3, facecolor=PAN, labelcolor='#1a237e',
            edgecolor=GRD, loc='upper left')

fig2.savefig(os.path.join(PLOTDIR, 'fig2_dashboard.png'), dpi=150,
             facecolor=BG, bbox_inches='tight')
plt.close(fig2)
print("Fig2 saved.")

# ══════════════════════════════════════════════════════════════
# FIGURE 3  –  Rocket body close-up at 8 flight phases
# ══════════════════════════════════════════════════════════════
print("Building Figure 3: rocket snapshots...")
fig3 = plt.figure(figsize=(18,10), facecolor=BG)
fig3.suptitle('MONOCHOPTER — Rocket Attitude at Key Flight Phases',
              color='#1a237e', fontsize=14, fontweight='bold', y=0.99)

snap_t  = [1.0, 3.0, 7.0, 11.0, 15.0, 18.0, 20.5, 23.5]
snap_lb = ['Ground\n(idle)','Takeoff\n(thrust)','Roll\nManeuver',
           'Pitch\nManeuver','Yaw\nTurn','Level\nHover',
           'Descent','Landing']
snap_c  = [C3, C1, C2, C4, C5, C6, '#ff88aa', '#aaddff']

for si,(st,sl,sc_c) in enumerate(zip(snap_t,snap_lb,snap_c)):
    ki = int(np.argmin(np.abs(t - st)))
    pos_k = np.array([0.0, 0.0, 0.0])   # centred for close-up
    R_k   = rot_zyx(phi_r[ki], tht_r[ki], psi_r[ki])
    tl_k  = thr[ki]/100.0

    ax_s = fig3.add_subplot(2, 4, si+1, projection='3d')
    ax_s.set_facecolor('#ffffff')
    for pane in [ax_s.xaxis.pane, ax_s.yaxis.pane, ax_s.zaxis.pane]:
        pane.fill=False; pane.set_edgecolor('#c8cfe0')
    ax_s.grid(True, color='#c8cfe0', lw=0.4)
    ax_s.set_xlim(-0.22,0.22); ax_s.set_ylim(-0.22,0.22); ax_s.set_zlim(-0.12,0.35)
    ax_s.tick_params(colors='#334466', labelsize=5)
    ax_s.set_title(f'{sl}\nt={st:.0f}s | φ={phi_d[ki]:.1f}° θ={theta_d[ki]:.1f}°\nThr={thr[ki]:.0f}%',
                   color=sc_c, fontsize=7, pad=3, fontweight='bold')
    ax_s.view_init(elev=15, azim=220)

    # Ground dot
    ax_s.scatter([0],[0],[0], color='#90a4ae', s=50, zorder=1)

    bf,nf,pf,fins,ff = rocket_faces(R_k, pos_k, scale=0.9, tl=tl_k)

    pc_b = Poly3DCollection(bf, alpha=0.85)
    pc_b.set_facecolor((0.22,0.32,0.52)); pc_b.set_edgecolor('none')
    ax_s.add_collection3d(pc_b)

    pc_n = Poly3DCollection(nf, alpha=0.9)
    pc_n.set_facecolor((0.82,0.18,0.12)); pc_n.set_edgecolor('none')
    ax_s.add_collection3d(pc_n)

    pc_p = Poly3DCollection(pf, alpha=0.15+0.2*tl_k)
    pc_p.set_facecolor((0.5,0.88,1.0)); pc_p.set_edgecolor('none')
    ax_s.add_collection3d(pc_p)

    fc_list = [(0.9,0.6,0.1),(0.1,0.75,0.3),(0.9,0.6,0.1),(0.1,0.75,0.3)]
    for fv,fc in zip(fins,fc_list):
        pfin = Poly3DCollection([fv], alpha=0.9)
        pfin.set_facecolor(fc); pfin.set_edgecolor('none')
        ax_s.add_collection3d(pfin)

    if ff:
        pff = Poly3DCollection(ff, alpha=0.45)
        pff.set_facecolor((1.0, 0.3+0.4*tl_k, 0.05))
        pff.set_edgecolor('none')
        ax_s.add_collection3d(pff)

    # Thrust arrow
    td = R_k @ np.array([0,0,0.18])
    ax_s.quiver(0,0,0, td[0],td[1],td[2],
                color=sc_c, alpha=0.8, arrow_length_ratio=0.3, lw=1.6)

    # Body axes
    for bv,col in [(np.array([0.15,0,0]),'#ff4444'),
                   (np.array([0,0.15,0]),'#44ff44'),
                   (np.array([0,0,0.15]),'#4488ff')]:
        bvw = R_k @ bv
        ax_s.quiver(0,0,0, bvw[0],bvw[1],bvw[2],
                    color=col, alpha=0.5, arrow_length_ratio=0.2, lw=1.0)

plt.tight_layout(rect=[0,0,1,0.97])
fig3.savefig(os.path.join(PLOTDIR, 'fig3_rocket_snapshots.png'), dpi=150,
             facecolor=BG, bbox_inches='tight')
plt.close(fig3)
print("Fig3 saved.")

print("\n=== ALL FIGURES GENERATED ===")
print("  fig1_3d_flight.png       – Full 3D trajectory + rocket overlays")
print("  fig2_dashboard.png       – Complete control performance dashboard")
print("  fig3_rocket_snapshots.png– Rocket attitude at 8 flight phases")
