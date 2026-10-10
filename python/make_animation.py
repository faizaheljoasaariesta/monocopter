"""
============================================================
FILE: make_animation.py
TUJUAN: Render animasi 3D monochopter (roket) → MP4
============================================================
Versi konsisten dengan simulasi_full_system.py terbaru:
  - Path relatif (tidak hardcoded)
  - Fase sesuai skenario MATLAB (takeoff 2s, roll 5s, dst)
  - Sirip ikut berdefleksi sesuai data servo_deg
Render per-frame lalu disatukan ke MP4 via ffmpeg
(menghindari timeout FuncAnimation).
============================================================
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.gridspec import GridSpec
import os, subprocess

# ─── PATH RELATIF ───────────────────────────────────────────
HERE   = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, 'hasil-simulasi')
FRAMES = os.path.join(HERE, 'frames')
os.makedirs(FRAMES, exist_ok=True)

data     = np.load(os.path.join(OUTDIR, 'hasil_simulasi_full.npz'))
t        = data['time']
phi_d    = data['phi_deg'];   phi_r = np.deg2rad(phi_d)
theta_d  = data['theta_deg']; tht_r = np.deg2rad(theta_d)
yaw_d    = data['yaw_deg'];   psi_r = np.deg2rad(yaw_d)
alt      = data['altitude_m']
thr      = data['throttle_pct']
ref_roll = data['ref_roll']
ref_alt  = data['ref_alt']
xp       = data['x_pos']
yp       = data['y_pos']
servo    = data['servo_deg']      # 4 x N [deg]
N        = len(t)

BG='#f5f7fa'; PAN='#eef0f5'; GRD='#c8cfe0'
C1='#0277bd'; C2='#d84315'; C3='#2e7d32'; C5='#f57f17'

def rot_zyx(ph,th,ps):
    cp,sp=np.cos(ph),np.sin(ph); ct,st=np.cos(th),np.sin(th); cy,sy=np.cos(ps),np.sin(ps)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ \
           np.array([[ct,0,st],[0,1,0],[-st,0,ct]]) @ \
           np.array([[1,0,0],[0,cp,-sp],[0,sp,cp]])

def deflect_fin(fin_local, ang, axis):
    """Rotasikan sirip terhadap sumbu radialnya (defleksi servo)."""
    c,s=np.cos(ang),np.sin(ang); u=axis/np.linalg.norm(axis)
    ux,uy,uz=u
    Rr=np.array([[c+ux*ux*(1-c), ux*uy*(1-c)-uz*s, ux*uz*(1-c)+uy*s],
                 [uy*ux*(1-c)+uz*s, c+uy*uy*(1-c), uy*uz*(1-c)-ux*s],
                 [uz*ux*(1-c)-uy*s, uz*uy*(1-c)+ux*s, c+uz*uz*(1-c)]])
    ctr=fin_local.mean(axis=0)
    return (Rr@(fin_local-ctr).T).T + ctr

def rocket_faces(R, pos, scale=1.0, tl=0.5, servo_k=None):
    rb=0.04*scale; hb=0.20*scale; rn=0.04*scale; hn=0.08*scale; rp=0.10*scale; ns=16
    th_a=np.linspace(0,2*np.pi,ns+1)
    def tw(vl): return (R@vl.T).T+pos
    # Badan silinder
    bf=[]
    for i in range(ns):
        a0,a1=th_a[i],th_a[i+1]
        bf.append(tw(np.array([[rb*np.cos(a0),rb*np.sin(a0),0],[rb*np.cos(a1),rb*np.sin(a1),0],
                                [rb*np.cos(a1),rb*np.sin(a1),hb],[rb*np.cos(a0),rb*np.sin(a0),hb]])))
    # Nose cone
    nf=[]
    for i in range(ns):
        a0,a1=th_a[i],th_a[i+1]
        nf.append(tw(np.array([[rn*np.cos(a0),rn*np.sin(a0),hb],[rn*np.cos(a1),rn*np.sin(a1),hb],[0,0,hb+hn]])))
    # Piringan propeller
    pf=[]
    pz=hb-0.005
    for i in range(ns):
        a0,a1=th_a[i],th_a[i+1]
        pf.append(tw(np.array([[rp*np.cos(a0),rp*np.sin(a0),pz],[rp*np.cos(a1),rp*np.sin(a1),pz],[0,0,pz]])))
    # 4 sirip (dengan defleksi servo)
    fins=[]
    fin_angles=[0,np.pi/2,np.pi,3*np.pi/2]
    for fi,fa in enumerate(fin_angles):
        ca,sa=np.cos(fa),np.sin(fa); ri,ro=rb,rb+0.055*scale; fh=0.045*scale
        fin_local=np.array([[ri*ca,ri*sa,0],[ro*ca,ro*sa,0],[ro*ca,ro*sa,fh],[ri*ca,ri*sa,fh]])
        # Defleksi sesuai servo (rotasi terhadap sumbu radial)
        if servo_k is not None:
            axis=np.array([ca,sa,0.0])
            fin_local=deflect_fin(fin_local, np.deg2rad(servo_k[fi]), axis)
        fins.append(tw(fin_local))
    # Flame thrust
    ff=[]
    if tl>0.3:
        fl=0.07*scale*tl
        for i in range(ns):
            a0,a1=th_a[i],th_a[i+1]; rf=rb*0.55
            ff.append(tw(np.array([[rf*np.cos(a0),rf*np.sin(a0),0],[rf*np.cos(a1),rf*np.sin(a1),0],[0,0,-fl]])))
    return bf,nf,pf,fins,ff

# ─── FASE (konsisten skenario MATLAB) ───────────────────────
phases=[(0,'GROUND'),(2,'TAKEOFF'),(5,'HOVER'),(7,'ROLL MANEUVER'),
        (10,'LEVELING'),(12,'PITCH MANEUVER'),(15,'LEVELING'),(17,'YAW TURN'),
        (21,'HEADING HOME'),(22,'DESCENDING')]
def get_phase(tc):
    p=phases[0][1]
    for ts,name in phases:
        if tc>=ts: p=name
    return p

SKIP = max(1, N//180)    # ~180 frame
frames_to_render = list(range(0, N, SKIP))
n_frames = len(frames_to_render)

xmn,xmx=xp.min()-0.4,xp.max()+0.4; ymn,ymx=yp.min()-0.4,yp.max()+0.4; zmx=alt.max()+0.6
xg,yg=np.meshgrid(np.linspace(xmn,xmx,7),np.linspace(ymn,ymx,7))

# Warna sirip: pasangan roll (S1,S3) oranye, pasangan pitch (S2,S4) hijau
fin_colors=[(0.9,0.6,0.1),(0.1,0.75,0.3),(0.9,0.6,0.1),(0.1,0.75,0.3)]

print(f"Rendering {n_frames} frames (skip={SKIP})...")
for fi, k in enumerate(frames_to_render):
    fig = plt.figure(figsize=(14,9), facecolor=BG)
    gs  = GridSpec(3,3,figure=fig,left=0.02,right=0.97,top=0.95,bottom=0.06,
                   hspace=0.45,wspace=0.30)

    ax3 = fig.add_subplot(gs[:,0:2], projection='3d')
    ax3.set_facecolor('#f0f3f8')
    for pane in [ax3.xaxis.pane,ax3.yaxis.pane,ax3.zaxis.pane]:
        pane.fill=False; pane.set_edgecolor(GRD)
    ax3.grid(True,color=GRD,lw=0.3)
    ax3.set_xlim(xmn,xmx); ax3.set_ylim(ymn,ymx); ax3.set_zlim(0,zmx)
    ax3.tick_params(colors='#334466',labelsize=6)
    ax3.set_xlabel('X',color='#37474f',fontsize=8); ax3.set_ylabel('Y',color='#37474f',fontsize=8)
    ax3.set_zlabel('Alt(m)',color='#37474f',fontsize=8)
    ax3.view_init(elev=22, azim=200 + t[k]*3.5)

    ax3.plot_surface(xg,yg,np.zeros_like(xg),alpha=0.10,color='#90caf9',edgecolor=GRD)
    ax3.plot(xp,yp,alt,color='#b0bec5',lw=0.7,alpha=0.6)
    i0=max(0,k-100)
    if k> i0:
        trail_col = plt.cm.plasma(np.linspace(0.3,1.0,k-i0+1))
        for ti in range(i0, k):
            ax3.plot([xp[ti],xp[ti+1]],[yp[ti],yp[ti+1]],[alt[ti],alt[ti+1]],
                     color=trail_col[ti-i0], lw=1.8, alpha=0.85)

    pos_k = np.array([xp[k],yp[k],alt[k]])
    R_k   = rot_zyx(phi_r[k],tht_r[k],psi_r[k])
    tl_k  = thr[k]/100.0
    bf,nf,pf,fins,ff = rocket_faces(R_k,pos_k,scale=1.0,tl=tl_k,servo_k=servo[:,k])

    pc_b=Poly3DCollection(bf,alpha=0.88); pc_b.set_facecolor((0.22,0.32,0.52)); pc_b.set_edgecolor('none')
    ax3.add_collection3d(pc_b)
    pc_n=Poly3DCollection(nf,alpha=0.92); pc_n.set_facecolor((0.82,0.18,0.12)); pc_n.set_edgecolor('none')
    ax3.add_collection3d(pc_n)
    pc_p=Poly3DCollection(pf,alpha=0.15+0.18*tl_k); pc_p.set_facecolor((0.5,0.88,1.0)); pc_p.set_edgecolor('none')
    ax3.add_collection3d(pc_p)
    for fv,fc in zip(fins,fin_colors):
        pfin=Poly3DCollection([fv],alpha=0.9); pfin.set_facecolor(fc); pfin.set_edgecolor('none')
        ax3.add_collection3d(pfin)
    if ff:
        pff=Poly3DCollection(ff,alpha=0.5); pff.set_facecolor((1.0,0.3+0.4*tl_k,0.05)); pff.set_edgecolor('none')
        ax3.add_collection3d(pff)

    td=R_k@np.array([0,0,0.32])
    ax3.quiver(pos_k[0],pos_k[1],pos_k[2],td[0],td[1],td[2],
               color='#e65100',alpha=0.7*tl_k+0.15,arrow_length_ratio=0.25,lw=2.0)

    hud = (f"t = {t[k]:.2f} s\n"
           f"Phase: {get_phase(t[k])}\n"
           f"Alt: {alt[k]:.2f} m\n"
           f"Roll: {phi_d[k]:.1f}  Pitch: {theta_d[k]:.1f}\n"
           f"Yaw: {yaw_d[k]:.1f}  Thr: {thr[k]:.0f}%")
    ax3.text2D(0.02, 0.97, hud, transform=ax3.transAxes,
               color='#1a237e', fontsize=8, va='top', fontfamily='monospace',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='#f5f7fa',
                         edgecolor='#b0bec5', alpha=0.7))
    fig.text(0.5, 0.97, 'MONOCHOPTER FLIGHT SIMULATOR',
             ha='center', color='#1a237e', fontsize=11, fontweight='bold')

    # Panel kanan
    k1=min(k+1,N)
    ax_r = fig.add_subplot(gs[0,2])
    ax_r.set_facecolor(PAN); ax_r.grid(True,color=GRD,lw=0.4)
    for sp in ax_r.spines.values(): sp.set_color(GRD)
    ax_r.tick_params(colors='#8899bb',labelsize=6)
    ax_r.set_title('Roll (deg)',color='#1a237e',fontsize=8,pad=3)
    ax_r.plot(t,ref_roll,'--',color='#37474f',lw=0.7,alpha=0.6)
    ax_r.plot(t[:k1],phi_d[:k1],color=C2,lw=1.4)
    ax_r.set_xlim(0,float(t[-1])); ax_r.set_ylim(-18,18)
    ax_r.axvline(x=t[k],color='#e65100',lw=0.8,alpha=0.7)

    ax_a = fig.add_subplot(gs[1,2])
    ax_a.set_facecolor(PAN); ax_a.grid(True,color=GRD,lw=0.4)
    for sp in ax_a.spines.values(): sp.set_color(GRD)
    ax_a.tick_params(colors='#8899bb',labelsize=6)
    ax_a.set_title('Altitude (m)',color='#1a237e',fontsize=8,pad=3)
    ax_a.plot(t,ref_alt,'--',color='#37474f',lw=0.7,alpha=0.6)
    ax_a.fill_between(t[:k1],0,alt[:k1],alpha=0.15,color=C1)
    ax_a.plot(t[:k1],alt[:k1],color=C1,lw=1.4)
    ax_a.set_xlim(0,float(t[-1])); ax_a.set_ylim(-0.1,zmx)
    ax_a.axvline(x=t[k],color='#e65100',lw=0.8,alpha=0.7)

    ax_t = fig.add_subplot(gs[2,2])
    ax_t.set_facecolor(PAN); ax_t.grid(True,color=GRD,lw=0.4)
    for sp in ax_t.spines.values(): sp.set_color(GRD)
    ax_t.tick_params(colors='#8899bb',labelsize=6)
    ax_t.set_title('Throttle (%)',color='#1a237e',fontsize=8,pad=3)
    ax_t.fill_between(t[:k1],0,thr[:k1],alpha=0.2,color=C3)
    ax_t.plot(t[:k1],thr[:k1],color=C3,lw=1.3)
    ax_t.set_xlim(0,float(t[-1])); ax_t.set_ylim(0,100)
    ax_t.axvline(x=t[k],color='#e65100',lw=0.8,alpha=0.7)
    ax_t.set_xlabel('t (s)',color='#8899bb',fontsize=6)

    fname = os.path.join(FRAMES, f'frame_{fi:04d}.png')
    fig.savefig(fname, dpi=100, facecolor=BG, bbox_inches='tight')
    plt.close(fig)

    if fi % 20 == 0:
        print(f"  Frame {fi}/{n_frames} ({100*fi/n_frames:.0f}%)")

print("All frames rendered. Stitching to MP4...")
mp4_path = os.path.join(HERE, 'animasi_3D_monochopter.mp4')
cmd = [
    'ffmpeg', '-y', '-framerate', '20',
    '-i', os.path.join(FRAMES, 'frame_%04d.png'),
    '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2',
    '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
    '-crf', '22', '-preset', 'medium', mp4_path
]
result = subprocess.run(cmd, capture_output=True, text=True)
if result.returncode == 0:
    size = os.path.getsize(mp4_path)
    print(f"MP4 saved: {mp4_path} ({size//1024} KB)")
else:
    print("ffmpeg error:", result.stderr[-500:])
