import json, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'DejaVu Serif','font.size':8,'axes.grid':True,'grid.alpha':0.3})
pose=json.load(open('pose_results.json')); qr=json.load(open('qr_results.json'))
# Fig 3 pose error vs distance
f,ax=plt.subplots(figsize=(3.6,2.6))
for L,ls in ((0.15,'-'),(0.10,'--')):
    Z=[v['Z'] for v in pose.values() if v['L']==L]
    lat=[np.hypot(v['rms_x'],v['rms_y']) for v in pose.values() if v['L']==L]
    dep=[v['rms_z'] for v in pose.values() if v['L']==L]
    ax.plot(Z,dep,ls,color='black',marker='o',ms=3,label=f'depth, {int(L*100)} cm marker')
    ax.plot(Z,lat,ls,color='#777777',marker='s',ms=3,label=f'lateral, {int(L*100)} cm marker')
ax.set_yscale('log'); ax.set_xlabel('Camera-to-marker distance (m)'); ax.set_ylabel('RMS position error (mm)'); ax.legend(fontsize=6.5,frameon=False)
plt.savefig('fig3.png',dpi=220,bbox_inches='tight'); plt.close()
# Fig 4 QR read rate
f,axs=plt.subplots(1,2,figsize=(7.0,2.6),sharey=True)
for ax,tilt in zip(axs,(0,20)):
    for Ls,mk,c in ((0.05,'o','#aaaaaa'),(0.08,'s','#666666'),(0.10,'^','black')):
        r=[x for x in qr if x['label_m']==Ls and x['tilt']==tilt]
        ax.plot([x['dist'] for x in r],[100*x['read'] for x in r],marker=mk,ms=3,color=c,label=f'{int(Ls*100)} cm label')
    ax.set_title(f'Viewing angle {tilt}°',fontsize=8); ax.set_xlabel('Distance (m)')
axs[0].set_ylabel('Read rate per frame (%)'); axs[0].legend(fontsize=6.5,frameon=False)
plt.savefig('fig4.png',dpi=220,bbox_inches='tight'); plt.close()
# Fig 5 trajectory
import sim_flight as sf
from run_experiments import P
r,log=sf.run(sf.WAYPOINTS,P=P,seed=1,eps=0.10,record=True)
f,axs=plt.subplots(2,1,figsize=(7.0,3.4),sharex=True)
axs[0].plot(log[:,0],log[:,1],color='black',lw=1,label='true'); axs[0].plot(log[:,0],log[:,4],color='#888888',lw=0.7,label='filtered estimate'); axs[0].set_ylabel('Lateral x (m)'); axs[0].legend(fontsize=6.5,frameon=False,loc='upper right')
axs[1].plot(log[:,0],log[:,3],color='black',lw=1); axs[1].plot(log[:,0],log[:,6],color='#888888',lw=0.7); axs[1].set_ylabel('Height z (m)'); axs[1].set_xlabel('Time (s)')
for z in (0.5,1.0,1.5): axs[1].axhline(z,color='black',ls=':',lw=0.6)
axs[0].axhline(0.8,color='black',ls=':',lw=0.6)
plt.savefig('fig5.png',dpi=220,bbox_inches='tight'); plt.close()
# Fig 6 latency
fr=json.load(open('flight_results.json'))
ds=sorted(float(k.split('_')[1]) for k in fr if k.startswith('delay_'))
f,ax=plt.subplots(figsize=(3.6,2.6))
ax.plot([1000*d for d in ds],[100*fr[f'delay_{d}']['completion'] for d in ds],color='black',marker='o',ms=3)
ax.axvline(70,color='black',ls=':',lw=0.8); ax.text(72,50,'nominal\n70 ms',fontsize=6.5)
ax.set_xlabel('Total loop delay (ms)'); ax.set_ylabel('Mission completion (%)'); ax.set_ylim(-3,103)
plt.savefig('fig6.png',dpi=220,bbox_inches='tight'); plt.close()
print('figs ok')
