import json, numpy as np, sys
from multiprocessing import Pool
import sim_flight as sf
pose=json.load(open('pose_results.json')); qr=json.load(open('qr_results.json'))
# measurement noise from the pose-estimation test (marker 15 cm, camera-to-drone distance 2.5 m)
p=pose['0.15_2.5']; P=dict(sf.PARAMS)
P['sig_xy']=p['rms_x']/1000; P['sig_z']=p['rms_y']/1000; P['sig_d']=p['rms_z']/1000
print('sigma lateral/height/depth (m):',P['sig_xy'],P['sig_z'],P['sig_d'],flush=True)
N=int(sys.argv[1]) if len(sys.argv)>1 else 100
def one(a):
    seed,eps,use_kf,ds=a
    r=sf.run(sf.WAYPOINTS,P=P,seed=seed,eps=eps,use_kf=use_kf,delay_scale=ds)
    return r
def summarize(rs):
    comp=np.mean([r['completed'] for r in rs])
    done=[r for r in rs if r['completed']]
    t=np.mean([r['time'] for r in done]) if done else float('nan')
    rh=np.array([r['rms_hold'] for r in rs if r['completed']])
    rms=np.sqrt((rh**2).mean(0)).tolist() if len(rh) else [float('nan')]*3
    return dict(completion=float(comp),time=float(t),rms_hold_xyz=rms,viol=float(np.mean([r['viol']>0 for r in rs])))
# QR read probability vs distance (label 8 cm, tilt 0): interpolate Sim B
tab=sorted([(r['dist'],r['read']) for r in qr if r['label_m']==0.10 and r['tilt']==0])
D=np.array([a for a,b in tab]);R=np.array([b for a,b in tab])
def pread(d): return float(np.interp(d,D,R))
STAND=0.45
def qr_mission(rs,rng):
    out=[]
    for r in rs:
        if not r['completed']: continue
        k=0;ok=0
        for wi,pos,tgt in r['caps']:
            if wi in (1,2,3):
                d=STAND+(pos[1]-tgt[1]); p1=pread(d); ok+= rng.random()<1-(1-p1)**5; k+=1
        out.append(ok/3)
    return float(np.mean(out)) if out else float('nan')
if __name__=='__main__':
    res={}
    with Pool(2) as pool:
        for eps in (0.03,0.05,0.10):
            rs=pool.map(one,[(s,eps,True,None) for s in range(N)]); res[f'eps_{eps}']=summarize(rs)
            res[f'eps_{eps}']['qr_per_mission']=qr_mission(rs,np.random.default_rng(5)); print('eps',eps,res[f'eps_{eps}'],flush=True)
        rs=pool.map(one,[(s,0.10,False,None) for s in range(N)]); res['no_kf_eps_0.10']=summarize(rs); print('no KF',res['no_kf_eps_0.10'],flush=True)
        for tot in (0.04,0.07,0.10,0.12,0.14,0.16,0.20,0.30):
            rs=pool.map(one,[(s,0.10,True,tot/0.07) for s in range(50)]); res[f'delay_{tot}']=summarize(rs); print('delay',tot,res[f'delay_{tot}'],flush=True)
    json.dump(res,open('flight_results.json','w'),indent=1)
