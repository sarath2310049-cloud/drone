"""Closed-loop simulation of the camera-in-the-loop quadcopter controller (Python/NumPy).
Plant per axis: attitude-hold inner loop (first-order lag) -> acceleration -> double integrator with drag.
Measurement: delayed, noisy pose at 30 Hz with 2 % dropouts. Estimator: constant-velocity Kalman filter.
Controller: PID with anti-windup and saturation. Disturbance: Ornstein-Uhlenbeck acceleration (gusts / prop wash).
Vertical axis has an unknown hover-throttle bias (battery sag). All parameters are assumptions, listed in PARAMS."""
import numpy as np, json, sys
g=9.81
PARAMS=dict(dt=0.005,fs=30.0,tau_att=0.12,tau_thr=0.15,drag=0.3,theta_max=0.20,az_max=4.0,
            delay_meas=0.045,delay_cmd=0.025,jit=0.010,dropout=0.02,
            sig_xy=0.003,sig_z=0.003,sig_d=0.025,   # measurement noise (m): lateral, height, depth (filled from sim_pose)
            gust_sigma=0.15,gust_tau=0.5,zbias_sigma=0.05*g)
Kp_xy,Ki_xy,Kd_xy=0.23,0.05,0.31   # rad per m, per m s, per m/s
Kp_z,Ki_z,Kd_z=4.0,1.0,3.5         # m/s^2 per m ...
ICLAMP=np.array([0.5,0.5,2.0])   # integrator clamp (anti-windup); z allows Ki*clamp = 2 m/s^2 to cancel hover-throttle bias
class KF:
    def __init__(s,r,q=1.5,dt=1/30):
        s.x=np.zeros(2);s.P=np.diag([1.0,1.0]);s.F=np.array([[1,dt],[0,1]]);s.Q=q**2*np.array([[dt**4/4,dt**3/2],[dt**3/2,dt**2]]);s.r=r**2
    def step(s,z):
        s.x=s.F@s.x;s.P=s.F@s.P@s.F.T+s.Q
        if z is not None:
            S=s.P[0,0]+s.r;K=s.P[:,0]/S;s.x=s.x+K*(z-s.x[0]);s.P=s.P-np.outer(K,s.P[0,:])
        return s.x.copy()
def run(wps,P=PARAMS,seed=0,use_kf=True,eps=0.10,Th=1.0,tmax=120.0,hold=3.0,delay_scale=None,record=False):
    """wps: list of (target xyz, dwell?) ; each waypoint must be 'reached' (|e|<eps for Th) then held `hold` s for capture."""
    rng=np.random.default_rng(seed)
    dt=P['dt'];n_per=int(round(1/(P['fs']*dt)))
    dm=P['delay_meas'];dc=P['delay_cmd']
    if delay_scale is not None: dm,dc=dm*delay_scale,dc*delay_scale
    sig=np.array([P['sig_xy'],P['sig_d'],P['sig_z']])      # axes: x lateral, y depth/standoff, z height
    pos=np.array([0.0,0.0,0.0]);vel=np.zeros(3);th=np.zeros(3)   # th: lag state (command applied: roll/pitch/thrust)
    gust=np.zeros(3);zb=rng.normal(0,P['zbias_sigma'])
    kfs=[KF(sig[i]) for i in range(3)]
    integ=np.zeros(3);prev_est=np.zeros(3)
    buf_state=[];buf_cmd=[]      # history for delays
    cmd_applied=np.zeros(3);cmd_queue=[]
    t=0.0;k=0;wi=0;state='reach';t_in=0.0;t_hold=0.0
    est=np.zeros(3);vest=np.zeros(3);u=np.zeros(3)
    log=[];caps=[];viol=0;
    nsteps=int(tmax/dt)
    hist=np.zeros((nsteps+1,3))
    err_hold=[];done=False;t_arr=[]
    for s_i in range(nsteps):
        t=s_i*dt;hist[s_i]=pos
        if s_i%n_per==0:
            # measurement of pose at time t-dm (+jitter)
            d_idx=max(0,s_i-int((dm+rng.uniform(0,P['jit']))/dt))
            zm=hist[d_idx]+rng.normal(0,sig)
            drop=rng.random()<P['dropout']
            out=[]
            for i in range(3):
                x=kfs[i].step(None if drop else zm[i])
                out.append(x)
            if use_kf:
                est=np.array([o[0] for o in out])+np.array([o[1] for o in out])*dm   # delay-compensated
                vest=np.array([o[1] for o in out])
            else:
                est=zm if not drop else est
                vest=(est-prev_est)*P['fs'];prev_est=est.copy()
            # waypoint logic
            tgt=np.array(wps[wi])
            e=tgt-est
            if state=='reach':
                if np.linalg.norm(e)<eps: t_in+=1/P['fs']
                else: t_in=0.0
                if t_in>=Th: state='hold';t_hold=0.0;t_arr.append(t)
            elif state=='hold':
                t_hold+=1/P['fs'];err_hold.append(tgt-pos)
                if t_hold>=hold:
                    caps.append((wi,pos.copy(),tgt.copy()))
                    wi+=1;state='reach';t_in=0.0;integ[:]=0
                    if wi>=len(wps): done=True;break
            tgt=np.array(wps[wi]);e=tgt-est
            integ=np.clip(integ+e/P['fs'],-ICLAMP,ICLAMP)
            u[0]=np.clip(Kp_xy*e[0]+Ki_xy*integ[0]-Kd_xy*vest[0],-P['theta_max'],P['theta_max'])
            u[1]=np.clip(Kp_xy*e[1]+Ki_xy*integ[1]-Kd_xy*vest[1],-P['theta_max'],P['theta_max'])
            u[2]=np.clip(Kp_z*e[2]+Ki_z*integ[2]-Kd_z*vest[2],-P['az_max'],P['az_max'])
            cmd_queue.append((t+dc+rng.uniform(0,P['jit']),u.copy()))
        while cmd_queue and cmd_queue[0][0]<=t: cmd_applied=cmd_queue.pop(0)[1]
        # plant
        th+=(cmd_applied-th)/(P['tau_att'] if True else 1)*dt*np.array([1,1,P['tau_att']/P['tau_thr']])
        gust+=(-gust/P['gust_tau'])*dt+P['gust_sigma']*np.sqrt(2*dt/P['gust_tau'])*rng.normal(size=3)
        zbias=zb*(1+0.0*t)
        acc=np.array([g*th[0]+gust[0]-P['drag']*vel[0],g*th[1]+gust[1]-P['drag']*vel[1],th[2]+gust[2]*0.5-zbias-P['drag']*vel[2]])
        # ground: cannot go below z=0 unless at start
        vel+=acc*dt;pos+=vel*dt
        if pos[2]<0: pos[2]=0;vel[2]=max(vel[2],0)
        if abs(pos[1])>0.25 or pos[2]>3: viol+=1
        if record: log.append((t,*pos,*est))
        if abs(pos[0])>3 or abs(pos[1])>2: break
    eh=np.array(err_hold) if err_hold else np.zeros((1,3))
    res=dict(completed=bool(done),time=float(t),n_caps=len(caps),viol=int(viol),
             rms_hold=np.sqrt((eh**2).mean(0)).tolist(),caps=caps)
    return (res,np.array(log)) if record else res
WAYPOINTS=[(0.0,0.0,0.5),(0.8,0.0,0.5),(0.8,0.0,1.0),(0.8,0.0,1.5),(0.0,0.0,1.5),(0.0,0.0,0.3)]
