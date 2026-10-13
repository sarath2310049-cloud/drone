"""Software-in-the-loop test of ArUco pose estimation (OpenCV) on rendered images.
Camera: 1280x720 webcam, f = 900 px (about 70 deg horizontal FOV). Marker: DICT_4X4_50 id 0.
Degradations: Gaussian blur, sensor noise, in-plane/yaw/tilt jitter. Ground truth pose is known exactly."""
import numpy as np, cv2, json, sys
rng=np.random.default_rng(1)
W,H,F=1280,720,900.0
K=np.array([[F,0,W/2],[0,F,H/2],[0,0,1.0]]); D=np.zeros(5)
aru=cv2.aruco; dic=aru.getPredefinedDictionary(aru.DICT_4X4_50)
par=aru.DetectorParameters(); par.cornerRefinementMethod=aru.CORNER_REFINE_SUBPIX
det=aru.ArucoDetector(dic,par)
MS=200; PAD=40
m=aru.generateImageMarker(dic,0,MS); m=cv2.copyMakeBorder(m,PAD,PAD,PAD,PAD,cv2.BORDER_CONSTANT,value=255)
SP=MS+2*PAD
def Rx(a):c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(a):c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a):c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
R0=np.diag([1,-1,-1.0])
def render(L,t,R):
    Lp=L*SP/MS; h=Lp/2
    obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    pc=(R@obj.T).T+t
    uv=(K@pc.T).T; uv=(uv[:,:2]/uv[:,2:3]).astype(np.float32)
    src=np.array([[0,0],[SP,0],[SP,SP],[0,SP]],np.float32)
    Hm=cv2.getPerspectiveTransform(src,uv)
    img=cv2.warpPerspective(m,Hm,(W,H),flags=cv2.INTER_LINEAR,borderValue=140)
    return img
def trial(L,Z):
    t=np.array([rng.uniform(-0.5,0.5),rng.uniform(-0.3,0.3),Z])
    R=Ry(np.deg2rad(np.clip(rng.normal(0,10),-25,25)))@Rx(np.deg2rad(rng.normal(0,3)))@Rz(np.deg2rad(rng.normal(0,3)))@R0
    img=render(L,t,R).astype(np.float32)
    img=cv2.GaussianBlur(img,(0,0),rng.uniform(0.5,1.2))
    img=img*rng.uniform(0.8,1.1)+rng.normal(0,4,img.shape)
    img=np.clip(img,0,255).astype(np.uint8)
    c,ids,_=det.detectMarkers(img)
    if ids is None or 0 not in ids.flatten(): return None
    c=c[list(ids.flatten()).index(0)].reshape(4,2)
    h=L/2; obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
    ok,rv,tv=cv2.solvePnP(obj,c,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok: return None
    return (tv.flatten()-t)*1000.0
if __name__=='__main__':
    N=int(sys.argv[1]) if len(sys.argv)>1 else 300
    res={}
    for L in (0.10,0.15):
        for Z in (1.0,1.5,2.0,2.5,3.0,3.5,4.0):
            e=[trial(L,Z) for _ in range(N)]
            ok=[x for x in e if x is not None]; a=np.array(ok)
            rms=np.sqrt((a**2).mean(0)) if len(a) else [np.nan]*3
            res[f'{L}_{Z}']=dict(L=L,Z=Z,det=len(ok)/N,rms_x=float(rms[0]),rms_y=float(rms[1]),rms_z=float(rms[2]))
            print(res[f'{L}_{Z}'],flush=True)
    json.dump(res,open('pose_results.json','w'),indent=1)
