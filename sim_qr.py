"""Software-in-the-loop test of QR decoding on rendered ESP32-CAM-like frames.
Assumed camera: 640x480 (VGA), f = 600 px (about 56 deg horizontal FOV), fixed focus set to 0.5 m.
Degradations: defocus blur growing with focus error, horizontal motion blur 0-2 px, Gaussian noise, JPEG quality 60."""
import numpy as np, cv2, json, sys
rng=np.random.default_rng(2)
W,H,F=640,480,600.0
K=np.array([[F,0,W/2],[0,F,H/2],[0,0,1.0]])
DF=0.5
enc=cv2.QRCodeEncoder.create(); det=cv2.QRCodeDetector()
def make(text):
    q=enc.encode(text); q=cv2.resize(q,(250,250),interpolation=cv2.INTER_NEAREST)  # 25 modules x 10 px
    return cv2.copyMakeBorder(q,40,40,40,40,cv2.BORDER_CONSTANT,value=255)           # 4-module quiet zone
Q=make('ITEM-000123'); SP=Q.shape[0]
def Ry(a):c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def trial(Ls,d,tilt_deg):
    Lp=Ls*33/25; h=Lp/2
    obj=np.array([[-h,-h,0],[h,-h,0],[h,h,0],[-h,h,0]])
    R=Ry(np.deg2rad(tilt_deg*rng.choice([-1,1])))
    t=np.array([rng.uniform(-0.08,0.08)*d,rng.uniform(-0.06,0.06)*d,d])
    pc=(R@obj.T).T+t; uv=(K@pc.T).T; uv=(uv[:,:2]/uv[:,2:3]).astype(np.float32)
    Hm=cv2.getPerspectiveTransform(np.array([[0,0],[SP,0],[SP,SP],[0,SP]],np.float32),uv)
    img=cv2.warpPerspective(Q,Hm,(W,H),flags=cv2.INTER_LINEAR,borderValue=120).astype(np.float32)
    s=0.5+1.0*abs(1/d-1/DF)
    img=cv2.GaussianBlur(img,(0,0),s)
    L=int(rng.integers(1,4)); k=np.zeros((1,L),np.float32)+1.0/L
    img=cv2.filter2D(img,-1,k)
    img=img*rng.uniform(0.75,1.1)+rng.normal(0,5,img.shape)
    img=np.clip(img,0,255).astype(np.uint8)
    ok,b=cv2.imencode('.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,60]); img=cv2.imdecode(b,0)
    try: txt,_,_=det.detectAndDecode(img)
    except cv2.error: txt=''
    return txt=='ITEM-000123'
if __name__=='__main__':
    N=int(sys.argv[1]) if len(sys.argv)>1 else 100
    res=[]
    for Ls in (0.05,0.08,0.10):
        for tilt in (0,20,40):
            for d in (0.2,0.3,0.4,0.5,0.6,0.8,1.0,1.2):
                r=np.mean([trial(Ls,d,tilt) for _ in range(N)])
                ppm=F*Ls/d/25
                res.append(dict(label_m=Ls,tilt=tilt,dist=d,read=float(r),px_per_module=float(ppm)))
                print(res[-1],flush=True)
    json.dump(res,open('qr_results.json','w'),indent=1)
