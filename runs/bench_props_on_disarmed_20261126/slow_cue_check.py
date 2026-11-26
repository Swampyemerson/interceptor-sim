import sys, math, random, collections
sys.path.insert(0,'.'); sys.path.insert(0,'scripts/field'); sys.path.insert(0,'tests')
import crsf_cue_source as ccs
from flight.cue_link import CueReceiver
from flight.cue_relay import CueQualityError
import test_crsf_cue_source as T
R=6378137.0
own=(40.0,-105.0,1650.0)
def run(speed, course, pos_sigma, spd_sigma, crs_sigma, jit, seed):
    rnd=random.Random(seed)
    sent, src = T.collect(); rx=CueReceiver()
    vn,ve=speed*math.cos(math.radians(course)),speed*math.sin(math.radians(course))
    t=100.0
    # slowly wandering GPS error (first-order), plus reported speed/course noise
    en=ee=0.0
    for k in range(12):
        dt=0.5*k
        en=0.9*en+rnd.gauss(0,pos_sigma*0.44); ee=0.9*ee+rnd.gauss(0,pos_sigma*0.44)
        lat=own[0]+math.degrees((15.0+vn*dt+en)/R)
        lon=own[1]+math.degrees((5.0+ve*dt+ee)/(R*math.cos(math.radians(own[0]))))
        s=max(0.0,speed+rnd.gauss(0,spd_sigma)); c=(course+rnd.gauss(0,crs_sigma))%360
        raw=T.frame(0x02, ccs.build_gps_payload(lat,lon,s,c,1657,14))
        tr=t+dt+abs(rnd.gauss(0.1,jit))
        n0=len(sent); src.feed(raw,tr)
        if len(sent)>n0: rx.feed(sent[-1], t_rx=tr)
    try:
        sol=rx.latch(*own, now_t=t+5.5+0.3)
        dv=math.hypot(sol.speed_ms*math.cos(math.radians(sol.heading_deg))-vn, sol.speed_ms*math.sin(math.radians(sol.heading_deg))-ve)
        return ('ok',sol.speed_ms,sol.heading_deg,dv)
    except Exception as e:
        return ('refused: '+str(e)[:110],)
for label,speed,ps,ss,cs in [('2 mph, clean',0.9,0,0,0),('2 mph, GPS-like noise',0.9,0.3,0.15,25),('hover (0 m/s), noise',0.0,0.3,0.15,180),('9 m/s, noise',9.0,0.3,0.15,3)]:
    res=[run(speed,60.0,ps,ss,cs,0.05,s) for s in range(200)]
    c=collections.Counter(r[0] for r in res); oks=[r for r in res if r[0]=='ok']
    print(label, dict(c.most_common(4)))
    if oks:
        dv=sorted(r[3] for r in oks); print('   fitted speed median %.2f, velocity error median %.2f m/s, 90th pct %.2f m/s'%(sorted(r[1] for r in oks)[len(oks)//2], dv[len(dv)//2], dv[int(len(dv)*.9)]))
