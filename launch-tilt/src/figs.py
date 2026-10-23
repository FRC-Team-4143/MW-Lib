import math, cairosvg
exec(open('sim.py').read().split('for d,al')[0])
BLUE='#2a78d6'; ORANGE='#eb6834'; INK='#0b0b0b'; INK2='#52514e'; MUTED='#8a8984'; SURF='#fcfcfb'; GRID='#e4e3df'
FONT="font-family='Helvetica, Arial, sans-serif'"
def doc(w,h,body,title):
    return f"""<svg xmlns='http://www.w3.org/2000/svg' width='{w}' height='{h}' viewBox='0 0 {w} {h}' {FONT}>
<rect width='{w}' height='{h}' rx='12' fill='{SURF}'/>
<text x='24' y='36' font-size='20' font-weight='700' fill='{INK}'>{title}</text>
{body}</svg>"""
def traj(v,u,n=60,stop_desc=True):
    vx,vy,vz=[v*c for c in u]; disc=vz*vz-2*g*h; T=(vz+math.sqrt(disc))/g
    return [(vx*T*i/n,vy*T*i/n,vz*T*i/n-0.5*g*(T*i/n)**2) for i in range(n+1)]
def path(pts):return 'M'+' L'.join(f'{x:.1f},{y:.1f}' for x,y in pts)
def arrow_defs():
    return f"""<defs>{''.join(f"<marker id='a{n}' viewBox='0 0 10 10' refX='8' refY='5' markerWidth='7' markerHeight='7' orient='auto-start-reverse'><path d='M0,0 L10,5 L0,10 z' fill='{c}'/></marker>" for n,c in [('b',BLUE),('o',ORANGE),('k',INK2)])}</defs>"""
LZ=0.489; d=3.0; al=math.radians(71.8); v=speed(d,al); tilt=math.radians(5)

# ---------- Fig 1: side view, pitch along the shot
W,H=900,600; S=120; ox,oy=170,560   # launcher origin px, ground y
X=lambda x:ox+x*S; Y=lambda z:oy-z*S
b=[arrow_defs()]
b.append(f"<line x1='20' y1='{oy}' x2='{W-20}' y2='{oy}' stroke='{INK2}' stroke-width='2'/>")
# hub
hx=X(d); b.append(f"<path d='M{hx-70},{oy} L{hx-40},{Y(1.83)} L{hx+40},{Y(1.83)} L{hx+70},{oy} z' fill='#efeee9' stroke='{INK2}' stroke-width='1.5'/>")
b.append(f"<line x1='{hx-40}' y1='{Y(1.83)}' x2='{hx+40}' y2='{Y(1.83)}' stroke='{INK}' stroke-width='4'/>")
b.append(f"<text x='{hx}' y='{oy-60}' text-anchor='middle' font-size='15' fill='{INK2}'>HUB</text>")
# robot (shooter faces the hub, on the robot's back end); tilted nose-up along shot by 5deg, ball under far wheel
rw,rh=0.75,0.30; c,s=math.cos(tilt),math.sin(tilt)
base=(-0.70,0.06)
def R(px,pz):  # rotate around rear wheel contact at (base)
    return X(base[0]+px*c-pz*s), Y(base[1]+px*s+pz*c)
corners=[R(0,0),R(rw,0),R(rw,rh),R(0,rh)]
b.append(f"<polygon points='{' '.join(f'{x:.1f},{y:.1f}' for x,y in corners)}' fill='#dcdad3' stroke='{INK}' stroke-width='2'/>")
for wx in (0.1,rw-0.1):
    x,y=R(wx,-0.03); b.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='11' fill='{INK2}'/>")
bx,by=R(rw-0.1,-0.03); b.append(f"<circle cx='{bx:.1f}' cy='{oy-0.07*S:.1f}' r='{0.07*S:.0f}' fill='#f2c94c' stroke='{INK2}'/>")
b.append(f"<text x='{bx}' y='{oy+24}' text-anchor='middle' font-size='14' fill='{INK2}'>ball under wheel</text>")
# trajectories
for col,mk,u,lab in [(BLUE,'b',(math.cos(al),0,math.sin(al)),'what the map assumes (level robot)'),
                     (ORANGE,'o',(math.cos(al+tilt),0,math.sin(al+tilt)),'what actually happens (+5° nose-up)')]:
    pts=[(X(p[0]),Y(LZ+p[2])) for p in traj(v,u)]
    dash="stroke-dasharray='8 6'" if col==BLUE else ''
    b.append(f"<path d='{path(pts)}' fill='none' stroke='{col}' stroke-width='2.5' {dash}/>")
    b.append(f"<circle cx='{pts[-1][0]:.1f}' cy='{pts[-1][1]:.1f}' r='6' fill='{col}' stroke='{SURF}' stroke-width='2'/>")
# launch vectors with angle arcs
L=110
for col,mk,a in [(BLUE,'b',al),(ORANGE,'o',al+tilt)]:
    b.append(f"<line x1='{X(0)}' y1='{Y(LZ)}' x2='{X(0)+L*math.cos(a):.1f}' y2='{Y(LZ)-L*math.sin(a):.1f}' stroke='{col}' stroke-width='3' marker-end='url(#a{mk})'/>")
b.append(f"<line x1='{X(0)}' y1='{Y(LZ)}' x2='{X(0)+90}' y2='{Y(LZ)}' stroke='{MUTED}' stroke-width='1.5' stroke-dasharray='4 4'/>")
b.append(f"<text x='{X(0)-40}' y='{Y(LZ)-70}' text-anchor='end' font-size='15' fill='{ORANGE}'>α + θ</text>")
b.append(f"<text x='{X(0)+44}' y='{Y(LZ)-70}' font-size='15' fill='{BLUE}'>α</text>")
short=traj(v,(math.cos(al+tilt),0,math.sin(al+tilt)))[-1][0]-d
b.append(f"<text x='{X(d+short)-6:.0f}' y='{Y(1.83)+30}' text-anchor='end' font-size='15' font-weight='700' fill='{INK}'>lands {abs(short):.2f} m short</text>")
# legend
ly=70
for i,(col,lab,dash) in enumerate([(BLUE,'Level robot: hood α from the distance map',True),(ORANGE,'Robot pitched 5° along the shot: ball leaves at α + θ',False)]):
    y=ly+i*26; b.append(f"<line x1='24' y1='{y}' x2='60' y2='{y}' stroke='{col}' stroke-width='3' {'stroke-dasharray=\"8 6\"' if dash else ''}/><text x='70' y='{y+5}' font-size='15' fill='{INK}'>{lab}</text>")
b.append(f"<text x='{W-24}' y='{oy+30}' text-anchor='end' font-size='13' fill='{INK2}'>Side view, 3 m shot, α ≈ 72°. Steep shots: more elevation = shorter range.</text>")
open('fig/1-pitch-side-view.svg','w').write(doc(W,H+10,'\n'.join(b),'Pitch: tilting along the shot changes the launch angle'))

# ---------- Fig 2: rear view (looking down the shot), roll across the shot
W,H=900,600; S=120; ox,oy=420,560
X=lambda y:ox+y*S; Y=lambda z:oy-z*S
b=[arrow_defs()]
b.append(f"<line x1='20' y1='{oy}' x2='{W-20}' y2='{oy}' stroke='{INK2}' stroke-width='2'/>")
# hub opening seen far away: draw as bracket at height 1.83 lateral +-0.53
b.append(f"<line x1='{X(-0.53)}' y1='{Y(1.83)}' x2='{X(0.53)}' y2='{Y(1.83)}' stroke='{INK}' stroke-width='5'/>")
b.append(f"<text x='{X(-0.6)}' y='{Y(1.83)+5}' text-anchor='end' font-size='15' fill='{INK2}'>hub opening, 3 m away (≈1 m wide)</text>")
# robot rear view rolled
rw2=0.68; c,s=math.cos(tilt),math.sin(tilt)
base=(-0.34,0.06)
def R2(py,pz):return X(base[0]+py*c-pz*s),Y(base[1]+py*s+pz*c)
corners=[R2(0,0),R2(rw2,0),R2(rw2,0.30),R2(0,0.30)]
b.append(f"<polygon points='{' '.join(f'{x:.1f},{y:.1f}' for x,y in corners)}' fill='#dcdad3' stroke='{INK}' stroke-width='2'/>")
for wy in (0.08,rw2-0.08):
    x,y=R2(wy,-0.03); b.append(f"<rect x='{x-8:.1f}' y='{y-14:.1f}' width='16' height='28' rx='4' fill='{INK2}'/>")
bx2,_=R2(rw2-0.08,0); b.append(f"<circle cx='{bx2:.1f}' cy='{oy-0.07*S:.1f}' r='{0.07*S:.0f}' fill='#f2c94c' stroke='{INK2}'/>")
b.append(f"<text x='{bx2+20}' y='{oy+24}' text-anchor='start' font-size='14' fill='{INK2}'>ball under one side</text>")
# lateral trajectory projection: y vs z
for col,u in [(BLUE,(math.cos(al),0,math.sin(al))),(ORANGE,None)]:
    if u is None:
        T=mm(rx(-tilt),ry(0)); u=mv(T,[math.cos(al),0,math.sin(al)])
    pts=[(X(p[1]),Y(LZ+p[2])) for p in traj(v,u)]
    dash="stroke-dasharray='8 6'" if col==BLUE else ''
    b.append(f"<path d='{path(pts)}' fill='none' stroke='{col}' stroke-width='2.5' {dash}/>")
    b.append(f"<circle cx='{pts[-1][0]:.1f}' cy='{pts[-1][1]:.1f}' r='6' fill='{col}' stroke='{SURF}' stroke-width='2'/>")
lat=pts[-1][0]
b.append(f"<text x='{lat+14:.0f}' y='{Y(1.83)+5}' text-anchor='start' font-size='15' font-weight='700' fill='{INK}'>{abs(traj(v,u)[-1][1]):.2f} m to the side</text>")
ly=70
for i,(col,lab,dash) in enumerate([(BLUE,'Level robot: ball rises straight up the shot plane',True),(ORANGE,'Robot rolled 5°: the steep launch vector leans sideways',False)]):
    y=ly+i*26; b.append(f"<line x1='24' y1='{y}' x2='60' y2='{y}' stroke='{col}' stroke-width='3' {'stroke-dasharray=\"8 6\"' if dash else ''}/><text x='70' y='{y+5}' font-size='15' fill='{INK}'>{lab}</text>")
b.append(f"<text x='{W-24}' y='{oy+50}' text-anchor='end' font-size='13' fill='{INK2}'>Looking down the shot line from behind the robot. Sideways drift ≈ distance × tan α × sin(roll).</text>")
open('fig/2-roll-rear-view.svg','w').write(doc(W,H+30,'\n'.join(b),'Roll: leaning sideways throws a steep shot off-line'))

# ---------- Fig 3: miss vs tilt chart
W,H=900,530; L0,R0,T0,B0=90,220,120,450
xs=[i*0.25 for i in range(41)]
series=[(ORANGE,'Lean across the shot (roll) → sideways miss',[abs(miss(d,al,0,math.radians(t))[1]) for t in xs]),
        (BLUE,'Tilt along the shot (pitch) → short/long miss',[abs(miss(d,al,math.radians(t),0)[0]) for t in xs])]
ymax=1.75
PX=lambda t:L0+(t/10)*(W-R0-L0+130); PY=lambda m:B0-(m/ymax)*(B0-T0)
b=[]
for m in [0,0.25,0.5,0.75,1.0,1.25,1.5]:
    b.append(f"<line x1='{L0}' y1='{PY(m):.1f}' x2='{PX(10):.1f}' y2='{PY(m):.1f}' stroke='{GRID}' stroke-width='1'/>")
    b.append(f"<text x='{L0-10}' y='{PY(m)+5:.1f}' text-anchor='end' font-size='13' fill='{INK2}'>{m:g} m</text>")
for t in range(0,11,2):
    b.append(f"<text x='{PX(t):.1f}' y='{B0+22}' text-anchor='middle' font-size='13' fill='{INK2}'>{t}°</text>")
b.append(f"<line x1='{L0}' y1='{B0}' x2='{PX(10):.1f}' y2='{B0}' stroke='{INK2}' stroke-width='1.5'/>")
b.append(f"<text x='{(L0+PX(10))/2:.0f}' y='{B0+48}' text-anchor='middle' font-size='14' fill='{INK}'>Robot tilt (degrees)</text>")
b.append(f"<text x='24' y='62' font-size='14' fill='{INK2}'>Miss distance at the hub, 3 m shot, no compensation</text>")
# hub radius reference
b.append(f"<line x1='{L0}' y1='{PY(0.53):.1f}' x2='{PX(10):.1f}' y2='{PY(0.53):.1f}' stroke='{INK2}' stroke-width='1.5' stroke-dasharray='3 4'/>")
b.append(f"<text x='{PX(10)-4:.1f}' y='{PY(0.53)+18:.1f}' text-anchor='end' font-size='13' fill='{INK2}'>≈ hub opening radius: past this line the ball misses</text>")
for i,(col,lab,ys) in enumerate(series):
    pts=[(PX(t),PY(m)) for t,m in zip(xs,ys)]
    b.append(f"<path d='{path(pts)}' fill='none' stroke='{col}' stroke-width='2.5'/>")
    b.append(f"<circle cx='{pts[-1][0]:.1f}' cy='{pts[-1][1]:.1f}' r='5' fill='{col}' stroke='{SURF}' stroke-width='2'/>")
    lx=pts[-1][0]+12; b.append(f"<text x='{lx:.1f}' y='{pts[-1][1]+(-8 if i==0 else 18):.1f}' font-size='13' fill='{INK}'>{['roll','pitch'][i]} {ys[-1]:.2f} m</text>")
for i,(col,lab,_) in enumerate(series):
    y=56+i*0
    x=24+i*430
    b.append(f"<line x1='{x}' y1='90' x2='{x+30}' y2='90' stroke='{col}' stroke-width='3'/><text x='{x+38}' y='95' font-size='14' fill='{INK}'>{lab}</text>")
# callout at 3 deg
for t in (3,):
    b.append(f"<line x1='{PX(t):.1f}' y1='{T0}' x2='{PX(t):.1f}' y2='{B0}' stroke='{MUTED}' stroke-width='1' stroke-dasharray='2 4'/>")
    b.append(f"<text x='{PX(t)+6:.1f}' y='{T0+14}' font-size='13' fill='{INK2}'>3° of tilt</text>")
open('fig/3-miss-vs-tilt.svg','w').write(doc(W,H,'\n'.join(b),'A few degrees of tilt is enough to miss the hub'))

# ---------- Fig 4: pipeline
W,H=900,560
b=[arrow_defs()]
steps=[('1','Phase-delay pose prediction',0),('2','Launcher pose (robot_to_launcher)',0),('3','Launcher velocity',0),
       ('4','Lookahead loop → distance, time of flight',0),('5','Heading to target',0),('6','Map lookup → hood α, flywheel',0),
       ('7','Velocity feedforward (filters)',0),('7b','NEW: tilt compensation → hood a, heading + δ',1),('8','Range check / is_valid (+ tilt limits)',2)]
x0,y0,bw,bh,gap=60,70,470,40,11
for i,(n,lab,kind) in enumerate(steps):
    y=y0+i*(bh+gap)
    fill={0:'#efeee9',1:'#dbe9fa',2:'#efeee9'}[kind]; stroke={0:INK2,1:BLUE,2:INK2}[kind]; sw=3 if kind==1 else 1.5
    b.append(f"<rect x='{x0}' y='{y}' width='{bw}' height='{bh}' rx='8' fill='{fill}' stroke='{stroke}' stroke-width='{sw}'/>")
    b.append(f"<text x='{x0+16}' y='{y+26}' font-size='15' fill='{INK}'><tspan font-weight='700'>{n}</tspan>  {lab}</text>")
    if i<len(steps)-1:
        b.append(f"<line x1='{x0+bw/2}' y1='{y+bh}' x2='{x0+bw/2}' y2='{y+bh+gap}' stroke='{INK2}' stroke-width='1.5'/>")
    if kind==2:
        b.append(f"<text x='{x0+bw-12}' y='{y+26}' text-anchor='end' font-size='13' fill='{BLUE}'>changed</text>")
# inputs
iy=y0+7*(bh+gap)
bx=620
b.append(f"<rect x='{bx}' y='{iy-150}' width='240' height='120' rx='8' fill='#efeee9' stroke='{INK2}' stroke-width='1.5'/>")
for j,t in enumerate(['Pigeon2 pitch + roll','(Pigeon2Gyro → SwerveMech','→ MwSwerveSubsystem)','passed in as a Rotation3d']):
    b.append(f"<text x='{bx+16}' y='{iy-122+j*24}' font-size='{15 if j==0 else 13}' font-weight='{700 if j==0 else 400}' fill='{INK if j==0 else INK2}'>{t}</text>")
b.append(f"<path d='M{bx+120},{iy-30} L{bx+120},{iy+bh/2} L{x0+bw+6},{iy+bh/2}' fill='none' stroke='{BLUE}' stroke-width='2.5' marker-end='url(#ab)'/>")
b.append(f"<text x='{bx}' y='{iy+bh+40}' font-size='13' fill='{INK2}'>Runs after step 7 so the hood/heading</text>")
b.append(f"<text x='{bx}' y='{iy+bh+58}' font-size='13' fill='{INK2}'>feedforward stays smooth when the</text>")
b.append(f"<text x='{bx}' y='{iy+bh+76}' font-size='13' fill='{INK2}'>robot rocks.</text>")
open('fig/4-pipeline.svg','w').write(doc(W,H,'\n'.join(b),'Where it goes in LaunchCalculator.calculateLaunchParameters()'))

import glob
for f in glob.glob('fig/*.svg'):
    cairosvg.svg2png(url=f,write_to=f[:-4]+'.png',scale=1.5)
print('done')
