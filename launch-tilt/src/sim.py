import math
g=9.81; h=1.83-0.489
def rx(a):c,s=math.cos(a),math.sin(a);return [[1,0,0],[0,c,-s],[0,s,c]]
def ry(a):c,s=math.cos(a),math.sin(a);return [[c,0,s],[0,1,0],[-s,0,c]]
def mm(A,B):return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(A,v):return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def speed(d,al):return math.sqrt(g*d*d/(2*math.cos(al)**2*(d*math.tan(al)-h)))
def land(v,dirv):
    vx,vy,vz=[v*c for c in dirv]
    disc=vz*vz-2*g*h
    if disc<0:return None
    t=(vz+math.sqrt(disc))/g
    return vx*t,vy*t
def miss(d,al,along,across,hood=None,dyaw=0.0):
    v=speed(d,al); a=al if hood is None else hood
    T=mm(rx(across),ry(-along))  # along>0 = launcher nose up
    u=mv(T,[math.cos(a),0,math.sin(a)])
    c,s=math.cos(dyaw),math.sin(dyaw); u=[c*u[0]-s*u[1],s*u[0]+c*u[1],u[2]]
    x,y=land(v,u);return x-d,y
def comp(al,along,across):
    T=mm(rx(across),ry(-along))
    A=T[2][0];B=T[2][2];R=math.hypot(A,B);phi=math.atan2(A,B)
    a=math.asin(max(-1,min(1,math.sin(al)/R)))-phi
    u=mv(T,[math.cos(a),0,math.sin(a)])
    return a,-math.atan2(u[1],u[0])
for d,al in [(1.5,math.radians(78.5)),(3.0,math.radians(71.8)),(5.0,math.radians(64.2))]:
    print(d,round(math.degrees(al),1),"v",round(speed(d,al),2))
    for deg in [3,5,8]:
        t=math.radians(deg)
        mx,_=miss(d,al,t,0); _,my=miss(d,al,0,t)
        a,dy=comp(al,0,t); a2,dy2=comp(al,t,0)
        print(f"  {deg}deg along: range miss {mx:+.2f}  | across: lateral miss {my:+.2f}  comp across hood {math.degrees(a-al):+.2f} yaw {math.degrees(dy):+.2f} | comp along hood {math.degrees(a2-al):+.2f}  resid {miss(d,al,t,t,*comp(al,t,t))}")
