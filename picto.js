// Pictogram engine: solid geometric athletes + equipment, animated between two key poses.
(function(){
const L={tor:48,ua:29,fa:27,th:40,sh:38};
const R=Math.PI/180,dir=a=>[Math.sin(a*R),Math.cos(a*R)];
const add=(a,b,k=1)=>[a[0]+b[0]*k,a[1]+b[1]*k],sub=(a,b)=>[a[0]-b[0],a[1]-b[1]];
const len=v=>Math.hypot(v[0],v[1]),nrm=v=>{const l=len(v)||1;return[v[0]/l,v[1]/l]},perp=v=>[-v[1],v[0]];
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1],lerp=(a,b,t)=>a+(b-a)*t;
const f=n=>n.toFixed(1);
function ik(root,tgt,a,b,hint){
  let v=sub(tgt,root),d=Math.min(Math.max(len(v),Math.abs(a-b)+.01),a+b-.01);
  const u=nrm(v),A=Math.acos((a*a+d*d-b*b)/(2*a*d));
  const s1=[u[0]*Math.cos(A)-u[1]*Math.sin(A),u[0]*Math.sin(A)+u[1]*Math.cos(A)],
        s2=[u[0]*Math.cos(-A)-u[1]*Math.sin(-A),u[0]*Math.sin(-A)+u[1]*Math.cos(-A)];
  const s=dot(s1,hint)>=dot(s2,hint)?s1:s2,j=add(root,s,a);
  return[j,add(j,nrm(sub(tgt,j)),b)];
}
// tapered limb with muscle bulge
function limb(p,q,r1,rm,r2,bulge=0){
  const d=nrm(sub(q,p)),n=perp(d),m=[(p[0]+q[0])/2,(p[1]+q[1])/2];
  const a=add(p,n,r1),b=add(q,n,r2),c=add(q,n,-r2),e=add(p,n,-r1);
  const k1=add(m,n,rm*1.25+bulge),k2=add(m,n,-rm*1.25+bulge);
  return`M${f(a[0])} ${f(a[1])}Q${f(k1[0])} ${f(k1[1])} ${f(b[0])} ${f(b[1])}A${r2} ${r2} 0 0 1 ${f(c[0])} ${f(c[1])}Q${f(k2[0])} ${f(k2[1])} ${f(e[0])} ${f(e[1])}A${r1} ${r1} 0 0 1 ${f(a[0])} ${f(a[1])}Z`;
}
function smooth(pts){ // closed catmull-rom
  let s=`M${f(pts[0][0])} ${f(pts[0][1])}`;const n=pts.length;
  for(let i=0;i<n;i++){const p0=pts[(i-1+n)%n],p1=pts[i],p2=pts[(i+1)%n],p3=pts[(i+2)%n];
    s+=`C${f(p1[0]+(p2[0]-p0[0])/6)} ${f(p1[1]+(p2[1]-p0[1])/6)} ${f(p2[0]-(p3[0]-p1[0])/6)} ${f(p2[1]-(p3[1]-p1[1])/6)} ${f(p2[0])} ${f(p2[1])}`}
  return s+'Z';
}
const P=(d,c)=>`<path d="${d}" fill="${c}"/>`;
const C=(p,r,c,x='')=>`<circle cx="${f(p[0])}" cy="${f(p[1])}" r="${r}" fill="${c}" ${x}/>`;
const cap=(p,q,w,c,x='')=>`<line x1="${f(p[0])}" y1="${f(p[1])}" x2="${f(q[0])}" y2="${f(q[1])}" stroke="${c}" stroke-width="${w}" stroke-linecap="round" ${x}/>`;
const ln=(p,q,w,c)=>cap(p,q,w,c);
const ell=(p,rx,ry,ang,c,x='')=>`<ellipse cx="${f(p[0])}" cy="${f(p[1])}" rx="${rx}" ry="${ry}" transform="rotate(${f(ang)} ${f(p[0])} ${f(p[1])})" fill="${c}" ${x}/>`;
const angOf=v=>Math.atan2(v[1],v[0])/R;

// interpolate pose objects (numbers + [x,y] arrays)
function mix(A,B,t){const o={};for(const k in A){const a=A[k],b=B[k]??a;
  o[k]=Array.isArray(a)?[lerp(a[0],b[0],t),lerp(a[1],b[1],t)]:typeof a==='number'?lerp(a,b,t):a}return o}

// Solve joints
function solve(p){
  const J={v:p.v||'side'};
  if(J.v==='side'){
    const u=dir(p.tor??180),n=perp(u);J.u=u;J.n=n;J.H=p.H;J.S=add(p.H,u,L.tor);J.head=add(add(J.S,u,19),n,1.5);
    const rel=(v)=>v&&p.rel?add(add(J.S,u,v[0]),n,v[1]):v;
    const hand=rel(p.hand),hand2=rel(p.hand2)||add(hand,[-5,-3]);
    const hint=p.rel&&p.eh?add([0,0],add(u,[0,0],p.eh[0]).map((x,i)=>x+n[i]*p.eh[1])):(p.eh||[0,1]);
    const ua=L.ua*(p.uas||1),fa=L.fa;
    [J.E,J.W]=ik(J.S,hand,ua,fa,hint);[J.E2,J.W2]=ik(add(J.S,[-3,-1]),hand2,ua,fa,p.eh2||hint);
    const foot2=p.foot2||add(p.foot,[-6,0]);
    [J.K,J.F]=ik(J.H,p.foot,L.th,L.sh,p.kh||[1,0]);[J.K2,J.F2]=ik(add(J.H,[-2,-1]),foot2,L.th,L.sh,p.kh2||p.kh||[1,0]);
    J.fd=p.fd??90;J.fd2=p.fd2??J.fd;
  }else{
    const c=p.H[0],Sy=p.H[1]-L.tor,sh=p.shr||0;J.H=p.H;J.c=c;J.Sy=Sy;J.shr=sh;
    J.SR=[c+19,Sy+4-sh];J.SL=[c-19,Sy+4-sh];J.head=[c,Sy-17];
    const hR=p.hand,hL=[2*c-hR[0],hR[1]],eh=p.eh||[1,0],ehL=[-eh[0],eh[1]],ua=L.ua*(p.uas||1);
    [J.ER,J.WR]=ik(J.SR,hR,ua,L.fa,eh);[J.EL,J.WL]=ik(J.SL,hL,ua,L.fa,ehL);
    if(p.seat){J.KR=[c+11,p.H[1]+12];J.KL=[c-11,p.H[1]+12];J.FR=[c+13,p.H[1]+48];J.FL=[c-13,p.H[1]+48]}
    else{const fR=p.foot||[c+12,182];[J.KR,J.FR]=ik([c+9,p.H[1]],fR,L.th,L.sh,[1,0]);[J.KL,J.FL]=ik([c-9,p.H[1]],[2*c-fR[0],fR[1]],L.th,L.sh,[-1,0])}
    J.back=!!p.back;
  }
  return J;
}

// Body drawing
function sideLimbs(J,far,ink,hl,mus){
  const E=far?J.E2:J.E,W=far?J.W2:J.W,K=far?J.K2:J.K,F=far?J.F2:J.F,S=far?add(J.S,[-3,-1]):J.S,H=far?add(J.H,[-2,-1]):J.H,fd=far?J.fd2:J.fd;
  let leg=P(limb(H,K,11,10.5,7.5),ink)+P(limb(K,F,7.5,7,4.8,0),ink)+cap(F,add(F,dir(fd),12),9,ink)+C(K,7.6,ink)+C(H,10.5,ink);
  let arm=P(limb(S,E,7.5,7,5),ink)+P(limb(E,W,5.6,5.4,3.8),ink)+C(W,5,ink)+C(E,5.2,ink);
  if(!far&&hl){
    const side=(p,q,toward,r,neg)=>{const d=nrm(sub(q,p));let n=perp(d);let s=dot(n,sub(toward,p))>0?1:-1;if(neg)s=-s;if(Math.abs(dot(nrm(sub(toward,p)),n))<.12)s=(n[0]*s>0)?1:-1;
      return P(limb(add(p,n,s*r*.38),add(q,n,s*r*.35),r*.62,r*.6,r*.42),hl)};
    if(mus==='biceps')arm+=side(S,E,W,7.5,false);
    if(mus==='triceps')arm+=side(S,E,W,7.5,true);
    if(mus==='quads')leg+=side(H,K,F,11,true);
    if(mus==='hams')leg+=side(H,K,F,11,false);
    if(mus==='calves'){const d=nrm(sub(F,K));let n=perp(d);if(n[0]>0)n=[-n[0],-n[1]];leg+=P(limb(add(K,n,2.5),add(F,n,2),4.6,5,2.6,-1.5),hl)}
    if(mus==='shoulders'||mus==='rear')arm=arm+C(S,7.5,hl);
  }
  return far?leg+arm:{leg,arm};
}
function sideTorso(J,ink,hl,mus){
  const{H,S,u,n}=J,at=(a,b)=>add(add(H,u,a),n,b);
  const pts=[at(-6,-11),at(-3,7),at(16,9),at(32,12.5),at(44,9),S.map((x,i)=>x+u[i]*5),at(46,-9),at(30,-11),at(12,-10.5)];
  let s=P(smooth(pts),ink)+cap(add(S,u,4),add(S,u,13),10,ink)+C(J.head,10.5,ink);
  const ang=angOf(u);
  if(hl){
    if(mus==='chest')s+=ell(at(35,6.5),9,5.2,ang,hl);
    if(mus==='back')s+=ell(at(32,-6),13,5,ang,hl);
    if(mus==='lowback')s+=ell(at(10,-6.5),9,4.2,ang,hl);
    if(mus==='traps')s+=ell(at(49,-4),7,4,ang,hl);
  }
  return s;
}
function frontBody(J,ink,far,hl,mus){
  const{c,Sy,H}=J,sh=J.shr,y=H[1];
  const pts=[[c-13,y+2],[c-12,y-14],[c-15,Sy+24],[c-20,Sy+8-sh],[c-12,Sy-1-sh*.6],[c,Sy-3],[c+12,Sy-1-sh*.6],[c+20,Sy+8-sh],[c+15,Sy+24],[c+12,y-14],[c+13,y+2],[c,y+5]];
  let s='';
  // legs
  for(const[Hp,K,F]of[[[c-8,y],J.KL,J.FL],[[c+8,y],J.KR,J.FR]]){s+=P(limb(Hp,K,9.5,9,7),ink)+P(limb(K,F,7,6.5,4.6),ink)+C(K,7.2,ink)+cap(F,add(F,[Hp[0]<c?-4:4,2]),8,ink)}
  s+=P(smooth(pts),ink)+cap([c,Sy-2],[c,Sy-10],10,ink)+C(J.head,10.5,ink);
  if(J.back)s+=ln([c,Sy+6],[c,y-6],1.6,far)+ell([c-8,Sy+13],5,7,-12,far,'opacity=".55"')+ell([c+8,Sy+13],5,7,12,far,'opacity=".55"');
  if(hl){
    if(mus==='chest')s+=ell([c-8,Sy+12],8,6,-8,hl)+ell([c+8,Sy+12],8,6,8,hl);
    if(mus==='lats')s+=P(`M${c-16} ${Sy+10}L${c-9} ${Sy+18}L${c-12} ${Sy+36}Z`,hl)+P(`M${c+16} ${Sy+10}L${c+9} ${Sy+18}L${c+12} ${Sy+36}Z`,hl);
    if(mus==='traps')s+=P(`M${c-5} ${Sy-3}L${c-17} ${Sy+5-sh}L${c-5} ${Sy+6}Z`,hl)+P(`M${c+5} ${Sy-3}L${c+17} ${Sy+5-sh}L${c+5} ${Sy+6}Z`,hl);
    if(mus==='rear')s+=ell([c,Sy+14],7,6,0,hl);
  }
  for(const[S,E,W]of[[J.SL,J.EL,J.WL],[J.SR,J.ER,J.WR]]){
    s+=P(limb(S,E,7.5,7,5),ink)+P(limb(E,W,5.6,5.4,3.8),ink)+C(W,5,ink)+C(E,5.2,ink);
    if(hl&&(mus==='shoulders'||mus==='rear'))s+=C(S,7.8,hl);
  }
  return s;
}

// Props helpers (t = palette)
const plate=(p,r,t)=>C(p,r,t.k)+C(p,r*.45,t[1])+C(p,r*.18,t.k);
const db=(p,t,r=8)=>C(p,r,t.k)+C(p,r*.4,t[1]);
const dbSide=(J,t)=>{const d=nrm(sub(J.W,J.E)),n=perp(d);return cap(add(J.W,n,-11),add(J.W,n,11),3.5,t.k)+cap(add(J.W,n,-11),add(J.W,n,-6),10,t.k)+cap(add(J.W,n,6),add(J.W,n,11),10,t.k)};
const pad=(p,q,t,w=11)=>cap(p,q,w,t[1]);
const frame=(p,q,t,w=5)=>cap(p,q,w,t[0]);
const floor=t=>`<rect x="0" y="182" width="200" height="18" fill="${t[0]}" opacity=".35"/>`;
const pulley=(p,t)=>C(p,6,t[0])+C(p,2.2,t[3]);
const cable=(p,q,t)=>`<line x1="${f(p[0])}" y1="${f(p[1])}" x2="${f(q[0])}" y2="${f(q[1])}" stroke="${t[0]}" stroke-width="1.6"/>`;
const stack=(x,y,t)=>`<rect x="${x}" y="${y}" width="16" height="44" rx="2" fill="${t[0]}"/>`+[0,1,2,3,4].map(i=>`<rect x="${x+2}" y="${y+3+i*8}" width="12" height="6" rx="1" fill="${t[1]}"/>`).join('');
const seatSide=(J,t,dx=-12,w=34)=>{const y=J.H[1]+12;return pad([J.H[0]+dx,y],[J.H[0]+dx+w,y],t,10)+frame([J.H[0]+dx+w/2,y],[J.H[0]+dx+w/2,182],t)};
const backPad=(J,t,a=-4,b=40,off=13)=>pad(add(add(J.H,J.u,a),J.n,-off),add(add(J.H,J.u,b),J.n,-off),t,11);

// ---------- Exercise library ----------
// each: v, mus, A, B (poses), back(J,t) props behind, front(J,t) props in front, per (s)
const X={};
const side=(o)=>Object.assign({v:'side'},o);
X.butterfly={mus:'chest',A:{v:'front',seat:1,H:[100,128],hand:[150,58],eh:[1,.6]},B:{hand:[109,60],eh:[.2,1],uas:.45},
 back:(J,t)=>frame([36,20],[36,182],t,6)+frame([164,20],[164,182],t,6)+frame([36,20],[164,20],t,6)+`<rect x="83" y="64" width="34" height="66" rx="8" fill="${t[1]}"/>`+pad([78,140],[122,140],t,10)+frame([100,145],[100,182],t),
 front:(J,t)=>cap(J.ER,J.WR,13,t[1],'opacity=".9"')+cap(J.EL,J.WL,13,t[1],'opacity=".9"')};
const inclineP={mus:'chest',A:side({H:[100,140],tor:230,hand:[70,97],eh:[-.3,1],foot:[142,180],kh:[.3,-1]}),B:{hand:[63,55],eh:[-1,0]},
 back:(J,t)=>backPad(J,t,-4,58)+pad([J.H[0]-14,J.H[1]+12],[J.H[0]+14,J.H[1]+12],t,10)+frame([100,152],[100,182],t)+frame([60,130],[60,182],t)};
X['incline-dumbbell-bench-press']={...inclineP,front:(J,t)=>db(J.W,t,9)};
X['incline-barbell-press']={...inclineP,back:(J,t)=>inclineP.back(J,t)+frame([40,30],[40,182],t,4)+frame([34,52],[48,52],t,4),behindArm:(J,t)=>plate(J.W,20,t)};
const flatP={mus:'chest',A:side({H:[128,132],tor:270,hand:[86,115],eh:[0,1],foot:[162,180],kh:[0,-1]}),B:{hand:[82,77],eh:[-1,0]},
 back:(J,t)=>pad([50,147],[140,147],t,12)+frame([62,152],[62,182],t)+frame([128,152],[128,182],t)};
X['flat-dumbbell-bench-press']={...flatP,front:(J,t)=>db(J.W,t,9)};
X['flat-barbell-press']={...flatP,back:(J,t)=>flatP.back(J,t)+frame([70,40],[70,182],t,4)+frame([64,64],[78,64],t,4),behindArm:(J,t)=>plate(J.W,20,t)};
const pushP={mus:'triceps',A:side({H:[86,102],tor:172,hand:[114,64],eh:[-.2,1],foot:[92,181],foot2:[80,181]}),B:{hand:[101,108]},
 back:(J,t)=>`<rect x="150" y="8" width="14" height="174" rx="3" fill="${t[0]}"/>`+stack(132,130,t)+pulley([146,22],t)};
X['ez-handle-push-down']={...pushP,front:(J,t)=>cable([146,22],J.W,t)+cap(add(J.W,[-10,1]),add(J.W,[9,-1]),4,t.k)};
X['rope-push-down']={...pushP,front:(J,t)=>{const k=add(J.W,nrm(sub([146,22],J.W)),12);return cable([146,22],k,t)+C(k,3,t[0])+cap(k,add(J.W,[3,4]),4,t[0])+cap(k,add(J.W,[-4,2]),4,t[0])}};
X['triceps-dips-machine']={mus:'triceps',A:side({H:[92,130],tor:180,hand:[106,104],eh:[-1,0],foot:[142,180],kh:[1,-.3]}),B:{hand:[98,130]},
 back:(J,t)=>pad([72,66],[72,140],t,11)+pad([70,142],[116,142],t,10)+frame([92,146],[92,182],t)+frame([44,40],[44,182],t,6)+frame([44,100],J.W,t,5),
 front:(J,t)=>cap(add(J.W,[-4,0]),add(J.W,[8,0]),7,t.k)};
const pullP={mus:'lats',A:{v:'front',seat:1,H:[100,134],hand:[144,36],eh:[1,0]},B:{hand:[140,82],eh:[.6,1]},
 back:(J,t)=>frame([30,10],[170,10],t,6)+frame([30,10],[30,182],t,6)+pulley([100,14],t)+pad([78,146],[122,146],t,10)+frame([100,150],[100,182],t)};
X['lat-pull-down']={...pullP,front:(J,t)=>cable([100,14],[(J.WL[0]+J.WR[0])/2,J.WR[1]-2],t)+cap(add(J.WL,[-14,2]),add(J.WR,[14,2]),5,t.k)+pad([80,138],[120,138],t,9)};
X['neutral-grip-lat-pull-down-shoulder-width']={...pullP,A:{...pullP.A,hand:[124,32]},B:{hand:[124,82],eh:[.3,1]},
 front:(J,t)=>cable([100,14],[100,J.WR[1]-8],t)+cap([J.WL[0],J.WL[1]-6],[J.WR[0],J.WR[1]-6],4,t.k)+cap(add(J.WL,[0,-6]),add(J.WL,[0,6]),6,t.k)+cap(add(J.WR,[0,-6]),add(J.WR,[0,6]),6,t.k)+pad([80,138],[120,138],t,9)};
X['seated-rows-machine']={mus:'back',A:side({H:[78,132],tor:182,hand:[130,94],eh:[0,1],foot:[150,152],kh:[0,-1]}),B:{hand:[98,106],eh:[-1,.3]},
 back:(J,t)=>pad([56,144],[100,144],t,10)+frame([78,148],[78,182],t)+`<rect x="164" y="40" width="14" height="142" rx="3" fill="${t[0]}"/>`+frame([156,140],[160,168],t,7)+pulley([168,96],t),
 front:(J,t)=>cable([168,96],J.W,t)+cap(add(J.W,[0,-6]),add(J.W,[0,6]),6,t.k)};
const rowP={mus:'back',A:side({H:[70,120],tor:135,hand:[110,140],eh:[1,0],foot:[52,180],kh:[1,0]}),B:{hand:[98,112],eh:[-1,-.5]},
 back:(J,t)=>pad(add(add(J.H,J.u,6),J.n,14),add(add(J.H,J.u,46),J.n,14),t,12)+frame([96,116],[110,182],t)+frame([40,182],[70,176],t,6),behindArm:(J,t)=>plate(J.W,19,t)};
X['chest-supported-barbell-row']=rowP;X['chest-supported-barbell-row-close-grip']=rowP;
X['back-hyperextension-free-weight']={mus:'lowback',A:side({H:[92,110],tor:35,rel:1,hand:[-16,11],eh:[-1,.4],foot:[44,168],kh:[1,-1],fd:180}),B:{tor:140},
 back:(J,t)=>pad([84,126],[104,108],t,12)+frame([94,118],[70,182],t,5)+frame([40,182],[60,160],t,5)+C([42,160],6,t[1]),
 front:(J,t)=>C(add(add(J.S,J.u,-15),J.n,14),10,t.k)+C(add(add(J.S,J.u,-15),J.n,14),4,t[1])};
X['incline-bench-dumbbell-curl']={mus:'biceps',A:side({H:[92,138],tor:208,hand:[64,150],eh:[1,0],foot:[138,180],kh:[0,-1]}),B:{hand:[82,104],eh:[-.2,1]},
 back:(J,t)=>backPad(J,t,-4,56)+pad([J.H[0]-14,J.H[1]+12],[J.H[0]+18,J.H[1]+12],t,10)+frame([96,152],[96,182],t)+frame([60,120],[52,182],t),front:(J,t)=>db(J.W,t)};
X['dumbbell-preacher-curl']={mus:'biceps',A:side({H:[70,134],tor:170,hand:[122,121],eh:[-.3,1],foot:[112,180]}),B:{hand:[104,81]},
 back:(J,t)=>{const d=nrm(sub(J.E,J.S)),n=[-d[1],d[0]];const o=n[1]>0?n:[-n[0],-n[1]];return pad(add(add(J.S,d,10),o,10),add(add(J.E,d,4),o,10),t,12)+frame([102,124],[110,182],t)+seatSide(J,t)},front:(J,t)=>db(J.W,t)};
const legM=(mus,A,B,roll)=>({mus,A:side({H:[86,128],tor:192,hand:[94,132],eh:[0,1],foot:A,kh:[1,-.3]}),B:{foot:B},
 back:(J,t)=>backPad(J,t,-2,46)+pad([62,141],[124,141],t,10)+frame([90,146],[90,182],t)+frame([130,146],[130,182],t,6),
 front:(J,t)=>{const d=nrm(sub(J.F,J.K)),fr=[d[1],-d[0]],p=add(add(J.F,d,-5),fr,roll*9);return frame(J.K,p,t,4)+C(p,7,t[1])+(roll<0?pad([102,116],[126,116],t,8):'')}});
X['leg-extension']=legM('quads',[128,166],[163,121],1);
X['leg-curl']=legM('hams',[163,121],[110,162],-1);
X['leg-press']={mus:'quads',A:side({H:[86,146],tor:240,hand:[90,150],eh:[0,1],foot:[118,114],kh:[-.2,-1],fd:225}),B:{foot:[138,94]},
 back:(J,t)=>backPad(J,t,-4,52)+frame([96,160],[182,74],t,3)+frame([80,160],[80,182],t,6)+frame([60,182],[150,182],t,6),
 front:(J,t)=>{const p=add(J.F,[5,-5]);return cap(add(p,[-20,-20]),add(p,[18,18]),8,t[0])}};
X['hack-squat']={mus:'quads',A:side({H:[92,96],tor:190,hand:[78,50],eh:[0,1],foot:[106,174],kh:[1,0]}),B:{H:[106,126]},
 back:(J,t)=>frame([40,10],[150,182],t,4)+backPad(J,t,-4,50)+cap(add(J.S,J.n,2),add(add(J.S,J.n,10),J.u,-2),8,t[1])+cap([88,180],[132,172],7,t[0])};
X['standing-calf-raise']={mus:'calves',A:side({H:[90,100],tor:180,hand:[104,50],eh:[0,1],foot:[94,170],fd:75}),B:{H:[90,90],hand:[104,40],foot:[94,160],fd:130},
 back:(J,t)=>`<rect x="96" y="170" width="26" height="12" fill="${t[0]}"/>`+frame([60,8],[60,182],t,6)+frame([60,J.S[1]-4],[100,J.S[1]-4],t,4),front:(J,t)=>cap(add(J.S,[-6,-5]),add(J.S,[10,-5]),8,t[1])};
X['seated-calf-machine']={mus:'calves',A:side({H:[84,128],tor:180,hand:[112,112],eh:[0,1],foot:[126,166],kh:[1,-1],fd:75}),B:{foot:[126,157],fd:130},
 back:(J,t)=>seatSide(J,t,-16,34)+`<rect x="126" y="166" width="22" height="16" fill="${t[0]}"/>`+frame([150,60],[150,182],t,6),front:(J,t)=>cap(add(J.K,[-18,-11]),add(J.K,[6,-11]),8,t[1])};
X['overhead-cable-triceps-extension']={mus:'triceps',A:side({H:[86,104],tor:160,hand:[92,47],eh:[.4,-1],foot:[118,180],foot2:[58,180]}),B:{hand:[128,10],eh:[-.3,-1]},
 back:(J,t)=>`<rect x="6" y="10" width="14" height="172" rx="3" fill="${t[0]}"/>`+pulley([24,30],t),front:(J,t)=>cable([24,30],J.W,t)+C(J.W,3.5,t[0])+cap(J.W,add(J.W,[5,5]),4,t[0])};
X['one-arm-dumbbell-row']={mus:'back',A:side({H:[70,112],tor:100,hand:[120,152],eh:[1,0],hand2:[128,141],eh2:[1,0],foot:[90,180],kh:[1,0],foot2:[34,141],kh2:[.6,1],fd2:270}),B:{hand:[102,116],eh:[-1,-1]},
 back:(J,t)=>pad([24,148],[140,148],t,10)+frame([40,152],[40,182],t)+frame([126,152],[126,182],t),front:(J,t)=>db(J.W,t,9)};
X['dumbbell-hammer-curl']={mus:'biceps',A:side({H:[96,102],tor:180,hand:[99,108],eh:[-.3,1],foot:[100,181],foot2:[90,181]}),B:{hand:[106,58],eh:[0,1]},front:(J,t)=>dbSide(J,t)};
X['dumbbell-concentration-curl']={mus:'biceps',A:side({H:[76,132],tor:150,hand:[110,144],eh:[-.5,1],hand2:[118,124],foot:[130,180],kh:[0,-1]}),B:{hand:[112,93],eh:[0,1]},
 back:(J,t)=>seatSide(J,t,-20,40),front:(J,t)=>db(J.W,t)};
X['lateral-dumbbell-raise']={mus:'shoulders',A:{v:'front',H:[100,104],hand:[124,106],eh:[1,0]},B:{hand:[172,64],eh:[0,1]},front:(J,t)=>db(J.WR,t)+db(J.WL,t)};
X['shoulder-press-machine']={mus:'shoulders',A:{v:'front',seat:1,H:[100,134],hand:[140,76],eh:[1,1]},B:{hand:[132,28],eh:[1,0]},
 back:(J,t)=>`<rect x="83" y="62" width="34" height="74" rx="8" fill="${t[1]}"/>`+pad([78,146],[122,146],t,10)+frame([100,150],[100,182],t)+frame([158,14],[158,182],t,6)+frame([42,14],[42,182],t,6),
 front:(J,t)=>cap(J.WR,[158,J.WR[1]],5,t[0])+cap(J.WL,[42,J.WL[1]],5,t[0])+cap(add(J.WR,[0,-6]),add(J.WR,[0,6]),7,t.k)+cap(add(J.WL,[0,-6]),add(J.WL,[0,6]),7,t.k)};
X['reverse-butterfly-rear-delts']={mus:'rear',A:{v:'front',back:1,seat:1,H:[100,132],hand:[112,78],eh:[1,0],uas:.5},B:{hand:[168,82],eh:[0,1],uas:1},
 back:(J,t)=>frame([36,20],[36,182],t,6)+frame([164,20],[164,182],t,6)+frame([36,20],[164,20],t,6)+pad([78,144],[122,144],t,10)+frame([100,148],[100,182],t),
 front:(J,t)=>cap(add(J.WR,[0,-8]),add(J.WR,[0,8]),6,t.k)+cap(add(J.WL,[0,-8]),add(J.WL,[0,8]),6,t.k)};
X['dumbbell-shrugs']={mus:'traps',A:{v:'front',H:[100,104],hand:[123,110],eh:[1,0]},B:{shr:8,hand:[123,102]},front:(J,t)=>db(J.WR,t,9)+db(J.WL,t,9)};
X.treadmill={mus:'quads',per:1.3,A:side({H:[92,100],tor:174,hand:[80,104],eh:[0,1],hand2:[114,98],eh2:[0,1],foot:[116,176],foot2:[70,174],kh:[1,0]}),B:{hand:[114,98],hand2:[80,104],foot:[70,174],foot2:[116,176]},
 back:(J,t,ph)=>`<rect x="34" y="176" width="130" height="10" rx="5" fill="${t[0]}"/>`+[0,1,2,3,4,5].map(i=>{const x=40+((i*22-ph*80)%132+132)%132;return`<rect x="${x.toFixed(1)}" y="179" width="8" height="3" fill="${t[2]}"/>`}).join('')+frame([158,176],[170,92],t,5)+cap([150,90],[178,94],8,t[0])+frame([140,110],[162,112],t,4)};

const DEF={per:2.8};
function draw(key,t,ph,opts={}){
  const ex=X[key];if(!ex)return'';
  const A=ex.A,B={...A,...ex.B},k=opts.still?.55:(1-Math.cos(2*Math.PI*ph))/2;
  const p=mix(A,B,k),J=solve(p),ink=t.k,far=t.far,hl=opts.nohl?null:t.hl;
  let s=floor(t)+(ex.back?ex.back(J,t,ph):'');
  if(J.v==='side'){
    s+=sideLimbs(J,true,far);
    if(ex.behindArm)s+=ex.behindArm(J,t);
    const nl=sideLimbs(J,false,ink,hl,ex.mus);
    s+=nl.leg+sideTorso(J,ink,hl,ex.mus)+nl.arm;
  }else{s+=frontBody(J,ink,far,hl,ex.mus);if(ex.behindArm)s+=ex.behindArm(J,t)}
  if(ex.front)s+=ex.front(J,t,ph);
  return s;
}
window.Picto={X,draw,period:k=>(X[k]&&X[k].per)||DEF.per};
})();
