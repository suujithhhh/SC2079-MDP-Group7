"""Read legacy labels; produce a task2-only audited detection manifest."""
from pathlib import Path
from collections import defaultdict, Counter
from datetime import datetime
import hashlib, json, re
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2] / 'task1'
OUT = Path(__file__).resolve().parent
TARGETS = {27,28,30}
SEED = 2602

def canonical(stem):
    stem = re.split(r"_(?:jpg|png|jpeg)\.rf\.", stem)[0]
    stem = re.sub(r"^s\d+_", "", stem)
    return re.sub(r"-\d+-$", "", stem)

def fix_arrow(image, polygon, cls):
    H,W = image.shape[:2]
    p = np.array(polygon,dtype=np.float32)*[W,H]
    x1,y1 = p.min(0); x2,y2 = p.max(0)
    ow,oh = x2-x1,y2-y1
    # Legacy polygons normally label only triangular head. Bound search to card glyph region.
    if cls == 28:
        rx1,rx2 = int(max(0,x1-.18*oh)),int(min(W,x2+1.0*oh))
    else:
        rx1,rx2 = int(max(0,x1-1.0*oh)),int(min(W,x2+.18*oh))
    ry1,ry2 = int(max(0,y1-.16*oh)),int(min(H,y2+.16*oh))
    gray = cv2.cvtColor(image[ry1:ry2,rx1:rx2],cv2.COLOR_BGR2GRAY)
    seed = np.zeros_like(gray,np.uint8)
    cv2.fillPoly(seed,[np.round(p-[rx1,ry1]).astype(np.int32)],1)
    vals = gray[seed>0]
    if not len(vals): return None, {"status":"no_seed"}
    oldarea = abs(cv2.contourArea(p.astype(np.float32)))
    otsu,_ = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    thresholds = sorted(set(int(np.clip(t,20,240)) for t in [otsu,np.quantile(vals,.25)-15,np.quantile(vals,.5)*.65,np.quantile(vals,.5)*.80]))
    best = None
    for threshold in thresholds:
        mask = np.uint8(gray>threshold)
        count,components,stats,_ = cv2.connectedComponentsWithStats(mask,8)
        if count<2: continue
        overlaps = np.bincount(components[seed>0],minlength=count); overlaps[0]=0
        component = int(np.argmax(overlaps))
        if component==0: continue
        glyph = np.uint8(components==component)
        contours,_ = cv2.findContours(glyph,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        c = max(contours,key=cv2.contourArea)
        gx,gy,gw,gh = cv2.boundingRect(c); area = cv2.contourArea(c)
        coverage = float(overlaps[component]/max(1,seed.sum()))
        xr,yr,ar = gw/ow,gh/oh,area/max(1,oldarea)
        touches = gx<=0 or gy<=0 or gx+gw>=gray.shape[1] or gy+gh>=gray.shape[0]
        expanded = coverage>=.70 and 1.25<=xr<=3.1 and .78<=yr<=1.36 and .95<=ar<=3.8
        already_full = len(p)>=8 and coverage>=.80 and .85<=xr<=1.20 and .85<=yr<=1.20 and .70<=ar<=1.35
        acceptable = (expanded or already_full) and not touches
        score = coverage - .18*abs(xr-2) - .25*abs(yr-1) -.05*abs(ar-2)
        c = cv2.approxPolyDP(c, max(.35,cv2.arcLength(c,True)*.002),True).reshape(-1,2)+[rx1,ry1]
        corrected = (c/[W,H]).tolist()
        info={"status":"accepted" if acceptable else "ambiguous", "threshold":threshold,"seed_coverage":round(coverage,4),"width_ratio":round(float(xr),4),"height_ratio":round(float(yr),4),"area_ratio":round(float(ar),4),"touches_search_edge":bool(touches), "original_covered_full_arrow":bool(already_full)}
        rank = (acceptable,score)
        if best is None or rank>best[0]:best=(rank,corrected,info)
    return (None,{"status":"no_component"}) if best is None else (best[1],best[2])

def main():
    entries=[]; empty_scenes=set(); empty_records=[]
    for split in ['train','valid','test']:
        for f in sorted((ROOT/split/'labels').glob('*.txt')):
            rows=[r.split() for r in f.read_text().splitlines() if r.strip()]
            key=canonical(f.stem)
            if not rows:
                empty_scenes.add(key); empty_records.append(str(f)); continue
            if len(rows)!=1: raise ValueError(f'Multiple annotations: {f}')
            cls=int(rows[0][0]); p=np.array(list(map(float,rows[0][1:]))).reshape(-1,2)
            image=f.parent.parent/'images'/(f.stem+'.jpg')
            if not image.exists(): raise FileNotFoundError(image)
            entries.append({'path':str(image),'label':str(f),'source_split':split,'scene':key,'old_class':cls,'original_box':[*p.min(0),*p.max(0)],'original_polygon':p.tolist(),'box':[*p.min(0),*p.max(0)],'polygon':p.tolist(),'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest()})
    excluded=[]
    keep=[]
    for e in entries:
        if e['scene'] in empty_scenes:
            excluded.append({**e,'exclusion_reason':'same_scene_as_missing_label'})
            continue
        if e['old_class'] in {27,28}:
            image=cv2.imread(e['path']); corrected,info=fix_arrow(image,e['polygon'],e['old_class'])
            e['arrow_correction']=info
            if info['status']!='accepted':
                excluded.append({**e,'proposed_polygon':corrected,'exclusion_reason':'ambiguous_arrow_component'})
                continue
            e['polygon']=corrected; p=np.array(corrected);e['box']=[*p.min(0),*p.max(0)]
        else:e['arrow_correction']={'status':'not_applicable'}
        keep.append(e)
    print('kept',len(keep),'excluded',len(excluded),'empty',len(empty_records),flush=True)
    # Group all variants from the same event timestamp; adjacent three events stay together.
    events=defaultdict(list)
    for i,e in enumerate(keep):
        m=re.match(r'^(\d{8})_(\d{6})(?:[_-].*)?$',e['scene'])
        event=m.group(1)+'_'+m.group(2) if m else e['scene']
        events[(e['old_class'],event)].append(i)
    parent=list(range(len(keep)))
    def find(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    def union(a,b):
        a,b=find(a),find(b)
        if a!=b:parent[max(a,b)]=min(a,b)
    for cls in range(31):
        seq=sorted((event,idx) for (c,event),idx in events.items() if c==cls)
        block=[];last=None
        def merge_block(block):
            indices=[i for _,ii in block for i in ii]
            for i in indices[1:]:union(indices[0],i)
        for event,idx in seq:
            try:stamp=datetime.strptime(event,'%Y%m%d_%H%M%S')
            except ValueError:stamp=None
            near=stamp is not None and last is not None and 0<=(stamp-last).total_seconds()<=30
            if block and (len(block)>=3 or not near):merge_block(block);block=[]
            block.append((event,idx));last=stamp
        if block:merge_block(block)
    hash_to_first={}
    for i,e in enumerate(keep):
        sha=e['image_sha256']
        if sha in hash_to_first:union(i,hash_to_first[sha])
        else:hash_to_first[sha]=i
    groups=defaultdict(list)
    for i,e in enumerate(keep):groups[find(i)].append(i)
    classgroups=defaultdict(list)
    for indices in groups.values():
        classes={keep[i]['old_class'] for i in indices}
        if len(classes)!=1:raise ValueError('Identical images have contradictory class labels')
        cls=next(iter(classes)); scenes=sorted({keep[i]['scene'] for i in indices})
        gid=f'c{cls:02d}_'+hashlib.sha256('|'.join(scenes).encode()).hexdigest()[:16]
        classgroups[cls].append((gid,indices))
    for cls,gs in classgroups.items():
        gs.sort(key=lambda g:hashlib.sha256(f'{SEED}:{cls}:{g[0]}'.encode()).hexdigest())
        n=len(gs);nv=max(1,round(n*.125));nt=max(1,round(n*.125))
        for j,(gid,indices) in enumerate(gs):
            split='val' if j<nv else 'test' if j<nv+nt else 'train'
            for i in indices:keep[i]['group']=gid;keep[i]['split']=split
    summary={'seed':SEED,'grouping':'All image variants and same camera timestamp grouped; consecutive blocks of up to 3 same-class timestamp events within 30 seconds; exact SHA256 duplicates unioned; stratified by old class into 75/12.5/12.5 group percentages. Adjacent blocks may share settings, so this is not independent deployment data.','source_images':len(entries)+len(empty_records),'empty_labels_excluded':len(empty_records),'ambiguous_or_empty_scene_excluded':len(excluded),'retained':len(keep),'groups':len(groups),'by_split':{},'by_class':{}}
    for split in ['train','val','test']:
        es=[e for e in keep if e['split']==split]
        summary['by_split'][split]={'images':len(es),'groups':len({e['group'] for e in es}),'target_images':sum(e['old_class'] in TARGETS for e in es),'non_target_images':sum(e['old_class'] not in TARGETS for e in es)}
    for cls in range(31):summary['by_class'][cls]={s:sum(e['old_class']==cls and e['split']==s for e in keep) for s in ['train','val','test']}
    OUT.mkdir(exist_ok=True,parents=True)
    (OUT/'source_manifest.json').write_text(json.dumps(keep,indent=2))
    (OUT/'source_manifest_excluded.json').write_text(json.dumps({'empty_label_paths':empty_records,'excluded':excluded},indent=2))
    (OUT/'source_manifest_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)
    print('arrow correction accepted',sum(e['old_class'] in {27,28} for e in keep),'rejected',sum(e['exclusion_reason']=='ambiguous_arrow_component' for e in excluded),flush=True)

if __name__=='__main__':main()
