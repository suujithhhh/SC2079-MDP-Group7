"""Build three-class detection copies and source-disjoint synthetic small-target views.
Never modifies legacy images or labels. Requires audit/build_source_manifest.py first.
"""
from pathlib import Path
import hashlib, json, shutil, random
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]; PROJECT=ROOT.parent/'task1'
NAMES=['bullseye','left','right']; CLASS={30:0,28:1,27:2}
OUT=ROOT/'data'; RNG=np.random.default_rng(260213)

def read_image(rec):
    image=cv2.imread(str(PROJECT/rec['path']))
    if image is None:raise ValueError(rec['path'])
    return image

def labels_write(path, objects, w=640,h=640):
    lines=[]
    for cls,box in objects:
        x0,y0,x1,y1=box
        assert 0<=x0<x1<=w and 0<=y0<y1<=h,(path,box)
        lines.append(f'{cls} {(x0+x1)/2/w:.7f} {(y0+y1)/2/h:.7f} {(x1-x0)/w:.7f} {(y1-y0)/h:.7f}')
    path.write_text('\n'.join(lines)+('\n' if lines else ''))

def glyph_card(rec):
    """Returns a card and full-glyph contour; inner marker rings retain real pixels."""
    source=read_image(rec);h,w=source.shape[:2]
    box=np.asarray(rec['box'])*[w,h,w,h];x0,y0,x1,y1=box
    if rec['old_class']==30:
        a,b,c,d=np.clip(np.round(box).astype(int),0,639)
        crop=source[b:d+1,a:c+1]
        gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
        low,high=np.percentile(gray,[8,94]);gray=np.clip((gray.astype(float)-low)/max(25,high-low)*210+15,0,255).astype('uint8')
        card=np.full((160,160,3),int(RNG.integers(0,35)),np.uint8)
        card[18:142,18:142]=cv2.cvtColor(cv2.resize(gray,(124,124)),cv2.COLOR_GRAY2BGR)
        contour=np.array([[18,18],[142,18],[142,142],[18,142]],np.float32)
    else:
        pts=np.asarray(rec['polygon'],np.float32)*[w,h]
        pts=(pts-[x0,y0])/[max(1,x1-x0),max(1,y1-y0)]
        aspect=np.clip((x1-x0)/(y1-y0),.55,1.5)
        gw=116*min(1,aspect);gh=116*min(1,1/aspect)
        contour=(pts*[gw,gh]+[(160-gw)/2,(160-gh)/2]).astype(np.float32)
        card=np.full((160,160,3),int(RNG.integers(220,256)),np.uint8)
        cv2.fillPoly(card,[np.round(contour).astype(int)],(int(RNG.integers(0,35)),)*3,lineType=cv2.LINE_AA)
    # Slight illumination gradient; the arrow card itself stays white.
    gradient=np.linspace(RNG.uniform(.84,1),1.0,160)[None,:,None]
    return np.clip(card*gradient,0,255).astype(np.uint8),contour

def paste_card(background,rec,target_px,center):
    card,contour=glyph_card(rec)
    # Mild projective deformation without reflection or direction-changing rotation.
    corners=np.array([[0,0],[159,0],[159,159],[0,159]],np.float32)
    jitter=RNG.uniform(-8,8,(4,2)).astype(np.float32)
    dest=corners+jitter
    angle=RNG.uniform(-6,6)*np.pi/180
    rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]],np.float32)
    dest=(dest-80)@rot.T
    dest[:,0]*=RNG.uniform(.78,1.08)
    dest=dest*(target_px/124)+np.asarray(center,np.float32)
    matrix=cv2.getPerspectiveTransform(corners,dest.astype(np.float32))
    warped=cv2.warpPerspective(card,matrix,(640,640),flags=cv2.INTER_LINEAR)
    mask=cv2.warpPerspective(np.full((160,160),255,np.uint8),matrix,(640,640),flags=cv2.INTER_LINEAR)/255
    background[:]=np.round(background*(1-mask[...,None])+warped*mask[...,None]).astype(np.uint8)
    glyph=cv2.perspectiveTransform(contour[None],matrix)[0]
    return [float(glyph[:,0].min()),float(glyph[:,1].min()),float(glyph[:,0].max()),float(glyph[:,1].max())]

def degrade(image):
    if RNG.random()<.3:
        small=int(RNG.choice([320,416,512]));image=cv2.resize(cv2.resize(image,(small,small),interpolation=cv2.INTER_AREA),(640,640))
    if RNG.random()<.25:image=cv2.GaussianBlur(image,(3,3),RNG.uniform(.3,.8))
    image=np.clip(image.astype(np.float32)*RNG.uniform(.8,1.15)+RNG.uniform(-12,12)+RNG.normal(0,RNG.uniform(0,2),image.shape),0,255).astype(np.uint8)
    return image

def main():
    cv2.setNumThreads(1)
    manifest=json.loads((ROOT/'audit/source_manifest.json').read_text())
    records=manifest['records'] if isinstance(manifest,dict) else manifest
    records=[r for r in records if r.get('usable',True)]
    if (OUT/'manifest.json').exists():raise FileExistsError('Dataset already built; preserve it.')
    metadata=[]; summaries={}
    for split in ['train','val','test']:
        rows=[r for r in records if r['split']==split]
        pools={c:[r for r in rows if r['old_class']==c] for c in CLASS}
        negatives=[r for r in rows if r['old_class'] not in CLASS]
        # Keep limited original augmentations per group, all real holdout targets.
        selected=[];count={};seen_scenes=set()
        for r in sorted(rows,key=lambda r:r['path']):
            if r['old_class'] not in CLASS:continue
            if split!='train' and r['scene'] in seen_scenes:continue
            seen_scenes.add(r['scene'])
            count[r['group']]=count.get(r['group'],0)+1
            if split!='train' or count[r['group']]<=6:selected.append(r)
        rng=random.Random(2602+len(split));rng.shuffle(negatives)
        clean_neg=[];seen_neg=set()
        for r in negatives:
            if r['scene'] not in seen_neg:clean_neg.append(r);seen_neg.add(r['scene'])
        selected+=clean_neg[:360 if split=='train' else 120]
        for kind in ['images','labels']:(OUT/split/kind).mkdir(parents=True,exist_ok=True)
        def save(name,im,objects,info):
            path=OUT/split/'images'/f'{name}.jpg'
            cv2.imwrite(str(path),im,[cv2.IMWRITE_JPEG_QUALITY,94]);labels_write(OUT/split/'labels'/f'{name}.txt',objects)
            metadata.append({'path':str(path),'split':split,'kind':info.pop('kind'), 'objects':[{'class':int(c),'box':b} for c,b in objects],**info})
        for i,r in enumerate(selected):
            im=read_image(r);h,w=im.shape[:2];im=cv2.resize(im,(640,640))
            objs=[(CLASS[r['old_class']],(np.array(r['box'])*640).tolist())] if r['old_class'] in CLASS else []
            save(f'real_{i:05d}',im,objs,{'kind':'real','source':r['path'],'groups':[r['group']]})
        synthetic_count=1800 if split=='train' else 216
        for i in range(synthetic_count):
            old=[30,28,27][i%3];target=pools[old][int(RNG.integers(len(pools[old])))];bg=negatives[int(RNG.integers(len(negatives)))]
            im=cv2.resize(read_image(bg),(640,640));groups=[target['group'],bg['group']]
            size=float(RNG.choice([16,24,32,48,64,96])) if split=='train' else float([16,24,32,48,64,96][(i//3)%6])
            if split=='train':size*=RNG.uniform(.85,1.25)
            centers=[(float(RNG.uniform(100,540)),float(RNG.uniform(100,520)))]
            objs=[(CLASS[old],paste_card(im,target,size,centers[0]))]
            if split=='train' and i%5==0:
                old2=[30,28,27][int(RNG.integers(3))];r2=pools[old2][int(RNG.integers(len(pools[old2])))]
                center2=(100 if centers[0][0]>320 else 530,530)
                objs.append((CLASS[old2],paste_card(im,r2,float(RNG.uniform(20,70)),center2)));groups.append(r2['group'])
            im=degrade(im)
            save(f'synth_{i:05d}',im,objs,{'kind':'synthetic','groups':groups,'source':target['path'],'background_source':bg['path'],'nominal_target_px':size})
        summaries[split]={'images':sum(r['split']==split for r in metadata),'real_targets':sum(r['old_class'] in CLASS for r in selected),'real_negatives':sum(r['old_class'] not in CLASS for r in selected),'synthetic_images':synthetic_count,'source_groups':len(set(r['group'] for r in rows))}
    # Verify generated outputs never cross source-group boundaries.
    groups={s:{g for r in metadata if r['split']==s for g in r['groups']} for s in summaries}
    assert not groups['train']&groups['val'] and not groups['train']&groups['test'] and not groups['val']&groups['test']
    (OUT/'data.yaml').write_text(f'path: {OUT}\ntrain: train/images\nval: val/images\ntest: test/images\nnames:\n  0: bullseye\n  1: left\n  2: right\n')
    report={'classes':NAMES,'legacy_ids':{'bullseye':'marker','left':'39','right':'38'},'seed':260213,'source_manifest_sha256':hashlib.sha256((ROOT/'audit/source_manifest.json').read_bytes()).hexdigest(),'counts':summaries,'group_overlap':0,'synthetic_distance_units':'target pixels at 640 input, NOT physical meters','records':metadata}
    (OUT/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    # Small review grid with exact training labels.
    canvas=np.full((4*240,6*240,3),240,np.uint8)
    samples=[r for r in metadata if r['split']=='train' and r['kind']=='synthetic']
    for i,r in enumerate(samples[::73][:24]):
        im=cv2.imread(r['path'])
        for obj in r['objects']:
            a,b,c,d=map(int,obj['box']);cv2.rectangle(im,(a,b),(c,d),(0,255,0),1);cv2.putText(im,NAMES[obj['class']],(a,max(12,b-3)),0,.4,(0,255,0),1)
        canvas[(i//6)*240:(i//6+1)*240,(i%6)*240:(i%6+1)*240]=cv2.resize(im,(240,240))
    cv2.imwrite(str(ROOT/'audit/synthetic_samples.jpg'),canvas)
    print(json.dumps(summaries,indent=2),flush=True)
if __name__=='__main__':main()
