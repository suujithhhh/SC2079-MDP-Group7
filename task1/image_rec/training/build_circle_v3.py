"""Build black-circle v3 data from scene-split screenshots and legacy glyphs.

All recolored/reconstructed cards are synthetic. Evaluation screenshots stay
unchanged. Source files and v2 data are never overwritten.
"""
import json, hashlib
from collections import Counter
from pathlib import Path
import cv2, numpy as np, yaml
from circle_data import ROOT, NAMES, labels, label_for, notebook_geometry, read_source, save_preview, texture
from circle_targeted_synthesis import augment, checker_surface, write_pair

OUT=ROOT/'circle_data_v3'
SEED=20261008
# Fixed capture families: all derivatives from one family stay in one split.
TRAIN_IDS=list(range(2,13))+[16,17,18]
EVAL_IDS=[0,1,13,15,19]

def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def purple_contours(im):
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(112,60,30),(170,255,255))
    cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    return sorted([c for c in cs if cv2.contourArea(c)>300],key=cv2.contourArea,reverse=True)

def ellipse_poly(ellipse,n=64):
    (x,y),(w,h),deg=ellipse;t=np.arange(n)*2*np.pi/n;a=np.deg2rad(deg)
    return np.c_[x+w/2*np.cos(t)*np.cos(a)-h/2*np.sin(t)*np.sin(a),
                 y+w/2*np.cos(t)*np.sin(a)+h/2*np.sin(t)*np.cos(a)].astype(np.float32)

def annotate(i,path):
    im=cv2.imread(str(path));h,w=im.shape[:2]
    if i in (0,1):
        ell=((343.,123.),(37.,38.),0.) if i==0 else ((319.,179.),(200.,203.),0.)
        poly=ellipse_poly(ell);card=None
    else:
        cs=purple_contours(im);c=cs[0]
        if 2<=i<=8:
            ell=cv2.fitEllipse(c);poly=ellipse_poly(ell)
            lo=poly.min(0);hi=poly.max(0);pad=(hi-lo)*.17
            lo-=pad;hi+=pad
            card=np.float32([lo,[hi[0],lo[1]],hi,[lo[0],hi[1]]])
        else:
            x,y,bw,bh=cv2.boundingRect(c)
            # Inside the annotation rectangle, select the dark component nearest
            # its center; exclude card borders and distant dark scene regions.
            margin=max(4,int(min(bw,bh)*.05))
            roi=im[y+margin:y+bh-margin,x+margin:x+bw-margin]
            gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
            _,binary=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
            cc,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            rh,rw=gray.shape
            valid=[]
            for k in cc:
                bx,by,kw,kh=cv2.boundingRect(k)
                if len(k)>=5 and cv2.contourArea(k)>.05*rw*rh and bx>0 and by>0 and bx+kw<rw and by+kh<rh:
                    valid.append(k)
            if not valid:
                manual={12:((520.,277.),(170.,230.),5.),13:((468.,500.),(260.,200.),4.),15:((172.,459.),(90.,134.),10.),16:((602.,429.),(80.,114.),-8.),17:((459.,432.),(105.,110.),8.),18:((517.,460.),(40.,80.),-3.),19:((455.,394.),(98.,75.),0.)}
                if i not in manual: raise ValueError(f'No reliable circle in {i}: {path.name}')
                # Visually audited silhouette: texture joins Otsu component.
                poly=ellipse_poly(manual[i])
            else:
                target=max(valid,key=cv2.contourArea).astype(np.float32)
                target+=np.float32([[[x+margin,y+margin]]])
                poly=ellipse_poly(cv2.fitEllipse(target))
            eps=cv2.approxPolyDP(c,.025*cv2.arcLength(c,True),True).reshape(-1,2)
            if len(eps)==4:
                center=eps.mean(0);angles=np.arctan2(eps[:,1]-center[1],eps[:,0]-center[0]);eps=eps[np.argsort(angles)]
                # Top-left, top-right, bottom-right, bottom-left in image coordinates.
                eps=np.roll(eps,-np.argmin(eps.sum(1)),axis=0);card=eps.astype(np.float32)
            else:card=np.float32([[x,y],[x+bw,y],[x+bw,y+bh],[x,y+bh]])
            center=card.mean(0);card=center+(card-center)*1.035
    row={'index':i,'path':str(path),'sha256':digest(path),'size':[w,h],
         'circle_polygon':poly.tolist(),'card_quad':None if card is None else card.tolist(),
         'split':'train_derivatives' if i in TRAIN_IDS else 'evaluation',
         'has_visible_annotation':i!=0,'synthetic_derivatives_only':i in TRAIN_IDS}
    return im,row

def load_glyph(source,mask_fn):
    parsed=read_source(Path(source),mask_fn)
    if parsed is None:raise ValueError(source)
    im,cls,points,mask=parsed;x,y,w,h=cv2.boundingRect(points)
    side=256;scale=192/max(w,h);dx=(side-w*scale)/2;dy=(side-h*scale)/2
    matrix=np.float32([[scale,0,dx-x*scale],[0,scale,dy-y*scale]])
    out=cv2.warpAffine(mask,matrix,(side,side),flags=cv2.INTER_LINEAR)
    poly=cv2.transform(points[None].astype(np.float32),matrix)[0]
    return cls,out,poly

def user_variant(im,row,glyph,rng,index):
    out=im.copy();h,w=out.shape[:2]
    if 2<=row['index']<=8:
        # These scenes have a second face marker. Remove its complete annotation
        # and tag in the synthetic derivative, so no visible symbol is unlabeled.
        for c in purple_contours(out)[1:]:
            x,y,bw,bh=cv2.boundingRect(c);pad=12
            cv2.rectangle(out,(x-pad,y-pad),(x+bw+pad,y+bh+pad),(18,18,18),-1)
    family=['dense_checker','checker','weave','speckle','glitter','wood','mottle','stripes'][index%8]
    if family=='dense_checker':patch,_=checker_surface(rng,256)
    else:patch=texture(256,rng,family).astype(np.uint8)
    cls,mask,poly=glyph
    patch=patch.astype(np.float32)
    if cls is not None:
        alpha=cv2.GaussianBlur(mask.astype(np.float32)/255,(0,0),.45)[...,None]
        shade=rng.uniform(3,26)+rng.normal(0,1,(256,256,1))
        patch=patch*(1-alpha)+shade*alpha
    card=np.asarray(row['card_quad'],np.float32)
    matrix=cv2.getPerspectiveTransform(np.float32([[0,0],[255,0],[255,255],[0,255]]),card)
    painted=cv2.warpPerspective(patch,matrix,(w,h),borderMode=cv2.BORDER_REPLICATE)
    region=np.zeros((h,w),np.uint8);cv2.fillConvexPoly(region,np.rint(card).astype(np.int32),255)
    alpha=cv2.GaussianBlur(region.astype(np.float32)/255,(0,0),.6)[...,None]
    out=out.astype(np.float32)*(1-alpha)+painted*alpha
    mapped=cv2.perspectiveTransform(poly[None].astype(np.float32),matrix)[0]
    # Preserve aspect ratio, then zoom/translate around the symbol. This changes
    # distance while retaining packaging and scene context from the source.
    natural=640/max(w,h);scale=natural*rng.uniform(.6,1.7)
    center=mapped.mean(0);target=np.float32([rng.uniform(230,410),rng.uniform(190,420)])
    transform=cv2.getRotationMatrix2D(tuple(center.astype(float)),rng.uniform(-5,5),scale)
    transform[:,2]+=target-center
    transformed=cv2.transform(mapped[None],transform)[0]
    if (transformed<8).any() or (transformed>632).any():
        transform=cv2.getRotationMatrix2D(tuple(center.astype(float)),0,natural)
        transform[:,2]+=np.float32([320,320])-center
        transformed=cv2.transform(mapped[None],transform)[0]
    border=tuple(float(v) for v in np.median(im.reshape(-1,3),axis=0))
    out=cv2.warpAffine(out,transform,(640,640),borderMode=cv2.BORDER_CONSTANT,borderValue=border)
    out=np.clip(out*rng.uniform(.75,1.8)+rng.uniform(-4,12),0,255).astype(np.uint8)
    if rng.random()<.6:out=cv2.GaussianBlur(out,(0,0),rng.uniform(.25,.8))
    assert (transformed>=0).all() and (transformed<=640).all()
    return out,cls,transformed/640,{'family':family,'source':row['path'],'source_index':row['index'],
        'affine':transform.tolist(),'circle_black':True,'synthetic':True}

def main():
    cv2.setNumThreads(1)
    OUT.mkdir(exist_ok=True)
    if (OUT/'manifest.json').exists():raise FileExistsError('Dataset already built; do not mutate it')
    paths=sorted((ROOT/'wrong_detected_circle_image').glob('*.png'))
    if len(paths)!=20 or '11.59.39' not in paths[2].name or '12.08.54' not in paths[19].name:
        raise ValueError('Upload changed; review source mapping first')
    audit=[];sources={};annotations=[]
    for i,p in enumerate(paths):
        if i==14:continue # circle clipped by screenshot edge; unreliable full silhouette
        im,row=annotate(i,p);sources[i]=im;annotations.append(row)
        display=im.copy();cv2.polylines(display,[np.rint(row['circle_polygon']).astype(np.int32)],True,(0,255,0),3)
        if row['card_quad'] is not None:cv2.polylines(display,[np.rint(row['card_quad']).astype(np.int32)],True,(0,200,255),2)
        audit.append((f'{i} {row["split"]}',display))
    (OUT/'source_annotations.json').write_text(json.dumps(annotations,indent=2)+'\n')
    save_preview(audit,OUT/'source_annotation_audit.jpg')
    mask_fn,quad_fn=notebook_geometry()
    old_rows=[json.loads(s) for s in (ROOT/'circle_data_v2/samples.jsonl').read_text().splitlines()]
    pools={}
    for split in ['train','val']:
        pools[split]={}
        for row in old_rows:
            if row['split']==split and row['image_id'] is not None:
                pools[split].setdefault(row['image_id'],[])
                if row['source'] not in pools[split][row['image_id']]:pools[split][row['image_id']].append(row['source'])
    # Reuse the existing scene split for legacy-derived cards, never train on val glyphs.
    assert not {p for ps in pools['train'].values() for p in ps}&{p for ps in pools['val'].values() for p in ps}
    rng=np.random.default_rng(SEED);records=[];previews=[]
    circle_mask=np.zeros((256,256),np.uint8);cv2.circle(circle_mask,(128,128),96,255,-1)
    circle_poly=ellipse_poly(((128.,128.),(192.,192.),0.))
    cache={}
    def glyph(ident,split,i):
        if ident=='40':return 29,circle_mask,circle_poly
        if ident=='empty':return None,circle_mask,circle_poly
        p=pools[split][ident][i%len(pools[split][ident])]
        if p not in cache:cache[p]=load_glyph(p,mask_fn)
        return cache[p]
    other=[x for x in pools['train'] if x not in ['40','16','24']]
    for row in annotations:
        if row['index'] not in TRAIN_IDS:continue
        for category,count in [('40',40),('16',12),('24',8),('other',12),('empty',8)]:
            for i in range(count):
                ident=other[i%len(other)] if category=='other' else category
                im,cls,poly,meta=user_variant(sources[row['index']],row,glyph(ident,'train',i),rng,i)
                pair=write_pair(im,cls,poly,f'upload_{row["index"]:02d}_{category}_{i:03d}',OUT/'train/images',OUT/'train/labels',rng)
                records += [dict(r,split='train',category=category,image_id=None if cls is None else NAMES[cls],**meta) for r in pair]
                if row['index'] in (2,11,16) and i<2:previews.append((f'upload {row["index"]} ID {ident}',im))
        print('generated upload scene',row['index'],flush=True)
    for split,counts in [('train',{'40':200,'16':100,'24':60,'other':100,'empty':80}),('val',{'40':60,'16':40,'24':20,'other':28,'empty':40})]:
        other_ids=[x for x in pools[split] if x not in ['40','16','24']]
        for category,count in counts.items():
            for i in range(count):
                ident=other_ids[i%len(other_ids)] if category=='other' else '40' if category=='empty' else category
                source=pools[split][ident][i%len(pools[split][ident])]
                width=float(rng.uniform(24,65) if i%3==0 else rng.uniform(65,220))
                im,cls,poly,meta=augment(source,mask_fn,quad_fn,rng,width,blank=category=='empty')
                pair=write_pair(im,cls,poly,f'dense_{category}_{i:04d}',OUT/split/'images',OUT/split/'labels',rng)
                records += [dict(r,split=split,category=category,image_id=None if cls is None else NAMES[cls],**meta) for r in pair]
        print('generated legacy',split,flush=True)
    # Real screenshots remain separate from synthetic val used by the trainer.
    real=[]
    for row in annotations:
        if row['index'] not in EVAL_IDS:continue
        im=sources[row['index']];h,w=im.shape[:2]
        poly=np.asarray(row['circle_polygon'])/[w,h]
        for size in [max(w,h),160]:
            scale=size/max(w,h);dest=OUT/'real_eval/images'/f'capture_{row["index"]:02d}_r{size}.png'
            dest.parent.mkdir(parents=True,exist_ok=True);(OUT/'real_eval/labels').mkdir(exist_ok=True)
            cv2.imwrite(str(dest),cv2.resize(im,(round(w*scale),round(h*scale)),interpolation=cv2.INTER_AREA))
            label_for(dest).write_text('29 '+' '.join(f'{x:.7f}' for x in poly.ravel())+'\n')
            real.append(dict(path=str(dest),source=row['path'],source_index=row['index'],size=size,annotated=row['has_visible_annotation']))
    parent=ROOT/'circle_data_v2'
    train=(parent/'train.txt').read_text().splitlines()+[r['path'] for r in records if r['split']=='train']
    target=[r['path'] for r in records if r['split']=='val']
    selection=(parent/'selection_val.txt').read_text().splitlines()+target
    for name,items in [('train',train),('selection_val',selection),('target_val',target),('real_eval',[r['path'] for r in real])]:
        assert len(items)==len(set(items));(OUT/f'{name}.txt').write_text('\n'.join(items)+'\n')
    config={'path':str(OUT),'train':str(OUT/'train.txt'),'val':str(OUT/'selection_val.txt'),'nc':31,'names':NAMES}
    (OUT/'data.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    (OUT/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (OUT/'real_eval_manifest.json').write_text(json.dumps(real,indent=2)+'\n')
    save_preview(previews,OUT/'synthetic_preview.jpg')
    manifest={'complete':True,'seed':SEED,'parent':str(parent),'training_images':len(train),'selection_validation_images':len(selection),
      'added_by_category':dict(Counter(f'{r["split"]}/{r["category"]}' for r in records)),'upload_training_source_indices':TRAIN_IDS,'upload_evaluation_source_indices':EVAL_IDS,
      'excluded_source_indices':{'14':'Circle clipped at top of screenshot'},'evaluation_used_for_training':False,'source_hashes':{p.name:digest(p) for p in paths},
      'limitations':['Most inputs are annotated/darkened screenshots. Reconstructed cards are synthetic, not clean captures.',
      'Held-out capture families: original packaging scene and outdoor walkway. Five source screenshots, not ten independent captures.',
      'Legacy dataset contains near duplicates; no claim of independent deployment accuracy.'],
      'builder_sha256':digest(Path(__file__)),'renderer_sha256':digest(ROOT/'circle_targeted_synthesis.py')}
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2),flush=True)

if __name__=='__main__':main()
