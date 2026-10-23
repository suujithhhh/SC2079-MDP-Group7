"""Append arrow-plus-bullseye scenes without changing frozen existing records.

Every frame has one black-on-white arrow and one or two original square-ring
bullseyes. Shared-obstacle examples put all cards on one common projected
face. Other examples use separate dark supports. Source splits are retained.
Run only after training that reads this dataset has stopped.
"""
from pathlib import Path
from collections import Counter
import hashlib
import json
import shutil
import cv2
import numpy as np

from arrow_augmentation import _arrow_mask, _camera_degrade, _tight

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT.parent
SEED = 261014
COUNTS = {'train': 1200, 'val': 180, 'test': 180}


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _arrow_patch(rng, cls, holdout):
    glyph, span = _arrow_mask(rng, cls, holdout)
    n = glyph.shape[0]
    yy, xx = np.mgrid[:n, :n].astype(np.float32)
    shade = rng.uniform(228, 252) + rng.uniform(-15, 15) * (xx/n-.5) + rng.uniform(-12, 12) * (yy/n-.5)
    paper = np.repeat(np.clip(shade, 198, 255)[..., None], 3, axis=2)
    alpha = glyph[..., None]/255.
    card = np.clip(paper*(1-alpha) + rng.uniform(0, 25)*alpha, 0, 255).astype(np.uint8)
    return card, glyph, span


def _marker_patch(record, rng):
    source = cv2.imread(record['path'])
    if source is None:
        raise ValueError(record['path'])
    h, w = source.shape[:2]
    x0, y0, x1, y1 = np.asarray(record['box'])*[w, h, w, h]
    x0, y0 = np.maximum(0, np.floor([x0, y0])).astype(int)
    x1, y1 = np.minimum([w, h], np.ceil([x1, y1])).astype(int)
    crop = source[y0:y1, x0:x1]
    if not crop.size:
        raise ValueError('Empty marker crop')
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    low, high = np.percentile(gray, [8, 95])
    gray = np.clip((gray.astype(np.float32)-low)/max(30., high-low)*227+8, 0, 255).astype(np.uint8)
    # Preserve the photographed ring aspect and interior white/black pattern.
    aspect = crop.shape[1]/crop.shape[0]
    gw, gh = int(round(236*min(1., aspect))), int(round(236*min(1., 1/aspect)))
    gw, gh = max(4, gw), max(4, gh)
    ox, oy = (320-gw)//2, (320-gh)//2
    card = np.full((320, 320, 3), int(rng.integers(3, 27)), np.uint8)
    card[oy:oy+gh, ox:ox+gw] = cv2.cvtColor(cv2.resize(gray, (gw, gh)), cv2.COLOR_GRAY2BGR)
    mask = np.zeros((320, 320), np.uint8)
    mask[oy:oy+gh, ox:ox+gw] = 255
    return card, mask, float(max(gw, gh))


def _intersection(a, b):
    return max(0., min(a[2], b[2])-max(a[0], b[0]))*max(0., min(a[3], b[3])-max(a[1], b[1]))


def make_scene(background, marker_records, rng, arrow_class, arrow_size, marker_sizes, shared, holdout=False):
    """Compose all cards in one scene plane, then project them together."""
    bg = cv2.imread(background['path'])
    if bg is None:
        raise ValueError(background['path'])
    bg = cv2.resize(bg, (640, 640), interpolation=cv2.INTER_AREA)
    arrow = _arrow_patch(rng, arrow_class, holdout)
    markers = [_marker_patch(r, rng) for r in marker_records]
    entries = [(arrow_class, arrow, float(arrow_size), 'arrow')]
    entries += [(0, patch, float(size), 'bullseye') for patch, size in zip(markers, marker_sizes)]
    # Arrow between two markers; one-marker layouts alternate sides.
    order = [1, 0, 2] if len(markers) == 2 else ([1, 0] if rng.random() < .5 else [0, 1])
    resized = []
    for cls, (card, mask, span), size, role in entries:
        scale = size/span
        cw, ch = max(6, int(round(card.shape[1]*scale))), max(6, int(round(card.shape[0]*scale)))
        resized.append((cls, card, mask, cw, ch, role))
    gap = float(rng.uniform(12, 27))
    border = float(rng.uniform(10, 22))
    width = int(np.ceil(sum(resized[i][3] for i in order)+gap*(len(order)-1)+2*border))
    max_height = max(r[4] for r in resized)
    height = int(np.ceil(max_height*float(rng.uniform(1.50, 1.90))+2*border))
    ss = 3
    shade = int(rng.integers(10, 48))
    plane = np.full((height*ss, width*ss, 3), shade, np.uint8)
    plane_alpha = np.full((height*ss, width*ss), 255 if shared else 0, np.uint8)
    masks = []
    cursor = border
    for idx in order:
        cls, card, mask, cw, ch, role = resized[idx]
        x = int(round(cursor*ss))
        y = int(round((border+(max_height-ch)/2+rng.uniform(-2, 2))*ss))
        ww, hh = cw*ss, ch*ss
        if not shared:
            # Separate support silhouettes, with room below each card.
            cv2.rectangle(plane_alpha, (max(0,x-5*ss),max(0,y-5*ss)),
                          (min(width*ss-1,x+ww+5*ss),height*ss-1),255,-1)
        plane[y:y+hh,x:x+ww] = cv2.resize(card,(ww,hh),interpolation=cv2.INTER_AREA)
        plane_alpha[y:y+hh,x:x+ww] = 255
        fullmask = np.zeros((height*ss,width*ss),np.uint8)
        fullmask[y:y+hh,x:x+ww] = cv2.resize(mask,(ww,hh),interpolation=cv2.INTER_AREA)
        masks.append((cls,fullmask,role))
        cursor += cw+gap
    source = np.asarray([[0,0],[width*ss-1,0],[width*ss-1,height*ss-1],[0,height*ss-1]],np.float32)
    dest = np.asarray([[-width/2,-height/2],[width/2,-height/2],[width/2,height/2],[-width/2,height/2]],np.float32)
    dest[:,0] *= float(rng.uniform(.82,1.02))
    dest += rng.uniform(-1,1,(4,2)).astype(np.float32)*[width*.028,height*.045]
    theta = float(rng.uniform(-5,5))*np.pi/180
    rot = np.asarray([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]],np.float32)
    dest = dest@rot.T
    radius = np.max(np.abs(dest),axis=0)
    fit = min(1., 278./float(radius.max()))
    dest *= fit
    radius *= fit
    center = np.asarray([rng.uniform(radius[0]+15,625-radius[0]),rng.uniform(radius[1]+15,625-radius[1])],np.float32)
    dest += center
    matrix = cv2.getPerspectiveTransform(source,dest.astype(np.float32))
    # If a new panel partly covers an existing non-target glyph, erase that
    # complete glyph in this copy first, preventing a clipped accidental arrow.
    sx0,sy0,sx1,sy1 = np.asarray(background['box'])*640
    panel_box = [dest[:,0].min(),dest[:,1].min(),dest[:,0].max(),dest[:,1].max()]
    erased = bool(_intersection(panel_box,[sx0,sy0,sx1,sy1]) > 0)
    if erased:
        erase = np.zeros((640,640),np.uint8)
        a,b = np.maximum(0,np.floor([sx0-3,sy0-3])).astype(int)
        c,d = np.minimum(640,np.ceil([sx1+3,sy1+3])).astype(int)
        erase[b:d,a:c] = 255
        bg = cv2.inpaint(bg,erase,3,cv2.INPAINT_TELEA)
    warped = cv2.warpPerspective(plane,matrix,(640,640),flags=cv2.INTER_AREA)
    alpha = cv2.warpPerspective(plane_alpha,matrix,(640,640),flags=cv2.INTER_LINEAR).astype(np.float32)/255
    image = np.clip(bg*(1-alpha[...,None])+warped*alpha[...,None],0,255).astype(np.uint8)
    objects=[]
    for cls,mask,role in masks:
        warped_mask=cv2.warpPerspective(mask,matrix,(640,640),flags=cv2.INTER_LINEAR)
        objects.append({'class':int(cls),'box':_tight(warped_mask)})
    assert len(objects)==1+len(marker_records)
    assert sum(o['class'] in (1,2) for o in objects)==1
    assert sum(o['class']==0 for o in objects)==len(marker_records)
    for i,a in enumerate(objects):
        box=a['box'];assert 0<=box[0]<box[2]<=640 and 0<=box[1]<box[3]<=640
        for b in objects[i+1:]:
            assert _intersection(a['box'],b['box'])==0,(a,b)
    return _camera_degrade(image,rng,min([arrow_size]+list(marker_sizes))),objects,erased


def main():
    cv2.setNumThreads(1)
    manifest_path=ROOT/'data/manifest.json'
    original_bytes=manifest_path.read_bytes()
    manifest=json.loads(original_bytes)
    if any(r['kind'].startswith('white_card_multi_') for r in manifest['records']):
        raise FileExistsError('Multi-object extension already exists; preserve it.')
    original_records=json.dumps(manifest['records'],sort_keys=True)
    source=json.loads((OLD/'audit/source_manifest.json').read_text())
    source_split={r['group']:r['split'] for r in source}
    original_hash=hashlib.sha256(original_bytes).hexdigest()
    snapshot=ROOT/'reports/pre_multi_manifest.json'
    if snapshot.exists():
        assert _sha(snapshot)==original_hash,'Existing pre-multi snapshot differs'
    else:
        snapshot.write_bytes(original_bytes)
    rng=np.random.default_rng(SEED)
    additions=[];preview=[];shared_count=Counter();smaller_arrow=Counter()
    for split,total in COUNTS.items():
        negatives=[r for r in source if r['split']==split and r['old_class'] not in (27,28,30)]
        markers=[r for r in source if r['split']==split and r['old_class']==30]
        assert negatives and markers
        for i in range(total):
            cls=1+i%2
            nmarkers=1+(i//2)%2
            bg=negatives[int(rng.integers(len(negatives)))]
            mark=[markers[int(rng.integers(len(markers)))] for _ in range(nmarkers)]
            arrow_size=float(rng.choice([16,20,24,32,48,64,96],p=[.08,.10,.14,.20,.20,.16,.12]))
            marker_sizes=[float(rng.choice([8,12,16,24,32,48,64,80])) for _ in range(nmarkers)]
            if i%3==0:
                arrow_size=float(rng.choice([16,20,24,32]))
                marker_sizes=[float(rng.choice([48,64,80])) for _ in range(nmarkers)]
            if split=='train':
                arrow_size*=float(rng.uniform(.94,1.07))
                marker_sizes=[s*float(rng.uniform(.95,1.06)) for s in marker_sizes]
            shared=i%5!=0
            image,objects,erased=make_scene(bg,mark,rng,cls,arrow_size,marker_sizes,shared,holdout=split!='train')
            kind=f'white_card_multi_{nmarkers}bull'
            name=f'{kind}_{split}_{i:05d}'
            path=ROOT/'data'/split/'images'/f'{name}.jpg'
            label=ROOT/'data'/split/'labels'/f'{name}.txt'
            # Deterministic crash recovery accepts only byte-identical files
            # from this extension; it never replaces unrelated existing data.
            if path.exists():
                ok, encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,95])
                assert ok and path.read_bytes()==encoded.tobytes(),f'Existing generated image differs: {path}'
            else:
                assert cv2.imwrite(str(path),image,[cv2.IMWRITE_JPEG_QUALITY,95])
            lines=[]
            for obj in objects:
                x0,y0,x1,y1=obj['box']
                lines.append(f"{obj['class']} {(x0+x1)/1280:.8f} {(y0+y1)/1280:.8f} {(x1-x0)/640:.8f} {(y1-y0)/640:.8f}")
            label_text='\n'.join(lines)+'\n'
            if label.exists():
                assert label.read_text()==label_text,f'Existing generated label differs: {label}'
            else:
                label.write_text(label_text)
            groups=sorted(set([bg['group']]+[r['group'] for r in mark]))
            assert all(source_split[g]==split for g in groups)
            row={'path':str(path),'split':split,'kind':kind,'objects':objects,'groups':groups,
                 'background_source':bg['path'],'marker_sources':[r['path'] for r in mark],
                 'nominal_arrow_px':arrow_size,'nominal_marker_px':marker_sizes,'synthetic':True,
                 'shared_obstacle_face':shared,'background_symbol_erased_in_copy':erased,
                 'renderer':'train_polygon' if split=='train' else 'holdout_composite'}
            additions.append(row);shared_count[split]+=int(shared)
            actual_arrow=next(o['box'] for o in objects if o['class'] in (1,2))
            aw=max(actual_arrow[2]-actual_arrow[0],actual_arrow[3]-actual_arrow[1])
            smaller_arrow[split]+=any(max(o['box'][2]-o['box'][0],o['box'][3]-o['box'][1])>aw for o in objects if o['class']==0)
            if i in (0,1,2,3,12,13,14,15):
                show=image.copy()
                for obj in objects:
                    a,b,c,d=map(round,obj['box']);cv2.rectangle(show,(a,b),(c,d),(0,255,0),1)
                    cv2.putText(show,['bull','left','right'][obj['class']],(a,max(13,b-2)),0,.4,(0,255,0),1)
                show=cv2.resize(show,(256,256));cv2.putText(show,f'{split} {nmarkers}bull shared={shared}',(5,250),0,.38,(0,0,255),1);preview.append(show)
            if (i+1)%100==0:
                print('MULTI_BUILD',split,i+1,total,flush=True)
    assert json.dumps(manifest['records'],sort_keys=True)==original_records
    assert manifest_path.read_bytes()==original_bytes,'Dataset manifest changed during generation'
    manifest['records'].extend(additions)
    groups={sp:{g for r in manifest['records'] if r['split']==sp for g in r['groups']} for sp in COUNTS}
    overlap={a+'_'+b:len(groups[a]&groups[b]) for a,b in [('train','val'),('train','test'),('val','test')]}
    assert not any(overlap.values()),overlap
    manifest['counts']={sp:dict(Counter(r['kind'] for r in manifest['records'] if r['split']==sp)) for sp in COUNTS}
    manifest['group_overlap']=overlap
    manifest['notes'].append('Multi-object extension: exactly one arrow plus one/two bullseyes per added frame; 80% share a common obstacle face. All originals retained unchanged.')
    summary={'seed':SEED,'added_images':dict(COUNTS),'added_objects':sum(len(r['objects']) for r in additions),
             'pre_multi_manifest_sha256':original_hash,'pre_multi_records':len(json.loads(original_bytes)['records']),
             'original_records_unchanged':True,'overlapping_added_boxes':0,'group_overlap':overlap,
             'shared_obstacle_images':dict(shared_count),'arrow_smaller_than_marker_images':dict(smaller_arrow),
             'added_class_counts':{sp:dict(Counter(str(o['class']) for r in additions if r['split']==sp for o in r['objects'])) for sp in COUNTS}}
    manifest['multiobject_extension']=summary
    temporary=manifest_path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(manifest,indent=2)+'\n');temporary.replace(manifest_path)
    # Ultralytics label caches must be regenerated after appending labels.
    for split in COUNTS:
        (ROOT/'data'/split/'labels.cache').unlink(missing_ok=True)
    cv2.imwrite(str(ROOT/'reports/multiobject_preview.jpg'),np.vstack([np.hstack(preview[i:i+4]) for i in range(0,len(preview),4)]))
    (ROOT/'reports/multiobject_integrity.json').write_text(json.dumps(summary,indent=2)+'\n')
    (ROOT/'reports/dataset_summary.json').write_text(json.dumps({k:v for k,v in manifest.items() if k!='records'},indent=2)+'\n')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    main()
